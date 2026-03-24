/*
 * VoodooSMBusControllerDriver.cpp
 * SMBus Controller Driver for macOS X
 *
 * Copyright (c) 2019 Leonard Kleinhans <leo-labs>
 *
 * some functions are ported from the linux kernel driver at:
 * https://github.com/torvalds/linux/blob/master/drivers/i2c/i2c-core-smbus.c
 * by Frodo Looijaard <frodol@dds.nl>
 * by Mark Studebaker <mdsxyz123@yahoo.com> and
 * Jean Delvare <jdelvare@suse.de>
 */

#include "VoodooSMBusControllerDriver.hpp"

OSDefineMetaClassAndStructors(VoodooSMBusControllerDriver, IOService)

#define super IOService
#define MILLI_TO_NANO 1000000

bool VoodooSMBusControllerDriver::init(OSDictionary *dict) {
    bool result = super::init(dict);
    
    device_nubs = OSDictionary::withCapacity(2);
    adapter = reinterpret_cast<i801_adapter*>(IOMalloc(sizeof(i801_adapter)));
    awake = true;
    isAMD = false;
    poller = nullptr;
    
    return result;
}

void VoodooSMBusControllerDriver::free(void) {
    IOFree(adapter, sizeof(i801_adapter));
    OSSafeReleaseNULL(device_nubs);
    super::free();
}

IOService *VoodooSMBusControllerDriver::probe(IOService *provider, SInt32 *score) {
    IOService *result = super::probe(provider, score);
    return result;
}

bool VoodooSMBusControllerDriver::start(IOService *provider) {
    bool result = super::start(provider);
    
    pci_device = OSDynamicCast(IOPCIDevice, provider);
    
    if (!(pci_device = OSDynamicCast(IOPCIDevice, provider))) {
        IOLogError("Failed to cast provider");
        return false;
    }

    pci_device->setIOEnable(true);
   
    adapter->pci_device = pci_device;
    adapter->name = getMatchedName(provider);
    adapter->features = 0;
    
    pci_device->retain();
    if (!pci_device->open(this)) {
        IOLogError("%s::%s Could not open provider", getName(), pci_device->getName());
        return false;
    }

    /* Detect AMD FCH KERNCZ SMBus controller (pci1022:790b) */
    uint16_t vendorID = pci_device->configRead16(kIOPCIConfigVendorID);
    uint16_t deviceID = pci_device->configRead16(kIOPCIConfigDeviceID);
    isAMD = (vendorID == AMD_VENDOR_ID) && ((deviceID & 0xFFFF) == AMD_KERNCZ_SMBUS_ID);

    if (isAMD) {
        /*
         * AMD FCH KERNCZ (rev >= 0x49) uses MMIO at 0xFED80300 to report the
         * SMBus I/O base address, instead of a PCI config register.
         * Reference: Linux arch/x86/include/asm/amd/fch.h + drivers/i2c/busses/i2c-piix4.c
         * Confirmed: pci1022:790b rev 0x51 on ThinkPad T14 Gen 1 AMD → smba = 0x0B00
         */
        adapter->smba = getAMDSMBusBase();
        if (adapter->smba == 0) {
            IOLogError("AMD FCH SMBus: failed to read base address from MMIO 0xFED80300");
            return false;
        }
        IOLogInfo("AMD FCH SMBus at I/O 0x%04lx (polling mode)", adapter->smba);

        /*
         * AMD FCH uses SMI for SMBus transactions rather than a PCI IRQ.
         * No PCI interrupt is available, so we omit FEATURE_IRQ and FEATURE_HOST_NOTIFY.
         * The i2c_i801 transaction code falls back to its built-in polling loop when
         * FEATURE_IRQ is not set.  We drive attention delivery via an IOTimerEventSource
         * at 8 ms intervals (125 Hz, matching USB HID polling rate).
         */
        adapter->features |= FEATURE_I2C_BLOCK_READ;
        adapter->features |= FEATURE_SMBUS_PEC;
        adapter->features |= FEATURE_BLOCK_BUFFER;
        adapter->original_hstcfg = 0;
        adapter->original_slvcmd = 0;
    } else {
        /* Intel i801 path */
        uint32_t host_config = pci_device->configRead8(SMBHSTCFG);
        if ((host_config & SMBHSTCFG_HST_EN) == 0) {
            IOLogError("SMBus disabled");
            return false;
        }
        adapter->smba = pci_device->configRead16(ICH_SMB_BASE) & 0xFFFE;
        if (host_config & SMBHSTCFG_SMB_SMI_EN) {
            IOLogError("No PCI IRQ. Poll mode is not implemented. Unloading.");
            return false;
        }
        adapter->original_hstcfg = host_config;
        adapter->original_slvcmd = pci_device->ioRead8(SMBSLVCMD(adapter));
        adapter->features |= FEATURE_I2C_BLOCK_READ;
        adapter->features |= FEATURE_IRQ;
        adapter->features |= FEATURE_SMBUS_PEC;
        adapter->features |= FEATURE_BLOCK_BUFFER;
        adapter->features |= FEATURE_HOST_NOTIFY;
    }

    adapter->retries = 3;
    adapter->timeout = 200 * MILLI_TO_NANO;
    
    work_loop = reinterpret_cast<IOWorkLoop*>(getWorkLoop());
    if (!work_loop) {
        IOLogError("%s Could not get work loop", getName());
        goto exit;
    }

    if (!isAMD) {
        /* Intel: register PCI interrupt */
        interrupt_source = IOInterruptEventSource::interruptEventSource(
            this,
            OSMemberFunctionCast(IOInterruptEventAction, this, &VoodooSMBusControllerDriver::handleInterrupt),
            provider);
        if (!interrupt_source || work_loop->addEventSource(interrupt_source) != kIOReturnSuccess) {
            IOLogError("%s Could not add interrupt source to work loop", getName());
            goto exit;
        }
    }
    
    command_gate = IOCommandGate::commandGate(this);
    if (!command_gate || (work_loop->addEventSource(command_gate) != kIOReturnSuccess)) {
        IOLogError("%s Could not open command gate", getName());
        goto exit;
    }
    adapter->command_gate = command_gate;
    work_loop->retain();
    
    PMinit();
    provider->joinPMtree(this);
    registerPowerDriver(this, VoodooSMBusPowerStates, kVoodooSMBusPowerStates);
    pci_device->enablePCIPowerManagement(kPCIPMCSPowerStateD0);

    publishMultipleNubs();

    if (isAMD) {
        /* AMD: drive attention delivery with a periodic timer */
        poller = IOTimerEventSource::timerEventSource(
            this,
            OSMemberFunctionCast(IOTimerEventSource::Action, this, &VoodooSMBusControllerDriver::pollTimerCallback));
        if (!poller || work_loop->addEventSource(poller) != kIOReturnSuccess) {
            IOLogError("%s Could not add polling timer", getName());
            goto exit;
        }
        poller->setTimeoutMS(8);
        IOLogInfo("AMD FCH SMBus polling active at 8 ms intervals");
    } else {
        interrupt_source->enable();
        enableHostNotify();
    }

    registerService();

    return result;
    
exit:
    releaseResources();
    return false;
}

void VoodooSMBusControllerDriver::releaseResources() {
    if (!isAMD) {
        disableHostNotify();
        pci_device->ioWrite8(SMBHSTCFG, adapter->original_hstcfg);
    }
    
    if (device_nubs) {
        OSCollectionIterator* iterator = OSCollectionIterator::withCollection(device_nubs);
        
        while (VoodooSMBusDeviceNub *device_nub = OSDynamicCast(VoodooSMBusDeviceNub, iterator->getNextObject())) {
            IOLogDebug("Detaching device nub");
            device_nub->detach(this);
        }
        device_nubs->flushCollection();
    }
    
    if (command_gate) {
        work_loop->removeEventSource(command_gate);
        command_gate->release();
        command_gate = NULL;
    }
    
    if (poller) {
        poller->cancelTimeout();
        work_loop->removeEventSource(poller);
        poller->release();
        poller = NULL;
    }
    
    if (interrupt_source) {
        interrupt_source->disable();
        work_loop->removeEventSource(interrupt_source);
        interrupt_source->release();
        interrupt_source = NULL;
    }
    
    OSSafeReleaseNULL(work_loop);
    pci_device->close(this);
    OSSafeReleaseNULL(pci_device);
}


void VoodooSMBusControllerDriver::stop(IOService *provider) {
    releaseResources();
    PMstop();
    super::stop(provider);
}

IOReturn VoodooSMBusControllerDriver::setPowerState(unsigned long whichState, IOService* whatDevice) {
    if (whatDevice != this)
        return kIOPMAckImplied;
    
    if (whichState == kIOPMPowerOff) {
        if (!isAMD) {
            disableHostNotify();
            pci_device->ioWrite8(SMBHSTCFG, adapter->original_hstcfg);
        }
        if (poller) poller->cancelTimeout();
        command_gate->runAction(OSMemberFunctionCast(IOCommandGate::Action, this, &VoodooSMBusControllerDriver::disableCommandGate));
        awake = false;

    } else {
        if (!awake) {
            pci_device->enablePCIPowerManagement(kPCIPMCSPowerStateD0);
            command_gate->enable();
            if (!isAMD) {
                enableHostNotify();
            } else if (poller) {
                poller->setTimeoutMS(8);
            }
            awake = true;
        }
        
    }
    return kIOPMAckImplied;
}

void VoodooSMBusControllerDriver::disableCommandGate() {
    command_gate->disable();
}

/*
 * getAMDSMBusBase - read the SMBus I/O base address from the AMD FCH PM MMIO region.
 *
 * For AMD KERNCZ (pci1022:790b, rev >= 0x49), the SMBus I/O base is stored in
 * FCH PM registers at 0xFED80300, not in a PCI config space BAR:
 *   Byte 0 (smba_en_lo): bit 4 = enable flag
 *   Byte 1 (smba_en_hi): upper byte of the I/O base → piix4_smba = en_hi << 8
 *
 * Confirmed on ThinkPad T14 Gen 1 AMD: smba = 0x0B00 (piix4_smbus at 0xb00).
 * Source: Linux i2c-piix4.c, piix4_sb800_get_ports() / piix4_sb800_use_mmio().
 */
unsigned short VoodooSMBusControllerDriver::getAMDSMBusBase() {
    IOMemoryDescriptor *desc = IOMemoryDescriptor::withPhysicalAddress(
        AMD_FCH_PM_BASE, AMD_FCH_PM_SIZE, kIODirectionIn);
    if (!desc) {
        IOLogError("AMD FCH: failed to create memory descriptor for 0xFED80300");
        return 0;
    }
    if (desc->prepare() != kIOReturnSuccess) {
        IOLogError("AMD FCH: failed to prepare memory descriptor");
        desc->release();
        return 0;
    }
    IOMemoryMap *map = desc->map();
    if (!map) {
        IOLogError("AMD FCH: failed to map 0xFED80300");
        desc->complete();
        desc->release();
        return 0;
    }

    volatile const UInt8 *mmio = reinterpret_cast<volatile const UInt8 *>(map->getVirtualAddress());
    UInt8 en_lo = mmio[0];  /* smba_en_lo */
    UInt8 en_hi = mmio[1];  /* smba_en_hi */

    map->release();
    desc->complete();
    desc->release();

    if (!(en_lo & AMD_SMBA_ENABLE_BIT)) {
        IOLogError("AMD FCH: SMBus not enabled (smba_en_lo=0x%02x)", en_lo);
        return 0;
    }

    return static_cast<unsigned short>(en_hi) << 8;
}

/*
 * pollTimerCallback - periodic attention delivery for AMD (no PCI IRQ available).
 *
 * AMD FCH uses SMI for SMBus host transactions, so there is no PCI interrupt to
 * detect SMBALERT# (host notify) from the touchpad.  We first attempt to read the
 * SMBSLVSTS register; if the hardware-reported host-notify bit is set we dispatch
 * only that event.  Otherwise we deliver an unconditional attention pulse on every
 * tick so that VoodooRMI can drain any pending RMI interrupt data.  This mirrors
 * the 8 ms (125 Hz) polling used by USB HID devices.
 */
void VoodooSMBusControllerDriver::pollTimerCallback(OSObject* owner, IOTimerEventSource* src) {
    if (!awake || !device_nubs) {
        if (poller) poller->setTimeoutMS(8);
        return;
    }

    bool notified = false;

    /* Check hardware-reported host notify (may be cleared by SMI handler on some BIOSes) */
    if (adapter->smba) {
        UInt8 slvsts = pci_device->ioRead8(SMBSLVSTS(adapter));
        if (slvsts & SMBSLVSTS_HST_NTFY_STS) {
            UInt8 addr = pci_device->ioRead8(SMBNTFDADD(adapter)) >> 1;
            char key[5];
            addrToDictKey(addr, key);
            VoodooSMBusDeviceNub *nub = OSDynamicCast(VoodooSMBusDeviceNub, device_nubs->getObject(key));
            if (nub) {
                nub->handleHostNotify();
                notified = true;
            }
            /* clear the notify bit */
            pci_device->ioWrite8(SMBSLVSTS(adapter), SMBSLVSTS_HST_NTFY_STS);
        }
    }

    /*
     * Fallback: if hardware host-notify was not visible (SMI handler cleared it),
     * unconditionally poll the Synaptics RMI nub.  VoodooRMI's handleAttention()
     * reads the RMI interrupt status register and returns immediately when there
     * is no pending data, so this is low-overhead.
     */
    if (!notified) {
        char key[5];
        addrToDictKey(0x2C, key);  /* Synaptics RMI4 address */
        VoodooSMBusDeviceNub *nub = OSDynamicCast(VoodooSMBusDeviceNub, device_nubs->getObject(key));
        if (nub) {
            nub->handleHostNotify();
        }
    }

    if (poller) poller->setTimeoutMS(8);
}


IOReturn VoodooSMBusControllerDriver::publishMultipleNubs() {
    addresses = OSDynamicCast(OSArray, getProperty("Addresses"));
    if (!addresses) {
        return kIOReturnError;
    }
    
    OSIterator *iter = OSCollectionIterator::withCollection(addresses);
    
    while (OSNumber *addr = OSDynamicCast(OSNumber, iter->getNextObject()))
    {
        IOReturn res = publishNub(addr->unsigned8BitValue());
        if (res) {
            return res;
        }
    }
    
    return kIOReturnSuccess;
}

IOReturn VoodooSMBusControllerDriver::publishNub(UInt8 address) {
    
    VoodooSMBusDeviceNub* device_nub = OSTypeAlloc(VoodooSMBusDeviceNub);
    
    if (!device_nub || !device_nub->init()) {
        IOLogError("%s::%s Could not initialise nub", getName(), adapter->name);
        goto exit;
    }
    
    if (!device_nub->attach(this, address)) {
        IOLogError("%s::%s Could not attach nub", getName(), adapter->name);
        goto exit;
    }
    
    if (!device_nub->start(this)) {
        IOLogError("%s::%s Could not start nub", getName(), adapter->name);
        goto exit;
    }
    
    char key[5];
    addrToDictKey(address, key);
    device_nubs->setObject(key, device_nub);
    IOLogDebug("Publishing nub for slave device at address %#04x", address);

    OSSafeReleaseNULL(device_nub);
    return kIOReturnSuccess;
    
exit:
    OSSafeReleaseNULL(device_nub);
    return kIOReturnError;
}

IOWorkLoop* VoodooSMBusControllerDriver::getWorkLoop() {
    // Do we have a work loop already?, if so return it NOW.
    if ((vm_address_t) work_loop >> 1)
        return work_loop;
    
    if (OSCompareAndSwap(0, 1, reinterpret_cast<IOWorkLoop*>(&work_loop))) {
        // Construct the workloop and set the cntrlSync variable
        // to whatever the result is and return
        work_loop = IOWorkLoop::workLoop();
    } else {
        while (reinterpret_cast<IOWorkLoop*>(work_loop) == reinterpret_cast<IOWorkLoop*>(1)) {
            // Spin around the cntrlSync variable until the
            // initialization finishes.
            thread_block(0);
        }
    }
    
    return work_loop;
}


void VoodooSMBusControllerDriver::handleInterrupt(OSObject* owner, IOInterruptEventSource* src, int intCount) {
    u8 status;

    if (adapter->features & FEATURE_HOST_NOTIFY) {
        status = adapter->inb_p(SMBSLVSTS(adapter));
        if (status & SMBSLVSTS_HST_NTFY_STS) {
            UInt8 addr;
            
            addr = adapter->inb_p(SMBNTFDADD(adapter)) >> 1;
            
            /*
             * With the tested platforms, reading SMBNTFDDAT (22 + (p)->smba)
             * always returns 0. Our current implementation doesn't provide
             * data, so we just ignore it.
             */
            
            char key[5];
            addrToDictKey(addr, key);
            VoodooSMBusDeviceNub* nub = OSDynamicCast(VoodooSMBusDeviceNub, device_nubs->getObject(key));
            if (nub) {
                nub->handleHostNotify();
            } else {
                IOLogError("Received Host Notify Interrupt for unknown device at address %#04x", addr);
            }
            
            /* clear Host Notify bit and return */
            adapter->outb_p(SMBSLVSTS_HST_NTFY_STS, SMBSLVSTS(adapter));
            return;
        }
    }
    
    status = adapter->inb_p(SMBHSTSTS(adapter));

    if (status & SMBHSTSTS_BYTE_DONE) {
        i801_isr_byte_done(adapter);
    }
    
    /*
     * Clear irq sources and report transaction result.
     * ->status must be cleared before the next transaction is started.
     */
    status &= SMBHSTSTS_INTR | STATUS_ERROR_FLAGS;
    if (status) {
        adapter->outb_p(status, SMBHSTSTS(adapter));
        adapter->status = status;
        command_gate->commandWakeup(&adapter->status);
    }
}


void VoodooSMBusControllerDriver::enableHostNotify() {
    
    if(!(adapter->original_slvcmd & SMBSLVCMD_HST_NTFY_INTREN)) {
        pci_device->ioWrite8(SMBSLVCMD(adapter), SMBSLVCMD_HST_NTFY_INTREN | adapter->original_slvcmd);
    }

    /* clear Host Notify bit to allow a new notification */
    pci_device->ioWrite8(SMBSLVSTS(adapter), SMBSLVSTS_HST_NTFY_STS);
}

void VoodooSMBusControllerDriver::disableHostNotify() {
    pci_device->ioWrite8(SMBSLVCMD(adapter), adapter->original_slvcmd);
}

IOReturn VoodooSMBusControllerDriver::readByteData(VoodooSMBusSlaveDevice *client, u8 command) {
    union i2c_smbus_data data;
    IOReturn status;
    
    status = transfer(client, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
    if (status != kIOReturnSuccess)
        return status;
    
    return data.byte;
}

IOReturn VoodooSMBusControllerDriver::readBlockData(VoodooSMBusSlaveDevice *client, u8 command, u8 *values) {
    union i2c_smbus_data data;
    IOReturn status;
    
    status = transfer(client, I2C_SMBUS_READ, command, I2C_SMBUS_BLOCK_DATA, &data);
    if (status != kIOReturnSuccess)
        return status;
    
    memcpy(values, &data.block[1], data.block[0]);
    return data.block[0];
}

IOReturn VoodooSMBusControllerDriver::writeByteData(VoodooSMBusSlaveDevice *client, u8 command, u8 value) {
    union i2c_smbus_data data;
    data.byte = value;
    
    return transfer(client, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}


IOReturn VoodooSMBusControllerDriver::writeByte(VoodooSMBusSlaveDevice *client, u8 value) {
    return transfer(client, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}


IOReturn VoodooSMBusControllerDriver::writeBlockData(VoodooSMBusSlaveDevice *client, u8 command,
                                                     u8 length, const u8 *values) {
    union i2c_smbus_data data;
    
    if (length > I2C_SMBUS_BLOCK_MAX)
        length = I2C_SMBUS_BLOCK_MAX;
    data.block[0] = length;
    memcpy(&data.block[1], values, length);
    return transfer(client, I2C_SMBUS_WRITE, command, I2C_SMBUS_BLOCK_DATA, &data);
}

IOReturn VoodooSMBusControllerDriver::transfer(VoodooSMBusSlaveDevice *client, char  read_write, u8 command, int protocol, union i2c_smbus_data *data) {
    VoodooSMBusControllerMessage message = {
        .slave_device = client,
        .read_write = read_write,
        .command = command,
        .protocol = protocol,
    };
    
    return command_gate->runAction(OSMemberFunctionCast(IOCommandGate::Action, this, &VoodooSMBusControllerDriver::transferGated), &message, data);
}

// __i2c_smbus_xfer
IOReturn VoodooSMBusControllerDriver::transferGated(VoodooSMBusControllerMessage *message, union i2c_smbus_data *data) {
    int _try;
    s32 res;

    VoodooSMBusSlaveDevice* slave_device = message->slave_device;
    slave_device->flags &= I2C_M_TEN | I2C_CLIENT_PEC | I2C_CLIENT_SCCB;
    
    /* Retry automatically on arbitration loss */
    for (res = 0, _try = 0; _try <= adapter->retries; _try++) {
        res = i801_access(adapter, slave_device->addr, slave_device->flags, message->read_write, message->command, message->protocol, data);
        if (res != -EAGAIN)
            break;
    }
    
    return res;
}
