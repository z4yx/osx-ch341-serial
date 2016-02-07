//
//  osx_ch341_dev.cpp
//  osx-ch341
//
//  Created by zhang on 16/2/7.
//
//
#include <IOKit/IOLib.h>
#include <IOKit/IOTypes.h>
#include <IOKit/IOMessage.h>


#include "osx_ch341.h"
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/usb/IOUSBInterface.h>
#include <IOKit/usb/IOUSBLog.h>
#include <kern/clock.h>


extern "C" {
#include <pexpert/pexpert.h>
}



IOReturn osx_wch_driver_ch341::ch341_control_out(UInt8 req, UInt16 value, UInt16 index)
{
    IOReturn rtn;
    IOUSBDevRequest request;
    request.bmRequestType = VENDOR_WRITE_REQUEST_TYPE;
    request.bRequest = req;
    request.wValue =  value;
    request.wIndex = index;
    request.wLength = 0;
    request.pData = NULL;
    
    rtn = fpDevice->DeviceRequest(&request);
    DEBUG_IOLog(5,"%s(%p)::ch341_control_out 0x%x:0x%x:0x%x  %d\n", getName(), this,req,value,index,rtn);
    return rtn;
}

IOReturn osx_wch_driver_ch341::ch341_control_in(UInt8 req, UInt16 value, UInt16 index, void *buf, UInt16 len, UInt32 *lenDone)
{
    IOReturn rtn;
    IOUSBDevRequest request;
    request.bmRequestType = VENDOR_READ_REQUEST_TYPE;
    request.bRequest = req;
    request.wValue =  value;
    request.wIndex = index;
    request.wLength = len;
    request.pData = buf;
    
    rtn = fpDevice->DeviceRequest(&request);
    DEBUG_IOLog(5,"%s(%p)::ch341_control_in 0x%x:0x%x:0x%x  %d got:%d\n", getName(), this,req,value,index,rtn,request.wLenDone);
    if(lenDone)
        *lenDone = request.wLenDone;
    return rtn;
}


//
// startSerial
//
// assumes createSerialStream is called once at usb start time
// calls allocateResources to open endpoints
//
bool osx_wch_driver_ch341::startSerial()
{
    const unsigned int startSerial_buf_size = 8;
    uint8_t * buf;
    DEBUG_IOLog(1,"%s(%p)::startSerial ------- \n", getName(), this);
    
    
    
    /* Ugly hack to make device clean */
    /*
     DEBUG_IOLog(5,"%s(%p)::startSerial RESET DEVICE \n", getName(), this);
     fUSBStarted = false;
     DEBUG_IOLog(5,"%s(%p)::startSerial close device-1\n", getName(), this);
     if(fpDevice) { fpDevice->close( fpDevice ); }
     DEBUG_IOLog(5,"%s(%p)::startSerial reset device-1 \n", getName(), this);
     if(fpDevice) { fpDevice->ResetDevice(); }
     
     int i;
     for (i=0; !fUSBStarted && (i < 10);i++)
     IOSleep(10);
     
     DEBUG_IOLog(5,"%s(%p)::startSerial close device-2 timout: %d \n", getName(), this, i);
     if(fpDevice) { fpDevice->close( fpDevice ); }
     DEBUG_IOLog(5,"%s(%p)::startSerial reset device-2 \n", getName(), this);
     if(fpDevice) { fpDevice->ResetDevice(); }
     */
    /*    ****************************     */
    
    
    if (!fNub) {
        IOLog("%s(%p)::startSerial fNub not available\n", getName(), this);
        goto	Fail;
    }
    
    
    buf = (uint8_t *) IOMalloc(startSerial_buf_size);
    if (!buf) {
        IOLog("%s(%p)::startSerial could not alloc memory for buf\n", getName(), this);
        goto	Fail;
    }
    
    ch341_control_in(0x5f, 0, 0,buf,startSerial_buf_size);
    DEBUG_IOLog(5, "Got two bytes 0x%x 0x%x\n",(uint32_t)buf[0],(uint32_t)buf[1]);  /* expect two bytes 0x27 0x00 */

    ch341_control_out (0xa1, 0, 0);
    ch341_set_baudrate(DEFAULT_BAUD_RATE);
    
    ch341_control_in(0x95, 0x2518, 0,buf,startSerial_buf_size);
    DEBUG_IOLog(5, "Got two bytes 0x%x 0x%x\n",(uint32_t)buf[0],(uint32_t)buf[1]); /* expect two bytes 0x56 0x00 */
    
    ch341_control_out (0x9a, 0x2518, 0x0050);
    ch341_get_status(fPort);   /* expect 0xff 0xee */
    
    ch341_control_out (0xa1, 0x501f, 0xd90a);
    ch341_set_baudrate(DEFAULT_BAUD_RATE);
    ch341_set_handshake(fPort->LineControl);
    ch341_get_status(fPort);  /* expect 0x9f 0xee */
    
    IOFree(buf, startSerial_buf_size);
    
    // open the pipe endpoints
    if (!allocateResources() ) {
        IOLog("%s(%p)::start Allocate resources failed\n", getName(), this);
        goto	Fail;
    }
    
    
    startPipes();                           // start reading on the usb pipes
    
    return true;
    
Fail:
    return false;
}

/****************************************************************************************************/
//
//		Function:	SetBreak
//
//		Inputs:		Channel - The port
//				break - true(send break), false(clear break)
//
//		Outputs:
//
//		Desc:		Set and clear line break.
//
/****************************************************************************************************/

IOReturn osx_wch_driver_ch341::setBreak( bool data){
    IOReturn rtn;
    
    const uint16_t ch341_break_reg =
    CH341_REG_BREAK1 | ((uint16_t) CH341_REG_BREAK2 << 8);
    UInt16 reg_contents;
    UInt8  *break_reg;
    int break_state = data;
    
    break_reg = (UInt8*)IOMalloc(2);
    if (!break_reg){
        return kIOReturnNoMemory;
    }
    
    rtn = ch341_control_in(CH341_REQ_READ_REG,
                           ch341_break_reg, 0, break_reg, 2);
    if (rtn != kIOReturnSuccess) {
        DEBUG_IOLog(4,"%s(%p)::setBreak Failed - ch341_control_in return: %d \n", getName(), this,  rtn);
        goto out;
    }
    DEBUG_IOLog(4,"%s(%p)::setBreak - initial ch341 break register contents - reg1: 0x%x, reg2: 0x%x \n",
                getName(), this,  break_reg[0], break_reg[1]);
    
    if (break_state != 0) {
        DEBUG_IOLog(4,"%s(%p)::setBreak Failed - Enter break state requested\n", getName(), this);
        break_reg[0] &= ~CH341_NBREAK_BITS_REG1;
        break_reg[1] &= ~CH341_NBREAK_BITS_REG2;
    } else {
        DEBUG_IOLog(4,"%s(%p)::setBreak Failed - Leave break state requested\n", getName(), this);
        break_reg[0] |= CH341_NBREAK_BITS_REG1;
        break_reg[1] |= CH341_NBREAK_BITS_REG2;
    }
    DEBUG_IOLog(4,"%s(%p)::setBreak - New ch341 break register contents - reg1: 0x%x, reg2: 0x%x \n",
                getName(), this,  break_reg[0], break_reg[1]);
    
    reg_contents = break_reg[0] | (UInt16(break_reg[1]) << 8);
    
    rtn = ch341_control_out(CH341_REQ_WRITE_REG,
                            ch341_break_reg, reg_contents);
out:
    IOFree(break_reg, 2);
    
    DEBUG_IOLog(4,"%s(%p)::setBreak - return: %d \n", getName(), this,  rtn);
    return rtn;
}


IOReturn osx_wch_driver_ch341::ch341_set_baudrate(UInt32 baud_rate)
{
    
    short a, b;
    IOReturn r;
    UInt32 factor;
    short divisor;
    
    IOLog("%s(%p)::ch341_set_baudrate %d\n", getName(), this, baud_rate);
    
    if (!baud_rate)
        return kIOReturnInvalid;
    factor = (CH341_BAUDBASE_FACTOR / baud_rate);
    divisor = CH341_BAUDBASE_DIVMAX;
    
    while ((factor > 0xfff0) && divisor) {
        factor >>= 3;
        divisor--;
    }
    
    if (factor > 0xfff0)
        return kIOReturnInvalid;
    
    factor = 0x10000 - factor;
    a = (factor & 0xff00) | divisor;
    b = factor & 0xff;
    
    r = ch341_control_out(0x9a, 0x1312, a);
    if (!r)
        r = ch341_control_out(0x9a, 0x0f2c, b);
    
    return r;
}

IOReturn osx_wch_driver_ch341::ch341_get_status(PortInfo_t *port)
{
    UInt32 done;
    uint8_t *buf = (uint8_t *) IOMalloc(8);
    if (!buf) {
        IOLog("%s(%p)::ch341_get_status could not alloc memory for buf\n", getName(), this);
        return kIOReturnNoMemory;
    }
    IOReturn rtn = ch341_control_in(0x95, 0x0706, 0, buf, 8, &done);
    if(done == 2){
        IOLog("%s(%p)::ch341_get_status two bytes: 0x%x 0x%x\n", getName(), this, (uint32_t)buf[0], (uint32_t)buf[1]);
        port->lineState = (~(*buf)) & CH341_BITS_MODEM_STAT;
    }
    IOFree(buf, 8);
    return rtn;
}

IOReturn osx_wch_driver_ch341::ch341_set_handshake(UInt8 control)
{
    IOLog("%s(%p)::ch341_set_handshake %d\n", getName(), this, (UInt32)control);
    return ch341_control_out(0xa4, ~control, 0);
}
