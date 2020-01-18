/*
	Authored 2016-2018. Phillip Stanley-Marbell.
	
	Additional contributions, 2018: Jan Heck, Chatura Samarakoon, Youchao Wang, Sam Willis.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/


/*
Copyright © 2020 Bjarte Johansen <Bjarte.Johansen@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the 
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Based on code by Dr.Leong ( WWW.B2CQSHOP.COM ) and
Miguel Balboa (https://github.com/miguelbalboa/rfid).
*/


#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devRFID.h"

extern volatile uint32_t		gWarpSPIBaudRateKbps;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;
extern volatile WarpSPIDeviceState	deviceRFIDState;

/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kRFIDPinMOSI	= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kRFIDPinMISO	= GPIO_MAKE_PIN(HW_GPIOA, 6),
	kRFIDPinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kRFIDPinCSn		= GPIO_MAKE_PIN(HW_GPIOA, 5),
	kRFIDPinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kRFIDPinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};


WarpStatus
writeSensorRegisterRFID(uint8_t deviceRegister, uint8_t writeValue, int numberOfBytes)
{	
	/*
	 *	Populate the shift-out register with the register to read,
	 *	followed by a the byte to be written.
	 */
	deviceRFIDState.spiSourceBuffer[0] = deviceRegister;
	deviceRFIDState.spiSourceBuffer[1] = writeValue;

	deviceRFIDState.spiSinkBuffer[0] = 0x00;
	deviceRFIDState.spiSinkBuffer[1] = 0x00;

	/*
	 *	First, create a falling edge on chip-select.
	 *
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinCSn);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kRFIDPinCSn);

	/*
	 *	The result of the SPI transaction will be stored in deviceRFIDState.spiSinkBuffer.
	 *
	 *	Providing a device structure here is optional since it 
	 *	is already provided when we did SPI_DRV_MasterConfigureBus(),
	 *	so we pass in NULL.
	 *
	 *	TODO: the "master instance" is always 0 for the KL03 since
	 *	there is only one SPI peripheral. We however should remove
	 *	the '0' magic number and place this in a Warp-HWREV0 header
	 *	file.
	 */
	deviceRFIDState.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceRFIDState.spiSourceBuffer,
					(uint8_t * restrict)deviceRFIDState.spiSinkBuffer,
					numberOfBytes /* transfer size */,
					gWarpSpiTimeoutMicroseconds);

	/*
	 *	Disengage the RFID
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinCSn);

	return kWarpStatusOK;
}

/*
* To read, execute a write and then read the SPI buffer
*/
WarpStatus
readSensorRegisterRFID(uint8_t deviceRegister, int numberOfBytes)
{	

	return writeSensorRegisterRFID( deviceRegister, 0x00 /* writeValue */, numberOfBytes);
}



void 
write_RFID(uint8_t addr, uint8_t val)
{
    /*
    * Apply the required bit mask for the address
    */
	writeSensorRegisterRFID(((addr<<1)&0x7E), val, 2);


}

uint8_t
read_RFID(uint8_t addr)
{
	/*
    * Apply the required bit mask for the address
    */
	readSensorRegisterRFID(((addr<<1)&0x7E) | 0x80,2);
	
	return deviceRFIDState.spiSinkBuffer[1];
}

/*
* Set a mask onto bits in a register
*/
void
setBitMask(uint8_t addr, uint8_t mask)
{
	uint8_t current = read_RFID(addr);
	write_RFID(addr, current | mask);
}

/*
* Clear a mask onto bits in a register
*/
void 
clearBitMask(uint8_t addr, uint8_t mask)
{
	uint8_t current;
	current = read_RFID(addr);
	write_RFID(addr, current & (~mask));
}

void
reset_RFID(void)
{
	write_RFID(CommandReg, MFRC522_SOFTRESET);
}

uint8_t
getFirmwareVersion(void)
{
	volatile uint8_t response;
	response = read_RFID(VersionReg);
	return response;

}

/*
* Detect whether a tag is present in range of the reader
*/
uint8_t request_tag(uint8_t mode, uint8_t *data)
{	
	int status, len;
	write_RFID(BitFramingReg, 0x07);
	data[0] = mode;

	status = commandTag(MFRC522_TRANSCEIVE, data, 1, data, &len);

	if((status != MI_OK) || (len != 0x10)){
		status = MI_ERR;
	};

	return status;
}

/*
* Receive a serial ID from the tag, commonly used to avoid
* tag collisions when reading. Applied here to arm and disarm 
* the bike alarm
*/
uint8_t anti_collision(uint8_t *serial){
	int status, i, len;
	uint8_t check = 0x00;

	write_RFID(BitFramingReg, 0x00);

	serial[0] = MF1_ANTICOLL;
	serial[1] = 0x20;
	status = commandTag(MFRC522_TRANSCEIVE, serial, 2, serial, &len);

	len = len/8;
	if(status == MI_OK){

		for(i=0; i<len-1; i++){
			check ^= serial[i];
		}
		if (check != serial[i]){
			status = MI_ERR;
		}
	}

	return status;

}

/*
* Send a command to the tag
*/
uint8_t commandTag(uint8_t cmd, uint8_t *data, int dlen, uint8_t *result, int *rlen)
{
	int status = MI_ERR;
  	//uint8_t irqEn = 0x70;
	uint8_t irqEn = 0x77;
  	uint8_t waitIRq = 0x30;
  	uint8_t lastBits, n;
  	int i;


	write_RFID(CommIEnReg, irqEn|0x80);    /* interrupt request */
	clearBitMask(CommIrqReg, 0x80);             /* Clear interrupt requests bits. */
	setBitMask(FIFOLevelReg, 0x80);             /* FlushBuffer=1, FIFO initialization */

	write_RFID(CommandReg, MFRC522_IDLE);  // Cancel current command

	/* Write data to FIFO */
	for (i=0; i < dlen; i++) {
		write_RFID(FIFODataReg, data[i]);
	}

	/* Execute the command. */
	write_RFID(CommandReg, cmd);
	if (cmd == MFRC522_TRANSCEIVE) {
		setBitMask(BitFramingReg, 0x80);  // StartSend=1, transmission of data starts
	}

	/* Waiting for the command to complete so we can receive data. */
	i = 25; 
	do {
		OSA_TimeDelay(1); 
		n = read_RFID(CommIrqReg);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	clearBitMask(BitFramingReg, 0x80);  /* StartSend=0 */

	if (i != 0) { /* Request receieved a reply from tag reader. */
		if(!(read_RFID(ErrorReg) & 0x1B)) {  /* No errors were generated in the command */
		status = MI_OK;
		if (n & irqEn & 0x01) {
			status = MI_NOTAGERR;
		}

		if (cmd == MFRC522_TRANSCEIVE) {
			n = read_RFID(FIFOLevelReg);
			lastBits = read_RFID(ControlReg) & 0x07;
			if (lastBits) {
				*rlen = (n-1)*8 + lastBits;
			} else {
				*rlen = n*8;
			}

			if (n == 0) {
				n = 1;
			}

			if (n > MAX_LEN) {
				n = MAX_LEN;
			}

			/* Read the recieved data from FIFO */
			for (i=0; i<n; i++) {
			result[i] = read_RFID(FIFODataReg);
			}
		}
		} else {
		status = MI_ERR;
		}
	}
	return status;
}


void
devRFIDinit(WarpSPIDeviceState volatile *  deviceStatePointer){
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 6u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 5u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	// Set CS pin high
	GPIO_DRV_SetPinOutput(kRFIDPinCSn);

	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kRFIDPinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kRFIDPinRST);
	OSA_TimeDelay(100);


	//reset();
    /* Set up as described in C++ library ported to Warp */

	/*Timer: TPrescaler*TreloadVal/6.78MHz = 24ms */
	write_RFID(TModeReg, 0x8D);       /* Tauto=1; f(Timer) = 6.78MHz/TPreScaler */
	write_RFID(TPrescalerReg, 0x3E);  /* TModeReg[3..0] + TPrescalerReg */
	write_RFID(TReloadRegL, 30);
	write_RFID(TReloadRegH, 0);

	write_RFID(TxAutoReg, 0x40);      /* 100%ASK */
	write_RFID(ModeReg, 0x3D);        /* CRC initial value 0x6363 */

	setBitMask(TxControlReg, 0x03);        /* Turn antenna on */

	return ; 
}
