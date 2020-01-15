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
	 *	Populate the shift-out register with the read-register command,
	 *	followed by the register to be read, followed by a zero byte.
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
	//enableSPIpins();
	deviceRFIDState.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceRFIDState.spiSourceBuffer,
					(uint8_t * restrict)deviceRFIDState.spiSinkBuffer,
					numberOfBytes /* transfer size */,
					gWarpSpiTimeoutMicroseconds);
	//disableSPIpins();

	/*
	 *	Disengage the RFID
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinCSn);

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterRFID(uint8_t deviceRegister, int numberOfBytes)
{	
	//Number of bytes + 1 as 
	return writeSensorRegisterRFID( deviceRegister, 0x00 /* writeValue */, numberOfBytes);
}



void 
write_RFID(uint8_t addr, uint8_t val)
{
	writeSensorRegisterRFID(((addr<<1)&0x7E), val, 2);


}

uint8_t
read_RFID(uint8_t addr)
{
	//Address formatted as 0XXXXXX0
	readSensorRegisterRFID(((addr<<1)&0x7E) | 0x80,2);
	
	return deviceRFIDState.spiSinkBuffer[1];
}

void
setBitMask(uint8_t addr, uint8_t mask)
{
	uint8_t current = read_RFID(addr);
	write_RFID(addr, current | mask);
}

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

uint8_t commandTag(uint8_t cmd, uint8_t *data, int dlen, uint8_t *result, int *rlen)
{
	int status = MI_ERR;
  	uint8_t irqEn = 0x70;
  	uint8_t waitIRq = 0x30;
  	uint8_t lastBits, n;
  	int i;


	write_RFID(CommIEnReg, irqEn|0x80);    // interrupt request
	clearBitMask(CommIrqReg, 0x80);             // Clear all interrupt requests bits.
	setBitMask(FIFOLevelReg, 0x80);             // FlushBuffer=1, FIFO initialization.

	write_RFID(CommandReg, MFRC522_IDLE);  // No action, cancel the current command.

	// Write to FIFO
	for (i=0; i < dlen; i++) {
		write_RFID(FIFODataReg, data[i]);
	}

	// Execute the command.
	write_RFID(CommandReg, cmd);
	if (cmd == MFRC522_TRANSCEIVE) {
		setBitMask(BitFramingReg, 0x80);  // StartSend=1, transmission of data starts
	}

	// Waiting for the command to complete so we can receive data.
	i = 25; // Max wait time is 25ms.
	do {
		OSA_TimeDelay(1); // Currently hoping time delay is in ms, will adjust if not
		// CommIRqReg[7..0]
		// Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = read_RFID(CommIrqReg);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	clearBitMask(BitFramingReg, 0x80);  // StartSend=0

	if (i != 0) { // Request did not time out.
		if(!(read_RFID(ErrorReg) & 0x1B)) {  // BufferOvfl Collerr CRCErr ProtocolErr
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

			// Reading the recieved data from FIFO.
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

	//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
	write_RFID(TModeReg, 0x8D);       // Tauto=1; f(Timer) = 6.78MHz/TPreScaler
	write_RFID(TPrescalerReg, 0x3E);  // TModeReg[3..0] + TPrescalerReg
	write_RFID(TReloadRegL, 30);
	write_RFID(TReloadRegH, 0);

	write_RFID(TxAutoReg, 0x40);      // 100%ASK
	write_RFID(ModeReg, 0x3D);        // CRC initial value 0x6363

	setBitMask(TxControlReg, 0x03);        // Turn antenna on.
	SEGGER_RTT_WriteString(0, "\nSetup nominally complete\n");
	return ; 
}

uint8_t check_tag(uint8_t * idf, int request_status, uint8_t FSM_state)
{
	uint8_t data_rfid[MAX_LEN];
	request_status = request_tag(MF1_REQIDL, data_rfid);
	uint8_t uip[5];
	memcpy(uip, data_rfid, 5);
	if(request_status == MI_OK)
	{
		bool correct = true;
		for (int i = 0; i < 5; i++)
		{
			if (uip[i] != idf[i]){
				correct = false;
				}
		}

		if(correct)
		{
			SEGGER_RTT_WriteString(0, "Same tag, unset\n");
			FSM_state = 1;
		}
		else
		{	
			FSM_state = 3;
			SEGGER_RTT_WriteString(0, "Different tag, alarm triggered\n");
			for (int j = 0; j < 5; j++)
			{
				SEGGER_RTT_printf(0, "\n\r\tRead value for tag is: %d", uip[j]);
			}
		}
	}
		else
		{
			SEGGER_RTT_WriteString(0, "No read, set\n");
		}
		return FSM_state;

}
