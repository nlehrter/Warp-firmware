#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devRFID.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];
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


void
initRFID(WarpSPIDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->signalType	= (	kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ |
						kWarpTypeMaskTemperature
					);
	return;
}


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
	enableSPIpins();
	deviceRFIDState.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceRFIDState.spiSourceBuffer,
					(uint8_t * restrict)deviceRFIDState.spiSinkBuffer,
					numberOfBytes /* transfer size */,
					gWarpSpiTimeoutMicroseconds);
	disableSPIpins();

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
read_RFID(uint8_t addr, uint8_t number_of_bytes)
{
	//Address formatted as 0XXXXXX0
	readSensorRegisterRFID(((addr<<1)&0x7E) | 0x80,number_of_bytes+1);
	
	return deviceRFIDState.spiSinkBuffer[1];
}

void
setBitMask(uint8_t addr, uint8_t mask)
{
	uint8_t current = read_RFID(addr,1);
	write_RFID(addr, current | mask);
}
/*
*Reset the device by writing the reset to the command register
*/

void reset_RFID(void){
	write_RFID(CommandReg, MFRC522_SOFTRESET);
}

/*
uint8_t get_version_RFID(void){
	uint8_t response;

  	response = readRFID(VersionReg, 1);

	return response;
}
*/

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
