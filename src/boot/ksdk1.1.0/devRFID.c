#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devRFID.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kRFIDPinMOSI	= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kRFIDPinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kRFIDPinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kRFIDPinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kRFIDPinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kRFIDPinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kRFIDPinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinCSn);

	return status;
}

static int
readCommand(uint8_t address, uint8_t number_bytes)
{
	spi_status_t status;
    uint8_t read_loc = (address <<1) | 0b10000000;
    uint8_t command_buff[number_bytes];

    /*
     The RFID reader SPI returns data as long as the address is beign provided with the read bit set
     to 1.
    */
    for(int i =0; i < number_bytes; i++)
    {
        command_buff[i] = read_loc;
    }


	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kRFIDPinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kRFIDPinDC);

    /*
    * We add 1 to the number of bytes recieved as the first byte transmitted has no corresponding received
    *data.
    */
    
	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&command_buff[0],
					(uint8_t * restrict)&inBuffer[0],
					number_bytes +1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinCSn);

	return status;
}



int
devRFIDinit(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kRFIDPinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kRFIDPinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kRFIDPinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-RFID-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kRFIDCommandDISPLAYOFF);	// 0xAE
	writeCommand(kRFIDCommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kRFIDCommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kRFIDCommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kRFIDCommandNORMALDISPLAY);	// 0xA4
	writeCommand(kRFIDCommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kRFIDCommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kRFIDCommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kRFIDCommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kRFIDCommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kRFIDCommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kRFIDCommandPRECHARGEB);	// 0x8B
	writeCommand(0x64);
	writeCommand(kRFIDCommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kRFIDCommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3E);
	writeCommand(kRFIDCommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kRFIDCommandMASTERCURRENT);	// 0x87
	writeCommand(0xF);
	writeCommand(kRFIDCommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kRFIDCommandCONTRASTB);		// 0x82
	writeCommand(0xFF);
	writeCommand(kRFIDCommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kRFIDCommandDISPLAYON);		// Turn on oled panel
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kRFIDCommandFILL);
	writeCommand(0x01);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kRFIDCommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");



	/*
	 *	Read the manual for the RFID (RFID_1.2.pdf) to figure
	 *	out how to fill the entire screen with the brightest shade
	 *	of green.
	 */

	writeCommand(kRFIDCommandFILL);
	writeCommand(0x01);

	writeCommand(kRFIDCommandDRAWRECT);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	//Colour lines
	writeCommand(0x00);
	writeCommand(0x3F);
	writeCommand(0x00);
	//Colour fill
	writeCommand(0x00);
	writeCommand(0x3F);
	writeCommand(0x00);



//	SEGGER_RTT_WriteString(0, "\r\n\tDone with draw rectangle...\n");



	return 0;
}
