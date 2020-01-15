/*
	Authored 2016-2018. Phillip Stanley-Marbell, Youchao Wang.

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
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceINAState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initINA(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ |
						kWarpTypeMaskTemperature
					);
	return;
}



WarpStatus
writeSensorRegisterINA(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	status;

	payloadByte[0] = payload >> 8 & 0xFF;
	payloadByte[1] = payload & 0xFF;

	switch (deviceRegister)
	{
		case 0x00: case 0x05: 
		
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceINAState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload >> 8 ;
	payloadByte[1] = payload & 0xFF;

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							2,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
WriteCalibINA(uint16_t payload,uint16_t menuI2cPullupValue)
{
	enableI2Cpins(menuI2cPullupValue);
	writeSensorRegisterINA(0x05, payload, menuI2cPullupValue);

}
void
WriteConfigINA(uint16_t payload,uint16_t menuI2cPullupValue)
{
	enableI2Cpins(menuI2cPullupValue);
	writeSensorRegisterINA(0x00, payload, menuI2cPullupValue);

}

/*
WarpStatus
configureSensorINA(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1, uint16_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;

	i2cWriteStatus1 = writeSensorRegisterINA(kWarpSensorConfigurationRegisterINAF_SETUP ,
							payloadF_SETUP ,
							menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterINA(kWarpSensorConfigurationRegisterINACTRL_REG1 ,
							payloadCTRL_REG1 ,
							menuI2cPullupValue);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}
*/

WarpStatus
readSensorRegisterINA(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00:  case 0x01: case 0x02:
		case 0x03: case 0x04: case 0x05:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}


	i2c_device_t slave =
	{
		.address = deviceINAState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceINAState.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

uint16_t
ReadCurrentINA(uint16_t menuI2cPullupValue)
{
	enableI2Cpins(menuI2cPullupValue);
	readSensorRegisterINA(0x04, 2);

	uint8_t readSensorRegisterValueMSB = deviceINAState.i2cBuffer[0];
	uint8_t readSensorRegisterValueLSB = deviceINAState.i2cBuffer[1];

	uint16_t current_value = ((readSensorRegisterValueMSB &0xFF) <<8) | (readSensorRegisterValueLSB & 0xFF);
	return current_value;


}

/*
SEGGER_RTT_WriteString(0, "\nWe have arrived at the register read location\n");
				struct machine_states{
					uint8_t FSM;
					uint8_t request_status;
				}machine_state;
				
				devRFIDinit(&deviceRFIDState);

				uint8_t uid[5];
				machine_state.FSM = 1;
				machine_state.request_status = MI_ERR;

				while(1){
					uint8_t data_rfid[MAX_LEN];

					switch(machine_state.FSM){
						case(1):
							
							machine_state.request_status = request_tag(MF1_REQIDL, data_rfid);
							if(machine_state.request_status == MI_OK){
								memcpy(uid, data_rfid, 5);
								for (int j = 0; j < 5; j++){
									SEGGER_RTT_printf(0, "\n\r\tRead value for tag is: %d", uid[j]);
								}
								SEGGER_RTT_WriteString(0, "\n Set \n");
								machine_state.FSM = 2;
							}
							else{
								SEGGER_RTT_WriteString(0, "No read\n");

							}
						
							break;
						case(2):

							check_tag(&machine_state);
							break;
					
						case(3):
							check_tag(&machine_state);

					}
					machine_state.request_status = MI_ERR;
					SEGGER_RTT_printf(0, "\n\r\tCurrent state: %d", machine_state.FSM);



					bool
check_tag(struct FSM_state *)
{
	FSM_state -> request_status = request_tag(MF1_REQIDL, data_rfid);
	if(FSM_state -> request_status == MI_OK)
	{
		bool correct = true;
		for (int i = 0; i < 5; i++)
		{
			if (data_rfid[i] != uid[i])
			{
				correct = false;
			}
		}

		if(correct)
		{
			SEGGER_RTT_WriteString(0, "Same tag, unset\n");
			FSM_state -> FSM = 1;
		}
		else
		{
			SEGGER_RTT_WriteString(0, "Different tag, Alarm triggered\n");
			FSM_state -> FSM = 3;
		}
	}
	else
	{
		SEGGER_RTT_WriteString(0, "No read, set\n");
	}
}

*/