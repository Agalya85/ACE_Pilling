/*
 * applicationDefines.h
 *
 *  Created on: Mar 25, 2022
 *      Author: admin
 */

#ifndef INC_APPLICATIONDEFINES_H_
#define INC_APPLICATIONDEFINES_H_

#include "user_modbus_rs485.h"

#define FALSE (0)
#define TRUE  (!FALSE)

#define SUCCESS TRUE
#define FAIL    FALSE

#define HIGH (1)
#define LOW  (0)

#define PAYLOAD_DATA_STRING_RADIX (10)
#define CAN_PAYLOADSTRING_RADIX	  (16)



#define MODEL_NUMBER    		"TorMini4G-v2.0-2302/ACEPILLING/"
#define YEAR 	2023
#define MONTH	06
#define DATE	28


/*Parity enum*/
typedef enum
{
	enum_USART_PARITY_NONE = 0,
	enum_USART_PARITY_ODD,
	enum_USART_PARITY_EVEN,
}enum_UART_PARITY;

/*Parity enum*/
typedef enum
{
	enum_USART_STOPBITS_1 = 0,
	enum_USART__STOPBITS_2
}enum_UART_STOP_BIT;

typedef enum
{
	enum_BAUD_RATE_4800=0,
	enum_BAUD_RATE_9600,
	enum_BAUD_RATE_19200,
	enum_BAUD_RATE_38400,
	enum_BAUD_RATE_57600,
	enum_BAUD_RATE_115200
}enum_UART_BAUD_RATE;

typedef struct
{
	uint32_t u32DataUploadingFreq;
	/* Buffer memory Parameters*/
	uint8_t u8BufferMemEnFlag;
	/*Modbus Master Parameters*/
	uint8_t u8ModbusMaterEnFlag;
	enum_UART_BAUD_RATE enumMbBaudRate;
	enum_UART_PARITY enumMBParity;
	enum_UART_STOP_BIT enumMbStopBit;
	/*Modbus Query Buffer*/
	uint16_t u16MbTotalNoOfQuerys;						/* Total Query */
	St_MBMaterQueryData stMbMasterQuerysArr[MB_MAX_SLAVES];
	uint8_t u8BleEscortSensEnableFlag; //If This Flag is 0 then Serial Debug is automatically Enable Else it is Disable.
    char u8EscortSensID[10];
    uint32_t u32Slagbyte;
	uint32_t u32Checksum;  //Structure Checksum for data validation.

}ST_Config;


#define CONFIG_STRUCT_SIZE   sizeof(ST_Config)

extern ST_Config st_DeviceConfig;

#endif /* INC_APPLICATIONDEFINES_H_ */
