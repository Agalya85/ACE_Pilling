/*
 * modbusmaster.h
 *
 *  Created on: Sep 29, 2021
 *      Author: svp100093
 */

#ifndef INC_USER_MODBUS_RS485_H_
#define INC_USER_MODBUS_RS485_H_

/* Modbus Defines */

#include "externs.h"
#include "timer.h"

#define MB_MAX_SLAVES         		  (50)

#define MB_ADDRESS_DEREF				0
#define MB_QUERY_RETRY_ATTEMPTS			2


#define DATA_UPLADING_FREQUENCY			(uint32_t)ONE_MIN //ONE_MIN

typedef enum
{
	enmMODBUS_IDLE = 0,
	enmMODBUS_SEND_QUERY,
	enmMODBBUS_AWAIT_RESPONSE,
	enmMODBUS_RETRY_QUERY,
	enmMODBUS_RESPONSE_TIMEOUT,
	enmMODBUS_PARSE_RESPONSE,

}enmMODBUSFSMState;

typedef enum{
	enmMB_ILLEGAL_FUCNTION = 1,  /* The function code received in the query is not
								    an allowable action for the slave. */
	enmMB_ILLEGAL_DATA_ADDRESS,  /* The data address received in the query
								    is not an allowable address for the
								    slave.*/
	enmMB_ILLEGAL_DATA_VALUE,    /* A value contained in the query data
								    field is not an allowable value for the
								    slave.*/
	enmMB_SLAVE_DEVICE_FAILURE,  /* An unrecoverable error occurred while
								    the slave was attempting to perform the
								    requested action.*/
	enmMB_ACKNOWLEDGE,			/*  The slave has accepted the request
									and is processing it, but a long duration
									of time will be required to do so. This
									response is returned to prevent a
									timeout error from occurring in the
									master. The master can next issue a
									Poll Program Complete message to
									determine if processing is completed.*/
	enmMB_SLAVE_DEVICE_BUSY,	/*  The slave is engaged in processing a
									long�duration program command. The
									master should retransmit the message
									later when the slave is free.*/
	enmMB_NEGATIVE_ACKNOWLEDGE, /* 	The slave cannot perform the program
									function received in the query. This
									code is returned for an unsuccessful
									programming request using function
									code 13 or 14 decimal. The master
									should request diagnostic or error
									information from the slave.*/
	enmMB_MEMORY_PARITY_ERROR,   /* The slave attempted to read extended
									memory, but detected a parity error in
									the memory. The master can retry the
									request, but service may be required on
									the slave device.*/
	enmMB_NO_EXCEPTION,

}enmModbusExceptionCode;

typedef enum{
	enmNO_ERROR = 0,
	enmMEMORY_ERROR ,   /* malloc error in init*/
	enmRESPONSE_TIMEOUT
}enmERROR;

typedef enum{
	enmMBFC_READ_COIL_STATUS = 0x01,
	enmMBFC_READ_INPUT_STATUS,
	enmMBFC_READ_HOLDING_REGISTERS,
	enmMBFC_READ_INPUT_REGISTERS,
	enmMBFC_FORCE_SINGLE_COIL,
	enmMBFC_PRESET_SINGLE_REGISTER,
	enmMBFC_READ_EXCEPTION_STATUS,
	enmMBFC_DIAGNOSTICS,
	enmMBFC_PROGRAM_485,
	enmMBFC_POLL_485,
	enmMBFC_FETCH_COMM_EVENT_CONTR,
	enmMBFC_FETCH_COMM_EVENT_LOG,
	enmMBFC_PROGRAM_CONTROLLER,
	enmMBFC_POLL_CONTROLLER,
	enmMBFC_FORCE_MULTIPLE_COIL,
	enmMBFC_PRESET_MULTIPLE_REGISTERS,
	enmMBFC_REPORT_SLAVE_ID,
	enmMBFC_PROGRAM_884_M84,
	enmMBFC_RESET_COMM_LINK,
	enmMBFC_READ_GENERAL_REFERENCE,
	enmMBFC_WRITE_GENERAL_REFERENCE,
	enmMBFC_MASK_WRITE_4X_REGISTER,
	enmMBFC_RW_4X_REGISTER,
	enmMBFC_READ_FIFO_QUEUE
}enmMBFunctionCode;

typedef enum
{
	enmMBREG_DISCRETE_OUT_COILS = 0,
	enmMBREG_DISCRETE_IN_CONTACTS = 1,
	enmMBREG_ANALOG_IN_REGISTERS = 3,
	enmMBREG_AN_OUT_HOLDING_REGISTER = 4,
}enmMBRegisterType;

typedef enum
{
	enm_MBMasterState_Tx = 0,
	enm_MBMasterState_Rx
}enmModbusMasterState;


typedef enum
{
	enm_ResponseSucccess = 0,
	enm_ResponseException = 1,
	enm_ResponseTimeOut = 2,
	enm_ResponseCRCInvalid = 3 /*CRC invalid or Garbage data Received*/
}enmQueryResponseStatus;


typedef struct
{
	uint8_t u8MbSlaveID; 				/* Slave Device Address */
	uint8_t u8MbFunctionCode;			/*Slave Function Code */
	uint16_t u16MbMbAddress;  			/* Starting Address*/
	uint16_t u16MBNoPoints;       			/* No of Points */
}St_MBMaterQueryData;


typedef struct
{

	uint32_t u32MbCycleRestartTmr;
	uint32_t gu32ModbusPollDelay;
	uint32_t u32MbResponseTimeout;
	uint16_t u16NoOfBytesForValidResponse[MB_MAX_SLAVES];       			/*Number of Byes will receive for Valid Response*/
	enmQueryResponseStatus enum_MBResponseStatusBuff[MB_MAX_SLAVES];			/* Stores Response status  0 : Success   1: Exception    2: Timeout */
	uint16_t u16QueryNCCntrBufff[MB_MAX_SLAVES];
	uint8_t u8MBNoQueryAttempts;
	uint8_t u8QueryFrame[8]; 					/* Modbus Query TX buffer */
	uint8_t u8SlaveResponseArray[200];			/* Modbus Slave Response Array */
	uint8_t u8MBResponseCharacterCounter;		/* Counts received characters */
	uint8_t u8MBOperationStatus;				/* Tracks Slave Data Completion 0:-> Not Started 1: In process 2: Completed */
	uint8_t  u8SlaveData[1000];					/* Holds Temperory Data Received From Slaves  If Response is Invalid or Timeout then store as a zero*/

}strctModbusMaster;


typedef struct{
    int8_t ModbusDataAddress0[100];
    int8_t ModbusDataAddress1[400];
    int8_t ModbusDataAddress2[250];
    int8_t ModbusDataAddress3[250];
    int8_t ModbusDataAddress4[250];
    int8_t ModbusDataAddress5[250];
}ModbusData;



extern volatile uint32_t gu16ModbusFrameEndTimer;
extern volatile uint32_t gu32ModbusResponseTimeout;
extern volatile uint32_t gu32ModbusPollDelay;
extern volatile uint32_t gu32ModbusCycelRestartTmr;


extern char gu8MBRTUPayloadString[1100];
extern strctModbusMaster master;

/*Modbus Master Function Prototypes*/

uint16_t modbusCRC16(uint8_t * data , uint8_t length);
void MODBUS_CharReception_Callback(void);
void MODBUS_Error_Callback(void);
void ModbusMaster_FSM(void);
void resetModbusPort();
void setupModbus(void);
void parseModbusResponse(void);
void MB_UpdateModbusPayload(void);


#endif /* INC_USER_MODBUS_RS485_H_ */
