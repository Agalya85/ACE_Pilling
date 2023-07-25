/*
 * user_can.h
 *
 *  Created on: Mar 25, 2022
 *      Author: admin
 */

#ifndef INC_USER_CAN_H_
#define INC_USER_CAN_H_

#define MAX_CAN_IDS_SUPPORTED (54)
//#define CAN_BUFFER_LENGTH     (80)
#define CAN_BUFFER_LENGTH     (200)

#define J1939_21 	21
#define J1939		1
#define CAN_ID		0

#define J1939_MODE  (J1939_21)

typedef enum
{
	enmCANQUERY_IDLE = 0,
	enmCANQUERY_UPDATEQUERY,
	enmCANQUERY_SENDQUERY,
	enmCANQUERY_AWAITRESPONSE,
	enmCANQUERY_PASRERESPONSE,
	enmCANQUERY_RESPONSETIMEOUT
}enmCanQueryState;
/**
 * This union stores Command IDs and its parsed info.
 * This is used to filter PGNs from incomming IDs
 */

#if(J1939_MODE == J1939)
typedef union{
	struct{
		uint8_t u16J1939SA : 8;		// bit 0 to 8
		uint32_t u16J1939PGN : 16;  // bit 9 to 24
		uint8_t u16J1939DataPage:1; // bit 25
		uint8_t u16J1939Reserved:1; // bit 26
		uint8_t u16J1939Priority:3; // bit 27,28,29
	};
	uint32_t u32J1939CommandId;
}unCan1939CommandId;

#elif(J1939_MODE == J1939_21)
typedef union{
	struct{
		uint8_t u16J1939SA : 8;		// bit 0 to 7
		uint8_t u16J1939PDU_SPECIFIC : 8;  // bit 8 to 15
		uint8_t u16J1939PDU_FORMAT:8; // bit 16 to 23
		uint8_t u16J1939DataPage:1; //bit 24
		uint8_t u16J1939Reserved:1; // bit 25
		uint8_t u16J1939Priority:3; // bit 26,27,28
		uint8_t u16J1939EXTID:3; // bit 29,30,31
	};
	uint32_t u32J1939CommandId;
}unCan1939CommandId;
#endif

typedef struct
{
	uint8_t u8CanMsgByte7;
	uint8_t u8CanMsgByte6;
	uint8_t u8CanMsgByte5;
	uint8_t u8CanMsgByte4;
	uint8_t u8CanMsgByte3;
	uint8_t u8CanMsgByte2;
	uint8_t u8CanMsgByte1;
	uint8_t u8CanMsgByte0;
	uint32_t u32CanMsgTimestamp;
}strCanReceivedMsg;

typedef enum
{
	enumTeleHearBeat_MSG_PERIODICITY = 0,
	enumTeleHearBeat_GPS_STATUS,
	enumTeleHearBeat_NW_STATUS,
	enumTeleHearBeat_SIGNAL_QUALITY,
	enumTeleHearBeat_SIM_CARD_ACTIVE_STATUS,
	enumTeleHearBeat_INTERNAL_BATT_SOC_PER,
	enumTeleHearBeat_INTERNAL_BATT_STATUS,
	enumTeleHearBeat_DEVICE_CODE,
}enmTeleHeartBeat;

void canFilterConfig(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
uint32_t isCommandIdConfigured(uint32_t canId);
void parseCanMessageQueue(void);
void executeCANQueries(void);
void updateCANQuery(void);
void sendMessageCAN (void);
void CAN_HeartBeat_Signal_Data(void);
void uploaddynamicCanQueue(void);
void update95_96CanPayload(uint8_t data[] ,char * systemPayload,uint8_t CANId);

#endif /* INC_USER_CAN_H_ */
