/*
 * user_can.c
 *
 *  Created on: Mar 25, 2022
 *      Author: admin
 */

#include "main.h"
#include "stm32l4xx_hal.h"
#include "user_can.h"
#include "externs.h"
#include "applicationdefines.h"
#include "error_handler.h"
#include "string.h"
#include "timer.h"
#include <time.h>
#include<stdio.h>
#include <stdlib.h>
#include "payload.h"

#define TELEMATIC_HEARTBEAT 		0x10C8FFFB
#define TELEMATIC_FIRMWARE_VER 		0x10C9FFFB
#define TELEMATIC_TIME		 		0x10CAFFFB
#define TELEMATIC_HWID_IMEI			0x10CBFFFB
#define TELEMATIC_SLEEP_RESP		0x10CFFFFB
#define TELEMATIC_IMOBI_RESP		0x1092FFFB
#define TELEMATIC_IMOBI_STATUS_RESP 0x1090FFFB
#define TELE_MSG_PERIOD				5
#define TELE_DEVICE_ID				0x01

uint8_t TIME_PERIOD_CNT = 0;
uint16_t cu16UTC_TIME = 0;


#define PGNREQUESTCANID 	0x1894F0FF




CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
CAN_FilterTypeDef  sFilterConfig;
extern CAN_HandleTypeDef hcan1;

/* Configuration Array for Can Peripheral
 * Prerequisite : [0] - Will / Should always contain ONLY CAN BaudRate / Bit Rate !
 * CommandIds   : [1 to 50] Sequential list of CAN Command Ids to be captured .
 * Defaults / Example : { 500, 1, 2, 3, 0x3AD,0x1FF,6,7,0x7FF,9  ,0x3AB};
 *                        BR ,Id,Id,Id, Id  , Id  ,Id , Id  ,Id , Id
 * */

#if(J1939_MODE == CAN_ID)
uint32_t gu32CanConfigurationArray[(MAX_CAN_IDS_SUPPORTED + 1)] = { 250,
																	0x0C01FFF4,0x0C02FFF4,0x0C03FFF4,0x0C04FFF4,0x1805FFF4,
																	0x0C06FFF4,0x1807FFF4,0x1808FFF4,0x0C10FFF4,0x0C11FFF4,
																	0x0C12FFF4,0x0C13FFF4,0x0C14FFF4,0x0C15FFF4,0x0C17FFF4,
																	0x0C18FFF4,0x0C19FFF4,0x0C1AFFF4,0x0C20FFF4,0x0C21FFF4,
																	0x0C22FFF4,0x0C23FFF4,0x0C24FFF4,0x0C25FFF4,0x0C30FFF4,
																	0x0C31FFF4,0x0C32FFF4,0x0C33FFF4,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
															 	   };

#elif(J1939_MODE == J1939_21)
uint8_t Priority = 0; // bit(28-26)
uint8_t EDP = 0; //bit(25) Extended Data Page: combined with DP to identify different message definitions
uint8_t DP = 0; //bit(24)Data Page: combined with EDP to identify different message definitions
//PGN
/*256,512,768,1024,1280,
																		1536,1792,2048,200960,4096,
																		4352,4608,4864,5120,5376,
																		5632,5888,6144,6400,6656,
																		6912,7168,8192,8448,8704,
																		8960,9216,9472,12288,12544,
																		12800,13056,32256,32512,32768,
																		33024,33280,33536,33792,34048,
																		34304,34560,34816,35072,35328,
																		35584,35840,36096,36352,29952,
//AS PER DBC ON 15th March23																		30208,30464,29696,2,*/
//uint32_t gu32CanConfigurationArray[(MAX_CAN_IDS_SUPPORTED + 1)] = { 250,
//																	164608,163840,164096,167936,168192,
//																	168448,168704,168960,164352,164864,
//																	13056,256,1792,5376,8192,
//																	8448,8704,8960,9216,9472,
//																	7424,1024,4352,4864,4608,
//																	6656,4096,7168,41216,197376,
//																	197632,197888,12288,12544,12800,
//																	5888,2304,6400,205056,512,
//																	13312,6144,2816,768,5120,
//																	2560,2,
//																	};

////As per dbc on 23rd March 23
//uint32_t gu32CanConfigurationArray[(MAX_CAN_IDS_SUPPORTED + 1)] = { 250,
//																	164608,163840,164096,167936,168192,
//																	168448,168704,168960,164352,164864,
//																	13056,256,1792,5376,8192,
//																	8448,8704,8960,9216,9472,
//																	7424,1024,4352,4864,4608,
//																	6656,4096,7168,41216,197376,
//																	197632,197888,12288,12544,12800,
//																	5888,2304,6400,205056,512,
//																	6144,2816,768,5120,2560,
//																	2,
//																	};

//As per Log9-BMS-LV-J1939_v12_170523.dbc on 17th May 23
uint32_t gu32CanConfigurationArray[(MAX_CAN_IDS_SUPPORTED + 1)] = { 250,
																	0x289,0x18A,0x387,0x187,0x187,
																	0x489,0x387,0x287,0x487,0x187,
																	0x488,0x18D,0x784,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
																	2,2,2,2,2,
																	};

#elif(J1939_MODE == 1)
uint32_t gu32CanConfigurationArray[(MAX_CAN_IDS_SUPPORTED + 1)] = { 250,
																		256,512,768,1024,1280,
																		1536,1792,2048,200960,4096,
																		4352,4608,4864,5120,5376,
																		5632,5888,6144,6400,6656,
																		6912,7168,8192,8448,8704,
																		8960,9216,9472,12288,12544,
																		12800,13056,32256,32512,32768,
																		33024,33280,33536,33792,34048,
																		34304,34560,34816,35072,35328,
																		35584,35840,36096,36352,29952,
																		30208,30464,29696,
																	};
#endif

volatile enmCanQueryState canCurrentState = enmCANQUERY_IDLE;
uint32_t gu32CANQueryCommandResponseReceivedFlag = FALSE;

uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
uint32_t u32CanIdsReceived[CAN_BUFFER_LENGTH] = {'0'};
uint32_t u32CanRxMsgLoopCounter = 0;
uint32_t canTestVariable = 0;
uint32_t u32IdSearchFlag = FALSE;

uint64_t gu64CanMessageIDPGN[MAX_CAN_IDS_SUPPORTED] = {0};
strCanReceivedMsg unCanReceivedMsgs[CAN_BUFFER_LENGTH];
uint32_t unCanReceivedMsgsTimestamp[CAN_BUFFER_LENGTH];

uint64_t gu64CanMessageArray[MAX_CAN_IDS_SUPPORTED] = {0};
uint32_t gu32CanMessageArrayTimestamp[MAX_CAN_IDS_SUPPORTED]={0};

uint64_t u64CanMessageReceived[CAN_BUFFER_LENGTH] = {0};
unCan1939CommandId unCan1939ReceivedId[CAN_BUFFER_LENGTH]={0};
unCan1939CommandId ReqPGN = {0};


uint32_t CANTxFlag = FALSE;

volatile uint32_t gu32ProcessCanMessage = 0;

/*Tx CAN*/

uint32_t gu32TxCANId[18] = {TELEMATIC_HEARTBEAT, TELEMATIC_FIRMWARE_VER,TELEMATIC_TIME,TELEMATIC_HWID_IMEI,0x18900140,
							0x18910140,0x18920140,0x18930140,0x18940140,0x18950140,
							0x18960140,0x18970140,0x18980140,2,2};
uint32_t gu32TxCANCounter = 0;

uint8_t TelematicHeartBeat[8] = {0};



_Bool Flag_95 = FALSE;
_Bool Flag_96 = FALSE;

uint8_t CAN_ID_95[70]="";
uint8_t CAN_ID_96[70]="";

uint32_t u3295IDrespTime = 0;
uint32_t u3296IDrespTime = 0;

uint8_t counter_95 = 0;
uint8_t counter_96 = 0;



/*Epoch time conversion
 * use #include <time.h>
 */
struct tm RTC_UTC_time;
time_t Epoch_time;   // time_t is macro as long long int
/****************************************************************************
 Function: canFilterConfig
 Purpose: Init CAN peripheral with filter configuration
 Input: None.
 Return value: None
 Refer Link for timing calculations :
 http://www.bittiming.can-wiki.info/
 Clock = 80 Mhz (Refer Clock Configuration in CubeMX for details)
 Bit Rate    Pre-scaler  time quanta  Seg 1  Seg 2   Sample Point
 kbps
 1000			5			16			13	   2	    87.5
 500			10			16			13     2		87.5
 250			20			16			13     2    	87.5
 125			40			16			13     2		87.5
 100			50			16			13     2		87.5
 83.33			60			16			13     2		87.5
 50				100			16			13     2		87.5
 20				250			16			13     2		87.5
 10				500			16			13     2		87.5
 Note(s)(if-any) :
 Change History:
 Author            	Date                Remarks
 KloudQ Team        22/03/2020			initial Definitions
 kloudq				27/03/2020			Bit Calculation Added
 kloudq				20/04/2021			Added support for STM32L433 MCU
******************************************************************************/
void canFilterConfig(void)
{
	hcan1.Instance = CAN1;
//	hcan1.Init.Prescaler = 6;
//	hcan1.Init.TimeTriggeredMode = ENABLE;//DISABLE;
//	hcan1.Init.AutoBusOff = ENABLE;
//	hcan1.Init.AutoWakeUp = ENABLE;//DISABLE;
//	hcan1.Init.AutoRetransmission = DISABLE;//ENABLE;//
//	hcan1.Init.ReceiveFifoLocked = DISABLE;//ENABLE;//
//	hcan1.Init.TransmitFifoPriority =ENABLE;// DISABLE;//
//	hcan1.Init.Mode = CAN_MODE_NORMAL;

	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = ENABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = ENABLE;
	hcan1.Init.Mode = CAN_MODE_NORMAL;

//	hcan1.Init.Mode = CAN_MODE_SILENT;

	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;

	switch(gu32CanConfigurationArray[0])
  	{
  		case 1000:
  			hcan1.Init.Prescaler = 5;
  		break;
  		case 500:
  			hcan1.Init.Prescaler = 10;
  		break;
  		case 250:
  			hcan1.Init.Prescaler = 20;
  		break;
  		case 125:
  			hcan1.Init.Prescaler = 40;
  		break;
  		case 100:
  			hcan1.Init.Prescaler = 50;
  		break;
  		case 83:
  			hcan1.Init.Prescaler = 60;
  		break;
  		case 50:
  			hcan1.Init.Prescaler = 100;
  		break;
  		case 20:
  			hcan1.Init.Prescaler = 250;
  		break;
  		case 10:
  			hcan1.Init.Prescaler = 500;
  		break;
  		default:
  		/* Illegal BaudRate Configured . Use Default 500 Kbps */
  			hcan1.Init.Prescaler = 10;
  		break;
  	}

  	if (HAL_CAN_Init(&hcan1) != HAL_OK)
  		assertError(enmTORERRORS_CAN1_INIT,enmERRORSTATE_ACTIVE);
  	else
  		 assertError(enmTORERRORS_CAN1_INIT,enmERRORSTATE_NOERROR);

  	/*##-2- Configure the CAN Filter ###########################################*/
  	  sFilterConfig.FilterBank = 0;
  	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  	  sFilterConfig.FilterIdHigh = 0x0000;
  	  sFilterConfig.FilterIdLow = 0x0000;
  	  sFilterConfig.FilterMaskIdHigh = 0x0000;
  	  sFilterConfig.FilterMaskIdLow = 0x0000;
  	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  	  sFilterConfig.FilterActivation = ENABLE;
  	  sFilterConfig.SlaveStartFilterBank = 14;

  	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  		assertError(enmTORERRORS_CAN1_CONFIGFILTER,enmERRORSTATE_ACTIVE);
  	else
  		assertError(enmTORERRORS_CAN1_CONFIGFILTER,enmERRORSTATE_NOERROR);

  	/*##-3- Start the CAN peripheral ###########################################*/
  	  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  	  {
  	    /* Start Error */
//  	    Error_Handler();
  	  }

  	/*##-4- Activate CAN RX notification #######################################*/
  	  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  	  {
  		/* Notification Error */
  		  assertError(enmTORERRORS_CAN1_CONFIGFILTER,enmERRORSTATE_ACTIVE);
  	  }

  	  /*##-5- Configure Transmission process #####################################*/
  	TxHeader.StdId = 0x321;
  	TxHeader.ExtId = 0x01;
  	TxHeader.RTR = CAN_RTR_DATA;
  	TxHeader.IDE = CAN_ID_STD;
  	TxHeader.DLC = 2;
  	TxHeader.TransmitGlobalTime = DISABLE;

  	memset(u32CanIdsReceived,0x00,sizeof(u32CanIdsReceived));
  	memset(unCanReceivedMsgs,0x00,sizeof(unCanReceivedMsgs));
}

/******************************************************************************
* Function : HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
*//**
* \b Description:
*
* This function is Rx complete callback in non blocking mode
*
* PRE-CONDITION: Enable CAN interface in CubeMx . Enable CAN Rx Interrupt
*
* POST-CONDITION: Buffers received data
*
* @return 		None.
*
* \b Example Example:
* @code
*
*
*
* @endcode
*
* @see
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 01/06/2019 </td><td> 0.0.1            </td><td> HL100133 </td><td> Interface Created </td></tr>
*
* </table><br><br>
* <hr>
*
*******************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* LED Only for testing/ Indication . Can be removed in production if not required  */
//#if(SLEEP_MOOD == ON)
//	if(gu32MCUCurrentWorkingMode != enmMCUMode_SLEEP)
//#endif
		HAL_GPIO_TogglePin(Comm_LED_GPIO_Port,Comm_LED_Pin);

	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
	/* Parse the incoming data only if array location is available
	 * Added on 3/3/21 - For payload overwrite issue */


//#if(J1939_MODE == J1939_21)
//	ReqPGN.u32J1939CommandId = RxHeader.ExtId;
//#if(SLEEP_MOOD == ON)
//				if((gu32MCUCurrentWorkingMode == enmMCUMode_RUN) && (((((ReqPGN.u16J1939Reserved << 17) | (ReqPGN.u16J1939DataPage << 16))| (ReqPGN.u16J1939PDU_FORMAT << 8)) | ((ReqPGN.u16J1939PDU_FORMAT) < 240 ? 0 : (ReqPGN.u16J1939PDU_SPECIFIC << 0))) == 52736)) //Sleep Request
//				{
//					if((RxData[0]== 0x01) && (RxData[1] == 0xE0))
//					{
//						Sleep_Req = TRUE;
//					}
//					else
//					{
//						gu32MCUCurrentWorkingMode = enmMCUMode_RUN;
//					}
//				}
//				else if((gu32MCUCurrentWorkingMode == enmMCUMode_SLEEP)&&(((((ReqPGN.u16J1939Reserved << 17) | (ReqPGN.u16J1939DataPage << 16))| (ReqPGN.u16J1939PDU_FORMAT << 8)) | ((ReqPGN.u16J1939PDU_FORMAT) < 240 ? 0 : (ReqPGN.u16J1939PDU_SPECIFIC << 0))) == 52480)) //Wakeup from Sleep Request
//				{
//					gu32MCUCurrentWorkingMode = enmMCUMode_RUN;
//				}
//				else if(((((ReqPGN.u16J1939Reserved << 17) | (ReqPGN.u16J1939DataPage << 16))| (ReqPGN.u16J1939PDU_FORMAT << 8)) | ((ReqPGN.u16J1939PDU_FORMAT) < 240 ? 0 : (ReqPGN.u16J1939PDU_SPECIFIC << 0))) == 37632) //Immobilaization Request
//				{
//					if((RxData[0]== 0xB0) && (RxData[1] == 0x04))
//					{
//						if(RxData[3]== 0x01)
//						{
//							Immobilization_Flag = FALSE;
//						}
//					}
//				}
//				else if(((((ReqPGN.u16J1939Reserved << 17) | (ReqPGN.u16J1939DataPage << 16))| (ReqPGN.u16J1939PDU_FORMAT << 8)) | ((ReqPGN.u16J1939PDU_FORMAT) < 240 ? 0 : (ReqPGN.u16J1939PDU_SPECIFIC << 0))) == 0x3C100) //OTA Response
//				{
//					uint8_t count = 0;
//					for(count=0;count<8;count++)
//					{
//						UDS_BMS_Rx[count] = 0;
//					}
//					for(count=0;count<8;count++)
//					{
//						UDS_BMS_Rx[count] = RxData[count];
//					}
//					BoolUDSQueryCommandResponseReceivedFlag = TRUE;
//					gu32UDSQueryPollTimer = 0;
//				}
//				else{}
//#endif
//#endif

	if(u32CanIdsReceived[u32CanRxMsgLoopCounter] == 0)
	{
		if(RxHeader.IDE == CAN_ID_EXT)
		{
			u32CanIdsReceived[u32CanRxMsgLoopCounter] = RxHeader.ExtId;
		}
		else if(RxHeader.IDE == CAN_ID_STD)
		{
			u32CanIdsReceived[u32CanRxMsgLoopCounter] = RxHeader.StdId;

		}

		if(RxHeader.ExtId == 0x18954001)
		{
//				Flag_95 = TRUE;
//			u3295IDrespTime = FIFTY_MS;
			counter_95 = ((8 * RxData[0])-8); // 8 byte data, RxData[0] will be from 1 to 8
			CAN_ID_95[counter_95++] = RxData[0];
			CAN_ID_95[counter_95++] = RxData[1];
			CAN_ID_95[counter_95++] = RxData[2];
			CAN_ID_95[counter_95++] = RxData[3];
			CAN_ID_95[counter_95++] = RxData[4];
			CAN_ID_95[counter_95++] = RxData[5];
			CAN_ID_95[counter_95++] = RxData[6];
			CAN_ID_95[counter_95] 	= RxData[7];
			if((counter_95>63) || (u3295IDrespTime == 0))
			{
				counter_95 = 0;
			}
		}
		else if(RxHeader.ExtId == 0x18964001)
		{
//			u3296IDrespTime = FIFTY_MS;
			counter_96 = ((8 * RxData[0])-8); // 8 byte data, RxData[0] will be from 1 to 8
			CAN_ID_96[counter_96++] = RxData[0];
			CAN_ID_96[counter_96++] = RxData[1];
			CAN_ID_96[counter_96++] = RxData[2];
			CAN_ID_96[counter_96++] = RxData[3];
			CAN_ID_96[counter_96++] = RxData[4];
			CAN_ID_96[counter_96++] = RxData[5];
			CAN_ID_96[counter_96++] = RxData[6];
			CAN_ID_96[counter_96]	= RxData[7];
			if((counter_96>63) || (u3296IDrespTime == 0))
			{
				counter_95 = 0;
			}
		}
		else
		{
//			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte0 = (RxData[0]);
//			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte1 = (RxData[1]);
//			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte2 = (RxData[2]);
//			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte3 = (RxData[3]);
//			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte4 = (RxData[4]);
//			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte5 = (RxData[5]);
//			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte6 = (RxData[6]);
//			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte7 = (RxData[7]);


			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte0 = (RxData[7]);
			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte1 = (RxData[6]);
			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte2 = (RxData[5]);
			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte3 = (RxData[4]);
			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte4 = (RxData[3]);
			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte5 = (RxData[2]);
			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte6 = (RxData[1]);
			unCanReceivedMsgs[u32CanRxMsgLoopCounter].u8CanMsgByte7 = (RxData[0]);
			u32CanRxMsgLoopCounter++;


			if(u32CanRxMsgLoopCounter >= CAN_BUFFER_LENGTH)
					u32CanRxMsgLoopCounter = 0;
		}

	}

	if(u32CanRxMsgLoopCounter >= CAN_BUFFER_LENGTH)
		u32CanRxMsgLoopCounter = 0;
}

/******************************************************************************
* Function : HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
*//**
* \b Description:
*
* This function is CAN Error callback
*
* PRE-CONDITION: Enable CAN interface in CubeMx . Enable CAN Interrupt
*
* POST-CONDITION: Gives Error Code
*
* @return 		None.
*
* \b Example Example:
* @code
*
*
* @endcode
*
* @see
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 01/06/2020 </td><td> 0.0.1            </td><td> HL100133 </td><td> Interface Created </td></tr>
*
* </table><br><br>
* <hr>
*
*******************************************************************************/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	canTestVariable = hcan->ErrorCode;
	HAL_CAN_DeInit(&hcan1);
	canFilterConfig();
//	HAL_GPIO_TogglePin(Comm_LED_GPIO_Port, Comm_LED_Pin);
}

/******************************************************************************
* Function : isCommandIdConfigured(uint32_t canId)
*//**
* \b Description:
*
* This function Checks if received Id is configured for CAN
*
* PRE-CONDITION: Enable CAN interface in CubeMx . Enable CAN Interrupt
*
* POST-CONDITION: Gives Error Code
*
* @return 		uint32_t Id Position in PGN configuration Array.
*
* \b Example Example:
* @code
*	uint32_t idIndex = 0;
	do
	{
		idIndex = isCommandIdConfigured(0x0803FF00);
	}while(u32IdSearchFlag != 2);
	*
* @endcode
*
* @see
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 01/06/2020 </td><td> 0.0.1            </td><td> HL100133 </td><td> Interface Created </td></tr>
*
* </table><br><br>
* <hr>
*
*******************************************************************************/
uint32_t isCommandIdConfigured(uint32_t canId)
{
	static uint32_t LoopCounter = 0;
	static uint32_t u32PositioninConfigArray = 0;

	if(LoopCounter == 0)
	{
		u32IdSearchFlag = 1;
		u32PositioninConfigArray = 0;
	}

	if(u32IdSearchFlag == 1)
	{
		if(gu32CanConfigurationArray[LoopCounter] == canId)
		{
			/*
			 * If Received CanID is found in configuration Array
			 * then parse the frame else ignore .
			 */
			u32PositioninConfigArray = LoopCounter;
			u32IdSearchFlag = 2;
			LoopCounter = 0;
		}
		else
		{
			LoopCounter++;
			if (LoopCounter == MAX_CAN_IDS_SUPPORTED)
			{
				LoopCounter = 0;
				u32IdSearchFlag = 2;
			}
		}
	}
	return u32PositioninConfigArray;
}

/******************************************************************************
* Function : parseCanMessageQueue(uint32_t canId)
*//**
* \b Description:
*
* This function is used Parse CAN Message . If command ID is configured the store the message
*
* PRE-CONDITION: Enable CAN interface in CubeMx . Enable CAN Interrupt
*
* POST-CONDITION: Stored messsage of configured ID
*
* @return 		None.
*
* \b Example Example:
* @code
*
	parseCanMessageQueue();
*
* @endcode
*
* @see
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 01/06/2020 </td><td> 0.0.1            </td><td> HL100133 </td><td> Interface Created </td></tr>
*
* </table><br><br>
* <hr>
*
*******************************************************************************/
uint32_t temp = 0;
uint32_t gu32CanIdParserCounter = 0;
void parseCanMessageQueue(void)
{
	static uint32_t u32CanMsgID = 0;
	static uint32_t u32ParserState = 0;
	static uint32_t u32IdStatus = 0;

	if(u32CanIdsReceived[gu32CanIdParserCounter] != 0)
	{
		if(u32ParserState == 0)
		{
			/* Message Available. Parse The Message */

			u32CanMsgID = u32CanIdsReceived[gu32CanIdParserCounter];


			u32ParserState = 1;
		}
		else if(u32ParserState == 1)
		{
			/* Existing CAN parsing based on configured IDs
			 * Modified by 100133 for dynamic CAN ID support on 23/2/23 */
			/* In Process */
			if(u32IdSearchFlag == 2)
			{
				/* Search Process Completed */
				if(u32IdStatus != 0)
				{
					if(u32IdStatus == 53)
					{
						if(u32IdStatus != 0){}
					}
					gu64CanMessageArray[u32IdStatus] =  ((uint64_t)unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte7 << 56)|
														((uint64_t)unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte6 << 48)|
														((uint64_t)unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte5 << 40)|
														((uint64_t)unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte4 << 32)|
														((uint64_t)unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte3 << 24)|
														((uint64_t)unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte2 << 16)|
														((uint64_t)unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte1 << 8) |
														((uint64_t)unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte0);



					// Reset Array Value for new Message
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte7 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte6 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte5 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte4 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte3 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte2 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte1 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte0 = 0;
					u32CanIdsReceived[gu32CanIdParserCounter] = 0;
					u32IdStatus = 0;
				}
				else if(u32IdStatus == 0)
				{
					/* Command Id is not Configured . Discard the Message*/
	//					gu64CanMessageArray[gu32CanIdParserCounter] = 0; // change by VEDANT on 21/11/22
					//u64CanMessageReceived[gu32CanIdParserCounter] = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte7 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte6 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte5 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte4 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte3 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte2 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte1 = 0;
					unCanReceivedMsgs[gu32CanIdParserCounter].u8CanMsgByte0 = 0;
					u32CanIdsReceived[gu32CanIdParserCounter] = 0;
					unCan1939ReceivedId[gu32CanIdParserCounter].u32J1939CommandId = 0;
				}
				u32IdSearchFlag = 0;
				u32ParserState = 0;
				u32CanMsgID = 0;
				gu32CanIdParserCounter++;
			}
			else
				u32IdStatus = isCommandIdConfigured(u32CanMsgID);
		}
	}
	else
	{
		gu32CanIdParserCounter++;
	}
	if(gu32CanIdParserCounter >= CAN_BUFFER_LENGTH)
		gu32CanIdParserCounter = 0;
}


void executeCANQueries(void)
{

	if((gu32CANQueryPollTimer != 0) || (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0))
		return;

	switch(canCurrentState)
	{
		case enmCANQUERY_IDLE:
			/* Do not Query  */
			canCurrentState = enmCANQUERY_UPDATEQUERY;
			break;

		case enmCANQUERY_UPDATEQUERY:
			/* Update counter for Next query */

			updateCANQuery();
			if(CANTxFlag == TRUE)
				canCurrentState = enmCANQUERY_SENDQUERY;
			else
				canCurrentState = enmCANQUERY_IDLE;
			break;

		case enmCANQUERY_SENDQUERY:
			/* Transmit Query */
			sendMessageCAN();
//			gu32CANCommandResponseTimeout = ONE_SEC;
			gu32CANCommandResponseTimeout = FIVE_MS;
			gu32CANQueryPollTimer = FIFTY_MS;;
			canCurrentState = enmCANQUERY_AWAITRESPONSE;

			break;

		case enmCANQUERY_AWAITRESPONSE:
			if((gu32CANQueryCommandResponseReceivedFlag == TRUE) && (gu32CANCommandResponseTimeout != 0))
			{
				canCurrentState = enmCANQUERY_PASRERESPONSE;
				gu32CANCommandResponseTimeout = 0;
			}
			else if(gu32CANCommandResponseTimeout == 0)
			{
				canCurrentState = enmCANQUERY_RESPONSETIMEOUT;
			}
			else{}

			break;
		case enmCANQUERY_PASRERESPONSE:
			/* Store / Parse  received response */
			canCurrentState = enmCANQUERY_IDLE;
			if(gu32CANQueryCommandResponseReceivedFlag == TRUE)
				gu32CANQueryCommandResponseReceivedFlag = FALSE;
			gu32CANQueryPollTimer = FIFTY_MS;;
//			gu32CANQueryPollTimer = FIVE_MS;
			break;

		case enmCANQUERY_RESPONSETIMEOUT:
			/*Response not received */
			canCurrentState = enmCANQUERY_IDLE;
			gu32CANQueryCommandResponseReceivedFlag = FALSE;
//			gu32CANQueryPollTimer = ONE_SEC;
			gu32CANQueryPollTimer = FIFTY_MS;
//			u32ChangeInCycleFlag = FALSE;
			break;

		default:
			/* Undefined State */
			canCurrentState = enmCANQUERY_IDLE;
			break;
	}
}



void updateCANQuery(void)
{

	uint8_t temp[20] ="";
	TxHeader.StdId = gu32TxCANId[gu32TxCANCounter];
	TxHeader.ExtId = gu32TxCANId[gu32TxCANCounter];
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_EXT;//CAN_ID_STD;//
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = ENABLE;
	TxData[0] = 0;
	TxData[1] = 0;
	TxData[2] = 0;
	TxData[3] = 0;
	TxData[4] = 0;
	TxData[5] = 0;
	TxData[6] = 0;
	TxData[7] = 0;

	if((gu32TxCANId[gu32TxCANCounter] == TELEMATIC_HEARTBEAT) && (gu32HeartBeatDelay== 0))
	{
		CAN_HeartBeat_Signal_Data();
		TelematicHeartBeat[enumTeleHearBeat_MSG_PERIODICITY] = TELE_MSG_PERIOD;
		TelematicHeartBeat[enumTeleHearBeat_DEVICE_CODE]= TELE_DEVICE_ID;

		TxData[0] = TelematicHeartBeat[enumTeleHearBeat_MSG_PERIODICITY];
		TxData[1] = TelematicHeartBeat[enumTeleHearBeat_GPS_STATUS];
		TxData[2] = TelematicHeartBeat[enumTeleHearBeat_NW_STATUS];
		TxData[3] = TelematicHeartBeat[enumTeleHearBeat_SIGNAL_QUALITY];
		TxData[4] = TelematicHeartBeat[enumTeleHearBeat_SIM_CARD_ACTIVE_STATUS];
		TxData[5] = TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER];
		if(g_stAdcData.u32PwrSupplyVtg > 1)
		{
			TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_STATUS] = 0;

		}
		else
		{
			TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_STATUS] = 1;
		}
		TxData[6] = TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_STATUS];
		TxData[7] = TelematicHeartBeat[enumTeleHearBeat_DEVICE_CODE];
		gu32HeartBeatDelay = (TELE_MSG_PERIOD * ONE_SEC);
		if(++TIME_PERIOD_CNT == 2)
		{
			gu32TxCANCounter++;
		}
		else
		{
			gu32TxCANCounter = 0;
		}
		CANTxFlag = TRUE;
	}
	else if((gu32TxCANId[gu32TxCANCounter] == TELEMATIC_FIRMWARE_VER) && (TIME_PERIOD_CNT == 2))
	{
//		strcpy(temp,BOOT_REGION);
//		strcat(temp,FIRMWARE_VER);
		TxData[0] = (YEAR & 0xFF00) >> 8;
		TxData[1] = YEAR & 0x00FF;
		TxData[2] = MONTH;
		TxData[3] = DATE;
		strcpy((char*)temp,(char*)BOOT_REGION);
		if(strcmp((char*)temp,(char*)"Xv") == 0)
		{
			TxData[4] = 0<<7; // X region
			strcpy((char*)temp,(char*)FIRMWARE_VER);
			TxData[4] = TxData[4] | (temp[0]<<4);
			TxData[4] = TxData[4] | ((temp[2]&0x0F)) ;
		}
		else if(strcmp((char*)temp,(char*)"Yv") == 0)
		{
			TxData[4] = 1<<7;// Y region
			strcpy((char*)temp,(char*)FIRMWARE_VER);
			TxData[4] = TxData[4] | (temp[0]<<4);// 0 to 7
			TxData[4] = TxData[4] | ((temp[2]&0x0F)) ; // 0 to 7
		}
		else
		{
			TxData[4] = 0;
		}
		TxData[5] = 0;
		TxData[6] = 0;
		TxData[7] = 0;
		TIME_PERIOD_CNT++;
		gu32TxCANCounter++;
		CANTxFlag = TRUE;

	}
	else if((gu32TxCANId[gu32TxCANCounter] == TELEMATIC_TIME) && (TIME_PERIOD_CNT == 3))
	{

		if(GSMInitCompleteFlag == TRUE)
		{
			getrtcStamp();
			RTC_UTC_time.tm_year  = ((gu8YY *100) + atoi(gau8Year))-1900;  // Year - 1900
			RTC_UTC_time.tm_mon   = atoi(gau8Month) - 1;           // Month, where 0 = jan
			RTC_UTC_time.tm_mday  = atoi(gau8Date);          // Day of the month
			RTC_UTC_time.tm_hour  = atoi(gau8Hour);
			RTC_UTC_time.tm_min   = atoi(gau8Minutes);
			RTC_UTC_time.tm_sec   = atoi(gau8Seconds);
			RTC_UTC_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
			Epoch_time = mktime(&RTC_UTC_time);
//				Epoch_time = 1677649660;
			TxData[0] = 0x01;
			TxData[1] = 0;
			TxData[2] = 0;
			TxData[3] = 0;
			TxData[4] = (Epoch_time)&0xFF;;
			TxData[5] = (Epoch_time>>8)&0xFF;
			TxData[6] = (Epoch_time>>16)&0xFF;
			TxData[7] = (Epoch_time>>24)&0xFF;

		}
		else
		{
			TxData[0] = 0;
			TxData[1] = 0;
			TxData[2] = 0;
			TxData[3] = 0;
			TxData[4] = 0;
			TxData[5] = 0;
			TxData[6] = 0;
			TxData[7] = 0;
		}


		TIME_PERIOD_CNT++;
		gu32TxCANCounter++;
		CANTxFlag = TRUE;
	}
	else if((gu32TxCANId[gu32TxCANCounter] == TELEMATIC_HWID_IMEI) && (TIME_PERIOD_CNT == 4))
	{
//		860987055194077
//		cu32IMEI = 860987055194077;

		TxData[0] = (cu32IMEI)&0xFF;
		TxData[1] = (cu32IMEI>>8)&0xFF;
		TxData[2] = (cu32IMEI>>16)&0xFF;
		TxData[3] = (cu32IMEI>>24)&0xFF;
		TxData[4] = (cu32IMEI>>32)&0xFF;
		TxData[5] = (cu32IMEI>>40)&0xFF;
		TxData[6] = (cu32IMEI>>48)&0xFF;
		TxData[7] = 0x00;
//		TIME_PERIOD_CNT = 0;
//		gu32TxCANCounter = 0;
		TIME_PERIOD_CNT++;
		gu32TxCANCounter++;
		CANTxFlag = TRUE;
	}
	else if(TIME_PERIOD_CNT == 5)
	{

		TxData[0] = 0xFF;
		TxData[1] = 0xFF;
		TxData[2] = 0xFF;
		TxData[3] = 0xFF;
		TxData[4] = 0xFF;
		TxData[5] = 0xFF;
		TxData[6] = 0xFF;
		TxData[7] = 0xFF;

		CANTxFlag = TRUE;

		if(gu32TxCANCounter == 8)
		{
			gu32TxCANCounter+=3;
		}
		else if(gu32TxCANCounter == 13)
		{
			TIME_PERIOD_CNT = 0;
			gu32TxCANCounter = 0;
			CANTxFlag = FALSE;
		}
		else
		{
			gu32TxCANCounter++;
		}
	}

	else
	{
//			TxData[0] = 0;
//			TxData[1] = 0;
//			TxData[2] = 0;
//			TxData[3] = 0;
//			TxData[4] = 0;
//			TxData[5] = 0;
//			TxData[6] = 0;
//			TxData[7] = 0;
		canCurrentState = enmCANQUERY_IDLE;
		CANTxFlag = FALSE;
	}

//	gu32TxCANCounter++;
//	if(gu32TxCANCounter >= 1)
//		gu32TxCANCounter = 0;
}

void sendMessageCAN (void)
{
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  Error_Handler();

	}
//	TxData[0] = 0;
//	TxData[1] = 0;
//	TxData[2] = 0;
//	TxData[3] = 0;
//	TxData[4] = 0;
//	TxData[5] = 0;
//	TxData[6] = 0;
//	TxData[7] = 0;
//	CANTxFlag = FALSE;
}

/*
 *
 *
 *
 *
 */
void CAN_HeartBeat_Signal_Data()
{
	if(signal_qaulity > 31)
	{
		TelematicHeartBeat[enumTeleHearBeat_SIGNAL_QUALITY] = 0x00;//Invalid
	}
	else if((signal_qaulity >=26) && (signal_qaulity <=31))
	{
		TelematicHeartBeat[enumTeleHearBeat_SIGNAL_QUALITY] = 0x04;
	}
	else if((signal_qaulity >= 19) && (signal_qaulity <=25))
	{
		TelematicHeartBeat[enumTeleHearBeat_SIGNAL_QUALITY] = 0x03;
	}
	else if((signal_qaulity >=12) && (signal_qaulity <=18))
	{
		TelematicHeartBeat[enumTeleHearBeat_SIGNAL_QUALITY] = 0x02;
	}

	else if(signal_qaulity >=5 && signal_qaulity <= 11)
	{
		TelematicHeartBeat[enumTeleHearBeat_SIGNAL_QUALITY] = 0x01;
	}
	else
	{
		TelematicHeartBeat[enumTeleHearBeat_SIGNAL_QUALITY] = 0x00;
	}

	if((strcmp(gau8GSM_NWINFO,"\"GSM\"")==0) || (strcmp(gau8GSM_NWINFO,"\"GPRS\"")==0) || (strcmp(gau8GSM_NWINFO,"\"EDGE\"")==0))
	{
		TelematicHeartBeat[enumTeleHearBeat_NW_STATUS] = NW_2G;
		u8daignostic|=(1<<1); // 2G
	}
	else if((strcmp(gau8GSM_NWINFO,"\"TDD LTE\"") == 0) || (strcmp(gau8GSM_NWINFO,"\"FDD LTE\"")==0))
	{
		TelematicHeartBeat[enumTeleHearBeat_NW_STATUS] = NW_4G;
		u8daignostic|=(1<<2); // 4G
	}
	else
	{
		TelematicHeartBeat[enumTeleHearBeat_NW_STATUS] = NO_SERVICE;
		u8daignostic&=~((1<<0)|(1<<1)|(1<<2)); //
	}

	if((g_stAdcData.u32IntBatVtg >= 4.12))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x64; //100%
	}
	if((g_stAdcData.u32IntBatVtg <= 4.11) && (g_stAdcData.u32IntBatVtg > 4.04))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x5A; //90%
	}
	if((g_stAdcData.u32IntBatVtg <= 4.04) && (g_stAdcData.u32IntBatVtg > 3.96))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x50; //80%
	}
	if((g_stAdcData.u32IntBatVtg <= 3.96) && (g_stAdcData.u32IntBatVtg > 3.88))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x46; //70%
	}
	if((g_stAdcData.u32IntBatVtg <= 3.88) && (g_stAdcData.u32IntBatVtg > 3.08))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x3C; //60%
	}
	if((g_stAdcData.u32IntBatVtg <= 3.08) && (g_stAdcData.u32IntBatVtg > 3.72))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x32; //50%
	}
	if((g_stAdcData.u32IntBatVtg <= 3.72) && (g_stAdcData.u32IntBatVtg > 3.64))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x28; //40%
	}
	if((g_stAdcData.u32IntBatVtg <= 3.64) && (g_stAdcData.u32IntBatVtg > 3.56))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x1E; //30%
	}
	if((g_stAdcData.u32IntBatVtg <= 3.56) && (g_stAdcData.u32IntBatVtg > 3.48))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x14; //20%
	}
	if((g_stAdcData.u32IntBatVtg <= 3.48) && (g_stAdcData.u32IntBatVtg > 3.4))
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0x0A; //10%
	}
	if(g_stAdcData.u32IntBatVtg <=3.4)
	{
		TelematicHeartBeat[enumTeleHearBeat_INTERNAL_BATT_SOC_PER] = 0; //0%
	}


}


