/*
  *****************************************************************************
  * @file    User_Appliaction.c
  * @author  KloudQ Team
  * @version
  * @date
  * @brief   Functions for For Init Config settings and Debug Messages
*******************************************************************************
*/
/******************************************************************************

            Copyright (c) by KloudQ Technologies Limited.

  This software is copyrighted by and is the sole property of KloudQ
  Technologies Limited.
  All rights, title, ownership, or other interests in the software remain the
  property of  KloudQ Technologies Limited. This software may only be used in
  accordance with the corresponding license agreement. Any unauthorized use,
  duplication, transmission, distribution, or disclosure of this software is
  expressly forbidden.

  This Copyright notice may not be removed or modified without prior written
  consent of KloudQ Technologies Limited.

  KloudQ Technologies Limited reserves the right to modify this software
  without notice.

*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "applicationdefines.h"
#include "externs.h"
#include "user_modbus_rs485.h"



ST_Config st_DeviceConfig = {0};

/****************************************************************************
 Function: UserAppl_Init
 Purpose: Initialize the user editable settings.

 Input:	None
 Return value: None


 Note(s)(if-any) :

 Change History:
 Author            	Date                Remarks
 KloudQ Team        03/06/2022			100367
******************************************************************************/
void UserAppl_FillDefaultparameters(ST_Config *ptrStruct)
{

/*1.Data uploading frequency*/
	ptrStruct->u32DataUploadingFreq = DATA_UPLADING_FREQUENCY;

 /*4.if u8BleEscortSensEnableFlag is Set then BLE Sensor name*/
	 ptrStruct->u8BufferMemEnFlag = 0;

	 ptrStruct->u8BleEscortSensEnableFlag = 0;//
	 /*3.if u8BleEscortSensEnableFlag is Set then BLE Sensor name*/
	 strcpy((char*)ptrStruct->u8EscortSensID,(const char*)"TD_000000");

/*4.Modbus Settings*/
	ptrStruct->u8ModbusMaterEnFlag = 1;

	ptrStruct->enumMbBaudRate = enum_BAUD_RATE_19200;//enum_BAUD_RATE_9600; TODO remove for cummins testing
	ptrStruct->enumMBParity = enum_USART_PARITY_NONE; //enum_USART_PARITY_NONE;
	ptrStruct->enumMbStopBit = enum_USART_STOPBITS_1;

	/* IO CARD Data and Hour meter Data */
	ptrStruct->u16MbTotalNoOfQuerys = 47;  //Total Query this can be from 0 to 50

//	ptrStruct->stMbMasterQuerysArr[0].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[0].u8MbFunctionCode = 0x03;//
//	ptrStruct->stMbMasterQuerysArr[0].u16MbMbAddress = 5;  	ptrStruct->stMbMasterQuerysArr[0].u16MBNoPoints = 3;

	ptrStruct->stMbMasterQuerysArr[0].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[0].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[0].u16MbMbAddress = 9;  	ptrStruct->stMbMasterQuerysArr[0].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[1].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[1].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[1].u16MbMbAddress = 15;  	ptrStruct->stMbMasterQuerysArr[1].u16MBNoPoints = 5;

//	ptrStruct->stMbMasterQuerysArr[2].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[2].u8MbFunctionCode = 0x03;
//	ptrStruct->stMbMasterQuerysArr[2].u16MbMbAddress = 17;  	ptrStruct->stMbMasterQuerysArr[2].u16MBNoPoints = 3;

	ptrStruct->stMbMasterQuerysArr[2].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[2].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[2].u16MbMbAddress = 21;  	ptrStruct->stMbMasterQuerysArr[2].u16MBNoPoints = 3;

	ptrStruct->stMbMasterQuerysArr[3].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[3].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[3].u16MbMbAddress = 25;  	ptrStruct->stMbMasterQuerysArr[3].u16MBNoPoints = 3;

	ptrStruct->stMbMasterQuerysArr[4].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[4].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[4].u16MbMbAddress = 30;  	ptrStruct->stMbMasterQuerysArr[4].u16MBNoPoints = 9;

//	ptrStruct->stMbMasterQuerysArr[6].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[6].u8MbFunctionCode = 0x03;
//	ptrStruct->stMbMasterQuerysArr[6].u16MbMbAddress = 34;  	ptrStruct->stMbMasterQuerysArr[6].u16MBNoPoints = 4;

//	ptrStruct->stMbMasterQuerysArr[7].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[7].u8MbFunctionCode = 0x03;
//	ptrStruct->stMbMasterQuerysArr[7].u16MbMbAddress = 38;  	ptrStruct->stMbMasterQuerysArr[7].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[5].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[5].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[5].u16MbMbAddress = 39;  	ptrStruct->stMbMasterQuerysArr[5].u16MBNoPoints = 5;

//	ptrStruct->stMbMasterQuerysArr[9].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[9].u8MbFunctionCode = 0x03;
//	ptrStruct->stMbMasterQuerysArr[9].u16MbMbAddress = 43;  	ptrStruct->stMbMasterQuerysArr[9].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[6].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[6].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[6].u16MbMbAddress = 49;  	ptrStruct->stMbMasterQuerysArr[6].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[7].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[7].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[7].u16MbMbAddress = 60;  	ptrStruct->stMbMasterQuerysArr[7].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[8].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[8].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[8].u16MbMbAddress = 62;  	ptrStruct->stMbMasterQuerysArr[8].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[9].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[9].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[9].u16MbMbAddress = 63;  	ptrStruct->stMbMasterQuerysArr[9].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[10].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[10].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[10].u16MbMbAddress = 67;  	ptrStruct->stMbMasterQuerysArr[10].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[11].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[11].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[11].u16MbMbAddress = 69;  	ptrStruct->stMbMasterQuerysArr[11].u16MBNoPoints = 2;

//	ptrStruct->stMbMasterQuerysArr[16].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[16].u8MbFunctionCode = 0x03;
//	ptrStruct->stMbMasterQuerysArr[16].u16MbMbAddress = 70;  	ptrStruct->stMbMasterQuerysArr[16].u16MBNoPoints = 1;

//	ptrStruct->stMbMasterQuerysArr[16].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[16].u8MbFunctionCode = 0x03;
//	ptrStruct->stMbMasterQuerysArr[16].u16MbMbAddress = 73;  	ptrStruct->stMbMasterQuerysArr[16].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[12].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[12].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[12].u16MbMbAddress = 10000;  	ptrStruct->stMbMasterQuerysArr[12].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[13].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[13].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[13].u16MbMbAddress = 10081;  	ptrStruct->stMbMasterQuerysArr[13].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[14].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[14].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[14].u16MbMbAddress = 10083;  	ptrStruct->stMbMasterQuerysArr[14].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[15].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[15].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[15].u16MbMbAddress = 10089;  	ptrStruct->stMbMasterQuerysArr[15].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[16].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[16].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[16].u16MbMbAddress = 191;  	ptrStruct->stMbMasterQuerysArr[16].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[17].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[17].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[17].u16MbMbAddress = 193;  	ptrStruct->stMbMasterQuerysArr[17].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[18].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[18].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[18].u16MbMbAddress = 206;  	ptrStruct->stMbMasterQuerysArr[18].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[19].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[19].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[19].u16MbMbAddress = 399;  	ptrStruct->stMbMasterQuerysArr[19].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[20].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[20].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[20].u16MbMbAddress = 402;  	ptrStruct->stMbMasterQuerysArr[20].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[21].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[21].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[21].u16MbMbAddress = 406;  	ptrStruct->stMbMasterQuerysArr[21].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[22].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[22].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[22].u16MbMbAddress = 426;  	ptrStruct->stMbMasterQuerysArr[22].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[23].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[23].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[23].u16MbMbAddress = 1023;  	ptrStruct->stMbMasterQuerysArr[23].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[24].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[24].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[24].u16MbMbAddress = 1416;  	ptrStruct->stMbMasterQuerysArr[24].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[25].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[25].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[25].u16MbMbAddress = 1417;  	ptrStruct->stMbMasterQuerysArr[25].u16MBNoPoints = 1;

//	ptrStruct->stMbMasterQuerysArr[31].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[31].u8MbFunctionCode = 0x03;
//	ptrStruct->stMbMasterQuerysArr[31].u16MbMbAddress = 25;  	ptrStruct->stMbMasterQuerysArr[31].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[26].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[26].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[26].u16MbMbAddress = 3064;  	ptrStruct->stMbMasterQuerysArr[26].u16MBNoPoints = 2;

	ptrStruct->stMbMasterQuerysArr[27].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[27].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[27].u16MbMbAddress = 3561;  	ptrStruct->stMbMasterQuerysArr[27].u16MBNoPoints = 1;

	ptrStruct->stMbMasterQuerysArr[28].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[28].u8MbFunctionCode = 0x03;//
	ptrStruct->stMbMasterQuerysArr[28].u16MbMbAddress = 3792;  	ptrStruct->stMbMasterQuerysArr[28].u16MBNoPoints = 2;

	//adding for PS0602
	ptrStruct->stMbMasterQuerysArr[29].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[29].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[29].u16MbMbAddress = 10;  	ptrStruct->stMbMasterQuerysArr[29].u16MBNoPoints = 4;//upto14

	ptrStruct->stMbMasterQuerysArr[30].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[30].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[30].u16MbMbAddress = 299;  	ptrStruct->stMbMasterQuerysArr[30].u16MBNoPoints = 2;//upto 301

	ptrStruct->stMbMasterQuerysArr[31].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[31].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[31].u16MbMbAddress = 3745;  	ptrStruct->stMbMasterQuerysArr[31].u16MBNoPoints = 1;//read 3746

	ptrStruct->stMbMasterQuerysArr[32].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[32].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[32].u16MbMbAddress = 3744;  	ptrStruct->stMbMasterQuerysArr[32].u16MBNoPoints = 1;//read 3745

	ptrStruct->stMbMasterQuerysArr[33].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[33].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[33].u16MbMbAddress = 3827;  	ptrStruct->stMbMasterQuerysArr[33].u16MBNoPoints = 3;//upto 3830

	ptrStruct->stMbMasterQuerysArr[34].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[34].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[34].u16MbMbAddress = 3830;  	ptrStruct->stMbMasterQuerysArr[34].u16MBNoPoints = 2;//upto 3831(32 bit)

	ptrStruct->stMbMasterQuerysArr[35].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[35].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[35].u16MbMbAddress = 117;  	ptrStruct->stMbMasterQuerysArr[35].u16MBNoPoints = 3;//upto 120

	ptrStruct->stMbMasterQuerysArr[36].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[36].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[36].u16MbMbAddress = 121;  	ptrStruct->stMbMasterQuerysArr[36].u16MBNoPoints = 3;//upto 124

	ptrStruct->stMbMasterQuerysArr[37].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[37].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[37].u16MbMbAddress = 143;  	ptrStruct->stMbMasterQuerysArr[37].u16MBNoPoints = 2;//read 144(32 bit)

	ptrStruct->stMbMasterQuerysArr[38].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[38].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[38].u16MbMbAddress = 1021;  	ptrStruct->stMbMasterQuerysArr[38].u16MBNoPoints = 2;//upto 1023

	ptrStruct->stMbMasterQuerysArr[39].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[39].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[39].u16MbMbAddress = 3747;  	ptrStruct->stMbMasterQuerysArr[39].u16MBNoPoints = 1;//read 3748

	ptrStruct->stMbMasterQuerysArr[40].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[40].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[40].u16MbMbAddress = 616;  	ptrStruct->stMbMasterQuerysArr[40].u16MBNoPoints = 1;//read 617

	ptrStruct->stMbMasterQuerysArr[41].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[41].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[41].u16MbMbAddress = 2356;  	ptrStruct->stMbMasterQuerysArr[41].u16MBNoPoints = 2;//read 2357 & 2358

	ptrStruct->stMbMasterQuerysArr[42].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[42].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[42].u16MbMbAddress = 2359;  	ptrStruct->stMbMasterQuerysArr[42].u16MBNoPoints = 2;//read 2360 & 2361

	ptrStruct->stMbMasterQuerysArr[43].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[43].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[43].u16MbMbAddress = 2369;  	ptrStruct->stMbMasterQuerysArr[43].u16MBNoPoints = 1;//read 2370

	ptrStruct->stMbMasterQuerysArr[44].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[44].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[44].u16MbMbAddress = 2364;  	ptrStruct->stMbMasterQuerysArr[44].u16MBNoPoints = 1;//read 2365

	ptrStruct->stMbMasterQuerysArr[45].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[45].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[45].u16MbMbAddress = 2366;  	ptrStruct->stMbMasterQuerysArr[45].u16MBNoPoints = 1;//read 2367

	ptrStruct->stMbMasterQuerysArr[46].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[46].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[46].u16MbMbAddress = 263;  	ptrStruct->stMbMasterQuerysArr[46].u16MBNoPoints = 1;//read 2367

	ptrStruct->stMbMasterQuerysArr[47].u8MbSlaveID = 1;		ptrStruct->stMbMasterQuerysArr[47].u8MbFunctionCode = 0x03;
	ptrStruct->stMbMasterQuerysArr[47].u16MbMbAddress = 20;  	ptrStruct->stMbMasterQuerysArr[47].u16MBNoPoints = 1;//read 21
	

}


