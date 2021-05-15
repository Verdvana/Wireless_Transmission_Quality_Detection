
/***********************************************************************************
    Filename:     per_test.h

    Description:  PER test header file

***********************************************************************************/

#ifndef PER_TEST_H
#define PER_TEST_H

/***********************************************************************************
* INCLUDES
*/
#include "hal_types.h"
#include "per_test_menu.h"

/***********************************************************************************
* TYPEDEFS
*/

// PER test packet format
typedef struct {
    uint32 seqNumber;
    uint8 padding[6];
} perTestPacket_t;

// PER test receiver statistics
typedef struct {
    uint32 expectedSeqNum;
    uint32 rcvdPkts;
    uint32 lostPkts;
    int16 rssiSum;
} perRxStats_t;

/***********************************************************************************
* CONSTANTS AND DEFINES
*/

// BasicRF definitions
#define PAN_ID                0x2007
#define TX_ADDR               0x2520
#define RX_ADDR               0xBEEF
#define MAX_PAYLOAD_LENGTH       103
#define PACKET_SIZE           sizeof(perTestPacket_t)

#define RSSI_AVG_WINDOW_SIZE   32  // Window size for RSSI moving average

#endif

//����
	uint32 burstSize=0;
	uint32 pktsSent=0;
	uint8 appTxPower;
//����
        uint8  i;   
        uint8  lostpack_flag;
        uint32 cnt_wait;
        uint32 segNumber=0;                              // ���ݰ����к� 
	int16 perRssiBuf[RSSI_AVG_WINDOW_SIZE] = {0};    // Ring buffer for RSSI �洢RSSI�Ļ��λ�����
	int16 perBuf[100] = {0};                         // Ring buffer for RSSI �洢PER�Ļ��λ�����  //verdvana 2021.1.4
	uint8 perRssiBufCounter = 0;                     // Counter to keep track of the ����������RSSI������ͳ��
	uint8 perBufCounter = 0;                         // Counter to keep track of the ����������RSSI������ͳ�� //verdvana 2021.1.4
	// oldest newest byte in RSSI
    // ring buffer
	perRxStats_t rxStats = {0,0,0,0};      
	int16 rssi;
        int16 per;
	uint8 resetStats=FALSE;
	
	int8 strPer[5];        
	int8 strRssi[2];
	int8 strCount[4];
	int32 nPer;           //��ŵ�����
	int32 nRssi;          //���ǰ32��rssiֵ��ƽ��ֵ    
	int32 nCount;         //��Ž��յİ��ĸ���
        uint8 gain;
/***********************************************************************************
  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/
