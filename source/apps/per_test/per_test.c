/***********************************************************************************
  Filename: 	per_test.c

  Description:  This application functions as a packet error rate (PER) tester.
  One node is set up as transmitter and the other as receiver. The role and
  configuration parameters for the PER test of the node is chosen on initalisation
  by navigating the joystick and confirm the choices with S1.

  The configuration parameters are channel, burst size and tx power. Push S1 to
  enter the menu. Then the configuration parameters are set by pressing
  joystick to right or left (increase/decrease value) and confirm with S1.

  After configuration of both the receiver and transmitter, the PER test is
  started by pressing joystick up on the transmitter. By pressing joystick up
  again the test is stopped.

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_int.h"
#include "hal_timer_32k.h"
#include "hal_joystick.h"
#include "hal_button.h"
#include "hal_board.h"
#include "hal_rf.h"
#include "hal_assert.h"
#include "util_lcd.h"
#include "basic_rf.h"
#include "per_test.h"
#include "string.h"

/***********************************************************************************
* CONSTANTS
*/
// Application states
#define IDLE                      0
#define TRANSMIT_PACKET           1

/***********************************************************************************
* LOCAL VARIABLES
*/
static basicRfCfg_t basicRfConfig;
static perTestPacket_t txPacket;
static perTestPacket_t rxPacket;
static volatile uint8 appState;
static volatile uint8 appStarted;

/***********************************************************************************
* LOCAL FUNCTIONS
*/
static void appTimerISR(void);
static void appStartStop(void);
static void appTransmitter();
static void appReceiver();
void initUART(void);//**************************
void UartTX_Send_String(int8 *Data,int len);//**********************


/****************************************************************
* 串口初始化函数
****************************************************************/
void initUART(void)
{ 
    PERCFG = 0x00;		      //位置1 P0口
    P0SEL = 0x0c;		      //P0_2,P0_3用作串口（外部设备功能）
    P2DIR &= ~0XC0;                   //P0优先作为UART0
	
    U0CSR |= 0x80;		      //设置为UART方式
    U0GCR |= 8;				       
    U0BAUD |= 59;		      //波特率设为115200
    UTX0IF = 0;                       //UART0 TX中断标志初始置位0
}
/****************************************************************
串口发送字符串函数			
****************************************************************/
void UartTX_Send_String(int8 *Data,int len)
{
	int j;
	for(j=0;j<len;j++)
	{
		U0DBUF = *Data++;
		while(UTX0IF == 0);
		UTX0IF = 0;
	}
}


/***********************************************************************************
* @fn          appTimerISR
*
* @brief       32KHz timer interrupt service routine. Signals PER test transmitter
*              application to transmit a packet by setting application state.
*
* @param       none
*
* @return      none
*/
static void appTimerISR(void)
{
    appState = TRANSMIT_PACKET;
}


/***********************************************************************************
* @fn          appStartStop
*
* @brief       Joystick up interrupt service routine. Start or stop 32KHz timer,
*              and thereby start or stop PER test packet transmission.
*
* @param       none
*
* @return      none
*/
static void appStartStop(void)
{
    // toggle value
    appStarted ^= 1;
	
    if(appStarted) {
        halTimer32kIntEnable();
    }
    else {
        halTimer32kIntDisable();
    }
}


/***********************************************************************************
* @fn          appConfigTimer
*
* @brief       Configure timer interrupts for application. Uses 32KHz timer
*
* @param       uint16 rate - Frequency of timer interrupt. This value must be
*              between 1 and 32768 Hz
*
* @return      none
*/
static void appConfigTimer(uint16 rate)
{
    halTimer32kInit(TIMER_32K_CLK_FREQ/rate);
    halTimer32kIntConnect(&appTimerISR);
}


/***********************************************************************************
* @fn          appReceiver
*
* @brief       Application code for the receiver mode. Puts MCU in endless loop
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              rxPacket - file scope variable of type perTestPacket_t
*
* @return      none
*/
static void initReceiver(){
	//initUART();           // 初始化串口
	
#ifdef INCLUDE_PA
	
	
	// Select gain (for modules with CC2590/91 only)
	gain =appSelectGain();
	halRfSetGain(gain);
#endif
    
    // Initialize BasicRF
	basicRfConfig.myAddr = RX_ADDR;
	if(basicRfInit(&basicRfConfig)==FAILED) 
	{
		HAL_ASSERT(FALSE);
	}
	basicRfReceiveOn();
	
	//halLcdClear();
    
	//halLcdWriteCharString(0,HAL_LCD_LINE_1, "Mode:Receiver");
	//halLcdWriteCharString(0,HAL_LCD_LINE_3, "Ready");
}
static void appReceiver()
{

	/* 主循环 */
	//verdvana UartTX_Send_String("PER_test: ",strlen("PER_test: "));
    // Main loop
	//while (TRUE) 
	//{
		//while(!basicRfPacketIsReady());  // 等待新的数据包
                
                //verdvana 2020.12.1--------------------
                cnt_wait = 10000;
                lostpack_flag = 0;
                while((!basicRfPacketIsReady()) & (cnt_wait!=0)){ //防卡死
                    cnt_wait--;
                    if(cnt_wait==0)
                      lostpack_flag = 1;
                    else
                      lostpack_flag = lostpack_flag;
                }
                //--------------------------------------
                
		if(basicRfReceive((uint8*)&rxPacket, MAX_PAYLOAD_LENGTH, &rssi)>0) {
			halLedSet(3);//*************P1_4
			
			// Change byte order from network to host order
			UINT32_NTOH(rxPacket.seqNumber);  // 改变接收序号的字节顺序
			segNumber = rxPacket.seqNumber;
            
			// If statistics is reset set expected sequence number to
			// received sequence number 若统计被复位，设置期望收到的数据包序号为已经收到的数据包序号     
			if(resetStats)
			{
				rxStats.expectedSeqNum = segNumber;
				
				resetStats=FALSE;
			}      
			// Subtract old RSSI value from sum
			//rxStats.rssiSum -= perRssiBuf[perRssiBufCounter];  // 从sum中减去旧的RSSI值  //verdvana 2021.01.04
			// Store new RSSI value in ring buffer, will add it to sum later
			//perRssiBuf[perRssiBufCounter] =  rssi;  // 存储新的RSSI值到环形缓冲区，之后它将被加入sum  //verdvana 2021.01.04
			
			//rxStats.rssiSum += perRssiBuf[perRssiBufCounter];  // 增加新的RSSI值到sum  //verdvana 2021.01.04
                        //verdvana 2021.01.04
			//if(++perRssiBufCounter == RSSI_AVG_WINDOW_SIZE) {  
			//	perRssiBufCounter = 0;      // Wrap ring buffer counter 
			//}
                        
                        //verdvana 2021.01.04
                        if(++perBufCounter == 100) {
				perBufCounter = 0;      // Wrap ring buffer counter 
			}
			
			
			// 检查接收到的数据包是否是所期望收到的数据包
			if(rxStats.expectedSeqNum == segNumber)  // 是所期望收到的数据包 
			{
				rxStats.expectedSeqNum++;  
                                perBuf[perBufCounter]=0;  //verdvana 2021.01.04
			}
			
            // 不是所期望收到的数据包（大于期望收到的数据包的序号）认为丢包
			else if(rxStats.expectedSeqNum < segNumber)  
			{
				rxStats.lostPkts += segNumber - rxStats.expectedSeqNum;
				rxStats.expectedSeqNum = segNumber + 1;
                                perBuf[perBufCounter]=1;  //verdvana 2021.01.04
			}
			// If the sequence number is lower than the previous one, we will assume a
			// new data burst has started and we will reset our statistics variables.
			else  // (小于期望收到的数据包的序号）
			{      
				// Update our expectations assuming this is a new burst 认为是一个新的测试开始，复位统计变量
				rxStats.expectedSeqNum = segNumber + 1;
                                perBuf[perBufCounter]=1;  //verdvana 2021.01.04
				rxStats.rcvdPkts = 0;
				rxStats.lostPkts = 0;
			}
			rxStats.rcvdPkts++;
			
			// reset statistics if button 1 is pressed
            nCount = (int32)rxStats.rcvdPkts;
			if(nCount > 998)
                        {
			//	if(halButtonPushed()==HAL_BUTTON_1){
					resetStats = TRUE;
					rxStats.rcvdPkts = 1;
					rxStats.lostPkts = 0;
			//	}
                        }
                        
                        //verdvana 2021.01.04
                        per = 0;
                        for(i=0;i<100;i++){
                          per = per + perBuf[i];
                        }
                        
                        
            if(lostpack_flag==0)
              //nRssi=(0-(int32)rxStats.rssiSum/32);
              nRssi=(0-rssi);
            else
              nRssi=(99);
            
            
            
            nPer = (int32)(per);//verdvana 2021.01.04
            //nPer = (int32)((rxStats.lostPkts*1000)/(rxStats.lostPkts+rxStats.rcvdPkts));
						
            //串口输出
            UartTX_Send_String("Z", strlen("Z"));
            
            strCount[0]=nCount/100+'0';
            strCount[1]=nCount%100/10+'0';
            strCount[2]=nCount%10+'0';
            UartTX_Send_String(strCount, 3); 
            strPer[0]=nPer/100%10+'0';
            strPer[1]=nPer/10%10+'0'; 
            strPer[2]=nPer/1%10+'0';  //verdvana
            strPer[4]='%';
            UartTX_Send_String(strPer,3); //verdvana 
		
            
            strRssi[0]=nRssi/10+'0';
            strRssi[1]=nRssi%10+'0';
            UartTX_Send_String(strRssi,2);        
            
            
            //串口输出
            //strCount[0]=nCount/100+'0';
            //strCount[1]=nCount%100/10+'0';
            //strCount[2]=nCount%10+'0';
            //strCount[3]='\0';
            //UartTX_Send_String("RECV:", strlen("RECV:"));
            //UartTX_Send_String(strCount, 4);
            //UartTX_Send_String("  ", strlen("  "));   
            
            //strPer[0]=nPer/100%10+'0';
            //strPer[1]=nPer/10%10+'0'; 
            ////strPer[2]='.';
            //strPer[2]=nPer/1%10+'0';
            //strPer[3]='%';
            //UartTX_Send_String("PER:",strlen("PER:"));
            //UartTX_Send_String(strPer,4);
            //UartTX_Send_String("  ",strlen("  "));
			
            //strRssi[0]=nRssi/10+'0';
            //strRssi[1]=nRssi%10+'0';
            //UartTX_Send_String(" RSSI:-",strlen(" RSSI:-"));
            //UartTX_Send_String(strRssi,2);        
            //UartTX_Send_String("\r\n",strlen("\r\n"));
			
            
            
            
            
            
	    halLedClear(3);
			
			
	    //halMcuWaitMs(100);
            
            // Update LCD
            // PER in units per 1000
			/*         utilLcdDisplayValue(HAL_LCD_LINE_1, "PER: ", (int32)((rxStats.lostPkts*1000)/(rxStats.lostPkts+rxStats.rcvdPkts)), " /1000");
            utilLcdDisplayValue(HAL_LCD_LINE_2, "RSSI: ", (int32)rxStats.rssiSum/32, "dBm");
            #ifndef SRF04EB
            utilLcdDisplayValue(HAL_LCD_LINE_3, "Recv'd: ", (int32)rxStats.rcvdPkts, NULL);
            #endif
            halLedClear(3);*/
	}
		
  //}
}


/***********************************************************************************
* @fn          appTransmitter
*
* @brief       Application code for the transmitter mode. Puts MCU in endless
*              loop
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              txPacket - file scope variable of type perTestPacket_t
*              appState - file scope variable. Holds application state
*              appStarted - file scope variable. Controls start and stop of
*                           transmission
*
* @return      none
*/

static void initTransmitter()
{
	uint8 n;
	
	// Initialize BasicRF               /* 初始化Basic RF */
	basicRfConfig.myAddr = TX_ADDR;
	if(basicRfInit(&basicRfConfig)==FAILED) 
	{
		HAL_ASSERT(FALSE);
	}
	
	// Set TX output power                      /* 设置输出功率 */
	//appTxPower = appSelectOutputPower();
	//halRfSetTxPower(1);//HAL_RF_TXPOWER_MIN_20_DBM
	halRfSetTxPower(9);//HAL_RF_TXPOWER_4_DBM
	//  halRfSetTxPower(appTxPower);
	
	// Set burst size                 /* 设置进行一次测试所发送的数据包数量 */
	//burstSize = appSelectBurstSize();
	burstSize = 1000;
	
	// Basic RF puts on receiver before transmission of packet, and turns off
	// after packet is sent
	basicRfReceiveOff();
	
	// Config timer and IO          /* 配置定时器和IO *************************/
	//n= appSelectRate();
	appConfigTimer(0xC8);
	//halJoystickInit();
	
	// Initalise packet payload            /* 初始化数据包载荷 */
	txPacket.seqNumber = 0;
	for(n = 0; n < sizeof(txPacket.padding); n++) 
	{
		txPacket.padding[n] = n;
	}
	
	//halLcdClear();
	//halLcdWriteCharString(0,HAL_LCD_LINE_1, "Mode:Transmitter");
	//halLcdWriteCharString(0,HAL_LCD_LINE_2, "CENTER to start/stop");
}
static void appTransmitter()
{


	
    // Main loop          /* 主循环 */
	//while (TRUE)    //2020.12.1
	//{               //2020.12.1
		// Wait for user to start application
		//while(!halJoystickPushed());  // 等待用户启动应用
		//appStartStop();   
		
		//if(appStarted)
		//{			
		//	if (pktsSent < burstSize) 
			{
				//if( appState == TRANSMIT_PACKET ) 
				//{
				// Make sure sequence number has network byte order
				UINT32_HTON(txPacket.seqNumber);  // 改变发送序号的字节顺序
				basicRfSendPacket(RX_ADDR, (uint8*)&txPacket, PACKET_SIZE);
				
				// Change byte order back to host order before increment     /* 在增加序号前将字节顺序改回为主机顺序 */
				UINT32_NTOH(txPacket.seqNumber);
				txPacket.seqNumber++;
                                
				
                                pktsSent++;

				
				appState = IDLE;
				halLedToggle(1);   //改变LED1的亮灭状态
				//halMcuWaitMs(20);
                                
			}
		//	else
		//		appStarted = !appStarted;
			
			// Reset statistics and sequence number/* 复位统计和序号 */
			//pktsSent = 0;
		//}
	//}
}


/***********************************************************************************
* @fn          main
*
* @brief       This is the main entry of the "PER test" application.
*
* @param       basicRfConfig - file scope variable. Basic RF configuration data
*              appState - file scope variable. Holds application state
*              appStarted - file scope variable. Used to control start and stop of
*              transmitter application.
*
* @return      none
*/
void main (void)
{
    uint8 appMode = MODE_RX;   //appMode = MODE_TX; 时为发射模块
                               //appMode = MODE_RX; 时为接收模块 
    
    initUART();           // 初始化串口
    
    appState = IDLE;
    appStarted = TRUE;
	
    // Config basicRF
    basicRfConfig.panId = PAN_ID;
    basicRfConfig.ackRequest = FALSE;
	
    // Initialise board peripherals
    halBoardInit();
	
    // Initalise hal_rf
    if(halRfInit()==FAILED) {
		HAL_ASSERT(FALSE);
    }
    
    // Indicate that device is powered
    halLedSet(1);
	
    // Print Logo and splash screen on LCD
    utilPrintLogo("PER Tester");
	
    // Wait for user to press S1 to enter menu
	//    while (halButtonPushed()!=HAL_BUTTON_1);***********************
    halMcuWaitMs(350);
	//    halLcdClear();************************************
	
    // Set channel
	//  basicRfConfig.channel = appSelectChannel();
    basicRfConfig.channel = 0x0B;
    
    initTransmitter();
    initReceiver();
    while(1){
      appTransmitter();
      halMcuWaitMs(20);
      appReceiver();
      
      //UartTX_Send_String("A", strlen("A"));
    
    }
    /*  //2020.12.1
    // Transmitter application
    if(appMode == MODE_TX) {
        // No return from here
        appTransmitter();
    }
    // Receiver application
    else if(appMode == MODE_RX) {
        // No return from here
        appReceiver();
    }
    // Role is undefined. This code should not be reached
    HAL_ASSERT(FALSE);
    */
}


/***********************************************************************************
  Copyright 2008 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
