/*!
** Copyright 2019 NXP
** @file main.c
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/

//#define WS_EMIOS_INST     0U
//#define WS_EMIOS_CH       7U
//#define WS_T0H_TICKS      25U
//#define WS_T1H_TICKS      59U
/* MODULE main */


/* Including necessary configuration files. */
#include "Mcal.h"
#include "Clock_Ip.h"
#include "FlexCAN_Ip.h"
#include "IntCtrl_Ip.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "Emios_Pwm_Ip.h"
#include "Emios_Pwm_Ip_Sa_PBcfg.h"

#define MSG_ID 0xC0FFEEu
#define RX_MB_IDX 1U
#define TX_MB_IDX 0U
volatile int exit_code = 0;
/* User includes */
uint8 dummyData[8] = {1,2,3,4,5,6,7};
Flexcan_Ip_DataInfoType tx_info = {
		.msg_id_type = FLEXCAN_MSG_ID_STD,
		.data_length = 8u,
		.is_polling = TRUE,
		.is_remote = FALSE
	};
Flexcan_Ip_DataInfoType rx_info = {
		.msg_id_type = FLEXCAN_MSG_ID_STD,
		.data_length = 8u,
		.is_polling = TRUE,
		.is_remote = FALSE
	};
/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
extern void CAN0_ORED_0_31_MB_IRQHandler(void);

void setupCan()
{

	uint8 sendData[8]={0,0,0,0,0,0,0,0};
	/* Filling of the frame's payload for transmission */
	//Siul2_Dio_Ip_WritePin(PTB_L_HALF, 12, 0U);
	Siul2_Dio_Ip_WritePin(STB_PORT, STB_PIN, 1U);
	Siul2_Dio_Ip_WritePin(EN_PORT, EN_PIN, 1U);

	FlexCAN_Ip_Init(INST_FLEXCAN_0, &FlexCAN_State0, &FlexCAN_Config0);
	FlexCAN_Ip_SetStartMode(INST_FLEXCAN_0);
	tx_info.msg_id_type = FLEXCAN_MSG_ID_STD;
	tx_info.data_length=8;
	FlexCAN_Ip_SendBlocking(INST_FLEXCAN_0, TX_MB_IDX, &tx_info, 0x555, sendData, 1000);
	uint32_t ID_Stinger = 0x18DA00F1;
	uint32_t PAYLOAD_FEC[2];
	uint8_t COMMAND_BYTE_FEC = 0x10;
	uint8_t DLC_FEC = 6;
	uint8_t Filter_Element_Index_0 = 0;
	uint8_t Filter_Element_Type = 1;
	uint8_t Extended_Filter_Config = 3; /* Both TWL and BBL */
	uint32_t ID_TX = 0xC0FFEE;

	/* Formation of the payload */
	uint32_t Filter_Element_Contents =  (Extended_Filter_Config << 29) | ID_TX ;

	PAYLOAD_FEC[0] = (COMMAND_BYTE_FEC << 24) | (Filter_Element_Index_0 << 16) |
					 (Filter_Element_Type << 15) | (Filter_Element_Contents >> 16);

	PAYLOAD_FEC[1] = (Filter_Element_Contents & 0xFFFF) << 16;
	tx_info.msg_id_type = FLEXCAN_MSG_ID_EXT;
	tx_info.data_length=DLC_FEC;
	sendData[3]=(PAYLOAD_FEC[0] & 0x000000FF);
	sendData[2]=(PAYLOAD_FEC[0] & 0x0000FF00) >> 8;
	sendData[1]=(PAYLOAD_FEC[0] & 0x00FF0000) >> 16;
	sendData[0]=(PAYLOAD_FEC[0] & 0xFF000000) >> 24;
	sendData[7]=(PAYLOAD_FEC[1] & 0x000000FF);
	sendData[6]=(PAYLOAD_FEC[1] & 0x0000FF00) >> 8;
	sendData[5]=(PAYLOAD_FEC[1] & 0x00FF0000) >> 16;
	sendData[4]=(PAYLOAD_FEC[1] & 0xFF000000) >> 24;
	FlexCAN_Ip_SendBlocking(INST_FLEXCAN_0, TX_MB_IDX, &tx_info, ID_Stinger, sendData, 1000);

	PAYLOAD_FEC[0] = 0x71020304;
	PAYLOAD_FEC[1] = 0x05060708;
	sendData[3]=(PAYLOAD_FEC[0] & 0x000000FF);
	sendData[2]=(PAYLOAD_FEC[0] & 0x0000FF00) >> 8;
	sendData[1]=(PAYLOAD_FEC[0] & 0x00FF0000) >> 16;
	sendData[0]=(PAYLOAD_FEC[0] & 0xFF000000) >> 24;
	sendData[7]=(PAYLOAD_FEC[1] & 0x000000FF);
	sendData[6]=(PAYLOAD_FEC[1] & 0x0000FF00) >> 8;
	sendData[5]=(PAYLOAD_FEC[1] & 0x00FF0000) >> 16;
	sendData[4]=(PAYLOAD_FEC[1] & 0xFF000000) >> 24;
	tx_info.data_length=8;
	FlexCAN_Ip_SendBlocking(INST_FLEXCAN_0, TX_MB_IDX, &tx_info, ID_Stinger, sendData, 1000);
	//Siul2_Dio_Ip_WritePin(PTB_L_HALF, 12, 1U);
	Siul2_Dio_Ip_WritePin(PTC_H_HALF, 4, 1U);
	FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN_0, RX_MB_IDX, &rx_info, MSG_ID);
	rx_info.is_polling = FALSE;
}


/* ---- WS2812 test: 800 kHz on eMIOS_0 CH7 / PTE7 (header P8A pin 24) ----
 * EMIOS0_CLK 160 MHz, channel prescaler /2 (module clock) -> 80 MHz, 1 tick = 12.5 ns
 * Period 100 ticks = 1.25 us (800 kHz).
 * Target at the pin as seen by the analyzer: "0" bit = 1/3 period (~417 ns), "1" bit = 2/3 period (~833 ns).
 * The board's level shifter makes the measured high time ~100 ns longer than programmed
 * (32 ticks = 400 ns programmed measured ~500 ns), so program ~100 ns less than the target:
 *   0 bit: 25 ticks (312 ns programmed -> ~417 ns measured)
 *   1 bit: 59 ticks (737 ns programmed -> ~833 ns measured)
 * Tune each by +/-1 tick (12.5 ns) from what you measure. For a direct pin with no level shifter
 * (like the STM32 project) use 33 and 66. */
#define WS_EMIOS_INST     0U
#define WS_EMIOS_CH       7U
//#define WS_T0H_TICKS      25U
//#define WS_T1H_TICKS      59U


#define WS_T0H_TICKS      33U     /* 0 bit: 33 x 12.5 ns = 412.5 ns high */
#define WS_T1H_TICKS      66U     /* 1 bit: 66 x 12.5 ns = 825 ns high   */


static void Ws2812_PwmTestInit(void)
{
    /* Turn on the eMIOS_0 global prescaler + global time base (GPRE = 0 -> /1).
     * Emios_Pwm_Ip does not do this, and there is no eMIOS init config from Mcl here. */
    IP_EMIOS_0->MCR = eMIOS_MCR_GTBE_MASK;
    IP_EMIOS_0->MCR |= eMIOS_MCR_GPREN_MASK;

    /* Make sure channel 7 is not disabled (UCDIS bit 7 = 0 means enabled) */
    IP_EMIOS_0->UCDIS &= ~((uint32)1U << WS_EMIOS_CH);

    /* Channel 7 OPWFMB, period 100, duty 32 come from Emios_Pwm_Ip_Sa_I0_Ch7 */
    Emios_Pwm_Ip_InitChannel(WS_EMIOS_INST, &Emios_Pwm_Ip_Sa_I0_Ch7);

    /* Start with the "0" bit width (the tool value is only a starting duty) */
    (void)Emios_Pwm_Ip_SetDutyCycle(WS_EMIOS_INST, WS_EMIOS_CH, WS_T1H_TICKS);
    /* For the "1" bit width instead, use:
     * (void)Emios_Pwm_Ip_SetDutyCycle(WS_EMIOS_INST, WS_EMIOS_CH, WS_T1H_TICKS); */
}

int main(void)
{
	uint8_t byte_0 = 0;

	Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);


	    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals , g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

	Ws2812_PwmTestInit();   /* PWM starts here, before CAN setup */


	IntCtrl_Ip_EnableIrq(FlexCAN0_1_IRQn);
	IntCtrl_Ip_InstallHandler(FlexCAN0_1_IRQn, CAN0_ORED_0_31_MB_IRQHandler, NULL_PTR);


	Flexcan_Ip_MsgBuffType rxData;
    setupCan();

    tx_info.is_polling = FALSE;
    FlexCAN_Ip_Send(INST_FLEXCAN_0, TX_MB_IDX, &tx_info, MSG_ID, (uint8 *)&dummyData);

	while(1)
	{
		//FlexCAN_Ip_Send(INST_FLEXCAN_0, TX_MB_IDX, &rx_info, MSG_ID, (uint8 *)&dummyData);
		// check if previous transfer is finished, if yes send new message
		if(FLEXCAN_STATUS_BUSY!=FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX))
		{
			dummyData[0]=byte_0++;
			FlexCAN_Ip_Send(INST_FLEXCAN_0, TX_MB_IDX, &tx_info, MSG_ID, (uint8 *)&dummyData);
		}
		//FlexCAN_Ip_Send(INST_FLEXCAN_0, TX_MB_IDX, &tx_info, MSG_ID, (uint8 *)&dummyData);
		//Flexcan_Ip_StatusType status = FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX);
		int ii;
		for(ii=0;ii<50000;ii++);
		//FlexCAN_Ip_Receive(INST_FLEXCAN_0, RX_MB_IDX, &rxData, TRUE);
	}
	{ FlexCAN_Ip_MainFunctionRead(INST_FLEXCAN_0, RX_MB_IDX); }
	FlexCAN_Ip_SetStopMode(INST_FLEXCAN_0);
	FlexCAN_Ip_Deinit(INST_FLEXCAN_0);
	return 0;
}

/* END main */
/*!
** @}
*/
