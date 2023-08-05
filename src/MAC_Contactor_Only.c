/***********************************************************************
*
*  FILE        : MAC.c
*  DATE        : 2023-03-13
*  DESCRIPTION : Main Program
*
*  NOTE:THIS IS A TYPICAL EXAMPLE.
*
***********************************************************************/
#include "r_smc_entry.h"
#include "platform.h"
#include "string.h"
#include "math.h"
#include "r_fw_up_rx_if.h"
#include "deviceFlash.h"
#include "storage.h"
/***************** MAC PARAMETTERS *************************************************/
#define SAMPLES_NUM 64
#define PC1X_ID 0x01
#define DST4400_ID 0x01
#define D300_ID 0x01
#define DSE7320_ID 10 //baud 115200
#define GC315_ID 0x02
#define DKG307_ID 0x01
#define Smartgen_ID 0x01
#define Emko_ID 0x01
#define MAC_ID 0x06
#define MAC_timeout_level 600
#define MAC_VERSION 4
typedef enum {
	SMARTGEN =1,
	EMKO =2,
	DKG307 =3, //DKG307
	GC315 =4, //GC315, GC250
	DSE7320 =5, //DSE7320, DSE6110
	D300 =6, //D300,D500,D700
	DST4400 =7,
	PC1X =8 //Power command 1x, 3x
} generator_type;
generator_type GENTYPE = EMKO;
#define PHASE_ATS 0 // 3pha ATS
//#define PHASE_ATS 1 // 3pha khong ATS
//#define PHASE_ATS 2 // 1pha ATS
//#define PHASE_ATS 3 // 1pha khong ATS
/***************** MAC GPIO *************************************************/
#define LED_STT (PORT2.PODR.BIT.B7)
#define LED_GEN (PORTH.PODR.BIT.B3)
#define LED_MCC (PORTH.PODR.BIT.B0)
#define WDI (PORT3.PODR.BIT.B2)
#define BUZZER (PORTA.PODR.BIT.B0)
#define RS485_DE1 (PORTE.PODR.BIT.B0) // RS485 connect MCC
#define RS485_DE2 (PORTA.PODR.BIT.B6) // RS485 connect Generator
#define CTRL_RELAY2 (PORTB.PODR.BIT.B0)
#define CTRL_RELAY1 (PORTB.PODR.BIT.B1)
#define ATS_CTRL_CONTACTOR_GEN (PORTB.PODR.BIT.B7)
#define ATS_CTRL_CONTACTOR_GRID (PORTB.PODR.BIT.B6) // tiếp điểm thường đóng
#define ATS_CTRL_GEN_START_1 (PORTB.PODR.BIT.B5) // Start GENERATOR
#define ATS_CTRL_GEN_START_2 (PORTB.PODR.BIT.B3)
#define FB_GEN_COIL (PORT4.PIDR.BIT.B6)
#define FB_GRID_COIL (PORT4.PIDR.BIT.B7)
#define DOOR_OPEN_DETECT (PORTE.PIDR.BIT.B3)
#define CB_RUNG_DETECT (PORTE.PIDR.BIT.B4)

//static char version_12[40] ="\n MAC Dev. 2023 Mar. 16 - 10:23 12";
//static char version_5[40] = "\n MAC Dev. 2023 Mar. 16 - 10:23 5";
/***************** MAC SCI **********************************************/
char print_str[200];
extern char rx1_buff[60];
extern char rx12_buff[128];
extern char rx5_buff[260];
extern uint16_t  g_sci12_rx_count;
extern volatile uint8_t SCI5_rxdone;
extern storage_data_t storageData;
//extern uint16_t Mfr,ID;
/***************** MAC MEASUREMENT **********************************************/
extern float period0,period1,period4;
extern uint8_t state0,state1,state4;
extern volatile uint8_t l0_edge_detected;
extern volatile uint8_t l3a_edge_detected;
extern volatile uint8_t l3c_edge_detected;
extern float grid_freq1,grid_freq2,grid_freq3;
extern volatile bool Sample_done;
uint16_t ADC_VGen1[SAMPLES_NUM], ADC_VGen2[SAMPLES_NUM], ADC_VGen3[SAMPLES_NUM];
uint16_t ADC_VGrid1[SAMPLES_NUM], ADC_VGrid2[SAMPLES_NUM], ADC_VGrid3[SAMPLES_NUM];
uint16_t ADC_temperature[SAMPLES_NUM];
float grid_volt1,grid_volt2,grid_volt3;
float gen_volt1,gen_volt2,gen_volt3;
float load_volt1,load_volt2,load_volt3;
float load_cur1,load_cur2,load_cur3;
float load_freq;
extern bool gen_running_timeout;
uint32_t GenRunTime=0;
uint8_t GenStart=0;
extern uint32_t wait_time;
uint32_t Timer1Relay5=0;
uint32_t Timer2Relay5=0;
uint32_t MAC_timeout=0;
extern uint32_t freq_ustbl_time_count;
uint16_t freq_start; // for checking unstable frequency
uint8_t g_is_all_relay_off=0;
uint8_t trans_enable=0;
uint8_t GenIsConnected =0;
uint8_t error_check=0;

float cof_vol[12] = {-121.9066, 121.9066*3.3,  -122.1679, 122.1679*3.3,   -119.24,  119.24*3.3,
					 -123.7055, 123.7055*3.3,  -120.2011, 120.2011*3.3,   -120.7989,120.7989*3.3
};
float cof_vol_test[12] = {-11, 121.9066*3.3,  -12, 122.1679*3.3,   -13,  119.24*3.3,
					 -14, 123.7055*3.3,  -15, 120.2011*3.3,   -16,120.7989*3.3
};
/***************** MAC REGISTERS *************************************************/
uint16_t MAC_registers[130];
uint8_t Rs485_MasterResponse[265];// Response to MCC
uint16_t Gen_registers[120];
uint16_t previous_mode;

/***************** FUNCTION *************************************************/
MD_STATUS R_SCI12_AsyncTransmit (uint8_t * const tx_buf,uint16_t tx_num,uint8_t control);
uint16_t CRC16_bytewise (uint8_t *nData, int wLength);
void uint16_to_uint8(uint16_t input, uint8_t* Hi_byte, uint8_t* Low_byte);
void RS485_Slave_Mode(uint16_t* MAC_registers);
void RS485_Master_Mode(uint16_t *Slave_registers,uint16_t *MAC_registers, generator_type gen_type);
void Grid_Volt_Cal(void);
void Gen_Volt_Cal(void);
uint8_t Gen_check();
uint8_t Grid_check();
void RS485_M_Cmd04_and_Receive(uint8_t slaveID, uint32_t StartAdd, uint16_t NoR, uint16_t * Value);
void RS485_M_Cmd03_and_Receive(uint8_t slaveID, uint32_t StartAdd, uint16_t NoR, uint16_t * Value);
void RS485_M_Cmd01_and_Receive(uint8_t slaveID, uint32_t StartAdd, uint16_t NoR, uint16_t * Value);

void GENERATOR_START(generator_type gen_type);
void GENERATOR_STOP(generator_type gen_type);
void Smartgen_Stop(uint8_t slaveID, uint16_t Coil_address, uint16_t Value);
void Smartgen_Start(uint8_t slaveID, uint16_t Coil_address, uint16_t Value);
void Smartgen_Manual(uint8_t slaveID, uint16_t Coil_address, uint16_t Value);
void Emko_Stop();
void Emko_Start();
void DKG307_Stop();
void DKG307_Start();
void GC315_Start();
void GC315_Stop();
void DSE7320_Start();
void DSE7320_Stop();
void D300_Start();
void D300_Stop();
void DST4400_Start();
void DST4400_Stop();
void PC1X_Start();
void PC1X_Stop();

void Import_PC1X_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers);
void Import_DST4400_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers);
void Import_D300_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers);
void Import_DSE7320_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers);
void Import_GC315_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers);
void Import_DKG307_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers);
void Import_Smartgen_Reg(uint16_t *Smartgen_reg, uint16_t *MAC_registers);
void Import_Emko_Reg(uint16_t *Smartgen_reg, uint16_t *MAC_registers);

void Load_Check();
void ATM_CMD_read(uint8_t channel);
void ATM_CMD_FREQ();
void ATM_CMD_AUX();
void Mode_UseGrid();
void Mode_UseGen();
void Process_OffAll();
void Mode_Auto();
void Process_StartGen();
void Process_StartGen_Manual();
void Process_StoptGen_Manual();
void waittime(uint16_t second);
uint8_t Mode_DeactiveCheck();
void main(void);
void Buzzer(uint8_t times, uint16_t millisec);
void main_FW_update(void);
uint8_t ErrorCheck(void);
void MAC_reg_init();
void Grid_Contactor_Off();
void Grid_Contactor_On();
void Gen_Contactor_Off();
void Gen_Contactor_On();
void Generator_Switch();
void TimerMode();
void Relay5_On();
void Relay5_Off();

void main(void)
{
	memset(ADC_VGen1,0,sizeof(ADC_VGen1));
	memset(ADC_VGen2,0,sizeof(ADC_VGen2));
	memset(ADC_VGen3,0,sizeof(ADC_VGen3));
	memset(ADC_VGrid1,0,sizeof(ADC_VGrid1));
	memset(ADC_VGrid2,0,sizeof(ADC_VGrid2));
	memset(ADC_VGrid3,0,sizeof(ADC_VGrid3));
	MAC_reg_init(); // Init MAC registers
	R_Config_CMT0_Start();
	R_Config_CMT1_Start();
	R_Config_ICU_IRQ0_Start();
	R_Config_ICU_IRQ1_Start();
	R_Config_ICU_IRQ4_Start();
	MTU.TRWER.BIT.RWE = 1U;
	R_Config_MTU0_Start();
	R_Config_MTU3_Start();
	R_Config_RSPI0_Start();
	R_Config_SCI5_Start();// RS485 Connect Generator
	R_Config_SCI1_Start();// Current Sensor
	R_Config_SCI12_Start();// RS485 Connect MCC
	R_Config_SCI12_Serial_Receive((uint8_t*)&rx12_buff, sizeof(rx12_buff));//MAC buffer
	g_sci12_rx_count=0;


//	deviceFlash_readMfrID(&Mfr, &ID);
//	//print test
//	memset(print_str, 0, sizeof(print_str));
//	sprintf(print_str,"Mfr: %d, ID: %d",Mfr, ID);
//	RS485_DE1 = 1U; //RS485 send mode
//	R_SCI12_AsyncTransmit((uint8_t*)print_str,strlen(print_str),false);
//	RS485_DE1 = 0U; //RS485 send mode
//	memset(print_str, 0, sizeof(print_str));
//	//end_print_test

	R_BSP_SoftwareDelay(1000,BSP_DELAY_MILLISECS);
	for(uint8_t i=0; i<5;i++)
	{
		storage_readDataStorage();
	}
	if(storageData.isCalib)
	{
		for(int i = 0 ; i < 12; i++)
		{
			cof_vol[i] = storageData.valueFloat[i];
		}
	}
//	//print test
//	memset(print_str, 0, sizeof(print_str));
//	sprintf(print_str,"%.2f %.2f %.2f %.2f %.2f %.2f",cof_vol[0],cof_vol[2],cof_vol[4],cof_vol[6],cof_vol[8],cof_vol[10]);
//	RS485_DE1 = 1U; //RS485 send mode
//	R_SCI12_AsyncTransmit((uint8_t*)print_str,strlen(print_str),false);
//	RS485_DE1 = 0U; //RS485 send mode
//	memset(print_str, 0, sizeof(print_str));
//	//end_print_test
	Buzzer(3,50);

	RS485_Master_Mode(Gen_registers,MAC_registers,GENTYPE);
	Load_Check();
	freq_ustbl_time_count=0;
	freq_start = MAC_registers[0x0F];
	Grid_check();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
	Gen_check();
	wait_time=0;

	//for BLD - OTA - Firmware update
	fw_up_return_t ret_fw_up;
	ret_fw_up = fw_up_open_flash();


	while(1)
	{
		if(g_sci12_rx_count>7)
		{
//			SCI12.SCR.BIT.RIE = 0U;
//			SCI12.SCR.BIT.RE = 0U;
			RS485_Slave_Mode(MAC_registers);
		}
		else
		{
			Generator_Switch();
			RS485_Master_Mode(Gen_registers,MAC_registers,GENTYPE);
			Load_Check();
			Grid_check();
			R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
			Gen_check();
			error_check = ErrorCheck();
			// RELAY 5 CONTROL
			TimerMode();
			//Check Door Open
			if(!DOOR_OPEN_DETECT) MAC_registers[0x4B] =0;
			// No Connection with MCC
			if(MAC_timeout == MAC_timeout_level)
			{
				MAC_registers[0x3E] =1;
			}
			// Check unstable frequency very 3 mins
			if(freq_ustbl_time_count > 178)
			{
				if(abs(MAC_registers[0x0F]-freq_start)>100) MAC_registers[0x48] = 1;
				else MAC_registers[0x48] = 0;
				freq_start = MAC_registers[0x0F]; // update start frequency
				freq_ustbl_time_count =0; // reset counter
			}

			// MODE REGISTER [0x41] PROCESSING
			// MANUAL USE GRID
			if(MAC_registers[0x41] == 1) // Mode reg - 65
			{
				Mode_UseGrid();
				g_is_all_relay_off=0;
			}
			// MANUAL USE GEN
			else if(MAC_registers[0x41] == 2)
			{
				Mode_UseGen();
				g_is_all_relay_off=0;
			}
			// MANUAL OFF ALL
			else if(MAC_registers[0x41] == 3)
			{
				if(g_is_all_relay_off==0) Process_OffAll();
				if(MAC_registers[0x53]>0)
				{
					MAC_registers[0x4C]=0;
					MAC_registers[0x70]=0;
				}
			}
			// AUTO MODE
			else if(MAC_registers[0x41] == 0)
			{
				Mode_Auto();
				g_is_all_relay_off=0;
			}

			// DO COMMAND REGISTER [0x42] PROCESSING
			//MANUAL STOP GEN - DONE
			if(MAC_registers[0x42] == 0x0100)
			{
				Process_StoptGen_Manual();
			}

			//MANUAL START GEN - 40sec + [0x59] seconds
			else if(MAC_registers[0x42] == 0x0101)
			{
				Process_StartGen_Manual();
			}
		}
	}
}
void uint16_to_uint8(uint16_t input, uint8_t* Hi_byte, uint8_t* Low_byte)
{
	*Hi_byte = input>>8;
	*Low_byte = input & 0xFF;
}
uint16_t CRC16_bytewise (uint8_t *nData, int wLength)
{
    static const uint16_t wCRCTable[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

    uint8_t nTemp;
    uint16_t wCRCWord = 0xFFFF;

    while (wLength--)
    {
        nTemp = *nData++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord  ^= wCRCTable[(nTemp & 0xFF)];
    }
    return wCRCWord;
} // End: CRC16
void RS485_Slave_Mode(uint16_t *MAC_registers)
{
	uint16_t CRC16;
	uint8_t CRC8[2];

	//SCAN BUFFER
	for(uint16_t i=0;i<g_sci12_rx_count;i++)
	{
//		//print test
//		memset(print_str, 0, sizeof(print_str));
//		sprintf(print_str,"%02X ",rx12_buff[i]);
//		RS485_DE1 = 1U; //RS485 send mode
//		R_SCI12_AsyncTransmit((uint8_t*)print_str,strlen(print_str),trans_enable);
//		RS485_DE1 = 0U; //RS485 send mode
//		memset(print_str, 0, sizeof(print_str));
//		//end_print_test

		// MCC WRITE 14 REGISTERS - 37 bytes COMMAND
		if((rx12_buff[i]== MAC_ID)&&(rx12_buff[i+1]== 0x10)&&(rx12_buff[i+5]== 0x0E))
		{
//			// print test
//			memset(print_str, 0, sizeof(print_str));
//			sprintf(print_str,"\r\n WRITE 14 REGISTERS");
//			RS485_DE1 = 1U; //RS485 send mode
//			R_SCI12_AsyncTransmit((uint8_t*)print_str,strlen(print_str),trans_enable);
//			RS485_DE1 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
//			//end_print_test
			CRC16 = CRC16_bytewise((uint8_t*)rx12_buff+i, 35);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx12_buff[i+35])&&(CRC8[1]== rx12_buff[i+36]))
			{
				for(uint8_t j=0; j<rx12_buff[i+5]; j++)
				{
					MAC_registers[rx12_buff[i+3]+j] = (uint16_t)(rx12_buff[i+(j*2)+7]<<8) + (uint16_t)(rx12_buff[i+(j*2)+8]);
				}
				memset(Rs485_MasterResponse, 0, sizeof(Rs485_MasterResponse));
				// Rs485 response to master
				Rs485_MasterResponse[0] = MAC_ID;
				Rs485_MasterResponse[1] = 0x10;
				Rs485_MasterResponse[2] = rx12_buff[i+2]; // Starting address Hi
				Rs485_MasterResponse[3] = rx12_buff[i+3]; // Starting address Lo
				Rs485_MasterResponse[4] = rx12_buff[i+4]; // Number of written registers Hi
				Rs485_MasterResponse[5] = rx12_buff[i+5]; // Number of written registers Lo
				CRC16 = CRC16_bytewise(Rs485_MasterResponse, 6);
				Rs485_MasterResponse[6] = CRC16 & 0xff;
				Rs485_MasterResponse[7] = CRC16 >> 8;

				RS485_DE1 = 1U; //RS485 send mode
				R_SCI12_AsyncTransmit(Rs485_MasterResponse,8,trans_enable);
				RS485_DE1 = 0U; //RS485 receive mode

				MAC_timeout=0;
			}
			else
			{
				//WRONG CRC
			}
		}
		// MCC READ 03 COMMAND FULL FUNCTION - 8 BYTES COMMAND
		else if((rx12_buff[i]== MAC_ID)&&(rx12_buff[i+1]== 0x03)) //Slave address & Read command
		{
//			//print test
//			sprintf(print_str,"\n\r   RS485 READ Add: 0x%02X%02X, 0x%02X%02X Registers!\n\r",rx12_buff[i+2],rx12_buff[i+3],rx12_buff[i+4],rx12_buff[i+5]);
//			RS485_DE1 = 1U;
//			R_SCI12_AsyncTransmit((uint8_t*)print_str,strlen(print_str),trans_enable);
//			RS485_DE1 = 0U;
//			memset(print_str, 0, sizeof(print_str));
//			//end_print_test

			CRC16 = CRC16_bytewise((uint8_t*)rx12_buff+i, 6);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx12_buff[i+6])&&(CRC8[1]== rx12_buff[i+7])) //CRC check
			{
				memset(Rs485_MasterResponse, 0, sizeof(Rs485_MasterResponse));
				// Rs485 response to master
				Rs485_MasterResponse[0] = MAC_ID;
				Rs485_MasterResponse[1] = 0x03;
				Rs485_MasterResponse[2] = rx12_buff[i+5]*2; // 100 address * 2 bytes = 200
				for(uint16_t j=0;j<rx12_buff[i+5];j++)
				{
					uint16_to_uint8(MAC_registers[j+rx12_buff[i+3]], (Rs485_MasterResponse +((j*2)+3)), (Rs485_MasterResponse +((j*2)+4)));
				}

				CRC16 = CRC16_bytewise(Rs485_MasterResponse, rx12_buff[i+5]*2+3);
				Rs485_MasterResponse[rx12_buff[i+5]*2+3] = CRC16 & 0xff;
				Rs485_MasterResponse[rx12_buff[i+5]*2+4] = CRC16 >> 8;
				RS485_DE1 = 1U; //RS485 send mode
				R_SCI12_AsyncTransmit(Rs485_MasterResponse,rx12_buff[i+5]*2+5,trans_enable);
				RS485_DE1 = 0U; //RS485 receive mode
				MAC_timeout=0;
			}
			else
			{
				//Wrong CRC
			}
		}
		// MCC WRITE SINGLE REGISTER - 8 BYTES COMMAND. REGISTER: 0x3E: DC_LOW, 0x5C: Data read mode, 0x41: mode, 0x42: do_command,
		else if((rx12_buff[i]== MAC_ID)&&(rx12_buff[i+1]== 0x06))//slave address & write single register command
		{
			CRC16 = CRC16_bytewise((uint8_t*)rx12_buff+i, 6);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx12_buff[i+6])&&(CRC8[1]== rx12_buff[i+7])) //CRC check
			{
				//print test
	//			sprintf(print_str,"\n\r   RS485 WRITE Add: 0x%02X%02X, 0x%02X%02X Registers!\n\r",rx12_buff[2],rx12_buff[3],rx12_buff[4],rx12_buff[5]);
	//			RS485_DE2 = 1U;
	//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
	//			RS485_DE2 = 0U;
	//			memset(print_str, 0, sizeof(print_str));

				if(rx12_buff[i+3]==0x41) previous_mode = MAC_registers[rx12_buff[i+3]];
				MAC_registers[rx12_buff[i+3]] = (uint16_t)(rx12_buff[i+4]<<8) + (uint16_t)(rx12_buff[i+5]);
				//enable transmit before response
				if(MAC_registers[0x6F]==0) trans_enable =0;

				memset(Rs485_MasterResponse, 0, sizeof(Rs485_MasterResponse));
				// Rs485 response to master
				Rs485_MasterResponse[0] = MAC_ID;
				Rs485_MasterResponse[1] = 0x06;
				Rs485_MasterResponse[2] = rx12_buff[i+2]; // modified register address
				Rs485_MasterResponse[3] = rx12_buff[i+3]; // modified register address
				uint16_to_uint8(MAC_registers[rx12_buff[i+3]], (Rs485_MasterResponse +4), (Rs485_MasterResponse +5));
				CRC16 = CRC16_bytewise(Rs485_MasterResponse, 6);
				Rs485_MasterResponse[6] = CRC16 & 0xff;
				Rs485_MasterResponse[7] = CRC16 >> 8;
				RS485_DE1 = 1U; //RS485 send mode
				R_SCI12_AsyncTransmit(Rs485_MasterResponse,8,trans_enable);
				RS485_DE1 = 0U; //RS485 receive mode

				MAC_timeout=0;
				// Update firmware
				if(MAC_registers[0x66]==1)
				{
					main_FW_update();
				}
				//disable transmit to MCC after response
				if(MAC_registers[0x6F]==1) trans_enable =1;
			}
			else
			{
				//WRONG CRC
			}

		}
		// MCC WRITE 2 REGISTER - 13 BYTES COMMAND. REGISTER: 0x5F: Hour, 0x60: MINUTE
		else if ((rx12_buff[i]== MAC_ID)&&(rx12_buff[i+1]== 0x10)&&(rx12_buff[i+5]== 0x02))
		{

			CRC16 = CRC16_bytewise((uint8_t*)rx12_buff+i, 11);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx12_buff[i+11])&&(CRC8[1]== rx12_buff[i+12])) //Check CRC
			{
				MAC_registers[0x5F] = (uint16_t)(rx12_buff[i+7]<<8) + (uint16_t)(rx12_buff[i+8]);
				MAC_registers[0x60] = (uint16_t)(rx12_buff[i+9]<<8) + (uint16_t)(rx12_buff[i+10]);

				// Rs485 response to master
				memset(Rs485_MasterResponse, 0, sizeof(Rs485_MasterResponse));
				Rs485_MasterResponse[0] = MAC_ID;
				Rs485_MasterResponse[1] = 0x10;
				Rs485_MasterResponse[2] = rx12_buff[i+2]; // Starting address Hi
				Rs485_MasterResponse[3] = rx12_buff[i+3]; // Starting address Lo
				Rs485_MasterResponse[4] = rx12_buff[i+4]; // Number of written registers Hi
				Rs485_MasterResponse[5] = rx12_buff[i+5]; // Number of written registers Lo
				CRC16 = CRC16_bytewise(Rs485_MasterResponse, 6);
				Rs485_MasterResponse[6] = CRC16 & 0xff;
				Rs485_MasterResponse[7] = CRC16 >> 8;
				RS485_DE1 = 1U; //RS485 send mode
				R_SCI12_AsyncTransmit(Rs485_MasterResponse,8,trans_enable);
				RS485_DE1 = 0U; //RS485 receive mode
				memset(rx12_buff,0,sizeof(rx12_buff));
				MAC_timeout=0;
			}
			else
			{
				//WRONG CRC
			}
		}
	}
	g_sci12_rx_count=0;
	memset(rx12_buff, 0, sizeof(rx12_buff));
	R_Config_SCI12_Serial_Receive((uint8_t*)&rx12_buff, sizeof(rx12_buff));
}


void Gen_Volt_Cal()
{
	if(Sample_done)
	{
		gen_volt1=0;gen_volt2=0;gen_volt3=0;
		for(uint16_t i=0;i<SAMPLES_NUM;i++)
		{
			gen_volt1 = gen_volt1+  pow((ADC_VGen1[i]*0.000806),2);
			gen_volt2 = gen_volt2+  pow((ADC_VGen2[i]*0.000806),2);
			gen_volt3 = gen_volt3+  pow((ADC_VGen3[i]*0.000806),2);
		}
		gen_volt1 = sqrt(gen_volt1/SAMPLES_NUM);
		if(gen_volt1> 3.1) gen_volt1=0;
		else
		{
			gen_volt1 = (3.3-gen_volt1)*(-1)*cof_vol[6];
			if(gen_volt1<70) gen_volt1 =0;
		}
		gen_volt2 = sqrt(gen_volt2/SAMPLES_NUM);
		if(gen_volt2> 3.1) gen_volt2=0;
		else
		{
			gen_volt2 = (3.3-gen_volt2)*(-1)*cof_vol[8];
			if(gen_volt2<70) gen_volt2 =0;
		}
		gen_volt3 = sqrt(gen_volt3/SAMPLES_NUM) ;
		if(gen_volt3> 3.1) gen_volt3=0;
		else
		{
			gen_volt3 = (3.3-gen_volt3) *(-1)*cof_vol[10];
			if(gen_volt3<70) gen_volt3 =0;
		}
		Sample_done=0;
	}
}
uint8_t Gen_check()
{
	uint8_t status;
//	RS485_Master_Mode(Gen_registers,MAC_registers,GENTYPE);
	float gen_freq1,gen_freq2,gen_freq3;

	if(state4==2)
	{
		gen_freq1 = (1/period4)*1000;
		state4=0; //kHz
//		//print test
//		sprintf(print_str,"\n\rgen_freq1: %.2f",gen_freq1);
////		sprintf(print_str,"\n\rPeriod1: %d",period4);
//		RS485_DE2 = 1U;
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//		RS485_DE2 = 0U;
//		memset(print_str, 0, sizeof(print_str));
//		//	end_print_test
	}
	else
	{
		gen_freq1=0;
		state4=0;
	}
	if(state1==2)
	{
		gen_freq2 = (1/period1)*1000;
		state1=0;
//		//print test
//		sprintf(print_str,"\n\rgen_freq2: %.2f",gen_freq2);
////		sprintf(print_str,"\n\rPeriod2: %d",period1);
//		RS485_DE2 = 1U;
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//		RS485_DE2 = 0U;
//		memset(print_str, 0, sizeof(print_str));
//		//	end_print_test
	}
	else
	{
		gen_freq2=0;
		state1=0;
	}

	if(state0==2)
	{
		gen_freq3 = (1/period0)*1000;
		state0=0;
//		//print test
//		sprintf(print_str,"\n\rgen_freq3: %.2f",gen_freq3);
////		sprintf(print_str,"\n\rPeriod3: %d",period0);
//		RS485_DE2 = 1U;
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//		RS485_DE2 = 0U;
//		memset(print_str, 0, sizeof(print_str));
//		//	end_print_test
	}
	else
	{
		gen_freq3=0;
		state0=0;
	}

//	if((gen_freq1<0.045)||(gen_freq2<0.045)||(gen_freq3<0.045)) //<45Hz
//	{
//		//Fail
//		status =0;
//	}
//	else status =1;

	Gen_Volt_Cal();
	if(((gen_volt3>150)&&(gen_volt3<300))||(((gen_volt2>150)&&(gen_volt2<300)))||(((gen_volt1>150)&&(gen_volt1<300))))
	{
		status =1;
		MAC_registers[0x39] =1;
	}
	else
	{
		status =0;
		MAC_registers[0x39] =0;
		MAC_registers[0x3C]=0;
	}
	MAC_registers[0x3B] = FB_GEN_COIL;
	if(FB_GEN_COIL==1)// GEN RUN WITH LOAD
	{
		// Gen Volt = Load Volt
		MAC_registers[0x03] = MAC_registers[0x06];
		MAC_registers[0x04] = MAC_registers[0x07];
		MAC_registers[0x05] = MAC_registers[0x08];

		// Gen Power = Load Power
		MAC_registers[0x20] = MAC_registers[0x23];
		MAC_registers[0x21] = MAC_registers[0x24];
		MAC_registers[0x22] = MAC_registers[0x25];

		//Gen Freq
		MAC_registers[0x0C] = MAC_registers[0x0F];
		MAC_registers[0x0D] = MAC_registers[0x10];
		MAC_registers[0x0E] = MAC_registers[0x11];

		if(status == 1) MAC_registers[0x3C] = 3;
	}
	else // GEN RUN WITHOUT LOAD
	{
		if(MAC_registers[0x5C]==0) // Read from Board
		{
			//Gen Volt
			MAC_registers[0x03] = gen_volt1*100;
			MAC_registers[0x04] = gen_volt2*100;
			MAC_registers[0x05] = gen_volt3*100;

			//Gen Power
			MAC_registers[0x20] = 0;
			MAC_registers[0x21] = 0;
			MAC_registers[0x22] = 0;

			//Gen Freq
			MAC_registers[0x0C] = (uint16_t)(gen_freq1*100);
			MAC_registers[0x0D] = (uint16_t)(gen_freq2*100);
			MAC_registers[0x0E] = (uint16_t)(gen_freq3*100);

			if(status == 1) MAC_registers[0x3C] = 2;
		}
	}
	//Voltage out of range warnning
	if((MAC_registers[0x03]>MAC_registers[0x51])&&(MAC_registers[0x03]<MAC_registers[0x57])) MAC_registers[0x40] &= ~0x01;
	else MAC_registers[0x40] |= 0x01;

	if((MAC_registers[0x04]>MAC_registers[0x51])&&(MAC_registers[0x04]<MAC_registers[0x57])) MAC_registers[0x40] &= ~0x02;
	else MAC_registers[0x40] |= 0x02;

	if((MAC_registers[0x05]>MAC_registers[0x51])&&(MAC_registers[0x05]<MAC_registers[0x57])) MAC_registers[0x40] &= ~0x04;
	else MAC_registers[0x40] |= 0x04;

	//print test
//	sprintf(print_str,"\n\rgen_volt1: %.2f gen_volt2: %.2f gen_volt3: %.2f status %d",gen_volt1,gen_volt2,gen_volt3,status);
//	RS485_DE2 = 1U;
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//	RS485_DE2 = 0U;
//	memset(print_str, 0, sizeof(print_str));
	//end_print_test

	return status;
}
void Grid_Volt_Cal()
{
	if(Sample_done)
	{
		grid_volt1=0;grid_volt2=0;grid_volt3=0;
		for(uint16_t i=0;i<SAMPLES_NUM;i++)
		{
			grid_volt1 = grid_volt1+  pow((ADC_VGrid1[i]*0.000806),2);
			grid_volt2 = grid_volt2+  pow((ADC_VGrid2[i]*0.000806),2);
			grid_volt3 = grid_volt3+  pow((ADC_VGrid3[i]*0.000806),2);
		}
		grid_volt1 = sqrt(grid_volt1/SAMPLES_NUM);
		if(grid_volt1> 3.1) grid_volt1=0;
		else
		{
			grid_volt1 = (3.3-grid_volt1)*(-1)*cof_vol[0];
			if(grid_volt1<70) grid_volt1 =0;
		}

		grid_volt2 = sqrt(grid_volt2/SAMPLES_NUM);
		if(grid_volt2> 3.1) grid_volt2=0;
		else
		{
			grid_volt2 = (3.3-grid_volt2)*(-1)*cof_vol[2];
			if(grid_volt2<70) grid_volt2 =0;
		}

		grid_volt3 = sqrt(grid_volt3/SAMPLES_NUM) ;
		if(grid_volt3> 3.1) grid_volt3=0;
		else
		{
			grid_volt3 = (3.3-grid_volt3)*(-1)*cof_vol[4];
			if(grid_volt3<70) grid_volt3 =0;
		}
		Sample_done=0;
	}
}
uint8_t Grid_check()
{
	uint8_t f_sts, v_sts, status;
// GRID VOLTAGE CHECK
	Grid_Volt_Cal();
	if(((grid_volt3<330)&&(grid_volt3>100))||((grid_volt2<330)&&(grid_volt2>100))||((grid_volt1<330)&&(grid_volt1>100))) v_sts =1;
	else v_sts =0;

	if(MAC_registers[0x7B]==0) // tram 3 pha co ATS
	{
		//Update Contactor Status
		MAC_registers[0x3A] = FB_GRID_COIL;
		if(FB_GRID_COIL==0)
		{
			MAC_registers[0x00] = (uint16_t)(grid_volt1*100);
			MAC_registers[0x01] = (uint16_t)(grid_volt2*100);
			MAC_registers[0x02] = (uint16_t)(grid_volt3*100);

			// power grid
			MAC_registers[0x1D] = 0;
			MAC_registers[0x1E] = 0;
			MAC_registers[0x1F] = 0;
		}
		else // Grid Contactor ON ->
		{
			//Grid Volt = Volt from ATM
			MAC_registers[0x00] = 	MAC_registers[0x06];
			MAC_registers[0x01] = 	MAC_registers[0x07];
			MAC_registers[0x02] = 	MAC_registers[0x08];

			// Power grid = power load
			MAC_registers[0x1D] = MAC_registers[0x23];
			MAC_registers[0x1E] = MAC_registers[0x24];
			MAC_registers[0x1F] = MAC_registers[0x25];
		}
	}
	else if(MAC_registers[0x7B]== 2) // tram 1 pha co ATS
	{
		//Update Contactor Status
		MAC_registers[0x3A] = FB_GRID_COIL;
		if(FB_GRID_COIL==0)
		{
			MAC_registers[0x00] = (uint16_t)(grid_volt1*100);
			// power grid
			MAC_registers[0x1D] = 0;
		}
		else // Grid Contactor ON ->
		{
			//Grid Volt = Volt from ATM
			MAC_registers[0x00] = 	MAC_registers[0x06];
			// Power grid = power load
			MAC_registers[0x1D] = MAC_registers[0x23];
		}
	}
	else if (MAC_registers[0x7B]==1) //tram 3 pha khong co ATS
	{
		//Grid Volt = Load Volt
		MAC_registers[0x00] = 	MAC_registers[0x06];
		MAC_registers[0x01] = 	MAC_registers[0x07];
		MAC_registers[0x02] = 	MAC_registers[0x08];

		// Power grid = power load
		MAC_registers[0x1D] = MAC_registers[0x23];
		MAC_registers[0x1E] = MAC_registers[0x24];
		MAC_registers[0x1F] = MAC_registers[0x25];
	}
	else if (MAC_registers[0x7B]==3) //tram 1 pha khong co ATS
	{
		//Grid Volt = Load Volt
		MAC_registers[0x00] = 	MAC_registers[0x06];
		// Power grid = power load
		MAC_registers[0x1D] = MAC_registers[0x23];
	}

	// Grid Volt Warnning
	if((MAC_registers[0x00]>MAC_registers[0x52])&&(MAC_registers[0x00]<MAC_registers[0x4F])) MAC_registers[0x3F] &= ~0x01;
	else MAC_registers[0x3F] |= 0x01;

	if((MAC_registers[0x01]>MAC_registers[0x52])&&(MAC_registers[0x01]<MAC_registers[0x4F])) MAC_registers[0x3F] &= ~0x02;
	else MAC_registers[0x3F] |= 0x02;

	if((MAC_registers[0x02]>MAC_registers[0x52])&&(MAC_registers[0x02]<MAC_registers[0x4F])) MAC_registers[0x3F] &= ~0x04;
	else MAC_registers[0x3F] |= 0x04;

	//print test
//	memset(print_str, 0, sizeof(print_str));
//	sprintf(print_str,"\n\rgridV1: %d gridV2: %d gridV3: %d\n\r",MAC_registers[0x00],MAC_registers[0x01],MAC_registers[0x02]);
//	RS485_DE2 = 1U;
//	R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//	RS485_DE2 = 0U;
//	memset(print_str, 0, sizeof(print_str));
	//end_print_test

	status = v_sts;
	//Update Grid Volt Status
	MAC_registers[0x38]=status;

// GRID FREQUENCY CHECK
	if((l0_edge_detected==3)||(l3a_edge_detected==3)||(l3c_edge_detected==3))
	{

		if(l0_edge_detected!=3) grid_freq3=0;
		if(l3a_edge_detected!=3) grid_freq1=0;
		if(l3c_edge_detected!=3) grid_freq2=0;

		if(MAC_registers[0x7B]==0 ) // tram 3 pha co ATS
		{
			if(FB_GRID_COIL==0)
			{
				MAC_registers[0x09] = (uint16_t)(grid_freq1*100);
				MAC_registers[0x0A] = (uint16_t)(grid_freq2*100);
				MAC_registers[0x0B] = (uint16_t)(grid_freq3*100);
			}
			else // Grid Contactor ON -> Grid Freq = Load Freq
			{
				MAC_registers[0x09] = MAC_registers[0x0F];
				MAC_registers[0x0A] = MAC_registers[0x10];
				MAC_registers[0x0B] = MAC_registers[0x11];
			}
		}
		else if (MAC_registers[0x7B]==2) // tram 1 pha co ATS
		{
			if(FB_GRID_COIL==0)
			{
				MAC_registers[0x09] = (uint16_t)(grid_freq1*100);

			}
			else // Grid Contactor ON -> Grid Freq = Load Freq
			{
				MAC_registers[0x09] = MAC_registers[0x0F];

			}
		}
		else if (MAC_registers[0x7B]==1) // tram 3 pha khong ATS
		{
			//grid freq = load freq
			MAC_registers[0x09] = MAC_registers[0x0F];
			MAC_registers[0x0A] = MAC_registers[0x10];
			MAC_registers[0x0B] = MAC_registers[0x11];
		}
		else if (MAC_registers[0x7B]==3) // tram 1 pha khong ATS
		{
			//grid freq = load freq
			MAC_registers[0x09] = MAC_registers[0x0F];
		}

		if(((grid_freq1>45)&&(grid_freq1<57))||((grid_freq2>45)&&(grid_freq2<57))||((grid_freq3>45)&&(grid_freq3<57)))
		{
			f_sts =1;
//			MAC_registers[0x38] = 1;
		}
		else
		{
			f_sts =0;
//			MAC_registers[0x38] = 0;
		}

		l0_edge_detected=0;
		l3a_edge_detected=0;
		l3c_edge_detected=0;

		//print test
//			memset(print_str, 0, sizeof(print_str));
//			sprintf(print_str,"\n\rgridF1: %.2f gridF2: %.2f gridF3: %.2f",grid_freq3,grid_freq3,grid_freq3);
//			RS485_DE2 = 1U;
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			RS485_DE2 = 0U;
//			memset(print_str, 0, sizeof(print_str));
		//end_print_test
	}
	else
	{
		if(MAC_registers[0x7B]==0) // tram 3 pha co ATS
		{
			if(FB_GRID_COIL==0)
			{
				if(!v_sts)
				{
					MAC_registers[0x09] = 0;
					MAC_registers[0x0A] = 0;
					MAC_registers[0x0B] = 0;
				}
			}
			else // Grid Contactor ON -> Grid Freq = Freq from ATM
			{
				MAC_registers[0x09] = MAC_registers[0x0F];
				MAC_registers[0x0A] = MAC_registers[0x10];
				MAC_registers[0x0B] = MAC_registers[0x11];
			}

		}
		else if (MAC_registers[0x7B]==1)// tram 3 pha khong ATS
		{
			//Grid Freq = Freq from ATM
			MAC_registers[0x09] = MAC_registers[0x0F];
			MAC_registers[0x0A] = MAC_registers[0x10];
			MAC_registers[0x0B] = MAC_registers[0x11];
		}
		else if(MAC_registers[0x7B]==2)// 1 pha co ATS
		{
			if(FB_GRID_COIL==0)
			{
				if(!v_sts)
				{
					MAC_registers[0x09] = 0;
				}
			}
			else // Grid Contactor ON -> Grid Freq = Freq from ATM
			{
				MAC_registers[0x09] = MAC_registers[0x0F];
			}
		}
		else if(MAC_registers[0x7B]==3)// 1 pha ko ATS
		{
			//Grid Freq = Freq from ATM
			MAC_registers[0x09] = MAC_registers[0x0F];
		}
	}

	//print test
//		memset(print_str, 0, sizeof(print_str));
//		sprintf(print_str,"\n\rgridF1: %d gridF2: %d gridF3: %d",MAC_registers[0x09],MAC_registers[0x0A],MAC_registers[0x0B]);
//		RS485_DE2 = 1U;
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//		RS485_DE2 = 0U;
//		memset(print_str, 0, sizeof(print_str));
	//end_print_test

	return status;
}
void PC1X_Start()
{
	ATS_CTRL_GEN_START_1 = 1;
	MAC_registers[0x45] = 1;
	MAC_registers[0x3C] = 1;
}
void PC1X_Stop()
{
	ATS_CTRL_GEN_START_1 = 0;
	MAC_registers[0x45] = 0;
//	MAC_registers[0x3C] = 0;

	if(Gen_check()) waittime(MAC_registers[0x58]); //In case Gen off already -> dont have to wait
	if(Gen_check())
	{
		MAC_registers[0x4C] = 2; // GEN DOESNT STOP AFTER COOLING
	}
	else
	{
		MAC_registers[0x4C] = 0; // No error
		GenStart =0;
	}
}
void DST4400_Start()
{
	ATS_CTRL_GEN_START_1 = 1;
	MAC_registers[0x45] = 1;
	MAC_registers[0x3C] = 1;
}
void DST4400_Stop()
{
	ATS_CTRL_GEN_START_1 = 0;
	MAC_registers[0x45] = 0;
//	MAC_registers[0x3C] = 0;

	if(Gen_check()) waittime(MAC_registers[0x58]); //In case Gen off already -> dont have to wait
	if(Gen_check())
	{
		MAC_registers[0x4C] = 2; // GEN DOESNT STOP AFTER COOLING
	}
	else
	{
		MAC_registers[0x4C] = 0; // No error
		GenStart =0;
	}
}
void D300_Start()
{
	ATS_CTRL_GEN_START_1 = 1;
	MAC_registers[0x45] = 1;
	MAC_registers[0x3C] = 1;
}
void D300_Stop()
{
	ATS_CTRL_GEN_START_1 = 0;
	MAC_registers[0x45] = 0;
//	MAC_registers[0x3C] = 0;

	if(Gen_check()) waittime(MAC_registers[0x58]); //In case Gen off already -> dont have to wait
	if(Gen_check())
	{
		MAC_registers[0x4C] = 2; // GEN DOESNT STOP AFTER COOLING
	}
	else
	{
		MAC_registers[0x4C] = 0; // No error
		GenStart =0;
	}
}
void DSE7320_Start()
{
	ATS_CTRL_GEN_START_1 = 1;
	MAC_registers[0x45] = 1;
	MAC_registers[0x3C] = 1;
}
void DSE7320_Stop()
{
	ATS_CTRL_GEN_START_1 = 0;
	MAC_registers[0x45] = 0;
//	MAC_registers[0x3C] = 0;

	if(Gen_check()) waittime(MAC_registers[0x58]);
	if(Gen_check())
	{
		MAC_registers[0x4C] = 2; // GEN DOESNT STOP AFTER COOLING
	}
	else
	{
		MAC_registers[0x4C] = 0; // No error
		GenStart =0;
	}
}
void GC315_Start()
{
	ATS_CTRL_GEN_START_1 = 1;
	MAC_registers[0x45] = 1;
	MAC_registers[0x3C] = 1;
}
void GC315_Stop()
{
	ATS_CTRL_GEN_START_1 = 0;
	MAC_registers[0x45] = 0;
//	MAC_registers[0x3C] = 0;

	if(Gen_check()) waittime(MAC_registers[0x58]); //In case Gen off already -> dont have to wait
	if(Gen_check())
	{
		MAC_registers[0x4C] = 2; // GEN DOESNT STOP AFTER COOLING
	}
	else
	{
		MAC_registers[0x4C] = 0; // No error
		GenStart =0;
	}
}
void DKG307_Start()
{
	ATS_CTRL_GEN_START_1 = 1;
	MAC_registers[0x45] = 1;
	MAC_registers[0x3C] = 1;
}
void DKG307_Stop()
{
	ATS_CTRL_GEN_START_1 = 0;
	MAC_registers[0x45] = 0;
//	MAC_registers[0x3C] = 0;

	if(Gen_check()) waittime(MAC_registers[0x58]); //In case Gen off already -> dont have to wait
	if(Gen_check())
	{
		MAC_registers[0x4C] = 2; // GEN DOESNT STOP AFTER COOLING
	}
	else
	{
		MAC_registers[0x4C] = 0; // No error
		GenStart =0;
	}
}
void Emko_Start()
{
	ATS_CTRL_GEN_START_1 = 1;
	MAC_registers[0x45] = 1;
	MAC_registers[0x3C] = 1;
}
void Emko_Stop()
{
	ATS_CTRL_GEN_START_1 = 0;
	MAC_registers[0x45] = 0;
	MAC_registers[0x3C] = 0;

	if(Gen_check()) waittime(MAC_registers[0x58]); //In case Gen off already -> dont have to wait
	if(Gen_check())
	{
		MAC_registers[0x4C] = 2; // GEN DOESNT STOP AFTER COOLING
	}
	else
	{
		MAC_registers[0x4C] = 0; // No error
		GenStart =0;
	}
}
void Smartgen_Stop(uint8_t slaveID, uint16_t Coil_address, uint16_t Value)
{
	uint8_t request[8];
	memset(request, 0, sizeof(request));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x05;
	//Coil address
	uint16_to_uint8(Coil_address, request+2, request+3);
	uint16_to_uint8(Value, request+4, request+5);
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;
	RS485_DE2 = 1U; //RS485 send mode
	R_SCI5_AsyncTransmit(request,8);
	RS485_DE2 = 0U; //RS485 receive mode

	R_Config_SCI5_Serial_Receive((uint8_t*)&rx5_buff, 8);
	R_BSP_SoftwareDelay((100), BSP_DELAY_MILLISECS);
	if(g_sci12_rx_count>7) 	RS485_Slave_Mode(MAC_registers);
	if (SCI5_rxdone ==1)
	{
		//print test
//		uint8_t i=0;
//		for(i=0;i<8;i++)
//		{
//			sprintf(print_str,"%02X ",rx5_buff[i]);
//		RS485_DE2 = 1U;
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,3);
//		RS485_DE2 = 0U;
//			memset(print_str, 0, sizeof(print_str));
//		}
		//end_print_test

		if((rx5_buff[0]== slaveID)&&(rx5_buff[1]==0x05))
		{
			CRC16 = CRC16_bytewise((uint8_t*)rx5_buff, 6);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx5_buff[6])&&(CRC8[1]== rx5_buff[7])) //CRC check
			{
				// CONTROL SUCCESSFULLY
				//print test
//				sprintf(print_str,"\n\r  SUCCESSFUL! ");
//				RS485_DE2 = 1U;
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U;
//				memset(print_str, 0, sizeof(print_str));
				//end_print_test

				// Update Gen error register
				MAC_registers[0x4C] = 0; // No error
				MAC_registers[0x3C] = 0; // GEN STOP
				MAC_registers[0x39] = 0; // Gen no voltage
				GenStart =0;
			}
		}
		else
		{
			//wrong CRC
		}
		SCI5_rxdone =0;
	}
	else
	{
		waittime(MAC_registers[0x58]);
		if(Gen_check())
		{
			MAC_registers[0x4C] = 2; // GEN DOESNT STOP AFTER COOLING
//			MAC_registers[0x3C] = 2; // GEN RUN WITHOUT LOAD
			MAC_registers[0x39] = 1; // Gen has voltage
		}
		else
		{
			MAC_registers[0x4C] = 0; // No error
			MAC_registers[0x3C] = 0; // GEN STOP
			MAC_registers[0x39] = 0; // Gen no voltage
			GenStart =0;
		}
	}
}
void Smartgen_Manual(uint8_t slaveID, uint16_t Coil_address, uint16_t Value)
{
	uint8_t request[8];
	memset(request, 0, sizeof(request));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x05;
	//Coil address
	uint16_to_uint8(Coil_address, request+2, request+3);
	uint16_to_uint8(Value, request+4, request+5);
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;


	RS485_DE2 = 1U; //RS485 send mode
	R_SCI5_AsyncTransmit(request,8);
	RS485_DE2 = 0U; //RS485 receive mode

	R_Config_SCI5_Serial_Receive((uint8_t*)&rx5_buff, 8);
	if(g_sci12_rx_count>7) 	RS485_Slave_Mode(MAC_registers);
	R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);

	if (SCI5_rxdone ==1)
	{
		//print test
//		uint8_t i=0;
//		for(i=0;i<8;i++)
//		{
//			sprintf(print_str,"%02X ",rx5_buff[i]);
//			RS485_DE2 = 1U;
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,3);
//			RS485_DE2 = 0U;
//			memset(print_str, 0, sizeof(print_str));
//		}
		//end_print_test

		if((rx5_buff[0]== slaveID)&&(rx5_buff[1]==0x05))
		{
			CRC16 = CRC16_bytewise((uint8_t*)rx5_buff, 6);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx5_buff[6])&&(CRC8[1]== rx5_buff[7])) //CRC check
			{
				// CONTROL SUCCESSFULLY
				//print test
//				sprintf(print_str,"\n\r  SUCCESSFUL! ");
//				RS485_DE2 = 1U;
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U;
//				memset(print_str, 0, sizeof(print_str));
				//end_print_test

			}
		}
		else
		{
			//wrong CRC
		}
		SCI5_rxdone =0;

	}
	else
	{
		// NO response
	}

}
void Smartgen_Start(uint8_t slaveID, uint16_t Coil_address, uint16_t Value)
{
	uint8_t request[8];
	memset(request, 0, sizeof(request));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x05;
	//Coil address
	uint16_to_uint8(Coil_address, request+2, request+3);
	uint16_to_uint8(Value, request+4, request+5);
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;


	RS485_DE2 = 1U; //RS485 send mode
	R_SCI5_AsyncTransmit(request,8);
	RS485_DE2 = 0U; //RS485 receive mode

	R_Config_SCI5_Serial_Receive((uint8_t*)&rx5_buff, 8);
	if(g_sci12_rx_count>7) 	RS485_Slave_Mode(MAC_registers);
	R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);

	if (SCI5_rxdone ==1)
	{
		//print test
//		uint8_t i=0;
//		for(i=0;i<8;i++)
//		{
//			sprintf(print_str,"%02X ",rx5_buff[i]);
//			RS485_DE2 = 1U;
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,3);
//			RS485_DE2 = 0U;
//			memset(print_str, 0, sizeof(print_str));
//		}
		//end_print_test

		if((rx5_buff[0]== slaveID)&&(rx5_buff[1]==0x05))
		{
			CRC16 = CRC16_bytewise((uint8_t*)rx5_buff, 6);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			if ((CRC8[0]== rx5_buff[6])&&(CRC8[1]== rx5_buff[7])) //CRC check
			{
				// CONTROL SUCCESSFULLY
				//print test
//				sprintf(print_str,"\n\r  SUCCESSFUL! ");
//				RS485_DE2 = 1U;
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U;
//				memset(print_str, 0, sizeof(print_str));
				//end_print_test

				// Update Gen error register

				MAC_registers[0x4C] = 0;
//				MAC_registers[0x3C] = 2; // GEN RUN NO LOAD
//				MAC_registers[0x39] = 1;
				GenStart =1;

			}
		}
		else
		{
			//wrong CRC
		}
		SCI5_rxdone =0;
	}
	else
	{
		// No response
	}
}
void RS485_M_Cmd01_and_Receive(uint8_t slaveID, uint32_t StartAdd, uint16_t NoR, uint16_t * Value)
{
	uint8_t request[8];
	memset(request, 0, sizeof(request));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x01;
	//address register
	uint16_to_uint8(StartAdd, request+2, request+3);
	uint16_to_uint8(NoR, request+4, request+5);
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;
	RS485_DE2 = 1U; //RS485 send mode
	R_SCI5_AsyncTransmit(request,8);
	RS485_DE2 = 0U; //RS485 receive mode


	R_Config_SCI5_Serial_Receive((uint8_t*)&rx5_buff,6);
	while((!SCI5_rxdone) && (!R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS)));
	if (SCI5_rxdone ==1)
	{
		//print test
//		uint8_t i=0;
//		for(i=0;i<(NoR*2+5);i++)
//		{
//			sprintf(print_str,"%02X ",rx5_buff[i]);
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,3);
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
//		}
		//end_print_test

		if((rx5_buff[0]== slaveID)&&(rx5_buff[1]==0x03))
		{
			CRC16 = CRC16_bytewise((uint8_t*) rx5_buff, 4);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			//print test
//			sprintf(print_str,"\r\n CRC: %02X %02X  MSG: %02X %02X \r\n",CRC8[0],CRC8[1],rx5_buff[(NoR*2+3)],rx5_buff[(NoR*2+4)]);
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
			//end_print_test

			if ((CRC8[0]== rx5_buff[4])&&(CRC8[1]== rx5_buff[5])) //CRC check
			{


					Value[113] = (uint16_t)rx5_buff[3];


				//print test
//				sprintf(print_str,"\n\r  Value: %d \n\r",Value[22]);
//				RS485_DE2 = 1U; //RS485 send mode
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U; //RS485 send mode
//				memset(print_str, 0, sizeof(print_str));
				//end_print_test
			}
		}
		else
		{
			//wrong CRC
			//print test
//			sprintf(print_str,"\n\rWRONG CRC");
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
			//end_print_test
		}
		SCI5_rxdone =0;
	}
	else// timeout
	{

	}
}
void RS485_M_Cmd03_and_Receive(uint8_t slaveID, uint32_t StartAdd, uint16_t NoR, uint16_t * Value) //Read value stores in Value
{
	uint8_t request[8];
	memset(request, 0, sizeof(request));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x03;
	//address register
	uint16_to_uint8(StartAdd, request+2, request+3);
	uint16_to_uint8(NoR, request+4, request+5);
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;
	RS485_DE2 = 1U; //RS485 send mode
	R_SCI5_AsyncTransmit(request,8);
	RS485_DE2 = 0U; //RS485 receive mode


	R_Config_SCI5_Serial_Receive((uint8_t*)&rx5_buff, (NoR*2+5));
	while((!SCI5_rxdone) && (!R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS)));
	if (SCI5_rxdone ==1)
	{
		//print test
//		uint8_t i=0;
//		for(i=0;i<(NoR*2+5);i++)
//		{
//			sprintf(print_str,"%02X ",rx5_buff[i]);
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,3);
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
//		}
		//end_print_test

		if((rx5_buff[0]== slaveID)&&(rx5_buff[1]==0x03))
		{
			CRC16 = CRC16_bytewise((uint8_t*) rx5_buff, NoR*2+3);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			//print test
//			sprintf(print_str,"\r\n CRC: %02X %02X  MSG: %02X %02X \r\n",CRC8[0],CRC8[1],rx5_buff[(NoR*2+3)],rx5_buff[(NoR*2+4)]);
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
			//end_print_test

			if ((CRC8[0]== rx5_buff[(NoR*2+3)])&&(CRC8[1]== rx5_buff[(NoR*2+4)])) //CRC check
			{
				for(uint8_t i=0;i<NoR;i++)
				{
					Value[i] = (uint16_t)(rx5_buff[(i*2)+3]<<8) + (uint16_t)(rx5_buff[(i*2)+4]);
				}

				//print test
//				sprintf(print_str,"\n\r  Value: %d \n\r",Value[22]);
//				RS485_DE2 = 1U; //RS485 send mode
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U; //RS485 send mode
//				memset(print_str, 0, sizeof(print_str));
				//end_print_test
			}
		}
		else
		{
			//wrong CRC
			//print test
//			sprintf(print_str,"\n\rWRONG CRC");
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
			//end_print_test
		}
		SCI5_rxdone =0;
	}
	else// timeout
	{

	}
}
void RS485_M_Cmd04_and_Receive(uint8_t slaveID, uint32_t StartAdd, uint16_t NoR, uint16_t * Value) //Read value stores in Value
{
	uint8_t request[8];
	memset(request, 0, sizeof(request));
	uint16_t CRC16;
	uint8_t CRC8[2];

	*request = slaveID;
	*(request+1) = 0x04;
	//address register
	uint16_to_uint8(StartAdd, request+2, request+3);
	uint16_to_uint8(NoR, request+4, request+5);
	CRC16 = CRC16_bytewise(request, 6);
	*(request+6) = CRC16 & 0xff;
	*(request+7) = CRC16 >> 8;
	RS485_DE2 = 1U; //RS485 send mode
	R_SCI5_AsyncTransmit(request,8);
	RS485_DE2 = 0U; //RS485 receive mode


	R_Config_SCI5_Serial_Receive((uint8_t*)&rx5_buff, (NoR*2+5));
	while((!SCI5_rxdone) && (!R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS)));
	if (SCI5_rxdone ==1)
	{
		//print test
//		uint8_t i=0;
//		for(i=0;i<(NoR*2+5);i++)
//		{
//			sprintf(print_str,"%02X ",rx5_buff[i]);
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,3);
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
//		}
		//end_print_test

		if((rx5_buff[0]== slaveID)&&(rx5_buff[1]==0x04))
		{
			CRC16 = CRC16_bytewise((uint8_t*) rx5_buff, NoR*2+3);
			CRC8[0] = CRC16 & 0xff;
			CRC8[1] = CRC16 >> 8;
			//print test
//			sprintf(print_str,"\r\n CRC: %02X %02X  MSG: %02X %02X \r\n",CRC8[0],CRC8[1],rx5_buff[(NoR*2+3)],rx5_buff[(NoR*2+4)]);
//			RS485_DE2 = 1U; //RS485 send mode
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			RS485_DE2 = 0U; //RS485 send mode
//			memset(print_str, 0, sizeof(print_str));
			//end_print_test

			if ((CRC8[0]== rx5_buff[(NoR*2+3)])&&(CRC8[1]== rx5_buff[(NoR*2+4)])) //CRC check
			{
				for(uint8_t i=0;i<NoR;i++)
				{
					Value[i] = (uint16_t)(rx5_buff[(i*2)+3]<<8) + (uint16_t)(rx5_buff[(i*2)+4]);
				}
				GenIsConnected =1;
				//print test
//				sprintf(print_str,"\n\r  Value: %d \n\r",Value[22]);
//				RS485_DE2 = 1U; //RS485 send mode
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U; //RS485 send mode
//				memset(print_str, 0, sizeof(print_str));
				//end_print_test
			}
			else
			{
				//wrong CRC
			}
		}
		SCI5_rxdone =0;
	}
	else// timeout
	{
		GenIsConnected =0;
	}
}
void Import_PC1X_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers)
{
	// Volt
	MAC_registers[0x03] = Gen_reg[18]*100;
	MAC_registers[0x04] = Gen_reg[19]*100;
	MAC_registers[0x05] = Gen_reg[20]*100;
	//freq
	MAC_registers[0x0C] = Gen_reg[44]*1000;
	MAC_registers[0x0D] = Gen_reg[44]*1000;
	MAC_registers[0x0E] = Gen_reg[44]*1000;
	//Current
	MAC_registers[0x15] = Gen_reg[26]*1000;
	MAC_registers[0x16] = Gen_reg[27]*1000;
	MAC_registers[0x17] = Gen_reg[28]*1000;
//	//Power
//	MAC_registers[0x20] = Gen_reg[31];
//	MAC_registers[0x21] = Gen_reg[41];
//	MAC_registers[0x22] = Gen_reg[51];
//	MAC_registers[0x2A] = Gen_reg[35];
//	MAC_registers[0x2B] = Gen_reg[45];
//	MAC_registers[0x2C] = Gen_reg[55];
//	//power factor
//	MAC_registers[0x32] = Gen_reg[38];
//	MAC_registers[0x33] = Gen_reg[48];
//	MAC_registers[0x34] = Gen_reg[58];
//	//Fuel level
//	MAC_registers[0x6E] = Gen_reg[79];
}
void Import_DST4400_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers)
{
	// Volt
	MAC_registers[0x03] = Gen_reg[7]*100;
	MAC_registers[0x04] = Gen_reg[9]*100;
	MAC_registers[0x05] = Gen_reg[11]*100;
	//freq
	MAC_registers[0x0C] = Gen_reg[29]*100;
	MAC_registers[0x0D] = Gen_reg[29]*100;
	MAC_registers[0x0E] = Gen_reg[29]*100;
	//Current
	MAC_registers[0x15] = Gen_reg[21]*100;
	MAC_registers[0x16] = Gen_reg[23]*100;
	MAC_registers[0x17] = Gen_reg[25]*100;
	//Power
	MAC_registers[0x20] = Gen_reg[31];
	MAC_registers[0x21] = Gen_reg[41];
	MAC_registers[0x22] = Gen_reg[51];
	MAC_registers[0x2A] = Gen_reg[35];
	MAC_registers[0x2B] = Gen_reg[45];
	MAC_registers[0x2C] = Gen_reg[55];
	//power factor
	MAC_registers[0x32] = Gen_reg[38];
	MAC_registers[0x33] = Gen_reg[48];
	MAC_registers[0x34] = Gen_reg[58];
	//Fuel level
	MAC_registers[0x6E] = Gen_reg[79];
	if((Gen_reg[110]& 0x100)||(Gen_reg[110]& 0x200)) MAC_registers[0x4A] = 1;
}
void Import_D300_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers)
{
	// Volt
	MAC_registers[0x03] = Gen_reg[46]*10;
	MAC_registers[0x04] = Gen_reg[48]*10;
	MAC_registers[0x05] = Gen_reg[50]*10;
	//freq
	MAC_registers[0x0C] = Gen_reg[139];
	MAC_registers[0x0D] = Gen_reg[139];
	MAC_registers[0x0E] = Gen_reg[139];
	//Current
	MAC_registers[0x15] = Gen_reg[70]*10;
	MAC_registers[0x16] = Gen_reg[72]*10;
	MAC_registers[0x17] = Gen_reg[74]*10;
	//Power
	MAC_registers[0x20] = Gen_reg[86]/10;
	MAC_registers[0x21] = Gen_reg[88]/10;
	MAC_registers[0x22] = Gen_reg[90]/10;
	MAC_registers[0x2A] = Gen_reg[118]/10;
	MAC_registers[0x2B] = Gen_reg[120]/10;
	MAC_registers[0x2C] = Gen_reg[122]/10;
	//power factor
	MAC_registers[0x32] = Gen_reg[131]/10;
	MAC_registers[0x33] = Gen_reg[132]/10;
	MAC_registers[0x34] = Gen_reg[133]/10;
	//Fuel level
	MAC_registers[0x6E] = Gen_reg[163];
}
void Import_DSE7320_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers)
{
	// Volt
	MAC_registers[0x03] = Gen_reg[33]*1000;
	MAC_registers[0x04] = Gen_reg[35]*1000;
	MAC_registers[0x05] = Gen_reg[37]*1000;
	//freq
	MAC_registers[0x0C] = Gen_reg[31]*1000;
	MAC_registers[0x0D] = Gen_reg[31]*1000;
	MAC_registers[0x0E] = Gen_reg[31]*1000;
	//Current
	MAC_registers[0x15] = Gen_reg[45]*1000;
	MAC_registers[0x16] = Gen_reg[47]*1000;
	MAC_registers[0x17] = Gen_reg[49]*1000;
	//Power
	MAC_registers[0x20] = Gen_reg[53];
	MAC_registers[0x21] = Gen_reg[55];
	MAC_registers[0x22] = Gen_reg[57];
	//oil pressure
	MAC_registers[0x64] = Gen_reg[24];
	//Fuel level
	MAC_registers[0x6E] = Gen_reg[27];
}
void Import_GC315_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers)
{
	// Volt
	MAC_registers[0x03] = Gen_reg[3]*100;
	MAC_registers[0x04] = Gen_reg[4]*100;
	MAC_registers[0x05] = Gen_reg[5]*100;
	//freq
	MAC_registers[0x0C] = Gen_reg[13]*10;
	MAC_registers[0x0D] = Gen_reg[13]*10;
	MAC_registers[0x0E] = Gen_reg[13]*10;
	//Current
	MAC_registers[0x15] = Gen_reg[6]*100;
	MAC_registers[0x16] = Gen_reg[7]*100;
	MAC_registers[0x17] = Gen_reg[8]*100;
	//Power
	uint32_t tmp = ((uint32_t)Gen_reg[16]<<16 + (uint32_t)Gen_reg[17]);
	MAC_registers[0x20] = (uint16_t)(tmp/256);
	//power factor
	MAC_registers[0x2F] = Gen_reg[18];
	// oil pressure
	MAC_registers[0x64] = Gen_reg[0x2B];
}
void Import_DKG307_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers)
{
	// Volt
	MAC_registers[0x03] = Gen_reg[3]*100;
	MAC_registers[0x04] = Gen_reg[4]*100;
	MAC_registers[0x05] = Gen_reg[5]*100;
	//freq
	MAC_registers[0x0C] = Gen_reg[0x13]*10;
	MAC_registers[0x0D] = Gen_reg[0x13]*10;
	MAC_registers[0x0E] = Gen_reg[0x13]*10;
	//Current
	MAC_registers[0x15] = Gen_reg[6]*100;
	MAC_registers[0x16] = Gen_reg[7]*100;
	MAC_registers[0x17] = Gen_reg[8]*100;
	//Power
	uint32_t tmp = ((uint32_t)Gen_reg[0x16]<<16 + (uint32_t)Gen_reg[0x17]);
	MAC_registers[0x20] = (uint16_t)(tmp/256);
	//power factor
	MAC_registers[0x2F] = abs(Gen_reg[0x18]);
	// % fuel level
	MAC_registers[0x6E] = Gen_reg[0x2D];
}
void Import_Emko_Reg(uint16_t *Gen_reg, uint16_t *MAC_registers)
{
	//volt
	MAC_registers[0x03] = Gen_reg[0x26]*100;
	MAC_registers[0x04] = Gen_reg[0x27]*100;
	MAC_registers[0x05] = Gen_reg[0x28]*100;

	//freq
	MAC_registers[0x0C] = Gen_reg[0]*10; //freq
	MAC_registers[0x0D] = Gen_reg[0]*10;
	MAC_registers[0x0E] = Gen_reg[0]*10;

	//current
	MAC_registers[0x15] = Gen_reg[0x16];
	MAC_registers[0x16] = Gen_reg[0x16];
	MAC_registers[0x17] = Gen_reg[0x16];

	//power
	MAC_registers[0x20] = Gen_reg[0x63];
	MAC_registers[0x29] = Gen_reg[0x63];
	MAC_registers[0x32] = Gen_reg[0x63];
	//oil pressure
	MAC_registers[0x64] = Gen_reg[0x07];

}
void Import_Smartgen_Reg(uint16_t *Smartgen_reg, uint16_t *MAC_registers)
{
	//volt
	MAC_registers[0x03] = Smartgen_reg[7]*100;
	MAC_registers[0x04] = Smartgen_reg[8]*100;
	MAC_registers[0x05] = Smartgen_reg[9]*100;
	//freq
	MAC_registers[0x0C] = Smartgen_reg[13]*10; //freq
	//current
	MAC_registers[0x15] = Smartgen_reg[14];
	MAC_registers[0x16] = Smartgen_reg[15];
	MAC_registers[0x17] = Smartgen_reg[16];
	//power
	MAC_registers[0x20] = Smartgen_reg[26];
	MAC_registers[0x29] = Smartgen_reg[28];
	MAC_registers[0x32] = Smartgen_reg[29];
	//low fuel alarm
	MAC_registers[0x4A] = Smartgen_reg[113];
	//oil pressure
	MAC_registers[0x64] = Smartgen_reg[19];

}
void RS485_Master_Mode(uint16_t *Slave_registers,uint16_t *MAC_registers, generator_type gen_type)
{
	switch(gen_type)
	{
	case SMARTGEN:
		RS485_M_Cmd03_and_Receive(Smartgen_ID, 0,35, Slave_registers);
		RS485_M_Cmd01_and_Receive(Smartgen_ID, 113,1, Slave_registers);
		Import_Smartgen_Reg(Slave_registers,MAC_registers);
		break;
	case EMKO:
		RS485_M_Cmd04_and_Receive(Emko_ID, 0,100, Slave_registers);
		Import_Emko_Reg(Slave_registers,MAC_registers);
		break;
	case DKG307:
		RS485_M_Cmd03_and_Receive(DKG307_ID, 0,16, Slave_registers); //0-0x0F: 16 registers
		RS485_M_Cmd03_and_Receive(DKG307_ID, 16,4, Slave_registers+16); //0x10-0x13: 4 registers
		RS485_M_Cmd03_and_Receive(DKG307_ID, 22,3, Slave_registers+22);
		RS485_M_Cmd03_and_Receive(DKG307_ID, 42,5, Slave_registers+42);
		Import_DKG307_Reg(Slave_registers,MAC_registers);
		break;
	case GC315:
		RS485_M_Cmd04_and_Receive(GC315_ID, 0,44, Slave_registers);
		Import_GC315_Reg(Slave_registers,MAC_registers);
		break;
	case DSE7320:
//		//baudrate 115200
		RS485_M_Cmd03_and_Receive(DSE7320_ID, 1000,58, Slave_registers);
		Import_DSE7320_Reg(Slave_registers,MAC_registers);
		break;
	case D300:
		RS485_M_Cmd03_and_Receive(D300_ID, 10200,164, Slave_registers);
		Import_D300_Reg(Slave_registers,MAC_registers);
		break;
	case DST4400:
		RS485_M_Cmd04_and_Receive(DST4400_ID, 0,130, Slave_registers);
		Import_DST4400_Reg(Slave_registers,MAC_registers);
		break;
	case PC1X:
		RS485_M_Cmd03_and_Receive(PC1X_ID, 40000,45, Slave_registers);
		Import_PC1X_Reg(Slave_registers,MAC_registers);
		break;
	default:
		RS485_M_Cmd04_and_Receive(Emko_ID, 0,100, Slave_registers);
		Import_Emko_Reg(Slave_registers,MAC_registers);
	}
}
void ATM_CMD_AUX()
{
	char tmp[20];
	memset(tmp, 0, sizeof(tmp));
	memset(rx1_buff,0,sizeof(rx1_buff));
	uint16_t curr5=0;

	R_Config_SCI1_Serial_Receive((uint8_t*)&rx1_buff, 40);
	// SEND READ COMMAND TO ATM
	memset(print_str, 0, sizeof(print_str));
	sprintf(print_str,"AT+AUX?\r\n");
	R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
	R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);


		//print test
//		memset(print_str, 0, sizeof(print_str));
//		sprintf(print_str,"\r\nAUX: ");
//		RS485_DE2 = 1U; //RS485 send
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//		R_BSP_SoftwareDelay(10, BSP_DELAY_MILLISECS);
//		R_SCI5_AsyncTransmit((uint8_t*)rx1_buff,strlen(rx1_buff));
//		RS485_DE2 = 0U; //RS485 send
		//end_print_test

	if(strlen(rx1_buff)) //+AUX:
	{
		if((rx1_buff[3]=='X')&&(rx1_buff[4]==':'))
		{
			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,rx1_buff+5,strlen(rx1_buff)-5);
			curr5 = atoi(tmp);

			if(curr5*2.5>100) MAC_registers[0x1C] = (uint16_t)(curr5*2.5); //Curr
			else MAC_registers[0x1C] = 0;
		}
	}

}
void ATM_CMD_FREQ()
{
	char tmp[20];
	memset(tmp, 0, sizeof(tmp));
	memset(rx1_buff,0,sizeof(rx1_buff));
	uint16_t freq=0;

	R_Config_SCI1_Serial_Receive((uint8_t*)&rx1_buff, 12);
	// SEND READ COMMAND TO ATM
	memset(print_str, 0, sizeof(print_str));
	sprintf(print_str,"AT+FREQ?\r\n");
	R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
	R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);



	if(strlen(rx1_buff))
	{
		//print test
//		memset(print_str, 0, sizeof(print_str));
//		sprintf(print_str,"\r\nFreq:");
//		RS485_DE2 = 1U; //RS485 send
//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//		R_BSP_SoftwareDelay(10, BSP_DELAY_MILLISECS);
//		R_SCI5_AsyncTransmit((uint8_t*)rx1_buff,strlen(rx1_buff));
//		RS485_DE2 = 0U; //RS485 send
		//end_print_test
		if((rx1_buff[4]=='Q')&&(rx1_buff[5]==':'))
		{
			memset(tmp, 0, sizeof(tmp));
			strncpy(tmp,rx1_buff+6,strlen(rx1_buff)-6);
			freq = atoi(tmp);

			if(freq>200)
			{
				if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==1) // 3pha
				{
					MAC_registers[0x0F] = (uint16_t)freq;
					MAC_registers[0x10] = (uint16_t)freq;
					MAC_registers[0x11] = (uint16_t)freq;
				}
				else
				{
					MAC_registers[0x0F] = (uint16_t)freq;
				}
			}
			else
			{
				MAC_registers[0x0F] = 0;
				MAC_registers[0x10] = 0;
				MAC_registers[0x11] = 0;
			}


			memset(rx1_buff,0,sizeof(rx1_buff));

			//print test
//			memset(print_str, 0, sizeof(print_str));
//			sprintf(print_str,"\r\nFreq= %d\r\n",freq);
//			RS485_DE2 = 1U; //RS485 send
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			RS485_DE2 = 0U; //RS485 send
			//end_print_test
		}
	}


}
void ATM_CMD_read(uint8_t channel)
{
	uint8_t pos[4],cnt=0;
	int32_t para[5];
	char tmp[20];
	memset(tmp, 0, sizeof(tmp));
	memset(rx1_buff,0,sizeof(rx1_buff));

	R_Config_SCI1_Serial_Receive((uint8_t*)&rx1_buff, 40);
	// SEND READ COMMAND TO ATM
	memset(print_str, 0, sizeof(print_str));
	sprintf(print_str,"AT+READ?%d\r\n",channel);
	R_SCI1_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
	R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);
	//RESPONSE: +READ:<channel>,<voltage>,<current>,<power>,<energy>\r\n
	//Update version RESPONSE: +READ:<channel>,<voltage>,<current>,<power>,<power factor>,<energy>\r\n

	if(strlen(rx1_buff))
	{
		if((rx1_buff[4]=='D')&&(rx1_buff[5]==':'))
		{
//			//print test
//			memset(print_str, 0, sizeof(print_str));
//			sprintf(print_str,"\r\nCH%d: ",channel);
//			RS485_DE2 = 1U; //RS485 send
//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//			R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);
//			R_SCI5_AsyncTransmit((uint8_t*)rx1_buff,strlen(rx1_buff));
//			RS485_DE2 = 0U; //RS485 send
//			//end_print_test

			for(uint8_t i=8;i<strlen(rx1_buff);i++)
			{
				if(rx1_buff[i]==',')
				{
					pos[cnt]=i;
					cnt++;
				}
			}
			if(cnt==4) //for updated Ambo version
			{
				//volt
				memset(tmp, 0, sizeof(tmp));
				strncpy(tmp,rx1_buff+8,pos[0]-8);
				para[0] = atoi(tmp);
				//curr
				memset(tmp, 0, sizeof(tmp));
				strncpy(tmp,rx1_buff+pos[0]+1,pos[1]-pos[0]-1);
				para[1] = atoi(tmp);
				//pow
				memset(tmp, 0, sizeof(tmp));
				strncpy(tmp,rx1_buff+pos[1]+1,pos[2]-pos[1]-1);
				para[2] = atoi(tmp);
				//pow factor
				memset(tmp, 0, sizeof(tmp));
				strncpy(tmp,rx1_buff+pos[2]+1,pos[3]-pos[2]-1);
				para[3] = atoi(tmp);
				//energy
				memset(tmp, 0, sizeof(tmp));
				strncpy(tmp,rx1_buff+pos[3]+1,strlen(rx1_buff)-pos[3]);
				para[4] = atoi(tmp);

//				//print test
//				memset(print_str, 0, sizeof(print_str));
//				sprintf(print_str,"\r\nCH%d: curr = %d \r\n",channel,para[1]);
//				RS485_DE2 = 1U; //RS485 send
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U; //RS485 send
//				//end_print_test
				if(channel == 0)
				{
					if(para[0]>=1000) MAC_registers[0x06] = para[0]; //Volt
					else MAC_registers[0x06]=0;
					if(para[1]/10*2.5>100) MAC_registers[0x18] = (uint16_t)(para[1]/4); // Current
					else MAC_registers[0x18] =0;
					MAC_registers[0x23] = abs(para[2]/40); // Pow
					MAC_registers[0x35] = abs(para[3]*10);

//					//print test
//					memset(print_str, 0, sizeof(print_str));
//					sprintf(print_str,"\r\nCH%d: curr = %d \r\n",channel,MAC_registers[0x18]);
//					RS485_DE2 = 1U; //RS485 send
//					R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//					RS485_DE2 = 0U; //RS485 send
//					//end_print_test
				}
				if(channel == 1)
				{
					if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==1) // 3pha
					{
						if(para[0]>1000) MAC_registers[0x07] = para[0]; //Volt
						else MAC_registers[0x07]=0;
						if(para[1]/10*2.5>100) MAC_registers[0x19] = (uint16_t)(para[1]/4); //Curr
						else MAC_registers[0x19] =0;
						MAC_registers[0x24] = abs(para[2]/40); //Pow
						MAC_registers[0x36] = abs(para[3]*10);//Pow factor
					}
					else // 1pha
					{
						if(para[1]/10*2.5>100) MAC_registers[0x7C] = (uint16_t)(para[1]/4); //Curr
						else MAC_registers[0x7C] =0;
					}
//					//print test
//					memset(print_str, 0, sizeof(print_str));
//					sprintf(print_str,"\r\nCH%d: curr = %d \r\n",channel,MAC_registers[0x19]);
//					RS485_DE2 = 1U; //RS485 send
//					R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//					RS485_DE2 = 0U; //RS485 send
//					//end_print_test
				}
				if(channel == 2)
				{
					if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==1) // 3pha
					{
						if(para[0]>1000) MAC_registers[0x08] = para[0]; //Volt
						else MAC_registers[0x08]=0;
						if(para[1]/10*2.5>100) MAC_registers[0x1A] = (uint16_t)(para[1]/4); //Curr
						else MAC_registers[0x1A] = 0;
						MAC_registers[0x25] = abs(para[2]/40); //Pow
						MAC_registers[0x37] = abs(para[3]*10);//Pow factor
					}
					else // 1pha
					{
						if(para[1]/10*2.5>100) MAC_registers[0x7D] = (uint16_t)(para[1]/4); //Curr
						else MAC_registers[0x7D] =0;
					}
//					//print test
//					memset(print_str, 0, sizeof(print_str));
//					sprintf(print_str,"\r\nCH%d: curr = %d \r\n",channel,MAC_registers[0x1A]);
//					RS485_DE2 = 1U; //RS485 send
//					R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//					RS485_DE2 = 0U; //RS485 send
//					//end_print_test
				}
				if(channel == 3)
				{

					MAC_registers[0x1B] = (uint16_t)(para[1]/4); //Curr

				}
				memset(rx1_buff,0,sizeof(rx1_buff));
				//print test
//				memset(print_str, 0, sizeof(print_str));
//				sprintf(print_str,"\r\n Channel: %d Volt: %d Curr: %d Pow: %d PF: %d Energy: %d\r\n",channel,para[0],para[1],para[2],para[3],para[4]);
//				RS485_DE2 = 1U; //RS485 send
//				R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
//				RS485_DE2 = 0U; //RS485 send
				//end_print_test
			}
		}
	}
}
void Load_Check()
{
	R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
	ATM_CMD_FREQ();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
	ATM_CMD_read(2);
	R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
	ATM_CMD_read(1);
	R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
	ATM_CMD_read(0);
	R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
	ATM_CMD_read(3);
	R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
	ATM_CMD_AUX();
	R_BSP_SoftwareDelay(50, BSP_DELAY_MILLISECS);
}
void Mode_UseGrid()
{
	// OFF GEN CONTACTOR
	Gen_Contactor_Off();
	// ON GRID CONTACTOR
	if(FB_GEN_COIL==0) Grid_Contactor_On();
	if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
}
void Mode_UseGen()
{
	if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==2) // tram co ATS
	{
		// OFF GEN CONTACTOR
		Grid_Contactor_Off();
		// ON GRID CONTACTOR
		if(FB_GRID_COIL==0) Gen_Contactor_On();
		if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
	}
}
void Process_OffAll()
{
	if(g_is_all_relay_off==0)
	{
		// OFF GRID CONTACTOR
		Grid_Contactor_Off();
		//OFF GEN CONTACTOR
		Gen_Contactor_Off();
		// OFF gen
		if(Gen_check())
		{
//			DKG_Stop(); //execute time max [0x58] sec
			GENERATOR_STOP(GENTYPE);
		}
		if(GenStart==0) g_is_all_relay_off =1; // Gen stop succesfully
	}

	if(g_sci12_rx_count>7)	RS485_Slave_Mode(MAC_registers);
}
void Mode_Auto()
{
	if(Grid_check()) // RUN ON GRID
	{
//		//print test
//		memset(print_str, 0, sizeof(print_str));
//		sprintf(print_str,"\r\n GRID OK!");
//		RS485_DE1 = 1U; //RS485 send
//		R_SCI12_AsyncTransmit((uint8_t*)print_str,strlen(print_str),trans_enable);
//		RS485_DE1 = 0U; //RS485 send
//		//end_print_test
		MAC_registers[0x38]=1; // Grid volt sts
		//Wait Grid stable
		waittime(MAC_registers[0x5A]);
		if(Grid_check()) // check GRID
		{
			// OFF GEN CONTACTOR
			Gen_Contactor_Off();
			// ON GRID CONTACTOR
			Grid_Contactor_On();
			//STOP GEN
//			if(Gen_check())
//			{
//				DKG_Stop();
//				GENERATOR_STOP(GENTYPE);
//			}
			if(!Gen_check()) gen_running_timeout=0;
		}
		if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
	}
	else // NO GRID
	{
//		if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==2) // tram co ATS
//		{
//			MAC_registers[0x38]=0; // grid sts
//			// USE GEN if DC is LOW
//			if((MAC_registers[0x3E]==1) || (MAC_registers[0x3C]!=0)) // DC is low || Gen is on -> Use Gen
//			{
//				if(MAC_registers[0x56]==1) // GEN DEACTIVE MODE
//				{
//					if(Mode_DeactiveCheck()==1) // current time is in deactive time - OFF ALL
//					{
//						Process_OffAll();
//					}
//					else // current time is NOT in deactive time
//					{
//						// check gen run time
//						if(gen_running_timeout==0) // No violation -
//						{
//							// OFF GRID CONTACTOR
//							Grid_Contactor_Off();
//							//Check Gen
//							if(Gen_check()==1) // if gen ON
//							{
//								//ON GEN CONTACTOR
//								Gen_Contactor_On();
//							}
//							else // if gen OFF
//							{
//								// OFF GEN CONTACTOR
//								Gen_Contactor_Off();
//								// START GEN & ON GEN CONTACTOR
//								Process_StartGen();
//							}
//						}
//						else // Runtime violation - OFF ALL
//						{
//							Process_OffAll();
//						}
//					}
//					if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
//				}
//				else // NOT IN GEN DEACTIVE MODE
//				{
//					// check gen run time
//					if(gen_running_timeout==0) // No violation -
//					{
//						// OFF GRID CONTACTOR
//						Grid_Contactor_Off();
//						//Check Gen
//						if(Gen_check()==1) // if gen ON
//						{
//							//ON GEN CONTACTOR
//							Gen_Contactor_On();
//						}
//						else // if gen OFF
//						{
//							// OFF GEN CONTACTOR
//							Gen_Contactor_Off();
//							// START GEN & ON GEN CONTACTOR
//							Process_StartGen();
//						}
//					}
//					else // Runtime violation - OFF ALL
//					{
//						Process_OffAll();
//					}
//					if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
//				}
//			}
//			else // DC is ok && gen off
//			{
//				Process_OffAll();
//			}
//		}
//		else //tram khong co ATS
//		{
//			Process_OffAll();
//		}

		if(Gen_check()==1)
		{
			waittime(MAC_registers[0x59]);
			if(Gen_check()==1)
			{
				Grid_Contactor_Off();
				Gen_Contactor_On();
			}
			if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
		}
		else
		{
			Process_OffAll();
		}
	}
}
uint8_t Mode_DeactiveCheck()
{
	uint8_t sts=0;
	uint16_t current_time, start, stop;
	current_time = (uint16_t)(MAC_registers[0x5F]<<8) + (uint16_t)(MAC_registers[0x60]);
	start = (uint16_t)(MAC_registers[0x4E]<<8) + (uint16_t)(MAC_registers[0x5D]);
	stop = (uint16_t)(MAC_registers[0x54]<<8) + (uint16_t)(MAC_registers[0x5E]);

	if(start>stop)
	{
		if((current_time<start)&&(current_time>stop)) sts =0;
		else sts =1;
	}
	else
	{
		if((current_time<start)||(current_time>stop)) sts =0;
		else sts =1;
	}

	return sts;
}
void Process_StoptGen_Manual()
{
	for(uint8_t i=0;i<3;i++)
	{
//		DKG_Stop();
		GENERATOR_STOP(GENTYPE);
		if(!Gen_check())
		{
			MAC_registers[0x3C] =0;
			MAC_registers[0x4C] =0;
			i=3;
		}
		else
		{
			MAC_registers[0x4C] =2;
		}
		if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
	}
	if(Gen_check())
	{
		MAC_registers[0x4C] = 2;
	}
	else
	{
		MAC_registers[0x4C] = 0;
	}
	MAC_registers[0x42] =0;
}
void Process_StartGen_Manual()
{
	//GEN IN StOP MODE && ATS IN MANUAL MODE
	if((MAC_registers[0x3C]==0)&&(MAC_registers[0x41]!=0))
	{
		for(uint8_t i=0;i<MAC_registers[0x61];i++)
		{
	//		Smartgen_Manual(Smartgen_ID, 0x0004, 0x00FF); // Manual Mode
	//		Smartgen_Start(Smartgen_ID, 0x0000, 0x00FF);
//			DKG_Start();
			MAC_registers[0x3C] =1; //gen starts
			GENERATOR_START(GENTYPE);
			waittime(MAC_registers[0x62]); //thoi gian de [0x62]

			//print test
	//		memset(print_str, 0, sizeof(print_str));
	//		sprintf(print_str,"\r\nfinish wait ",i);
	//		RS485_DE2 = 1U; //RS485 send
	//		R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
	//		RS485_DE2 = 0U; //RS485 send
			//end_print_test

			if(Gen_check())
			{
				//print test
	//			memset(print_str, 0, sizeof(print_str));
	//			sprintf(print_str,"\r\nGen run ",i);
	//			RS485_DE2 = 1U; //RS485 send
	//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
	//			RS485_DE2 = 0U; //RS485 send
				//end_print_test

				MAC_registers[0x4C] = 0;
				GenStart=1; // start count run time
				i=3;

			}
			else
			{

	//			Smartgen_Stop(Smartgen_ID, 0x0001, 0x00FF);
//				DKG_Stop();
				MAC_registers[0x4C] = 1;
				GENERATOR_STOP(GENTYPE);
				waittime(MAC_registers[0x63]); //thoi gian nghi giua 2 lan de [0x63]
				//print test
	//			memset(print_str, 0, sizeof(print_str));
	//			sprintf(print_str,"\r\nGen Off %d ",i);
	//			RS485_DE2 = 1U; //RS485 send
	//			R_SCI5_AsyncTransmit((uint8_t*)print_str,strlen(print_str));
	//			RS485_DE2 = 0U; //RS485 send
				//end_print_test

			}
			if(g_sci12_rx_count>7) 	RS485_Slave_Mode(MAC_registers);
		}
		if(!Gen_check())
		{
			MAC_registers[0x4C] = 1; // Start Gen error
		}
		else
		{
			MAC_registers[0x4C] = 0;
		}
	}
	else
	{
		MAC_registers[0x42] =0;
	}
	MAC_registers[0x42] =0;
}
void Process_StartGen()
{
	if(MAC_registers[0x4C]==0) // HAS NO ERROR ALLOW TO START
	{
		for(uint8_t i=0;i<MAC_registers[0x61];i++)
		{
	//		Smartgen_Manual(Smartgen_ID, 0x0004, 0x00FF); // Manual Mode
	//		Smartgen_Start(Smartgen_ID, 0x0000, 0x00FF);
//			DKG_Start();
			MAC_registers[0x3C] =1; //gen start
			GENERATOR_START(GENTYPE);
			waittime(MAC_registers[0x62]); //thoi gian de [0x62]
			if(Gen_check())
			{
				MAC_registers[0x4C] = 0;
				GenStart=1; // run time count
				// wait stable
				waittime(MAC_registers[0x59]);

				// ON GEN CONTACTOR
				Gen_Contactor_On();
				i=3;
			}
			else
			{
	//			Smartgen_Stop(Smartgen_ID, 0x0001, 0x00FF);
//				DKG_Stop();
				GENERATOR_STOP(GENTYPE);
				waittime(MAC_registers[0x63]); // thoi gian nghi giua 2 lan de [0x63]
			}
		}
		if(!Gen_check())
		{
			MAC_registers[0x4C] = 1; // Start Gen error
		}
	}
	else // NOT ALLOW TO START -> OFF ALL
	{
		Process_OffAll(); // execute time max [0x58] second
	}
}
void waittime(uint16_t second)
{
	wait_time =0;
//	R_Config_SCI12_Serial_Receive((uint8_t*)&rx12_buff, 100);//MAC buffer
	while(wait_time<second) //
	{
		R_BSP_SoftwareDelay(200, BSP_DELAY_MILLISECS);
		if(g_sci12_rx_count>7) 	RS485_Slave_Mode(MAC_registers);

		//print test
//		memset(print_str, 0, sizeof(print_str));
//		sprintf(print_str,"\r\ntime count: %d ",wait_time);
//		RS485_DE1 = 1U; //RS485 send
//		R_SCI12_AsyncTransmit((uint8_t*)print_str,strlen(print_str),trans_enable);
//		RS485_DE1 = 0U; //RS485 send
		//end_print_test
		RS485_Master_Mode(Gen_registers,MAC_registers,GENTYPE);
		Load_Check();
		Gen_check();
		R_BSP_SoftwareDelay(150, BSP_DELAY_MILLISECS);
		Grid_check();
		R_BSP_SoftwareDelay(150, BSP_DELAY_MILLISECS);
	}
}

void main_FW_update(void)
{
    fw_up_return_t ret_fw_up;
    ret_fw_up = fw_up_open();
    if (FW_UP_SUCCESS != ret_fw_up)
    {
    }
    else
    {
        Buzzer(1, 300);
    	ret_fw_up = switch_start_up_and_reset();

        switch (ret_fw_up)
        {
            case FW_UP_ERR_NOT_OPEN:
                break;
            case FW_UP_ERR_INVALID_RESETVECT:
                break;
            case FW_UP_ERR_SWITCH_AREA:
                break;
            default:

                /** Do nothing */
            break;
        }

        ret_fw_up = fw_up_close();

        if (FW_UP_SUCCESS != ret_fw_up)
        {

        }
    }
}
void Buzzer(uint8_t times, uint16_t millisec)
{
	for(uint8_t i=0;i<times;i++)
	{
    	BUZZER =1;
    	R_BSP_SoftwareDelay(millisec, BSP_DELAY_MILLISECS);
    	BUZZER =0;
    	R_BSP_SoftwareDelay(millisec, BSP_DELAY_MILLISECS);
	}
}
uint8_t ErrorCheck(void)
{
	uint8_t sts;
	if((MAC_registers[0x3F]!=0)||(MAC_registers[0x40]!=0)||(MAC_registers[0x48]!=0)||(MAC_registers[0x49]!=0)||(MAC_registers[0x4A]!=0)||(MAC_registers[0x4B]!=0)||(MAC_registers[0x4C]!=0)||(MAC_registers[0x4D]!=0))
		sts =1;
	else sts =0;
	return 	sts;
}
void MAC_reg_init()
{
	MAC_registers[0x50] = 30; //Thời gian đợi thiết bị xoá lỗi và tiếp tục điều khiển máy phát (atsGenAutoResetTimeout)(s)
	MAC_registers[0x53] = 1; //Chế độ cho phép máy phát tư reset lại khi bị lỗi (atsGenAutoResetMode)
	MAC_registers[0x55] = 1;//Số lần tối đa cho phép xoá lỗi và reset máy phát (atsGenAutoResetMax)
	MAC_registers[0x58] = 130;
	MAC_registers[0x59] = 60;
	MAC_registers[0x5A] = 30;
	MAC_registers[0x5B] = 4; // Thoi gian chay may phat toi da
	MAC_registers[0x61] = 3; // Gen start try times
	MAC_registers[0x62] = 30; //thời gian đề
	MAC_registers[0x63] = 30; //thời gian giữa 2 lần đề
	MAC_registers[0x65] = 0; //OTA sts;
	MAC_registers[0x66] = 0;//OTA ctrl;
	MAC_registers[0x67] = MAC_VERSION; // version;
	MAC_registers[0x6F] = 0; // transmit disable
	MAC_registers[0x70] = 0; // gen error reset counter
	MAC_registers[0x73] = GENTYPE;
	MAC_registers[0x7B] = PHASE_ATS;

}
void Grid_Contactor_On()
{
	ATS_CTRL_CONTACTOR_GRID =0; // on grid contactor
	MAC_registers[0x43]=1;
	R_BSP_SoftwareDelay(200, BSP_DELAY_MILLISECS);
	MAC_registers[0x3A] =FB_GRID_COIL; // update grid contactor sts
	//check feedback
	if(FB_GRID_COIL==0) // incorrect
	{
		if(MAC_registers[0x4D] == 2) MAC_registers[0x4D] = 3; // 2 contactors error
		if(MAC_registers[0x4D] == 0) MAC_registers[0x4D] = 1; // Grid contactor error
	}
	else // correct
	{
		if(MAC_registers[0x4D] == 1) MAC_registers[0x4D] =0; // no contactor error
		if(MAC_registers[0x4D] == 3) MAC_registers[0x4D] =2; // Gen contactor error
	}
}
void Grid_Contactor_Off()
{
	ATS_CTRL_CONTACTOR_GRID =1; // off grid contactor
	MAC_registers[0x43]=0;
	R_BSP_SoftwareDelay(200, BSP_DELAY_MILLISECS);
	MAC_registers[0x3A] =FB_GRID_COIL; // update grid contactor sts
	//check feed back
	if(FB_GRID_COIL==0) //correct
	{
		if(MAC_registers[0x4D] == 3) MAC_registers[0x4D] = 2; // gen contactor error
		if(MAC_registers[0x4D] == 1) MAC_registers[0x4D] = 0; // no error
	}
	else // incorrect
	{
		if(MAC_registers[0x4D] == 0) MAC_registers[0x4D] =1; // grid contactor error
		if(MAC_registers[0x4D] == 2) MAC_registers[0x4D] =3; // 2 contactors error
	}
}
void Gen_Contactor_On()
{
	ATS_CTRL_CONTACTOR_GEN =1; // on Gen contactor
	MAC_registers[0x44]=1;
	R_BSP_SoftwareDelay(200, BSP_DELAY_MILLISECS);
	MAC_registers[0x3B] =FB_GEN_COIL; // update gen contactor sts
	//check feedback
	if(FB_GEN_COIL==0) // incorrect
	{
		if(MAC_registers[0x4D] == 1) MAC_registers[0x4D] = 3; // contactor error
		if(MAC_registers[0x4D] == 0) MAC_registers[0x4D] = 2;
	}
	else // correct
	{
		if(MAC_registers[0x4D] == 2) MAC_registers[0x4D] =0; // Gen contactor error
		if(MAC_registers[0x4D] == 3) MAC_registers[0x4D] =1; // Gen contactor error
	}
	Gen_check();// update [0x3C]
}
void Gen_Contactor_Off()
{
	ATS_CTRL_CONTACTOR_GEN =0; //off gen contactor
	MAC_registers[0x44]=0;
	R_BSP_SoftwareDelay(200, BSP_DELAY_MILLISECS);
	MAC_registers[0x3B] =FB_GEN_COIL; // update gen contactor sts
	if(FB_GEN_COIL==0) //corrects
	{
		if(MAC_registers[0x4D] == 2) MAC_registers[0x4D] =0; // no error
		if(MAC_registers[0x4D] == 3) MAC_registers[0x4D] =1; // Grid contactor error
	}
	else //incorrect
	{
		if(MAC_registers[0x4D] == 1) MAC_registers[0x4D] = 3; // 2 contactors error
		if(MAC_registers[0x4D] == 0) MAC_registers[0x4D] = 2; // Gen contactor error
	}
	Gen_check();// update [0x3C]
}
void Relay5_On()
{
	if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==2) // tram co ATS
	{
		CTRL_RELAY1 =1;
		MAC_registers[0x47] = 1;
	}
}

void Relay5_Off()
{
	if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==2) // tram co ATS
	{
		CTRL_RELAY1 =0;
		MAC_registers[0x47] = 0;
	}
}

void GENERATOR_START(generator_type gen_type)
{
	if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==2) // tram co ATS
	{
		MAC_registers[0x3C] = 1; //Start
		switch(gen_type)
		{
		case SMARTGEN:
			Smartgen_Manual(Smartgen_ID, 0x0004, 0x00FF); // Manual Mode
			Smartgen_Start(Smartgen_ID, 0x0000, 0x00FF);
			break;
		case EMKO:
			Emko_Start();
			break;
		case DKG307:
			DKG307_Start();
			break;
		case GC315:
			GC315_Start();
			break;
		case DSE7320:
			DSE7320_Start();
			break;
		case D300:
			D300_Start();
			break;
		case DST4400:
			DST4400_Start();
			break;
		case PC1X:
			PC1X_Start();
			break;
		default:
			Emko_Start();
		}
		if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
	}
}
void GENERATOR_STOP(generator_type gen_type)
{
	if(MAC_registers[0x7B]==0 || MAC_registers[0x7B]==2) // tram co ATS
	{
		switch(gen_type)
		{
		case SMARTGEN:
			Smartgen_Stop(Smartgen_ID, 0x0000, 0x00FF);
			break;
		case EMKO:
			Emko_Stop();
			break;
		case DKG307:
			DKG307_Stop();
			break;
		case GC315:
			GC315_Stop();
			break;
		case DSE7320:
			DSE7320_Stop();
			break;
		case D300:
			D300_Stop();
			break;
		case DST4400:
			DST4400_Stop();
			break;
		case PC1X:
			PC1X_Stop();
			break;
		default:
			Emko_Stop();
		}
		if(g_sci12_rx_count>7) RS485_Slave_Mode(MAC_registers);
	}
}
void Generator_Switch()
{
	//Gen type select
	switch(MAC_registers[0x73])
	{
	case 1:
		GENTYPE = SMARTGEN;
		break;
	case 2:
		GENTYPE = EMKO;
		break;
	case 3:
		GENTYPE = DKG307;
		break;
	case 4:
		GENTYPE = GC315; //GC250
		break;
	case 5:
		GENTYPE = DSE7320; //DSE6110
		break;
	case 6:
		GENTYPE = D300; //D500 D700
		break;
	case 7:
		GENTYPE = DST4400;
		break;
	case 8:
		GENTYPE = PC1X;
		break;
	default:
		GENTYPE = EMKO;
	}
}

void TimerMode()
{
	switch(MAC_registers[0x74]) //Timer Mode
	{
	case 0:
		Relay5_Off();
		break;
	case 1:
		if((MAC_registers[0x75]==MAC_registers[0x5F])&&(MAC_registers[0x76]==MAC_registers[0x60])) //check hour & min turn on relay 5, timer 1
		{
			Relay5_On();
			Timer1Relay5=(MAC_registers[0x77]*60); //start counting
		}
		if(Timer1Relay5 <= 60 ) Relay5_Off();
		break;
	case 2:
		if((MAC_registers[0x78]==MAC_registers[0x5F])&&(MAC_registers[0x79]==MAC_registers[0x60])) //check hour & min turn on relay 5, timer 2
		{
			Relay5_On();
			Timer2Relay5= (MAC_registers[0x7A]*60); //start counting
		}
		if(Timer2Relay5 <= 60 ) Relay5_Off();
		break;
	case 3:
		if((MAC_registers[0x75]==MAC_registers[0x5F])&&(MAC_registers[0x76]==MAC_registers[0x60])) //check hour & min turn on relay 5, timer 1
		{
			Relay5_On();
			Timer1Relay5=(MAC_registers[0x77]*60); //start counting
		}
		if(Timer1Relay5 <= 60 ) Relay5_Off();

		if((MAC_registers[0x78]==MAC_registers[0x5F])&&(MAC_registers[0x79]==MAC_registers[0x60])) //check hour & min turn on relay 5, timer 2
		{
			Relay5_On();
			Timer2Relay5= (MAC_registers[0x7A]*60); //start counting
		}
		if(Timer2Relay5 <= 60 ) Relay5_Off();
		break;

	}
}
