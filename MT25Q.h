/* 
 * File:   main.h
 * Author: Ganji
 *
 * Created on October 22, 2024, 3:31 PM
 */

 #ifndef MT25Q_H
 #define	MT25Q_H
 
 #ifdef	__cplusplus
 extern "C" {
 #endif
 #define SECT 65536

Static int32 ADD_INFO_ADDRESS = 4096;//for keeping the address data to flash memory
Static int32 ADDRESS_WRITING_COUNTER = 0;//if address writing more than 30000 times, change the store address

Static int32 FLAG_DATA_ADDRESS = SECT*4;//1sector
Static int32 RSV_DATA_ADDRESS=SECT*5;//1sector
Static int32 SAT_LOG = SECT*6;//2sector
Static int32 CAM_ADDRESS = SECT*8;//90sector
Static int32 FAB_HK_ADDRESS = SECT*98;//1000sector
Static int32 FAB_CW_ADDRESS = SECT*1098;//40sector
Static int32 ADCS_SENSOR_ADDRESS = SECT*1138;//500sector
Static int32 DC_STATUS_ADDRESS = SECT*1638;//1sector
Static int32 HIGH_SAMP_HK_ADDRESS = SECT*1639;//409sector
Static int32 FLAG_ADDRESS_EEPROM = 0x18000;//from 75 percent of the programming memory
Static int32 FLASH_AD_ADDRESS_EEPROM = 0x18000 + 28;//after flag info

#define FLAG_DATA_ADDRESS_END  SECT*5-26
#define RSV_DATA_ADDRESS_END  SECT*6-60
#define SAT_LOG_END  SECT*8-22
#define CAM_ADDRESS_END  SECT*98-SECT*2
#define FAB_HK_ADDRESS_END  SECT*1098-SECT-248
#define FAB_CW_ADDRESS_END  SECT*1138-10
#define FAB_CW_ADDRESS_FOR_MF_END FAB_CW_ADDRESS_END
#define ADCS_SENSOR_ADDRESS_END  SECT*1638-408000//408000 is for 2 hour data(85*3600*2/1.5)
#define DC_STATUS_ADDRESS_END SECT*1639-415
#define HIGH_SAMP_HK_ADDRESS_END  SECT*2047-178560//178560 is for 2 hour data(124*3600*2/5)

int8 BC_ATTEMPT_FLAG = 0;
int16 PASSED_DAYS = 0;
int8 RESERVE_CHECK = 0;
int8 RESERVE_MIN_FLAG = 0;
int8 RESERVE_TARGET_FLAG = 0;
int8 MISSION_CONTENTS = 0;
int8 MISSION_DETAIL = 0;
int8 Kill_FLAG_MAIN = 0;
int8 Kill_FLAG_FAB = 0;
int8 FIRST_HSSC_DONE = 0;
int8 AUTO_CAM_DONE = 0;
int8 AUTO_LDM_DONE = 0;
int8 AUTO_ADCS_DONE = 0;
int8 ANT_DEP_STATUS = 0;
int8 UPLINK_SUCCESS = 0;

int8 RESERVE_SEC_FLAG = 0;

//--------MAIN PIC Buffer------------------------------------------------------
int8 CMD_FROM_PC[6] = {};
unsigned int8 in_bffr_main[16] = {};
int8 COM_DATA= 0;
static int8 CW_IDENTIFIER = 0;
int8 OPERATION_MODE = 0x00;
int8 PC_DATA = 0;                                  
int8 COM_ONEBYTE_COMMAND = 0; //for GS testing

#define buffer_from_com  (in_bffr_main[0]==0xAA) && (in_bffr_main[15]==0xBB)
#define buffer_flash  (in_bffr_main[7]==0x73)
#define START_ADCS_MISSION  (in_bffr_main[1]==0x73)
#define ADCS_SENSOR_SIZE 78//for testing additional 6byte for checking duty

int8 ADCS_SENSOR_DATA[ADCS_SENSOR_SIZE] = {};
int8 ADCS_ACK = 0;
int8 ADCS_ACK_COMMING = 0; //for Checking ACK from ADCS
#define ATTEMPT_TIME 8


#define EX_PANEL_THRESHHOLD 0x14
#define HIGH_SAMP_TIMES 12//for 2 hours

//----------RESET--------------------------------------------------------------
#define Reset_size 11 //1byte ack, 4byte time, 10byte sensor
int8 RESET_DATA = 0;
int8 reset_bffr[Reset_size] = {};
int8 reset_flag = 0;

//---------OTHER FUNCTUON------------------------------------------------------
#define FLASH_ADD_SIZE 41//10 kinds of address(40byte), 1 frag(1byte), 1 WITING COUNTER(4byte)
#define FLAG_INFO_SIZE 16
unsigned int8 sec_add_bfr[FLASH_ADD_SIZE] = {};
unsigned int8 flag_info_bffr[FLAG_INFO_SIZE] = {};
 
 #ifdef	__cplusplus
 }
 #endif
 
 #endif	/* MAIN_H */
 
 