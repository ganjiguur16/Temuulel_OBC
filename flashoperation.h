/* 
 * File:   main.h
 * Author: Ganji
 *
 * Created on October 22, 2024, 3:31 PM
 */

 #ifndef MT25Q_H
#define MT25Q_H

// #ifdef __INTELLISENSE__
// // CCS C type stubs for VS Code IntelliSense only
// typedef int int32;
// typedef unsigned int uint32;
// typedef char int8;
// typedef unsigned char uint8;
// typedef short int16;
// typedef unsigned short uint16;
// #endif

#ifdef __cplusplus
extern "C" {
#endif
#include <flashoperation.c>

// ---- Function Prototypes ----
void CURRENT_FLAG_STATUS(void);
void STORE_ADRESS_DATA_TO_FLASH(void);
void CHANGE_ADDRESS_WRITING_LOCATION(void);
void CHECK_ADDRESS_DATA(void);
void TAKE_ADDRESS_DATA_FROM_OF(void);
void TAKE_ADDRESS_DATA_FROM_SCF(void);
void TAKE_ADDRESS_DATA_FROM_SMF(void);
void MAKE_ADDRESS_DATA(void);
void READ_WRTITING_ADDRESS_LOCATION(void);
void ERASE_EEPROM_INFO(void);
void STORE_FLAG_INFO(void);
void RESET_FLAG_DATA(void);
void WRITE_FLAG_to_EEPROM(void);
void MAKE_FLAG_from_EEPROM(void);
void CHECK_FLAG_INFO(void);
void TAKE_FLAG_INFO_FROM_OF(void);
void MAKE_FLAG_INFO(void);
void WRITE_AD_INFO_to_EEPROM(void);
void TAKE_AD_INFO_from_EEPROM(void);
void MEMORY_ERASE(void);
void LOOP_FLAG_DATA_ADDRESS(void);
void LOOP_RSV_DATA_ADDRESS(void);
void LOOP_SAT_LOG(void);
void LOOP_CAM_ADDRESS(void);
void LOOP_FAB_HK_ADDRESS(void);
void LOOP_FAB_CW_ADDRESS(void);
void LOOP_ADCS_SENSOR_ADDRESS(void);
void LOOP_DC_STATUS_ADDRESS(void);
void LOOP_HIGH_SAMP_HK_ADDRESS(void);

#ifdef __cplusplus
}
#endif

#endif /* MT25Q_H */