/* 
 * File:   main.h
 * Author: Ganji
 *
 * Created on October 22, 2024, 3:31 PM
 */

 #ifndef MAIN_H
 #define	MAIN_H
 
 #ifdef	__cplusplus
 extern "C" {
 #endif

 #include <18F67J94.h>

 #include <string.h>
#include <stdbool.h>

// #device ADC=16
// #device ICD=TRUE
// #include <main_functions.h>
 #FUSES NOWDT NOBROWNOUT    
 #use delay(clock=16M, crystal)
  
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 #use rs232(baud=9600, parity=N, xmit=PIN_E5, rcv=PIN_E4, bits=8, stream=EPS) //EPS DATA ACQUISITION
 #use rs232(baud=9600, parity=N, xmit=PIN_C6, rcv=PIN_C7, bits=8, stream=EXT) //MAIN RAB Rear access board 
 #use rs232(baud=57600, parity=N, xmit=PIN_D2, rcv=PIN_D3, bits=8, stream=COM, FORCE_SW) //MAIN COM Communication, send CW data 
 #use rs232(baud=57600, parity=N, xmit=PIN_F7, rcv=PIN_F6, bits=8, stream=CAM, FORCE_SW) //MAIN CAM Communicationx
 #use spi(MASTER, CLK=PIN_E1, DI=PIN_E0, DO=PIN_E6,  BAUD=10000, BITS=8, STREAM=MAIN_FM, MODE=0) //MAIN flash memory port
 #use spi(MASTER, CLK=PIN_B2, DI=PIN_B5, DO=PIN_B4,  BAUD=10000, BITS=8, STREAM=COM_FM, MODE=0) //COM shared flash memory port
 #use spi(MASTER, CLK=PIN_A3, DI=PIN_A0, DO=PIN_A1,  BAUD=10000, BITS=8, STREAM=MISSION_FM, MODE=0) //ADCS shared flash memory port, Camera module (ovcam,mvcam) only can access via mux selcent
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //SPI Stream alter name 
 #define SPIPORT MAIN_FM
 #define SPIPORT2 COM_FM
 #define SPIPORT3 MISSION_FM  //cam system futher added 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     


 //Flash memory chip select pins and mux control 
 #define CS_PIN_1 PIN_E2 //OBC_FLASH_SELECT
 #define CS_PIN_COM PIN_B3 //COM_CHIP_SELECT
 #define CS_PIN_MISSION PIN_A2 //ADCS_CHIP_SELECT
 #define MX_PIN_OVCAM PIN_G2 //OVCAM_MUX_SELECT
 #define MX_PIN_MVCAM PIN_G3 //MVCAM_MUX_SELECT
 #define MX_PIN_ADCS PIN_A5 //ADCS_MUX_SELECT
 #define MX_PIN_COM PIN_C4 //COM_MUX_SELECT
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
 //mt25q flash memory command assigment
 #define READ_ID              0x9F
 #define READ_STATUS_REG      0x05 
 #define READ_DATA_BYTES      0x13  //0x03 for byte
 #define ENABLE_WRITE         0x06
 #define WRITE_PAGE           0x12  //0x02 for 3byte 
 #define ERASE_SECTOR         0xDC  //0xD8 for 3byte
 #define ERASE_4KB_SUBSECTOR  0x21
 #define ERASE_32KB_SUBSECTOR 0x5C
 #define DIE_ERASE            0xC4
 #define FAST_READ            0x0B
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
 //memory maping     
 #define SHUTDOWN_COUNT_ADDRESS 0x00100011
 
 //digtal control pins 
 #define EN_SUP_3V3_1 PIN_B0
 #define EN_SUP_3V3_2 PIN_G1
 #define EN_SUP_3V3_DAQ PIN_D0
 #define EN_SUP_UNREG PIN_B1
 #define EN_SUP_5V0 PIN_D1
 #define KILL_SWITCH PIN_A4
 #define MVCAM_PWR PIN_G0
 #define OVCAM_PWR PIN_D7
 #define ADCS_PWR PIN_D6
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //FLAG variable
    //  int8 EPS_UART = 0;
    //  int8 COM_UART = 0;
    //  int8 EXT_UART = 0; 
    //  unsigned int8 in_bffr_main[16] = {};
    //  int8 COM_DATA= 0;
 
 //    int8 ADCS_HK_ADDRESS = 0x00010000;
 //    int8 ADCS_HK_ADDRESS_COUNTER = 0x00000000;
 //    int8 SHUT_DOWN_COUNT_ADD = 0x00010000;
 //    int8 ADCS_COMMAND = 0x02;
 //    unsigned char *READ_HK_ADCS[16];
     
 
 //SerialDataReceive(){
 //   int num = 0;
 //   for(num = 0 ;num < 16; num++)
 //   {
 //    in_bffr_main[num] = fgetc(EXT);
 //   }     
 //   return;
 //}
 //void Delete_Buffer() //delete com command buffer
 //{
 //   int num = 0;
 //   for(num = 0;num < 16; num++)
 //   {
 //    in_bffr_main[num] = 0x00;
 //   }
 //   COM_DATA = 0;
 //   return;
 //}
 //
 //void Transfer_Buffer(int PORT_NUM) //get buffer data one by one
 //                                   //1:EPS 2:EXT 3:COM 4:CAM
 //{
 //   int num = 0;
 //   switch(PORT_NUM)
 //   {
 //      case 1:
 //         for(;num < 16; num++)
 //         {
 //         fputc(in_bffr_main[num],EPS);
 //         }
 //         break;
 //      case 2:
 //         for(;num < 16; num++)
 //         {
 //         fputc(in_bffr_main[num],EXT);
 //         }
 //         break;
 //      case 3:
 //         for(;num < 16; num++)
 //         {
 //         fputc(in_bffr_main[num],COM);
 //         }
 //         break;
 //      case 4:
 //         for(;num < 16; num++)
 //         {
 //         fputc(in_bffr_main[num],CAM);
 //         }
 //         break;
 //   }
 //   return;
 //}
 //
 //void process_uart() {
 //    if (kbhit(EXT)) {  // Check if data is available
 //        SerialDataReceive();  // Load 16 bytes into the buffer
 //        fprintf(EXT, "Received Data: ");
 //        Transfer_Buffer(1);  // Transfer to PC for verification
 //        Delete_Buffer();  // Clear the buffer for the next message
 //    }
 //}
 
 void WRITE_ENABLE_OF(){
  output_low(CS_PIN_1);
  spi_xfer(SPIPORT,ENABLE_WRITE);                //Send 0x06
  output_high(CS_PIN_1);  
  return;
 }
 
 void WRITE_ENABLE_OF_COM(){
      // Lower CS to select the SPI device
     output_low(CS_PIN_COM);
     // Lower MX to connect to flash device
     output_low(MX_PIN_COM);
     spi_xfer(SPIPORT2,ENABLE_WRITE);                //Send 0x06
     output_high(CS_PIN_COM);
     output_high(MX_PIN_COM);
  return;
 }
 
 void WRITE_ENABLE_OF_ADCS(){
     // Lower MX to connect to flash device
     output_low(MX_PIN_ADCS);
      // Lower CS to select the SPI device
     output_low(CS_PIN_MISSION);
     spi_xfer(SPIPORT3,ENABLE_WRITE);                //Send 0x06
     output_high(CS_PIN_MISSION);  
     output_high(MX_PIN_ADCS);
  return;
 }
 void WRITE_ENABLE_IHS(){
    // lower MX to connect to flash device
    output_low(MX_PIN_MVCAM);
    // LowerCS pin to activate the flash device
    output_low(CS_PIN_MISSION);
    spi_xfer(SPIPORT3,ENABLE_WRITE);                //Send 0x06
    output_high(CS_PIN_MISSION);
    output_high(MX_PIN_MVCAM);
  return;
 }

 void WRITE_ENABLE_GENERIC(int STREAM, int CS_PIN, int MX_PIN) {
    // Lower MUX pin if applicable
    if (MX_PIN != -1) {
        output_low(MX_PIN);
    }

    // Lower CS to select the SPI device
    output_low(CS_PIN);

    if(STREAM == 1) {
        spi_xfer(SPIPORT, ENABLE_WRITE);  // Send ENABLE_WRITE command
    } else if(STREAM == 2) {
        spi_xfer(SPIPORT2, ENABLE_WRITE);  // Send ENABLE_WRITE command
    } else if(STREAM == 3) {
        spi_xfer(SPIPORT3, ENABLE_WRITE);  // Send ENABLE_WRITE command
    } 
    // Raise CS to deselect the SPI device
    output_high(CS_PIN);

    // Raise MUX pin if applicable
    if (MX_PIN != -1) {
        output_high(MX_PIN);
    }
}
 
 void SECTOR_ERASE_OF_ADCS(unsigned int32 sector_address) {
    unsigned int8 address[4];
    // Byte extraction for a 32-bit address
    address[0] = (unsigned int8)((sector_address >> 24) & 0xFF);
    address[1] = (unsigned int8)((sector_address >> 16) & 0xFF);
    address[2] = (unsigned int8)((sector_address >> 8) & 0xFF);
    address[3] = (unsigned int8)(sector_address & 0xFF);

    // Enable write operation
    WRITE_ENABLE_OF_ADCS();

    // Lower MX to connect to flash device
    output_low(MX_PIN_ADCS);
    // Lower CS to select the SPI device
    output_low(CS_PIN_MISSION);
    delay_us(2);  // Small delay for stabilization

    // Send ERASE command and address
    spi_xfer(SPIPORT3, ERASE_SECTOR);
    spi_xfer(SPIPORT3, address[0]);
    spi_xfer(SPIPORT3, address[1]);
    spi_xfer(SPIPORT3, address[2]);
    spi_xfer(SPIPORT3, address[3]);

    // Deselect SPI device and MUX
    output_high(CS_PIN_MISSION);
    output_high(MX_PIN_ADCS);

    // Wait for the erase operation to complete
    delay_ms(10);
}
 
 void WRITE_DATA_NBYTES(unsigned int32 ADDRESS, unsigned int8* data[], unsigned char data_number) {
     fprintf(EXT,"WRITE ADDRESS: 0x%08lx\n", ADDRESS);  // Print address as hex
     unsigned int8 adsress[4];
     // Byte extraction for a 32-bit address
     adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
     adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
     adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
     adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);
     WRITE_ENABLE_OF();  // Enable write operation
 
     // Lower CS to select the SPI device
     output_low(CS_PIN_1);
     delay_us(2);  // Small delay for stabilization
     // Send WRITE command and address
     spi_xfer(SPIPORT, WRITE_PAGE);
     spi_xfer(SPIPORT, adsress[0]);
     spi_xfer(SPIPORT, adsress[1]);
     spi_xfer(SPIPORT, adsress[2]);
     spi_xfer(SPIPORT, adsress[3]);
     // Write data bytes
     for (int i = 0; i < data_number; i++) {
         spi_xfer(SPIPORT, data[i]);  // Send data byte
         fprintf(EXT,"%02c", data[i]);    // Print each byte as hex (optional)
     }
 //    for (int i = 0; i < data_number; i++) {
 //        spi_xfer(SPIPORT, data[i]);  // Send data byte
 //        fprintf(EXT,"%02d", data[i]);    // Print each byte as hex (optional)
 //    } for futhre use this is for displaying in hex format 
     
     output_high(CS_PIN_1);  // Deselect SPI device
     
     fprintf(EXT,"\n%d BYTES WRITTEN IN MAIN!\n", data_number);
     return;
 }
 
 
 void WRITE_DATA_NBYTES_COM(unsigned int32 ADDRESS, unsigned int8 data[], unsigned char data_number) {
     fprintf(EXT,"WRITE ADDRESS IN COM: 0x%08lx\n", ADDRESS);  // Print address as hex
     unsigned int8 adsress[4];
     // Byte extraction for a 32-bit address
     adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
     adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
     adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
     adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);
     WRITE_ENABLE_OF_COM();  // Enable write operation
 
     // Lower MX to connect to flash device
     output_low(MX_PIN_COM);
     // Lower CS to select the SPI device
     output_low(CS_PIN_COM);
     delay_us(2);  // Small delay for stabilization
     // Send WRITE command and address
     spi_xfer(SPIPORT2, WRITE_PAGE);
     spi_xfer(SPIPORT2, adsress[0]);
     spi_xfer(SPIPORT2, adsress[1]);
     spi_xfer(SPIPORT2, adsress[2]);
     spi_xfer(SPIPORT2, adsress[3]);
     // Write data bytes
     for (int i = 0; i < data_number; i++) {
         spi_xfer(SPIPORT2, data[i]);  // Send data byte
         fprintf(EXT,"%02c", data[i]);    // Print each byte as hex (debugging purpose)
     }
     
     output_high(CS_PIN_COM);  // Deselect SPI devices
     output_high(MX_PIN_COM);  //Deselect MUX from flash
     
     fprintf(EXT,"\n%d BYTES WRITTEN IN COM!\n", data_number);
 }
 
 void WRITE_DATA_NBYTES_ADCS(unsigned int32 ADDRESS, unsigned int8 data[], unsigned char data_number) {
     fprintf(EXT,"WRITE ADDRESS IN ADCS: 0x%08lx\n", ADDRESS);  // Print address as hex
     unsigned int8 adsress[4];
     // Byte extraction for a 32-bit address
     adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
     adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
     adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
     adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);
     WRITE_ENABLE_OF_ADCS();  // Enable write operation and MX and CS pins are included in here 
 
     //Lower MX to connect to flash device
     output_low(MX_PIN_ADCS);
     // Lower CS to select the SPI device
     output_low(CS_PIN_MISSION);
     delay_us(2);  // Small delay for stabilization
     // Send WRITE command and address
     spi_xfer(SPIPORT3, WRITE_PAGE);
     spi_xfer(SPIPORT3, adsress[0]);
     spi_xfer(SPIPORT3, adsress[1]);
     spi_xfer(SPIPORT3, adsress[2]);
     spi_xfer(SPIPORT3, adsress[3]);
     // Write data bytes
     for (int i = 0; i < data_number; i++) {
         spi_xfer(SPIPORT3, data[i]);  // Send data byte
         fprintf(EXT,"%02c", data[i]);    // Print each byte as hex (debugging purpose)
     }
     
     output_high(CS_PIN_MISSION);  // Deselect SPI device5
     output_high(MX_PIN_ADCS);  //Deselect MUX from flash
     
     fprintf(EXT,"\n%d BYTES WRITTEN IN ADCS!\n", data_number);
 }
  
 
 char* READ_DATA_NBYTES(unsigned int32 ADDRESS, unsigned short data_number) {
     unsigned int8 adsress[4];
     unsigned char Data_return[256];  
 
     // Byte extraction for a 32-bit address
     adsress[0] = (unsigned int8)((ADDRESS >> 24) & 0xFF);
     adsress[1] = (unsigned int8)((ADDRESS >> 16) & 0xFF);
     adsress[2] = (unsigned int8)((ADDRESS >> 8) & 0xFF);
     adsress[3] = (unsigned int8)(ADDRESS & 0xFF);
 
     output_low(CS_PIN_1);  // Select SPI device
 
     // Send READ DATA COMMAND (0x13 or appropriate for your flash chip)
     spi_xfer(SPIPORT, READ_DATA_BYTES);
     // Send address bytes
     spi_xfer(SPIPORT, adsress[0]);
     spi_xfer(SPIPORT, adsress[1]);
     spi_xfer(SPIPORT, adsress[2]);
     spi_xfer(SPIPORT, adsress[3]);
 
     // Read the requested number of bytes
     for (int i = 0; i < data_number && i < 256; i++) {  // Avoid overflow
         Data_return[i] = spi_xfer(SPIPORT, 0x00);  // Send dummy byte to receive data
         fprintf(EXT, "%c", Data_return[i]);  // Print each byte as hex
     }
 
     output_high(CS_PIN_1);  // Deselect SPI device after reading
     fprintf(EXT, "\n");
 
     return Data_return;
 }
 
 
 
 char* READ_DATA_NBYTES_COM(unsigned int32 ADDRESS, unsigned short data_number) {
     unsigned int8 adsress[4];
     unsigned char Data_return[256];  // 
     
     // Byte extraction for a 32-bit address
     adsress[0] = (unsigned int8)((ADDRESS >> 24) & 0xFF);
     adsress[1] = (unsigned int8)((ADDRESS >> 16) & 0xFF);
     adsress[2] = (unsigned int8)((ADDRESS >> 8) & 0xFF);
     adsress[3] = (unsigned int8)(ADDRESS & 0xFF);
 
     output_low(MX_PIN_COM);  // Lower MX to connect to flash device
     output_low(CS_PIN_COM);  // Select SPI device
 
     // Send READ DATA COMMAND
     spi_xfer(SPIPORT2, READ_DATA_BYTES);
     // Send address bytes
     spi_xfer(SPIPORT2, adsress[0]);
     spi_xfer(SPIPORT2, adsress[1]);
     spi_xfer(SPIPORT2, adsress[2]);
     spi_xfer(SPIPORT2, adsress[3]);
 
     // Read the requested number of bytes
     for (int i = 0; i < data_number && i < 256; i++) {
         Data_return[i] = spi_xfer(SPIPORT2, 0x00);  // Send dummy byte to receive data
         fprintf(EXT, "%c", Data_return[i]);         // Print each byte
     }
 
     output_high(CS_PIN_COM);  // Deselect SPI device
     output_high(MX_PIN_COM);  // Deselect MUX from flash
     fprintf(EXT, "\n");
 
     return Data_return;
 }

#define MAX_DATA_SIZE 256  // Define the maximum data size

unsigned int8 Data_return[MAX_DATA_SIZE];  // Static buffer to hold the data

unsigned int8* READ_DATA_NBYTES_ADCS(unsigned int32 ADDRESS, unsigned short data_number) {
    unsigned int8 adsress[4];

    // Ensure data_number does not exceed the maximum buffer size
    if (data_number > MAX_DATA_SIZE) {
        data_number = MAX_DATA_SIZE;
    }

    // Byte extraction for a 32-bit address
    adsress[0] = (unsigned int8)((ADDRESS >> 24) & 0xFF);
    adsress[1] = (unsigned int8)((ADDRESS >> 16) & 0xFF);
    adsress[2] = (unsigned int8)((ADDRESS >> 8) & 0xFF);
    adsress[3] = (unsigned int8)(ADDRESS & 0xFF);

    output_low(MX_PIN_ADCS);  // Lower MX to connect to flash device
    output_low(CS_PIN_MISSION);  // Select SPI device

    // Send READ DATA COMMAND
    spi_xfer(SPIPORT3, READ_DATA_BYTES);
    // Send address bytes
    spi_xfer(SPIPORT3, adsress[0]);
    spi_xfer(SPIPORT3, adsress[1]);
    spi_xfer(SPIPORT3, adsress[2]);
    spi_xfer(SPIPORT3, adsress[3]);

    // Read the requested number of bytes
    for (int i = 0; i < data_number; i++) {
        Data_return[i] = spi_xfer(SPIPORT3, 0x00);  // Send dummy byte to receive data
        fprintf(EXT, "  0x%02x", Data_return[i]);  // Print each byte (optional)
        fprintf(EXT, "\n");
    }

    output_high(CS_PIN_MISSION);  // Deselect SPI device
    output_high(MX_PIN_ADCS);  // Deselect MUX from flash
    fprintf(EXT, "\n");

    return Data_return;
}
 
 
 int8 READ_DATA_BYTES_ADCS(unsigned int32 ADDRESS) {
     unsigned int8 adsress[4];
     unsigned int8 Data_return;
    
     // Byte extraction for a 32-bit address
     adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
     adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
     adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
     adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);
 
     output_low(MX_PIN_ADCS);  // Lower MX to connect to flash device
     output_low(CS_PIN_MISSION);  // Select SPI device
 
     // Send READ DATA COMMAND (0x13 or appropriate for your flash chip)
     spi_xfer(SPIPORT3, READ_DATA_BYTES);
     // Send address bytes
     spi_xfer(SPIPORT3, adsress[0]);
     spi_xfer(SPIPORT3, adsress[1]);
     spi_xfer(SPIPORT3, adsress[2]);
     spi_xfer(SPIPORT3, adsress[3]);
     // Read the requested number of bytes
         Data_return = spi_xfer(SPIPORT3, 0x00);  // Send dummy byte to receive data
 
     output_high(CS_PIN_MISSION);  // Deselect SPI device after reading
     output_high(MX_PIN_ADCS);  //Deselect MUX from flash
     return Data_return;
 }
 void READ_CHIP_ID_OF() {
     int8 chip_id[8];
     output_low(CS_PIN_1);  // Lower the CS PIN
     spi_xfer(SPIPORT, READ_ID);  // READ ID COMMAND (0x9F)
     
     // Receive 8 bytes of chip ID
     for (int i = 0; i < 8; i++) {
         chip_id[i] = spi_xfer(SPIPORT, 0x00);  // Send dummy bytes to receive data
         fprintf(EXT, "%02X ", chip_id[i]);
     }
     fprintf(EXT,"\n");
 
     output_high(CS_PIN_1);  // Raise CS PIN back
 }
 void READ_CHIP_ID_OF_COM() {
     int8 chip_id[8];
     output_low(MX_PIN_COM);
     output_low(CS_PIN_COM);  // Lower the CS PIN
     spi_xfer(SPIPORT2, READ_ID);  // READ ID COMMAND (0x9F)
     
     // Receive 8 bytes of chip ID
     for (int i = 0; i < 8; i++) {
         chip_id[i] = spi_xfer(SPIPORT2, 0x00);  // Send dummy bytes to receive data
         fprintf(EXT, "%02X ", chip_id[i]);
     }
     fprintf(EXT,"\n");
 
     output_high(CS_PIN_COM);  // Raise CS PIN back
     output_high(MX_PIN_COM);
 }
 
 void READ_CHIP_ID_OF_ADCS() {
     int8 chip_id[8];
     output_low(MX_PIN_ADCS);
     output_low(CS_PIN_MISSION);  // Lower the CS PIN
     spi_xfer(SPIPORT3, READ_ID);  // READ ID COMMAND (0x9F)
     
     // Receive 8 bytes of chip ID
     for (int i = 0; i < 8; i++) {
         chip_id[i] = spi_xfer(SPIPORT3, 0x00);  // Send dummy bytes to receive data
         fprintf(EXT, "%02X ", chip_id[i]);
     }
     fprintf(EXT,"\n");
 
     output_high(CS_PIN_MISSION);  // Raise CS PIN back
     output_high(MX_PIN_ADCS);
 }
//  void READ_CHIP_ID_GENERIC(int SPIPORT, int CS_PIN, int MX_PIN) {
//     int8 chip_id[8];

//     // Lower MUX pin if applicable
//     if (MX_PIN != -1) {
//         output_low(MX_PIN);
//     }

//     // Lower CS to select the SPI device
//     output_low(CS_PIN);

//     // Send READ ID command
//     spi_xfer(SPIPORT, READ_ID);

//     // Receive 8 bytes of chip ID
//     for (int i = 0; i < 8; i++) {
//         chip_id[i] = spi_xfer(SPIPORT, 0x00);  // Send dummy bytes to receive data
//         fprintf(EXT, "%02X ", chip_id[i]);    // Print each byte in hex
//     }
//     fprintf(EXT, "\n");

//     // Raise CS to deselect the SPI device
//     output_high(CS_PIN);

//     // Raise MUX pin if applicable
//     if (MX_PIN != -1) {
//         output_high(MX_PIN);
//     }
// }
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //Command operation
 //void SEND_COMMAND_ADCS(void){
 //    WRITE_DATA_NBYTES_ADCS(ADCS_HK_ADDRESS, ADCS_COMMAND,1);
 //}
 ////void READ_HK_ADCS(void){
 ////    int8 state_of_pin = 0;
 ////    
 ////    state_of_pin = input_state(EN_SUP_3V3_2);
 ////    if(state_of_pin = FALSE){ 
 ////        READ_DATA_NBYTES_ADCS(ADCS_HK_ADDRESS[16], READ_HK_ADCS, 16);
 ////        
 ////    }
 ////}
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 ////EEPROM operation 
 ////thus functions used to store essential data order to prevent loosing it in unexpected shutdowns  
 //void EEPROM_
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Main menu functions
 
 void startup_freeze(){
     delay_ms(2000);
     fprintf(EXT, "POWER ON!\n");
     //EPS power control all disabled when startup, using menu function will turn on
     output_low(EN_SUP_3V3_1);
     output_low(EN_SUP_3V3_2 );
     output_low(EN_SUP_3V3_DAQ);
     output_low(EN_SUP_UNREG);
     output_low(EN_SUP_5V0);
     output_low(MVCAM_PWR);
     output_low(OVCAM_PWR);
     output_low(ADCS_PWR); //turns on the power of ADCS instantly 
     output_high(CS_PIN_1);
     output_high(CS_PIN_COM );
     output_high(CS_PIN_MISSION );
     output_high(MX_PIN_OVCAM );
     output_high(MX_PIN_MVCAM );
     output_high(MX_PIN_ADCS );
     output_high(MX_PIN_COM );
     
     fprintf(EXT, "Digital pin out configured \n");
         
 }
 //set RTCC functions counting to all zero 
 void set_clock(rtc_time_t &date_time){
 
    date_time.tm_year=0000;
    date_time.tm_mon=00;
    date_time.tm_mday=00;
    date_time.tm_wday=00;
    date_time.tm_hour=00;
    date_time.tm_min=00;
    date_time.tm_sec=0; 
 }

 void RTC_initialize(){
     setup_lcd(LCD_DISABLED);
     rtc_time_t write_clock, read_clock;
     setup_rtc(RTC_ENABLE | RTC_CLOCK_SOSC | RTC_CLOCK_INT, 0);
     set_clock(write_clock);
     rtc_write(&write_clock);
     rtc_read(&read_clock);
     read_clock.tm_year - 2000; 
     fprintf(EXT, "\r%02u/%02u/20%02u %02u:%02u:%02u", read_clock.tm_mon, read_clock.tm_mday, read_clock.tm_year , read_clock.tm_hour, read_clock.tm_min, read_clock.tm_sec);

     fprintf(EXT, "RTCC setup finished!\n");
     fprintf(EXT, "RTCC setup finished!\n");
 
 }
 
 //this function will receive from EPS and sent to external port of EXT single character by character   
 void uart_repeater() {
     int received_data;
     while (TRUE) {
         // Check if data is available on the EPS stream
         if (kbhit(EPS)) {
             // Read one byte from the EPS stream
             received_data = fgetc(EPS);
             // Send the received byte to the EXT stream
             fputc(received_data, EXT);
         }
     }
 }
 // Function to receive exactly 16 bytes from UART
 void receive_16_bytes(int8* buffer) {
     for (int8 i = 0; i < 16; i++) {
         buffer[i] = getc(EPS);  // Blocks until a byte is received
     }
 }
 
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //adcs command
 //
 //#define SHUTDOWN_COUNT_ADDRESS  0x00001000  // Address where shutdown count is stored
 // this function is not working correctly need to fix it 
 int8 *read_data_adcs; //ADCS flash received data will be stored in here 
int8 adcs_mission[3] = {0x33, 0x05, 0x05}; // mission command for ADCS all sensors will run for 3 minutes 
int8 adcs_mag[3] = {0x22, 0x01, 0x05}; // mag each 9 bytes for measurement
int8 adcs_gyro[3] = {0x11, 0x01, 0x05}; // gyro each 9 bytes for measurement 
int8 last_adcs[5] = {0x00, 0x00, 0x00, 0x00, 0x00}; // last command for ADCS 
                // (how many measurements are taken), (last address of the data)x4 not inverted
                // we will multiply measurement size and how many measurements are taken and subtract from the last address
                // to get the first measurement address of the data
                // later we will read the data from the first address to the last address
int8 adcs_status_address = 0x00020000; // status address of the adcs first byte is measrument count /each 9 byte/, last 4 is last address
int8 adcs_gyro_first_address = 0x00030000; // first address of the gyro data 
 void adcs_mission_mode(void){
     output_high(ADCS_PWR);
delay_ms(1000);
READ_CHIP_ID_OF_ADCS();
fprintf(EXT, "deleting command address of the adcs\n");
SECTOR_ERASE_OF_ADCS(0x00000000);
delay_ms(100);
//SECTOR_ERASE_OF_ADCS(0x00020000);
delay_ms(100);
fprintf(EXT, "Command address of the adcs is deleted!\n");
delay_ms(100);
fprintf(EXT, "Sending command to the adcs\n");

// Write and read from ADCS flash memory
fprintf(EXT, "Starting to write data in ADCS flash memory\n");
WRITE_DATA_NBYTES_ADCS(0x00000000, adcs_gyro, sizeof(adcs_gyro));
delay_ms(100);
read_data_adcs = READ_DATA_NBYTES_ADCS(0x00000000, sizeof(adcs_gyro));
delay_ms(100);
for (int i = 0; i < sizeof(adcs_gyro); i++) {
    fprintf(EXT, "%c", read_data_adcs[i]);
    delay_ms(2);
}
fprintf(EXT, "\n"); 
fprintf(EXT, "ADCS command is written!\n");

// Wait 70 seconds for ADCS to complete the mission
for (int i = 0; i < 7; i++) {
    fprintf(EXT, "waiting for adcs to finish 70s/%d0s\n", i);
    delay_ms(10000);
}
fprintf(EXT, "ADCS command is finished!\n");

// Read status and last address from ADCS
fprintf(EXT, "reading the status address of the adcs!\n");
read_data_adcs = READ_DATA_NBYTES_ADCS(0x00020000, 5); // Read 5 bytes: 1 for status, 4 for last address
delay_ms(100);

// Extract status and last address
int8 adcs_status = read_data_adcs[0];  // Number of measurements (status)
int32 last_adcs_address = 0;
for (int i = 1; i < 5; i++) {
    last_adcs_address = (last_adcs_address << 8) | (int8)read_data_adcs[i]; // Combine 4 bytes into a 32-bit address
}
fprintf(EXT, "ADCS status: %d, last address: 0x%08x\n", adcs_status, last_adcs_address);

// Handle mission status
if (adcs_status <= 0) {  // Assuming 0 or -1 indicates failure based on your original code
    fprintf(EXT, "ADCS mission failed!\n");
    while (TRUE) {
        fprintf(EXT, "ADCS mission failed!\n");
        delay_ms(5000);
    }
} else {
    fprintf(EXT, "ADCS mission success with %d measurements!\n", adcs_status);

    // Calculate the first data address
    int32 first_adcs_data_address = last_adcs_address - (adcs_status * 9); // Each measurement is 9 bytes
    fprintf(EXT, "First data address: 0x%08x\n", first_adcs_data_address);

    // Read and print each measurement
    for (int i = 0; i < adcs_status; i++) {
        int32 current_address = first_adcs_data_address + (i * 9);
        read_data_adcs = READ_DATA_NBYTES_ADCS(current_address, 9); // Read 9 bytes per measurement
        fprintf(EXT, "Measurement %d at address 0x%08x: ", i + 1, current_address);
        for (int j = 0; j < 9; j++) {
            fprintf(EXT, "%02x ", (int8)read_data_adcs[j]); // Print each byte in hex
        }
        fprintf(EXT, "\n");
        delay_ms(2); // Small delay between reads
    }
}
output_low(ADCS_PWR);
 }
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // eps functions 
 char cmd = 'G'; // Command character to trigger EPS
unsigned int8 temperature[7];
unsigned int8 solar_panel_current[5];
unsigned int8 solar_panel_voltage[5];
unsigned int8 sc_pwr;
unsigned int8 sc_i_sens;
unsigned int8 batt_i_sens;
unsigned int8 batt_pwr;
unsigned int8 raw_i_sens;
unsigned int8 raw_v_sens;
unsigned int8 batt_temp;
unsigned int8 DCDC_output_i[5];
bool KS_OBC_STAT;
bool KS_EPS_STAT;

// Function to receive 33 bytes from EPS into the buffer
void get_data(char* buffer) {
    putc(cmd, EPS); // Send command to EPS
    for (int8 i = 0; i < 33; i++) {
        buffer[i] = fgetc(EPS); // Receive byte from EPS (blocks until received)
    }
}

// Function to send 33 bytes to EXT
void send_data(char* buffer) {
    for (int8 i = 0; i < 33; i++) {
        putc(buffer[i], EXT); // Send each byte to EXT
    }
    printf(EXT, "\n"); // Newline after sending all bytes
}
// Function to store data
void store_eps_data(unsigned int8* buffer) {
    if (buffer[0] == '}' && buffer[32] == 'f') {
        memcpy(temperature, &buffer[1], 7);
        memcpy(solar_panel_current, &buffer[8], 5);
        memcpy(solar_panel_voltage, &buffer[13], 5);
        sc_pwr = buffer[18];
        sc_i_sens = buffer[19];
        batt_i_sens = buffer[20];
        batt_pwr = buffer[21];
        raw_i_sens = buffer[22];
        raw_v_sens = buffer[23];
        batt_temp = buffer[24];
        memcpy(DCDC_output_i, &buffer[25], 5);
        KS_OBC_STAT = buffer[30] != 0;
        KS_EPS_STAT = buffer[31] != 0;
    }
}
void print_table() {
    printf(EXT, "%-15s | %-8s | %-12s\n", "Variable", "Original", "Processed (V/A)");
    printf(EXT, "--------------------------------------------------\n");
    printf(EXT, "%-15s | %-8u | %.6f\n", "sc_pwr", sc_pwr, ((sc_pwr * 3.3) / 256.0) / 20.0);
    printf(EXT, "%-15s | %-8u | %.6f\n", "sc_i_sens", sc_i_sens, ((sc_i_sens * 3.3) / 256.0) / 20.0);
    printf(EXT, "%-15s | %-8u | %.6f\n", "raw_i_sens", raw_i_sens, ((raw_i_sens * 3.3) / 256.0) / 20.0);
    printf(EXT, "%-15s | %-8u | %.6f\n", "raw_v_sens", raw_v_sens, ((raw_v_sens * 3.3) / 256.0) / 20.0);
    printf(EXT, "%-15s | %-8u | %.6f\n", "batt_pwr", batt_pwr, ((batt_pwr * 3.3) / 256.0) / 20.0);
    printf(EXT, "%-15s | %-8u | %.6f\n", "batt_i_sens", batt_i_sens, ((batt_i_sens * 3.3) / 256.0) / 20.0);
    printf(EXT, "\n"); // Add a newline after the table
    printf(EXT, "KS of OBC status is %d", KS_OBC_STAT);
    printf(EXT, "KS of EPS status is %d", KS_EPS_STAT);
}
// Main function to test UART communication
void eps_mission_mode(void) {
    output_high(EN_SUP_3V3_DAQ); // Enable 3.3V supply for DAQ
    printf(EXT, "EPS power on!\n");  // Initial message to EXT

    char buffer[33]; // Buffer to hold 33 bytes

        printf(EXT, "Sending request...\n");
        get_data(buffer);          // Get data from EPS
        store_eps_data(buffer);    // Parse buffer into variables
        print_table();             // Print the table
        send_data(buffer);         // Forward data to EXT
    
}
 
 int8 update_shutdown_count(void) {
     fprintf(EXT, "Shutdown count started\n");
 
     // Read shutdown count directly from memory
     unsigned char shutdown_count[1];
     shutdown_count[0] = READ_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, 1); // Updated call
     delay_ms(10);
 
     fprintf(EXT, "Read shutdown count: %u\n", shutdown_count[0]);
 
     shutdown_count[0]++;  // Increment the shutdown count
     fprintf(EXT, "Incremented shutdown count: %u\n", shutdown_count[0]);
 
     // Write the updated shutdown count back to memory
     WRITE_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, shutdown_count, 1);
     delay_ms(10);
 
     fprintf(EXT, "Now shutdown count is : %u\n\n", shutdown_count[0]);
 
     return shutdown_count[0];
 }
 
 
 //main flash memory consol for main_menu() function
 void write_to_main_flash_menu(){
             unsigned int32 address;
             unsigned int8 data[256]; // Buffer for data to be written (adjust size as needed)
             unsigned char data_number;
             unsigned char choice;
             unsigned int8 i;
             // Prompt user to enter the address
             fprintf(EXT, "\nEnter Address (hex, 0x1234): 0x");
             fscanf(EXT, "%x", &address); // Read address input in hex
 
             // Prompt user to enter the number of bytes
             fprintf(EXT, "\nEnter number of bytes to write (max 256): ");
             fscanf(EXT, "%x", &data_number); // Read number of bytes
 
//             if (data_number > 256) {
//                 fprintf(EXT, "Error: Maximum data length is 256 bytes.\n");
//                 continue;
//             }
 
             // Get data from user
             fprintf(EXT, "Enter %d bytes of data (in hex):\n", data_number);
             for (i = 0; i < data_number; i++) {
                 fprintf(EXT, "Byte %d: 0x", i + 1);
                 fscanf(EXT, "%x", &data[i]); // Read byte in hex format
             }
 
             // Call the function to write data to the address
             WRITE_DATA_NBYTES(address, data, data_number);
 
             fprintf(EXT, "\nData successfully written.\n");
 }
 //main flash memory consol for main_menu() function
 void handle_main_flash_memory() {
     char main_flash_option;
     unsigned int32 address;
     unsigned char data[32];
     unsigned char data_length;
 
     fprintf(EXT, "MAIN flash memory chosen\n");
     fprintf(EXT, "press a: Read ID of the chip\n");
     fprintf(EXT, "press b: Write data set in specified address\n");
     fprintf(EXT, "press c: Read data set in specified address\n");
     fprintf(EXT, "press x: Return to MAIN MENU\n");
 
     main_flash_option = fgetc(EXT);
 
     switch (main_flash_option) {
         case 'a':
             fprintf(EXT, "Started reading chip ID of MAIN flash memory\n");
             READ_CHIP_ID_OF();  
             break;
         case 'b':
             write_to_main_flash_menu();
             break;
         case 'c':
             fprintf(EXT, "Read data set in specified address\n");
             fprintf(EXT, "Enter your specified address and length (e.g., 0x1234 10): ");
             if (scanf("%x %d", &address, &data_length)) {
                 fprintf(EXT, "Address: 0x%09x, Length: %d\n", address, data_length);
                 READ_DATA_NBYTES(address, data_length);  // Replace with actual function
             } else {
                 fprintf(EXT, "Invalid input. Please enter a valid address and length.\n");
             }
             break;
 
         case 'x':
             return;
         default:
             fprintf(EXT, "Invalid MAIN flash memory option. Please try again.\n");
             break;
     }
 }
 //main flash memory consol for main_menu() function
 void handle_flash_memories() {
     char flash_option;
     fprintf(EXT, "pressed option d: Check Flash Memories\n\n");
     fprintf(EXT, "Please choose which flash memory to work on (a, b, c, d, e):\n");
     fprintf(EXT, "press a: MAIN flash memory\n");
     fprintf(EXT, "press b: COM shared flash memory\n");
     fprintf(EXT, "press c: ADCS shared flash memory\n");
     fprintf(EXT, "press d: OVCAM shared flash memory\n");
     fprintf(EXT, "press e: MVCAM shared flash memory\n");
     fprintf(EXT, "press x: Return to MAIN MENU\n");
 
     flash_option = fgetc(EXT);
 
     switch (flash_option) {
         case 'a':
             fprintf(EXT, "MAIN shared flash memory chosen\n");
             handle_main_flash_memory();
             break;
         case 'b':
             fprintf(EXT, "COM shared flash memory chosen\n");
             // Implement COM shared flash memory handling
             break;
         case 'c':
             fprintf(EXT, "ADCS shared flash memory chosen\n");
             // Implement ADCS shared flash memory handling
             break;
          case 'd':
             fprintf(EXT, "OVCAM shared flash memory chosen\n");
             // Implement ADCS shared flash memory handling
             break;
          case 'e':
             fprintf(EXT, "MVCAM shared flash memory chosen\n");
             // Implement ADCS shared flash memory handling
             break;
          case'x':
              break;
         default:
             fprintf(EXT, "Invalid flash memory option. Please try again.\n");
             break;
     }
 }
 
 //main RTCC functions consol for main_menu() function
 void handle_set_time() {
     char handle_set_time_option;
     fprintf(EXT, "Settings of RTC chosen\n");
     fprintf(EXT, "    press a: to reset the RTC /all current time will be set zero/\n");
     fprintf(EXT, "    press b: display current time\n");
     fprintf(EXT, "    press c: display current time nonstop\n");
     handle_set_time_option = fgetc(EXT);
 
     switch (handle_set_time_option) {
         case 'a':
     rtc_time_t write_clock, read_clock;
     rtc_read(&read_clock);
     fprintf(EXT, "Now time is\n");
     fprintf(EXT, "\r%02u/%02u/20%02u %02u:%02u:%02u", read_clock.tm_mon, read_clock.tm_mday, read_clock.tm_year, read_clock.tm_hour, read_clock.tm_min, read_clock.tm_sec);
     fprintf(EXT, "Time changing function activated\n");
     set_clock(write_clock);
     rtc_write(&write_clock);
     fprintf(EXT, "Time successfully changed. Current time is:\n");
     rtc_read(&read_clock);
     fprintf(EXT, "\r%02u/%02u/20%02u %02u:%02u:%02u", read_clock.tm_mon, read_clock.tm_mday, read_clock.tm_year, read_clock.tm_hour, read_clock.tm_min, read_clock.tm_sec);
     break;
         case 'b':
             rtc_read(&read_clock);
     fprintf(EXT, "\r%02u/%02u/20%02u %02u:%02u:%02u", read_clock.tm_mon, read_clock.tm_mday, read_clock.tm_year, read_clock.tm_hour, read_clock.tm_min, read_clock.tm_sec);
     break;
         case 'c':
                 while(true){
                     rtc_read(&read_clock);
                     fprintf(EXT, "Now time is\n");
                     fprintf(EXT, "\r%02u/%02u/20%02u %02u:%02u:%02u\n", read_clock.tm_mon, read_clock.tm_mday, read_clock.tm_year, read_clock.tm_hour, read_clock.tm_min, read_clock.tm_sec);   
                     delay_ms(1000);
                 }
             break;
         case 'x':
             break;
             return;
         default:
             fprintf(EXT, "Invalid IO option. Please try again.\n");
     break;
             
 }
 }
 //main IO control consol for main_menu() function
 void handle_io_control() {
     char io_option;
     int8 state_of_pin;
 
     fprintf(EXT, "IO control chosen\n");
 
     // Check and display the state of each pin before providing options
     state_of_pin = input_state(EN_SUP_3V3_1);
     fprintf(EXT, "    press a: Toggle EN_SUP_3V3_1 /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
     state_of_pin = input_state(EN_SUP_3V3_2);
     fprintf(EXT, "    press b: Toggle EN_SUP_3V3_2 /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
      state_of_pin = input_state(EN_SUP_3V3_DAQ);
     fprintf(EXT, "    press c: Toggle EN_SUP_3V3_DAQ /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
     state_of_pin = input_state(EN_SUP_UNREG);
     fprintf(EXT, "    press d: Toggle EN_SUP_UNREG /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
     state_of_pin = input_state(EN_SUP_5V0);
     fprintf(EXT, "    press e: Toggle EN_SUP_5V0 /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
     state_of_pin = input_state(KILL_SWITCH);
     fprintf(EXT, "    press f: Toggle KILL_SWITCH /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
         state_of_pin = input_state(MVCAM_PWR);
     fprintf(EXT, "    press g: Toggle MVCAM_PWR /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
         state_of_pin = input_state(OVCAM_PWR);
     fprintf(EXT, "    press h: Toggle OVCAM_PWR /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
         state_of_pin = input_state(ADCS_PWR);
     fprintf(EXT, "    press i: Toggle ADCS_PWR /is currently/");
     if(state_of_pin == 1 ){
         fprintf(EXT, "HIGH\n");
     }else if(state_of_pin == 0){
         fprintf(EXT, "LOW\n");
     }else {
         fprintf(EXT, "Invalid\n"); 
     }
     fprintf(EXT, "    press j: Toggle all Pins");
     
 
     io_option = fgetc(EXT);
 
     switch (io_option) {
         case 'a':
             output_toggle(EN_SUP_3V3_1);
             break;
         case 'b':
             output_toggle(EN_SUP_3V3_2);
             break;
         case 'c':
             output_toggle(EN_SUP_3V3_DAQ);
             break;
         case 'd':
             output_toggle(EN_SUP_UNREG);
             break;
         case 'e':
             output_toggle(EN_SUP_5V0);
             break;
         case 'f':
             output_toggle(KILL_SWITCH);
             break;
         case 'g':
             output_toggle(MVCAM_PWR);
             break;
         case 'h':
             output_toggle(OVCAM_PWR);        
             break;
         case 'i':
             output_toggle(ADCS_PWR);        
             break;    
         case 'j' :
             output_toggle(OVCAM_PWR);
             output_toggle(MVCAM_PWR);  
             output_toggle(KILL_SWITCH);
             output_toggle(EN_SUP_5V0);
             output_toggle(EN_SUP_UNREG);
             output_toggle(EN_SUP_3V3_DAQ);
             output_toggle(EN_SUP_3V3_2);
             output_toggle(EN_SUP_3V3_1);
             break;
         case 'k' :
             output_toggle(MX_PIN_COM);
             break;
         case 'x':
             break;
             return;
         default:
             fprintf(EXT, "Invalid IO option. Please try again.\n");
             break;
     }
 }
 void testmode(void){
    fprintf(EXT, "    press a: ADCS mission mode\n");
    fprintf(EXT, "    press b: EPS mission mode\n");
    fprintf(EXT, "    press c: Check Flash Memories\n");
          //fprintf(EXT, "    press e: See satellite Log\n");
          char io_option;
            io_option = fgetc(EXT);
            switch (io_option) {
              case 'a':
                fprintf(EXT, "ADCS mission mode\n");
                adcs_mission_mode();
                break;
                case 'b':
                fprintf(EXT, "EPS mission mode\n");
                eps_mission_mode();
                break;
                case 'c':
                handle_flash_memories();
                break;
                default:
                fprintf(EXT, "Invalid IO option. Please try again.\n");
                break;
   }
 }
         
void main_menu(void) {
    char option;
  fprintf(EXT, " __  __ _____ _   _ _   _   _____                 _   _             \n");
  fprintf(EXT, "|  \\/  | ____| \\ | | | | | |  ___|   _ _ __   ___| |_(_) ___  _ __  \n");
  fprintf(EXT, "| |\\/| |  _| |  \\| | | | | | |_ | | | | '_ \\ / __| __| |/ _ \\| '_ \\ \n");
  fprintf(EXT, "| |  | | |___| |\\  | |_| | |  _|| |_| | | | | (__| |_| | (_) | | | |\n");
  fprintf(EXT, "|_| _|_|_____|_| \\_|\\___/  |_|_  \\__,_|_| |_|\\___|\\__|_|\\___/|_| |_|\n");
  fprintf(EXT, "   / \\   ___| |_(_)_   ____ _| |_ ___  __| | |                      \n");
  fprintf(EXT, "  / _ \\ / __| __| \\ \\ / / _` | __/ _ \\/ _` | |                      \n");
  fprintf(EXT, " / ___ \\ (__| |_| |\\ V / (_| | ||  __/ (_| |_|                      \n");
  fprintf(EXT, "/_/   \\_\\___|\\__|_| \\_/ \\__,_|\\__\\___|\\__,_(_)                      \n");
    
    while (1) {
        // Display Main Menu
        fprintf(EXT, "\n-----------------Main Menu-----------------\n");
        //fprintf(EXT, "    press a: Get House keeping data\n");
        fprintf(EXT, "    press b: EPS Power output control\n");
        fprintf(EXT, "    press c: House keeping data collection\n");
        fprintf(EXT, "    press d: Check Flash Memories\n");
        //fprintf(EXT, "    press e: See satellite Log\n");
        fprintf(EXT, "    press f: Settings of RTC\n");
        //fprintf(EXT, "    press g: Satellite log down-link command\n");
        //fprintf(EXT, "    press h: IHC Mission start\n");
        //fprintf(EXT, "    press i: SEL current Measurement\n");
        //fprintf(EXT, "    press j: H8 COM Reset\n");
        fprintf(EXT, "    press k: UART repeater of EPS\n");
        fprintf(EXT, "    press l: testmode \n");
        fprintf(EXT, "    press x: Exit Main Menu\n");
        fprintf(EXT, "    DO NOT USE CAPITAL CHARACTERS TO WRITE!\n\n");

        // Read the user's choice
        option = fgetc(EXT);

        // Main menu switch
        switch (option) {
            case 'a':
                // Call a function to get housekeeping data
                // get_housekeeping_data();
                break;
            case 'b':
                // Call a function to control EPS power output
                handle_io_control();
                break;
            case 'c':
                // Call a function to collect housekeeping data
                // collect_housekeeping_data();
                break;
            case 'd':
                handle_flash_memories();
                break;
            case 'e':
                // Call a function to see satellite log
                // see_satellite_log();
                break;
            case 'f':
                handle_set_time();
                break;
            case 'g':
                // Call a function for satellite log downlink command
                // satellite_log_downlink_command();
                break;
            case 'h':
                // Call a function for IHC mission start
                // ihc_mission_start();
                break;
            case 'i':
                // Call a function for SEL current measurement
                // sel_current_measurement();
                break;
            case 'j':
                // Call a function for H8 COM reset
                // h8_com_reset();
                break;
            case 'k':
                fprintf(EXT, "UART Repeater Initialized.\n");
                uart_repeater();
                break;
            case 'l':
                fprintf(EXT, "Testmode initialized\n");
                testmode();
                break;
                
            case 'x':
                return;
            default:
                fprintf(EXT, "Invalid option. Please try again.\n");
                break;
        }
    }
}
//void receive_16_bytes(int16* buffer) {
//    for (int8 i = 0; i < 33; i++) {
//        buffer[i] = getc(EPS);  // Blocks until a byte is received
//    }
//}
//void EPS_DATA_ACQUISITION (){
//        output_high(EN_SUP_3V3_DAQ);
//    printf(EXT,"Power on!\n");
//    int16 buffer[33];  // Buffer to hold the 16 bytes
//    while (1) {
//        printf(EXT,"sending request...\n");
//        putc(cmd, EPS);
//        // Receive 16 bytes into the buffer
//        printf(EXT,"Waiting to respond...\n");
//        receive_16_bytes(buffer);
//        // Send the 16 bytes back via UART
//        for (int8 i = 0; i < 33; i++) {
//            putc(buffer[i], EXT);
//        }
//        printf("\n");
//    }
//}

#include <flashoperation.h>



 
 #ifdef	__cplusplus
 }
 #endif
 
 #endif	/* MAIN_H */
 
 