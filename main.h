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
 #device ADC=16
 #device ICD=TRUE
 #include <string.h>
#include <stdbool.h>
//#include <mt25q.h>
// #include <main_functions.h>
 #FUSES NOWDT NOBROWNOUT 
 #use delay(clock=16M, crystal)
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 #use rs232(baud=9600, parity=N, xmit=PIN_E5, rcv=PIN_E4, bits=8, stream=EPS) //EPS DATA ACQUISITION
 #use rs232(baud=57600, parity=N, xmit=PIN_C6, rcv=PIN_C7, bits=8, stream=EXT) //MAIN RAB Rear access board 
 #use rs232(baud=57600, parity=N, xmit=PIN_D2, rcv=PIN_D3, bits=8, stream=COM) //MAIN COM Communication, send CW data 
 #use rs232(baud=9600, parity=N, xmit=PIN_F6, rcv=PIN_F7, bits=8, stream=CAM) //MAIN CAM Communication that will sent camara data ov5642 
 #use rs232(baud=9600, parity=N, xmit=PIN_B7, rcv=PIN_B6, bits=8, stream=ADCS) //ADCS communication port
 #use spi(MASTER, CLK=PIN_E1, DI=PIN_E0, DO=PIN_E6,  BAUD=10000, BITS=8, STREAM=MAIN_FM, MODE=0) //MAIN flash memory port
 #use spi(MASTER, CLK=PIN_B2, DI=PIN_B5, DO=PIN_B4,  BAUD=10000, BITS=8, STREAM=COM_FM, MODE=0) //COM shared flash memory port
 #use spi(MASTER, CLK=PIN_A3, DI=PIN_A0, DO=PIN_A1,  BAUD=10000, BITS=8, STREAM=MISSION_FM, MODE=0) //ADCS shared flash memory port, Camera module (ovcam,mvcam) only can access via mux selcent
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     


 //Flash memory chip select pins and mux control 
 #define CS_PIN_MAIN PIN_E2 //OBC_FLASH_SELECT
 #define CS_PIN_COM PIN_B3 //COM_CHIP_SELECT
 #define CS_PIN_MISSION PIN_A2 //MISSION_CHIP_SELECT ADCS and CAMERA module shared flash memory ALL CONNECTED TO THIS CS PIN AND MANAGED BY MUX
 #define MX_PIN_FM2 PIN_G2 //OVCAM_MUX_SELECT
 #define MX_PIN_FM1 PIN_G3 //MVCAM_MUX_SELECT
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


    unsigned int16 hk_gyro_x = 0;
    unsigned int16 hk_gyro_y = 0;
    unsigned int16 hk_gyro_z = 0;
    unsigned int16 hk_mag_x = 0;
    unsigned int16 hk_mag_y = 0;
    unsigned int16 hk_mag_z = 0;
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
  output_low(CS_PIN_MAIN);
  spi_xfer(MAIN_FM,ENABLE_WRITE);                //Send 0x06
  output_high(CS_PIN_MAIN);  
  return;
 }
 
 void WRITE_ENABLE_OF_COM(){
      // Lower CS to select the SPI device
     output_low(CS_PIN_COM);
     // Lower MX to connect to flash device
     output_low(MX_PIN_COM);
     spi_xfer(COM_FM,ENABLE_WRITE);                //Send 0x06
     output_high(CS_PIN_COM);
     output_high(MX_PIN_COM);
  return;
 }
 
 void WRITE_ENABLE_OF_ADCS(){
     // Lower MX to connect to flash device
     output_low(MX_PIN_ADCS);
      // Lower CS to select the SPI device
     output_low(CS_PIN_MISSION);
     spi_xfer(MISSION_FM,ENABLE_WRITE);                //Send 0x06
     output_high(CS_PIN_MISSION);  
     output_high(MX_PIN_ADCS);
  return;
 }
 void WRITE_ENABLE_OF_IHS(unsigned int8 MX_PIN){
    // lower MX to connect to flash device
    output_low(MX_PIN);
    // LowerCS pin to activate the flash device
    output_low(CS_PIN_MISSION);
    spi_xfer(MISSION_FM,ENABLE_WRITE);                //Send 0x06
    output_high(CS_PIN_MISSION);
    output_high(MX_PIN);
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
        spi_xfer(MAIN_FM, ENABLE_WRITE);  // Send ENABLE_WRITE command
    } else if(STREAM == 2) {
        spi_xfer(COM_FM, ENABLE_WRITE);  // Send ENABLE_WRITE command
    } else if(STREAM == 3) {
        spi_xfer(MISSION_FM, ENABLE_WRITE);  // Send ENABLE_WRITE command
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
    spi_xfer(MISSION_FM, ERASE_SECTOR);
    spi_xfer(MISSION_FM, address[0]);
    spi_xfer(MISSION_FM, address[1]);
    spi_xfer(MISSION_FM, address[2]);
    spi_xfer(MISSION_FM, address[3]);

    // Deselect SPI device and MUX
    output_high(CS_PIN_MISSION);
    output_high(MX_PIN_ADCS);

    // Wait for the erase operation to complete
    delay_ms(10);
}


void SECTOR_ERASE_OF_IHS(unsigned int32 sector_address , unsigned int8 MX_PIN) {
    unsigned int8 address[4];
    // Byte extraction for a 32-bit address
    address[0] = (unsigned int8)((sector_address >> 24) & 0xFF);
    address[1] = (unsigned int8)((sector_address >> 16) & 0xFF);
    address[2] = (unsigned int8)((sector_address >> 8) & 0xFF);
    address[3] = (unsigned int8)(sector_address & 0xFF);

    // Enable write operation
    WRITE_ENABLE_OF_IHS(MX_PIN);

    // Lower MX to connect to flash device
    output_low(MX_PIN);
    // Lower CS to select the SPI device
    output_low(CS_PIN_MISSION);
    delay_us(2);  // Small delay for stabilization

    // Send ERASE command and address
    spi_xfer(MISSION_FM, ERASE_SECTOR);
    spi_xfer(MISSION_FM, address[0]);
    spi_xfer(MISSION_FM, address[1]);
    spi_xfer(MISSION_FM, address[2]);
    spi_xfer(MISSION_FM, address[3]);

    // Deselect SPI device and MUX
    output_high(CS_PIN_MISSION);
    output_high(MX_PIN);

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
     output_low(CS_PIN_MAIN);
     delay_us(2);  // Small delay for stabilization
     // Send WRITE command and address
     spi_xfer(MAIN_FM, WRITE_PAGE);
     spi_xfer(MAIN_FM, adsress[0]);
     spi_xfer(MAIN_FM, adsress[1]);
     spi_xfer(MAIN_FM, adsress[2]);
     spi_xfer(MAIN_FM, adsress[3]);
     // Write data bytes
     for (int i = 0; i < data_number; i++) {
         spi_xfer(MAIN_FM, data[i]);  // Send data byte
         fprintf(EXT,"%02c", data[i]);    // Print each byte as hex (optional)
     }
 //    for (int i = 0; i < data_number; i++) {
 //        spi_xfer(MAIN_FM, data[i]);  // Send data byte
 //        fprintf(EXT,"%02d", data[i]);    // Print each byte as hex (optional)
 //    } for futhre use this is for displaying in hex format 
     
     output_high(CS_PIN_MAIN);  // Deselect SPI device
     
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
     spi_xfer(COM_FM, WRITE_PAGE);
     spi_xfer(COM_FM, adsress[0]);
     spi_xfer(COM_FM, adsress[1]);
     spi_xfer(COM_FM, adsress[2]);
     spi_xfer(COM_FM, adsress[3]);
     // Write data bytes
     for (int i = 0; i < data_number; i++) {
         spi_xfer(COM_FM, data[i]);  // Send data byte
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
     spi_xfer(MISSION_FM, WRITE_PAGE);
     spi_xfer(MISSION_FM, adsress[0]);
     spi_xfer(MISSION_FM, adsress[1]);
     spi_xfer(MISSION_FM, adsress[2]);
     spi_xfer(MISSION_FM, adsress[3]);
     // Write data bytes
     for (int i = 0; i < data_number; i++) {
         spi_xfer(MISSION_FM, data[i]);  // Send data byte
         fprintf(EXT,"%02c", data[i]);    // Print each byte as hex (debugging purpose)
     }
     
     output_high(CS_PIN_MISSION);  // Deselect SPI device5
     output_high(MX_PIN_ADCS);  //Deselect MUX from flash
     
     fprintf(EXT,"\n%d BYTES WRITTEN IN ADCS!\n", data_number);
 }
  void WRITE_DATA_NBYTES_IHS(unsigned int32 ADDRESS, unsigned int8 data[], unsigned char data_number, unsigned int8 MX_PIN) {
     fprintf(EXT,"WRITE ADDRESS IN IHS: 0x%08lx\n", ADDRESS);  // Print address as hex
     unsigned int8 adsress[4];
     // Byte extraction for a 32-bit address
     adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
     adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
     adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
     adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);
     WRITE_ENABLE_OF_IHS(MX_PIN);  // Enable write operation and MX and CS pins are included in here 
 
     //Lower MX to connect to flash device
     output_low(MX_PIN);
     // Lower CS to select the SPI device
     output_low(CS_PIN_MISSION);
     delay_us(2);  // Small delay for stabilization
     // Send WRITE command and address
     spi_xfer(MISSION_FM, WRITE_PAGE);
     spi_xfer(MISSION_FM, adsress[0]);
     spi_xfer(MISSION_FM, adsress[1]);
     spi_xfer(MISSION_FM, adsress[2]);
     spi_xfer(MISSION_FM, adsress[3]);
     // Write data bytes
     for (int i = 0; i < data_number; i++) {
         spi_xfer(MISSION_FM, data[i]);  // Send data byte
         fprintf(EXT,"%02c", data[i]);    // Print each byte as hex (debugging purpose)
     }
     
     output_high(CS_PIN_MISSION);  // Deselect SPI device5
     output_high(MX_PIN);  //Deselect MUX from flash
     
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
 
     output_low(CS_PIN_MAIN);  // Select SPI device
 
     // Send READ DATA COMMAND (0x13 or appropriate for your flash chip)
     spi_xfer(MAIN_FM, READ_DATA_BYTES);
     // Send address bytes
     spi_xfer(MAIN_FM, adsress[0]);
     spi_xfer(MAIN_FM, adsress[1]);
     spi_xfer(MAIN_FM, adsress[2]);
     spi_xfer(MAIN_FM, adsress[3]);
 
     // Read the requested number of bytes
     for (int i = 0; i < data_number && i < 256; i++) {  // Avoid overflow
         Data_return[i] = spi_xfer(MAIN_FM, 0x00);  // Send dummy byte to receive data
         fprintf(EXT, "%c", Data_return[i]);  // Print each byte as hex
     }
 
     output_high(CS_PIN_MAIN);  // Deselect SPI device after reading
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
     spi_xfer(COM_FM, READ_DATA_BYTES);
     // Send address bytes
     spi_xfer(COM_FM, adsress[0]);
     spi_xfer(COM_FM, adsress[1]);
     spi_xfer(COM_FM, adsress[2]);
     spi_xfer(COM_FM, adsress[3]);
 
     // Read the requested number of bytes
     for (int i = 0; i < data_number && i < 256; i++) {
         Data_return[i] = spi_xfer(COM_FM, 0x00);  // Send dummy byte to receive data
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

    WRITE_ENABLE_OF_ADCS(); // Ensure write enable is set before read

    output_low(MX_PIN_ADCS);  // Lower MX to connect to flash device
    output_low(CS_PIN_MISSION);  // Select SPI device

    // Send READ DATA COMMAND
    spi_xfer(MISSION_FM, READ_DATA_BYTES);
    // Send address bytes
    spi_xfer(MISSION_FM, adsress[0]);
    spi_xfer(MISSION_FM, adsress[1]);
    spi_xfer(MISSION_FM, adsress[2]);
    spi_xfer(MISSION_FM, adsress[3]);

    // Read the requested number of bytes
    for (int i = 0; i < data_number; i++) {
        Data_return[i] = spi_xfer(MISSION_FM, 0x00);  // Send dummy byte to receive data
    }

    output_high(CS_PIN_MISSION);  // Deselect SPI device
    output_high(MX_PIN_ADCS);  // Deselect MUX from flash

    return Data_return;
}
 
#define MAX_DATA_SIZE_IHS 256  // Define the maximum data size

unsigned int8 Data_return_IHS[MAX_DATA_SIZE_IHS];  // Static buffer to hold the data

unsigned int8* READ_DATA_NBYTES_IHS(unsigned int32 ADDRESS, unsigned short data_number, unsigned int8 MX_PIN) {
    unsigned int8 addr_bytes[3]; // Only 3 bytes now!

    // Ensure data_number buffer limit logic exists here...

    // Extract 3 bytes (A23-A0)
    addr_bytes[0] = (unsigned int8)((ADDRESS >> 16) & 0xFF);
    addr_bytes[1] = (unsigned int8)((ADDRESS >> 8) & 0xFF);
    addr_bytes[2] = (unsigned int8)(ADDRESS & 0xFF);

    output_low(MX_PIN);  // Lower MX to connect to flash device
    output_low(CS_PIN_MISSION);

    // Send STANDARD READ Command (0x03)
    spi_xfer(MISSION_FM, 0x03); 
    
    // Send 3 Address Bytes
    spi_xfer(MISSION_FM, addr_bytes[0]);
    spi_xfer(MISSION_FM, addr_bytes[1]);
    spi_xfer(MISSION_FM, addr_bytes[2]);

    // Read Data
    for (int i = 0; i < data_number; i++) {
        Data_return_IHS[i] = spi_xfer(MISSION_FM, 0x00);
    }

    output_high(CS_PIN_MISSION);
    output_high(MX_PIN);

    return Data_return_IHS;
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
     spi_xfer(MISSION_FM, READ_DATA_BYTES);
     // Send address bytes
     spi_xfer(MISSION_FM, adsress[0]);
     spi_xfer(MISSION_FM, adsress[1]);
     spi_xfer(MISSION_FM, adsress[2]);
     spi_xfer(MISSION_FM, adsress[3]);
     // Read the requested number of bytes
         Data_return = spi_xfer(MISSION_FM, 0x00);  // Send dummy byte to receive data
 
     output_high(CS_PIN_MISSION);  // Deselect SPI device after reading
     output_high(MX_PIN_ADCS);  //Deselect MUX from flash
     return Data_return;
 }

int8 READ_DATA_BYTES_IHS(unsigned int32 ADDRESS) {
     unsigned int8 adsress[4];
     unsigned int8 Data_return;
    
     // Byte extraction for a 32-bit address
     adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
     adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
     adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
     adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);
 
     output_low(MX_PIN_FM2);  // Lower MX to connect to flash device
     output_low(CS_PIN_MISSION);  // Select SPI device
 
     // Send READ DATA COMMAND (0x13 or appropriate for your flash chip)
     spi_xfer(MISSION_FM, READ_DATA_BYTES);
     // Send address bytes
     spi_xfer(MISSION_FM, adsress[0]);
     spi_xfer(MISSION_FM, adsress[1]);
     spi_xfer(MISSION_FM, adsress[2]);
     spi_xfer(MISSION_FM, adsress[3]);
     // Read the requested number of bytes
         Data_return = spi_xfer(MISSION_FM, 0x00);  // Send dummy byte to receive data
 
     output_high(CS_PIN_MISSION);  // Deselect SPI device after reading
     output_high(MX_PIN_FM2);  //Deselect MUX from flash
     return Data_return;
 }

 // --- COMMAND DEFINITIONS FOR 3-BYTE MODE ---
#define CMD_READ_3B        0x03    // Standard Read
#define CMD_WRITE_3B       0x02    // Standard Page Program
#define CMD_ERASE_64K_3B   0xD8    // 64KB Block Erase

// --- ADDRESS MAPPING ---
#define IHS_CMD_ADDR       0x00000000  // Command sector on FM2
#define IHS_SLOT_BASE      0x00010000  // Slot 1 starts here
#define IHS_SLOT_SIZE      0x00010000  // 64KB per slot

void IHS_Test_Sequence(unsigned int8 IHS_CMD[4]) {
    unsigned int32 i;
    unsigned int8 MX_PIN;
    unsigned int32 status_addr;
    output_high(EN_SUP_3V3_2);
    output_high(OVCAM_PWR);  // Power ON OVCAM
    delay_ms(2000); // IDK this is how much time it need to stablelize 

    // 1. Determine which hardware path to use
    // IHS Logic: Command & Status ALWAYS on FM2. Data on FM1 (CAM1) or FM2 (CAM2).
    if (IHS_CMD[0] == 0xF0) {
        MX_PIN = MX_PIN_FM2;        // Data download from FM2
        status_addr = 0x00FF0001;   // Status byte for CAM2
        fprintf(EXT, "Target: CAM2 (MVCAM) | Data MUX: %u\n", MX_PIN);
    } else {
        MX_PIN = MX_PIN_FM1;        // Data download from FM1
        status_addr = 0x00FF0000;   // Status byte for CAM1
        fprintf(EXT, "Target: CAM1 (OVCAM) | Data MUX: %u\n", MX_PIN);
    }
    
    fprintf(EXT, "\n--- IHS MISSION START ---\n");

    // 2. Clear old command (ALWAYS on FM2)
    fprintf(EXT, "Erasing Command Sector on FM2...\n");
    SECTOR_ERASE_OF_IHS(IHS_CMD_ADDR, MX_PIN_FM2); 
    delay_ms(200);

    // 3. Send new mission parameters (ALWAYS on FM2)
    WRITE_DATA_NBYTES_IHS(IHS_CMD_ADDR, IHS_CMD, 4, MX_PIN_FM2);
    fprintf(EXT, "Command Sent: [%02X %02X %02X %02X]\n", IHS_CMD[0], IHS_CMD[1], IHS_CMD[2], IHS_CMD[3]);

    // 4. WAIT LOGIC
    // We must wait for: Initial Delay + (Interval + WriteTime) * Count
    unsigned int16 wait_sec = (unsigned int16)IHS_CMD[1] + (((unsigned int16)IHS_CMD[2] + 4) * (unsigned int16)IHS_CMD[3]);
    
    // Fixed printf: %u for int16
    fprintf(EXT, "Waiting %lu seconds for IHS execution...\n", wait_sec);
    for(i = 0; i < wait_sec; i++) {
        delay_ms(1000);
        if((i+1) % 5 == 0) fprintf(EXT, "Time: %lu/%lu s\n", (unsigned int16)(i+1), wait_sec);
    }

    // 5. Read Status (ALWAYS from FM2)
    // BUG FIX: Removed the extra += 1 here because status_addr was already set correctly in Step 1.
    unsigned int8* status_ptr = READ_DATA_NBYTES_IHS(status_addr, 1, MX_PIN_FM2);
    unsigned int8 num_pictures = status_ptr[0];
    
    if (num_pictures == 0xFF || num_pictures == 0x00) {
        fprintf(EXT, "Warning: No index found at 0x%06lX. Forcing 1-picture read.\n", status_addr);
        num_pictures = 1; 
    } else {
        fprintf(EXT, "IHS reports %u pictures captured.\n", num_pictures);
    }

    // 6. Download Data
    // Uses MX_PIN (FM1 for CAM1, FM2 for CAM2)
    for(i = 1; i <= num_pictures; i++) {
        unsigned int32 current_addr = IHS_SLOT_BASE + ((i - 1) * IHS_SLOT_SIZE);
        fprintf(EXT, "\n--- START PIC %lu (ADDR: 0x%06lX) ---\n", i, current_addr);
        
        unsigned int32 bytes_read = 0;
        unsigned int8 prev_byte = 0, curr_byte = 0;
        
        while(bytes_read < IHS_SLOT_SIZE) {
            unsigned int8* chunk = READ_DATA_NBYTES_IHS(current_addr + bytes_read, 64, MX_PIN);
            
            for(int k = 0; k < 64; k++) {
                curr_byte = chunk[k];
                fputc(curr_byte, EXT); // Stream to PC
                
                // Detection of JPEG End Marker
                if (prev_byte == 0xFF && curr_byte == 0xD9) {
                    fprintf(EXT, "\n[JPEG EOI FOUND]\n");
                    goto next_picture;
                }
                prev_byte = curr_byte;
            }
            bytes_read += 64;
            // Optional: restart_wdt(); 
        }
        next_picture:;
    }
    
    output_high(MX_PIN_FM1); 
    output_high(MX_PIN_FM2);
    output_low(OVCAM_PWR);  // Power ON OVCAM
    output_low(EN_SUP_3V3_2);
    fprintf(EXT, "--- IHS MISSION FINISHED ---\n");
}

 void READ_CHIP_ID_OF() {
     int8 chip_id[8];
     output_low(CS_PIN_MAIN);  // Lower the CS PIN
     spi_xfer(MAIN_FM, READ_ID);  // READ ID COMMAND (0x9F)
     
     // Receive 8 bytes of chip ID
     for (int i = 0; i < 8; i++) {
         chip_id[i] = spi_xfer(MAIN_FM, 0x00);  // Send dummy bytes to receive data
         fprintf(EXT, "%02X ", chip_id[i]);
     }
     fprintf(EXT,"\n");
 
     output_high(CS_PIN_MAIN);  // Raise CS PIN back
 }
 void READ_CHIP_ID_OF_COM() {
     int8 chip_id[8];
     output_low(MX_PIN_COM);
     output_low(CS_PIN_COM);  // Lower the CS PIN
     spi_xfer(COM_FM, READ_ID);  // READ ID COMMAND (0x9F)
     
     // Receive 8 bytes of chip ID
     for (int i = 0; i < 8; i++) {
         chip_id[i] = spi_xfer(COM_FM, 0x00);  // Send dummy bytes to receive data
         fprintf(EXT, "%02X ", chip_id[i]);
     }
     fprintf(EXT,"\n");
 
     output_high(CS_PIN_COM);  // Raise CS PIN back
     output_high(MX_PIN_COM);
 }
 
 void READ_CHIP_ID_OF_ADCS() {
    int8 chip_id[8];
    int i;

    // 1. Clean the bus - ensure other CS pins are HIGH before starting
    output_low(MX_PIN_ADCS); // Enable the MUX path to ADCS Flash
    delay_us(10);            // Short settle time
    
    // 4. Start SPI Transaction
    output_low(CS_PIN_MISSION);  
    
    // Send Command 0x9F
    spi_xfer(MISSION_FM, 0x9F); 
    
    // NO DELAY HERE - Read immediately
    for (i = 0; i < 8; i++) {
        chip_id[i] = spi_xfer(MISSION_FM, 0x00);
    }

    // 5. Close Transaction
    output_high(CS_PIN_MISSION);
    output_high(MX_PIN_ADCS);

    // 6. Print Results
    // fprintf(EXT, "ADCS RAW ID: ");
    for (i = 0; i < 8; i++) {
        fprintf(EXT, "%02X ", chip_id[i]);
    }
    fprintf(EXT, "\n");
}
 void READ_CHIP_ID_OF_IHS_MVCAM() {
     int8 chip_id[8];
     output_low(MX_PIN_FM1);
     output_low(CS_PIN_MISSION);  // Lower the CS PIN
     spi_xfer(MISSION_FM, READ_ID);  // READ ID COMMAND (0x9F)
     
     // Receive 8 bytes of chip ID
     for (int i = 0; i < 8; i++) {
         chip_id[i] = spi_xfer(MISSION_FM, 0x00);  // Send dummy bytes to receive data
         fprintf(EXT, "%02X ", chip_id[i]);
     }
     fprintf(EXT,"\n");
 
     output_high(CS_PIN_MISSION);  // Raise CS PIN back
     output_high(MX_PIN_FM1);
 }

 void READ_CHIP_ID_OF_IHS_OVCAM() {
     int8 chip_id[8];
     output_low(MX_PIN_FM2);
     output_low(CS_PIN_MISSION);  // Lower the CS PIN
     spi_xfer(MISSION_FM, READ_ID);  // READ ID COMMAND (0x9F)
     
     // Receive 8 bytes of chip ID
     for (int i = 0; i < 8; i++) {
         chip_id[i] = spi_xfer(MISSION_FM, 0x00);  // Send dummy bytes to receive data
         fprintf(EXT, "%02X ", chip_id[i]);
     }
     fprintf(EXT,"\n");
 
     output_high(CS_PIN_MISSION);  // Raise CS PIN back
     output_high(MX_PIN_FM2);
 }
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Main menu functions
 
 void startup_freeze(){
     delay_ms(2000);
     fprintf(EXT, "POWER ON!\n");
     //EPS power control all disabled when startup, using menu function will turn on
     output_high(EN_SUP_3V3_1); // can't control this pin because it's always on when the system is powered on
     output_low(EN_SUP_3V3_2); // camera system power control pin
     output_low(EN_SUP_5V0); 
     output_low(EN_SUP_UNREG);
     
    // OCP power control pins
     //  output_low(MVCAM_PWR); // Camera system main ocp power control pin
     output_low(OVCAM_PWR); // Camera system main ocp power control pin 
     output_low(ADCS_PWR); //turns on the power of ADCS instantly don't forget to on the ADCS system for usage in this case of it 
     output_low(EN_SUP_3V3_DAQ); // in here should be on when using DAQ system or ADCS system this is 
     
     // SPI flash CS and MUX pins default state
     output_high(CS_PIN_MAIN); // my own system FM OBC flash CS pin
     output_high(CS_PIN_COM); // COM FM OBC flash CS pin
     output_high(CS_PIN_MISSION); //share FM ADCS, CAM1, CAM2 flash CS pin
     output_high(MX_PIN_FM2);   // default mux to camera connection in high state, low to connect to OBC to flash
     output_high(MX_PIN_FM1); // default mux to camera connection in high state, low to connect to OBC to flash
     output_high(MX_PIN_ADCS);  // default mux to ADCS connection in high state, low to connect to OBC to flash
     output_high(MX_PIN_COM);   // IDK what is this for but just to be safe set it high
     
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
int8 adcs_quick[2] = {0x01, 0x01}; // quick mission for ADCS 
                // if mission is 0x66 it means two sensor will run together
                // |header 2 byte| |gyro 2 byte (XYZ)| |mag 2 byte (XYZ) + 1byte (2 bit lsb each)|
                //
unsigned int32 ADCS_STATUS_ADDR = 0x00020000;
unsigned int32 ADCS_CMD_ADDR    = 0x00000000;

void READ_DATA_TO_BUFFER_ADCS(unsigned int32 ADDRESS, unsigned int8* destination, unsigned int16 len) {
    unsigned int8 addr_bytes[4];
    unsigned int16 i;

    // 1. Extract address bytes
    addr_bytes[0] = (unsigned int8)((ADDRESS >> 24) & 0xFF);
    addr_bytes[1] = (unsigned int8)((ADDRESS >> 16) & 0xFF);
    addr_bytes[2] = (unsigned int8)((ADDRESS >> 8) & 0xFF);
    addr_bytes[3] = (unsigned int8)(ADDRESS & 0xFF);

    // 2. Hardware Control
    output_low(MX_PIN_ADCS);    // Switch MUX to ADCS Flash
    delay_us(10);
    output_low(CS_PIN_MISSION); // Select SPI device

    // 3. Send Read Command (Using 0x13 for 4-byte addressing)
    spi_xfer(MISSION_FM, 0x13);
    spi_xfer(MISSION_FM, addr_bytes[0]);
    spi_xfer(MISSION_FM, addr_bytes[1]);
    spi_xfer(MISSION_FM, addr_bytes[2]);
    spi_xfer(MISSION_FM, addr_bytes[3]);

    // 4. Fill the destination buffer
    for(i = 0; i < len; i++) {
        destination[i] = spi_xfer(MISSION_FM, 0x00);
    }

    // 5. Release Bus
    output_high(CS_PIN_MISSION);
    output_high(MX_PIN_ADCS);
}

int8 VERIFY_ADCS_HARDWARE(void) {
    unsigned int8 m_id, type, capacity;
    
    // 1. Point the MUX to the ADCS side
    output_low(MX_PIN_ADCS); 
    delay_ms(5); // Wait for MUX to settle

    // 2. Start SPI Transaction
    output_low(CS_PIN_MISSION);

    spi_xfer(MISSION_FM, READ_ID);  // READ ID COMMAND (0x9F)
    delay_ms(100); // Small delay for stability
    
    // 3. Send the "Read Identification" command
    spi_xfer(MISSION_FM, 0x9F); 

    // 4. Read the 3 bytes of the JEDEC ID
    m_id     = spi_xfer(MISSION_FM, 0x00);
    type     = spi_xfer(MISSION_FM, 0x00);
    capacity = spi_xfer(MISSION_FM, 0x00);

    // 5. End SPI Transaction
    output_high(CS_PIN_MISSION);
    output_high(MX_PIN_ADCS); // Release MUX

    // 6. Print for debugging
    fprintf(EXT, "ADCS Flash ID: %02X %02X %02X\n", m_id, type, capacity);

    // 7. Check if it's a valid Micron ID
    // 0x20 is Micron. If you get 0x00 or 0xFF, the hardware is dead/busy.
    if (m_id == 0xBA) {
        return 1; // SUCCESS
    } else {
        return 0; // FAIL
    }
}

void adcs_mission_mode(void) {
    unsigned int8 local_buffer[16]; // Safe local storage
    unsigned int8 adcs_status = 0;
    unsigned int32 last_addr = 0;
    unsigned int32 start_addr = 0;
    unsigned int8 i, j;

    fprintf(EXT, "\n--- ADCS MISSION START ---\n");
    
    // 1. Power ON and WAIT for stabilization
    output_high(EN_SUP_3V3_DAQ);
    output_high(ADCS_PWR);
    delay_ms(1500); 

    // 2. VERIFY Hardware is actually responding
    // If ID is 00 or FF, the MUX or Power is failing
     WRITE_ENABLE_OF_ADCS(); // to wake up the ADCS flash from deep power down
    if (!VERIFY_ADCS_HARDWARE()) {
        fprintf(EXT, "CRITICAL ERROR: ADCS Flash not responding. Aborting.\n");
        output_low(ADCS_PWR);
        return;
    }

    // 3. Send Quick Mission Command
    fprintf(EXT, "Clearing Command Sector...\n");
    SECTOR_ERASE_OF_ADCS(ADCS_CMD_ADDR);
    
    fprintf(EXT, "Writing Mission Code {01, 01}...\n");
    WRITE_DATA_NBYTES_ADCS(ADCS_CMD_ADDR, adcs_quick, 2);
    delay_ms(100);

    // 4. Wait for ADCS to finish (40 seconds)
    // Note: If ADCS is writing to flash now, OBC MUST NOT touch the MUX
    for (i = 0; i <= 4; i++) {
        fprintf(EXT, "ADCS Processing: %ds\n", i * 10);
        delay_ms(10000);
    }

    // 5. Read Status and Last Address (5 bytes)
    // We read into a local buffer so we don't lose the data
    READ_DATA_TO_BUFFER_ADCS(ADCS_STATUS_ADDR, local_buffer, 5);
    
    adcs_status = local_buffer[0];
    last_addr = ((unsigned int32)local_buffer[1] << 24) | 
                ((unsigned int32)local_buffer[2] << 16) | 
                ((unsigned int32)local_buffer[3] << 8)  | 
                 (unsigned int32)local_buffer[4];

    if (adcs_status == 0x00 || adcs_status == 0xFF) {
        fprintf(EXT, "FAILED: No data found (Status: 0x%02X)\n", adcs_status);
        output_low(ADCS_PWR);
        return;
    }
}





// This function must be standalone (not inside another function)
void ADCS_GET_RAW_HK(int16 *gx, int16 *gy, int16 *gz, int16 *mx, int16 *my, int16 *mz) 
{
    unsigned int8 cmd_seq[2];
    unsigned int8 local_buffer[20];          
    unsigned int32 last_addr = 0;
    unsigned int32 data_addr = 0;
    unsigned int8 i;
    unsigned int8* ptr;

    cmd_seq[0] = 0x66;
    cmd_seq[1] = 0x01;

    fprintf(EXT, "\n--- ADCS RAW HK START ---\n");
    output_high(EN_SUP_3V3_DAQ);
    output_high(ADCS_PWR);
    
    // there is secondary in line ocp to control the order to turn on the adcs power
    
    delay_ms(4000); 

    // Send Command
    SECTOR_ERASE_OF_ADCS(ADCS_CMD_ADDR);
    delay_ms(50);
    WRITE_DATA_NBYTES_ADCS(ADCS_CMD_ADDR, cmd_seq, 2);

    //check the data is written correctly
    ptr = READ_DATA_NBYTES_ADCS(ADCS_CMD_ADDR, 2);
    fprintf(EXT, "ADCS Command Written: %02X %02X\n", ptr[0], ptr[1]);
    if (ptr[0] != cmd_seq[0] || ptr[1] != cmd_seq[1]) {
        fprintf(EXT, "Error: ADCS Command Verification Failed.\n");
        output_low(ADCS_PWR);
        return;
    }

    // Wait for ADCS to finish measurement
    for(i=0; i<4; i++) {
        delay_ms(1000);
        fputc('.', EXT);
    }

    // Read Status (5 bytes)
    ptr = READ_DATA_NBYTES_ADCS(ADCS_STATUS_ADDR, 5);
    for(i=0; i<5; i++) {
        local_buffer[i] = ptr[i];
    }

    last_addr = ((unsigned int32)local_buffer[1] << 24) | 
                ((unsigned int32)local_buffer[2] << 16) | 
                ((unsigned int32)local_buffer[3] << 8)  | 
                (unsigned int32)local_buffer[4];

    // If count (local_buffer[0]) is 0 or 0xFF, it failed
    if(local_buffer[0] == 0 || local_buffer[0] == 0xFF) {
        fprintf(EXT, "ADCS Fail\n");
        output_low(ADCS_PWR);
        return;
    }

    // Read Data (Packet is 15 bytes)
    data_addr = last_addr - 15; 
    ptr = READ_DATA_NBYTES_ADCS(data_addr, 15);
    for(i=0; i<15; i++) {
        local_buffer[i] = ptr[i];
    }

    // Update the variables using the pointers
    // Use * to access the value at the address
    *gx = make16(local_buffer[2], local_buffer[3]);
    *gy = make16(local_buffer[4], local_buffer[5]);
    *gz = make16(local_buffer[6], local_buffer[7]);
    
    *mx = make16(local_buffer[8], local_buffer[9]);
    *my = make16(local_buffer[10], local_buffer[11]);
    *mz = make16(local_buffer[12], local_buffer[13]);

    output_low(ADCS_PWR);
    output_high(EN_SUP_3V3_DAQ);
    fprintf(EXT, "\nSuccess.\n");
}



 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // eps functions 
char cmd = 'g'; // Command character to trigger EPS
unsigned int8 eps_buffer[33];
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

void get_data(unsigned int8* buffer, char cmd) {
    unsigned int32 timeout_count = 0;
    unsigned int8 trash;
    int8 retries = 5; // Attempt to request data 5 times
    int1 found_header = false;

    while(retries > 0 && !found_header) {
        // A. CLEAR THE BUFFER (Flush any old garbage)
        while(kbhit(EPS)) { trash = fgetc(EPS); } 

        // B. REQUEST DATA
        fputc(cmd, EPS); 
        fprintf(EXT, "Requesting EPS (Try %d)...\n", 6 - retries);

        // C. SYNC ON HEADER with shorter timeout per attempt
        timeout_count = 0;
        while(timeout_count < 100000) { // ~100ms wait per try
            if(kbhit(EPS)) {
                unsigned int8 head = fgetc(EPS);
                if(head == '}') {
                    buffer[0] = head;
                    found_header = true;
                    break; 
                }
            }
            delay_us(1); 
            timeout_count++;
        }
        
        if(!found_header) {
            retries--;
            delay_ms(50); // Small pause before retrying
        }
    }

    if(!found_header) {
        printf(EXT, "Error: EPS did not respond after 5 attempts.\n");
        return;
    }

    // D. READ THE REMAINING 32 BYTES (If header was found)
    for (int i = 1; i < 33; i++) {
        timeout_count = 0;
        while(!kbhit(EPS)) {
            delay_us(10);
            if(++timeout_count > 10000) { 
                printf(EXT, "Timeout lost byte %d\n", i);
                return;
            }
        }
        buffer[i] = fgetc(EPS);
    }
    printf(EXT, "EPS Packet Received Successfully.\n");
}

// Function to send 33 bytes to EXT
void send_data(char* buffer) {
    for (int8 i = 0; i < 33; i++) {
        putc(buffer[i], EXT); // Send each byte to EXT
    }
    printf(EXT, "\n"); // Newline after sending all bytes
}

/// Update store_eps_data for accuracy
void store_eps_data(unsigned int8* buffer) {
    // Check start '}' and end 'f' frames for data integrity
    if (buffer[0] == '}' && buffer[32] == 'f') {
        memcpy(temperature, &buffer[1], 7);
        memcpy(solar_panel_current, &buffer[8], 5);
        memcpy(solar_panel_voltage, &buffer[13], 5);
        sc_pwr    = buffer[18];
        sc_i_sens = buffer[19];
        batt_i_sens = buffer[20];
        batt_pwr  = buffer[21];
        raw_i_sens = buffer[22];
        raw_v_sens = buffer[23];
        batt_temp = buffer[24];
        memcpy(DCDC_output_i, &buffer[25], 5);
        KS_OBC_STAT = (buffer[30] != 0);
        KS_EPS_STAT = (buffer[31] != 0);
    } else {
        printf(EXT, "CRC/Framing Error! Start: %c, End: %c\n", buffer[0], buffer[32]);
    }
}

void print_table() {
    // Use a constant for the ADC conversion to ensure float precision
    // Equation: (Value * Vref) / Res / Gain
    const float conv_factor = 3.3 / 256.0 / 20.0;

    printf(EXT, "\n%-15s | %-8s | %-12s\n", "Variable", "Raw", "Value (V/A)");
    printf(EXT, "--------------------------------------------------\n");
    
    // Using (float) cast ensures the compiler uses floating point math
    printf(EXT, "%-15s | %-8u | %.6f\n", "sc_pwr", sc_pwr, (float)sc_pwr * conv_factor);
    printf(EXT, "%-15s | %-8u | %.6f\n", "sc_i_sens", sc_i_sens, (float)sc_i_sens * conv_factor);
    printf(EXT, "%-15s | %-8u | %.6f\n", "raw_i_sens", raw_i_sens, (float)raw_i_sens * conv_factor);
    printf(EXT, "%-15s | %-8u | %.6f\n", "raw_v_sens", raw_v_sens, (float)raw_v_sens * conv_factor);
    printf(EXT, "%-15s | %-8u | %.6f\n", "batt_pwr", batt_pwr, (float)batt_pwr * conv_factor);
    printf(EXT, "%-15s | %-8u | %.6f\n", "batt_i_sens", batt_i_sens, (float)batt_i_sens * conv_factor);
    
    printf(EXT, "OBC Status: %s | EPS Status: %s\n", 
           KS_OBC_STAT ? "ON" : "OFF", 
           KS_EPS_STAT ? "ON" : "OFF");
}

void eps_mission_mode(void) {
    char cmd = 'g'; // Command character to trigger EPS
    unsigned int8 eps_buffer[33];
    memset(eps_buffer, 0, 33); // Clear buffer
    output_high(EN_SUP_3V3_DAQ); 
    delay_ms(500); // Give it more time to boot
    get_data(eps_buffer, cmd); 
    // Print raw HEX first so you can see if '7D' is at index 0
    printf(EXT, "RAW EPS: ");
    for(int i=0; i<33; i++) {
        fprintf(EXT, "[%d]:%02X ", i, eps_buffer[i]);
    }
    printf(EXT, "\n");
    store_eps_data(eps_buffer);
    print_table();
    output_low(EN_SUP_3V3_DAQ);
}
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // IHS functions
// Changed return type to signed int8 to allow -1 error code
// Helper function to wait for specific text from Camera
int8 wait_for_cam_confirmation(unsigned int16 timeout_ms) {
    char window[2] = {0, 0}; // Our small array to hold the last 2 chars
    unsigned int32 timer = 0;

    // Clear the software UART buffer first to remove old data
    while(kbhit(CAM)) { fgetc(CAM); } 

    while(timer < timeout_ms) {
        if(kbhit(CAM)) {
            window[0] = window[1];
            window[1] = fgetc(CAM);
            
            // Optional: Send to debug terminal
            fputc(window[1], EXT); 

            // Check the array
            if (window[0] == 'O' && window[1] == 'K') {
                return 1; // "OK" found!
            }
        } else {
            delay_ms(1);
            timer++;
        }
    }
    return 0; // Timeout
}

int8 CAM_MISSION_MAIN (unsigned int8 cam_select, unsigned int8 mission_number, unsigned int8 delay) {
    unsigned int8 cam_response; 
    
    fprintf(EXT, "\n--- CAM MISSION START ---\n");

    // 1. Send Capture Command (Append 'x' to end Arduino parsing instantly)
    if (cam_select == 1) {
        fprintf(CAM, "c%u t%u", mission_number, delay); 
        fprintf(EXT, "Command Sent: c%u (Capture) in delay of %u s\n", mission_number, delay);
    } else {
        fprintf(CAM, "v%u t%u", mission_number, delay); 
        fprintf(EXT, "Command Sent: v%u (Capture) in delay of %u s \n", mission_number, delay);
    }

    // 2. THE FIX: Wait for "CAM1 SAVE... OK"
    // This cleans the text out of the buffer so it doesn't corrupt the image
    fprintf(EXT, "Waiting for Save Confirmation...\n");
    wait_for_cam_confirmation(10000);
    fprintf(EXT, "\nConfirmation Received! Requesting Download...\n");
    
    delay_ms(1000); // Short safety pause

    // 3. Request Binary Download
    if (cam_select == 1) {
        fprintf(CAM, "d%u", mission_number);
        fprintf(EXT, "d%u Command sent!\n", mission_number);
    } else {
        fprintf(CAM, "f%u", mission_number);
        fprintf(EXT, "f%u Command sent!\n", mission_number);
    }
    
    // 4. Receive Binary Data
    // No delays here, just fast reading
    unsigned int32 inactivity_timer = 0;
    unsigned int32 bytes_read = 0;
    fprintf(EXT, "WAITIGN TO GET DATA\n");
    
    while (inactivity_timer < 100000) { 
        if (kbhit(CAM)) {
            cam_response = fgetc(CAM); 
            fputc(cam_response, EXT); // Send raw byte to terminal
            bytes_read++;
            inactivity_timer = 0;     // Reset timer because we got data
        }
        inactivity_timer++;
    }

    fprintf(EXT, "\n--- CAM MISSION ENDED (Bytes: %lu) ---\n", bytes_read);
    return 0;
}
void ihs_get_status(void){
    fprintf(EXT, "IHS status function called\n");
    // send status command to IHS e1 or e2 to get the status of the cameras
    fprintf(EXT, "select which flash memory to work on (a, b):\n");
    fprintf(EXT, "press a: OVCAM status\n");
    fprintf(EXT, "press b: MVCAM status\n");
    char ihs_status_option;
    ihs_status_option = fgetc(EXT);
    switch (ihs_status_option) {
        case 'a':
            fprintf(EXT, "OVCAM status chosen\n");
            fprintf(CAM, "e1"); // send status command to OVCAM
            break;
        case 'b':
            fprintf(EXT, "MVCAM status chosen\n");
            fprintf(CAM, "e2"); // send status command to MVCAM
            break;
        default:
            fprintf(EXT, "Invalid IHS status option. Please try again.\n");
            break;
    }
}

void IHS_MISSION_MENU(void){
    // Call a function for IHC mission start
                fprintf(EXT, "IHS Mission mode selected.\n");
                fprintf(EXT, "    press a: Command IHS to take picture\n");
                fprintf(EXT, "    press b: Download picture from IHS\n");
                fprintf(EXT, "    press c: Get IHS status\n");
                fprintf(EXT, "    press x: Return to Main Menu\n");
            char option;
                // Read the user's choice
            option = fgetc(EXT);

                        // IHS mission switch
                        switch (option) {
                     case 'a':
                         // Command IHS to take picture (cam_select=2 for IHS, mission_number=1)
                         fprintf(EXT, "Commanding IHS to capture image...\n");
                        int cmd[3] = {0x00, 0x01, 0x05}; // IHS defualt command to take picture
                        fprintf(EXT, "ENTER THE MISSION PARAMETER\n"); // take user input for mission parameters
                        fprintf(EXT, "First byte is always 0x0F or 0xF0\n");
                        if(kbhit(EXT)){
                            cmd[0] = fgetc(EXT);
                            fprintf(EXT, "First byte: %02X\n", cmd[0]);
                            if(kbhit(EXT)){
                                cmd[1] = fgetc(EXT);
                                fprintf(EXT, "Second byte: %02X\n", cmd[1]);
                                if(kbhit(EXT)){
                                    cmd[2] = fgetc(EXT);
                                    fprintf(EXT, "Third byte: %02X\n", cmd[2]);
                                }
                            }
                        }
                        
                         IHS_Test_Sequence(cmd);

                         break;
                     case 'b':
                         // Download picture from IHS (cam_select=2 for IHS, mission_number=1)
                         fprintf(EXT, "Downloading image from IHS...\n");
                         CAM_MISSION_MAIN(1, 247, 100); // 100s delay before capture
                         break;
                        case 'c':
                         // Get IHS status
                         fprintf(EXT, "Getting IHS status...\n");
                         // ihs_get_status();
                         break;
                     case 'x':
                         fprintf(EXT, "Returning to Main Menu.\n");
                         break;
                     default:
                         fprintf(EXT, "Invalid IHS mission option. Please try again.\n");
                         break;
}
}

// Definitions based on your system
#define CMD_ENTER_4B_MODE  0xB7    // Command to enter 4-byte addressing
#define CMD_READ_4B        0x13    // Read command for 4-byte addresses
#define SLOT0_BASE         0x00010000
#define SLOT_SIZE          0x00010000
#define DATA_OFFSET        256

int8 CAM_MISSION_DOWNLOAD(unsigned int8 cam_select, unsigned int16 mission_number) {
    unsigned int32 base_address;
    unsigned int32 data_address;
    unsigned int32 jpeg_length = 0;
    unsigned int8 addr_bytes[4];
    unsigned int32 i;
    unsigned int8 active_mux;

    // 1. Setup MUX and Port
    // Assuming 0 = Ocean View, 1 = Mountain View
    active_mux = (cam_select == 0) ? MX_PIN_FM2 : MX_PIN_FM1;

    // fprintf(EXT, "\n--- STARTING BINARY DOWNLOAD (SLOT %u) ---\n", mission_number);

    output_low(active_mux);      // Switch MUX to Flash
    delay_ms(10);
    
    // --- STEP 1: FORCE 4-BYTE MODE ---
    output_low(CS_PIN_MISSION);
    spi_xfer(COM_FM, CMD_ENTER_4B_MODE); 
    output_high(CS_PIN_MISSION);
    delay_us(10);

    // --- STEP 2: CALCULATE ADDRESSES ---
    // base_address = 0x00010000 + (n-1) * 0x10000
    base_address = SLOT0_BASE + ((unsigned int32)(mission_number - 1) * SLOT_SIZE);
    
    addr_bytes[0] = (unsigned int8)(base_address >> 24);
    addr_bytes[1] = (unsigned int8)(base_address >> 16);
    addr_bytes[2] = (unsigned int8)(base_address >> 8);
    addr_bytes[3] = (unsigned int8)(base_address);

    // --- STEP 3: READ LENGTH (First 4 bytes of slot) ---
    output_low(CS_PIN_MISSION);
    spi_xfer(COM_FM, CMD_READ_4B);
    spi_xfer(COM_FM, addr_bytes[0]);
    spi_xfer(COM_FM, addr_bytes[1]);
    spi_xfer(COM_FM, addr_bytes[2]);
    spi_xfer(COM_FM, addr_bytes[3]);

    // Read 4-byte big-endian length
    jpeg_length =  ((unsigned int32)spi_xfer(COM_FM, 0x00) << 24);
    jpeg_length |= ((unsigned int32)spi_xfer(COM_FM, 0x00) << 16);
    jpeg_length |= ((unsigned int32)spi_xfer(COM_FM, 0x00) << 8);
    jpeg_length |= (unsigned int32)spi_xfer(COM_FM, 0x00);
    output_high(CS_PIN_MISSION);

    if(jpeg_length == 0 || jpeg_length == 0xFFFFFFFF) {
        fprintf(EXT, "FAIL: Slot Empty.\n");
        output_high(active_mux);
        return 0;
    }

    // --- STEP 4: READ DATA (Base + Offset) ---
    data_address = base_address + DATA_OFFSET;
    addr_bytes[0] = (unsigned int8)(data_address >> 24);
    addr_bytes[1] = (unsigned int8)(data_address >> 16);
    addr_bytes[2] = (unsigned int8)(data_address >> 8);
    addr_bytes[3] = (unsigned int8)(data_address);

    output_low(CS_PIN_MISSION);
    spi_xfer(COM_FM, CMD_READ_4B);
    spi_xfer(COM_FM, addr_bytes[0]);
    spi_xfer(COM_FM, addr_bytes[1]);
    spi_xfer(COM_FM, addr_bytes[2]);
    spi_xfer(COM_FM, addr_bytes[3]);

    // Stream to Radio/Terminal
    for(i = 0; i < jpeg_length; i++) {
        unsigned int8 b = spi_xfer(COM_FM, 0x00);
        fputc(b, EXT); // Stream raw binary out
        
        // Optional Watchdog Clear for large files
        if((i % 512) == 0) restart_wdt();
    }

    output_high(CS_PIN_MISSION);
    delay_ms(10);
    output_high(active_mux); // Return control to Camera
    
    fprintf(EXT, "\n--- DOWNLOAD FINISHED ---\n");
    return 1;
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
             READ_CHIP_ID_OF_ADCS();
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
        fprintf(EXT, "    press a: Get House keeping data\n");
        fprintf(EXT, "    press b: EPS Power output control\n");
        fprintf(EXT, "    press c: House keeping data collection\n");
        fprintf(EXT, "    press d: Check Flash Memories\n");
        //fprintf(EXT, "    press e: See satellite Log\n");
        fprintf(EXT, "    press f: Settings of RTC\n");
        //fprintf(EXT, "    press g: Satellite log down-link command\n");
        fprintf(EXT, "    press h: IHC Mission start\n");
        fprintf(EXT, "    press i: ADCS mission start\n");
        //fprintf(EXT, "    press j: H8 COM Reset\n");
        fprintf(EXT, "    press k: UART TEST of EPS\n");
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
                IHS_MISSION_MENU();
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
                fprintf(EXT, "UART TEST Initialized.\n");
                eps_mission_mode();
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


// #include <flashoperation.h>



 
 #ifdef	__cplusplus
 }
 #endif
 
 #endif	/* MAIN_H */
 
 