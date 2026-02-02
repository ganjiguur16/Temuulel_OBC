#include <main.h>
#include "flashoperation.h"
char bichig[25] = "test data update of MAIN"; //test data for testing 
char bichigcom[24] = "test data update of COM"; //test data for testing 
char bichigadcs[25] = "test data update of ADCS"; //test data for testing 
char *read_data; //MAIN flash received data will be stored in here 
unsigned char buffer[40]; //secondary buffer 
char *read_data_com; //COM flash received data will be stored in here 
unsigned int8 IHS_CMD[5]  = {0xF0, 0x05, 0x03, 0x02, 0x02}; 
unsigned int8 IHS_CMD1[5] = {0x0F, 0x05, 0x03, 0x02, 0x02};
// Changed 0x33 to 0x11 (60 seconds)
unsigned int8 CMD_ADCS[2] = {0x02, 0x01};
// unsigned int8 local_buffer[10]; 
    // unsigned int8 adcs_status = 0;
    unsigned int32 last_adcs_address = 0;
    unsigned int32 first_adcs_data_address = 0;    
 
void main() {    
    HK_PACKET hk_packet; // call struct in the main to use it 
    CW_BEACON cw_packet;

    startup_freeze();
    
    RTC_initialize();
    update_shutdown_count(); 

//     //------------------------read_chip_ID----------------------------
//     fprintf(EXT, "Reading chip ID of main\n");
//     delay_ms(1000);
//     READ_CHIP_ID_OF();
//     // READ_CHIP_ID_GENERIC(SPIPORT, CS_PIN_1, -1);  // Pass the array to be filled by the function
//     fprintf(EXT, "Reading chip ID of COM\n");
//     delay_ms(1000);
//     READ_CHIP_ID_OF_COM();
//     fprintf(EXT, "Reading chip ID of ADCS\n");
//     delay_ms(1000);
//     output_high(EN_SUP_3V3_DAQ);
//     output_high(ADCS_PWR);
//     delay_ms(5000);
//     READ_CHIP_ID_OF_ADCS();
//     GENERIC_READ_CHIP_ID_MISSION(MX_PIN_ADCS);
//     // adcs_mission_mode();
//     output_low(ADCS_PWR);
//     output_low(EN_SUP_3V3_DAQ);
//     delay_ms(1000);
//     fprintf(EXT, "Reading chip ID of IHS OVCAM\n");
//     output_high(EN_SUP_3V3_2);
//     output_high(OVCAM_PWR);  // Power ON OVCAM
//     delay_ms(1000);
//     GENERIC_READ_CHIP_ID_MISSION(MX_PIN_FM2);
//     fprintf(EXT, "Reading chip ID of IHS MVCAM\n");
//     delay_ms(1000);
//     GENERIC_READ_CHIP_ID_MISSION(MX_PIN_FM1);
//     delay_ms(1000);
//     output_low(OVCAM_PWR);  // Power OFF OVCAM
//     output_low(EN_SUP_3V3_2);
//     fprintf(EXT, "Done reading chip ID\n");
//        //------------------------write_flash_memory--------------------------
//     // Write and read from MAIN flash memory
//     fprintf(EXT, "Starting to write data in MAIN flash memory\n");
//     WRITE_DATA_NBYTES(0x00005000, bichig[25], sizeof(bichig));
//     delay_ms(1000);
//     read_data = READ_DATA_NBYTES(0x00005000, sizeof(bichig));
//     delay_ms(1000);
//     for (int i = 0; i < sizeof(bichig); i++) {
//         fprintf(EXT, "%c", read_data[i]);
//         delay_ms(2);
//     }
//     fprintf(EXT, "\n");
//     // Write and read from COM flash memory
//     fprintf(EXT, "Starting to write data in COM flash memory\n");
//     WRITE_DATA_NBYTES_COM(0x00005000, bichigcom, sizeof(bichigcom));
//     delay_ms(1000);
//     read_data_com = READ_DATA_NBYTES_COM(0x00005000, sizeof(bichigcom));
//     delay_ms(1000);
//     for (int i = 0; i < sizeof(bichigcom); i++) {
//         fprintf(EXT, "%c", read_data_com[i]);
//         delay_ms(2);
//     }
//     fprintf(EXT, "\nMAIN TEST IS FINISHED!\n");
//     //------------------------send_and_write_adcs_mission-----------------------
//     fprintf(EXT, "Starting ADCS hk mode\n");
// ADCS_GET_RAW_HK(&hk_gyro_x, &hk_gyro_y, &hk_gyro_z, &hk_mag_x, &hk_mag_y, &hk_mag_z);
// delay_ms(1000);
// fprintf(EXT, "Starting ADCS MISSION mode\n");
// adcs_mission_mode(CMD_ADCS);
// delay_ms(1000); 
// adcs_mission_download();
// delay_ms(1000);

//-----------------------------     EPS TEST     --------------------------------
// eps_mission_mode();
// delay_ms(1000);
// //-----------------------------IHS OVCAM MVCAM TEST--------------------------------
// fprintf(EXT, "Starting IHS  TEST\n");
// IHS_Test_Sequence(IHS_CMD);
// delay_ms(1000);
// IHS_Test_Sequence(IHS_CMD1);
// delay_ms(1000);

// // if keyboard hit detected during delay, go to main menu or the time out hits 
    fprintf(EXT, "Press any key for Main Menu. Starting in 5 seconds...\n");

int1 interrupt_triggered = 0;

for (int i = 20; i > 0; i--) {
    fprintf(EXT, "Starting in %d... \r", i); // \r keeps the countdown on one line
    
    for (int j = 0; j < 100; j++) {
        if(kbhit(EXT)) {   
            getc(EXT); // Consume the key
            interrupt_triggered = 1;
            break; // Break inner loop
        }
        delay_ms(10); 
    }
    
    if(interrupt_triggered) break; // Break outer loop
}

if(interrupt_triggered) {
    fprintf(EXT, "\n\n>>> Main Menu Activated <<<\n");
    main_menu();
}

fprintf(EXT, "\nStarting ADCS Mission Mode...\n");
adcs_mission_mode(CMD_ADCS);
    // 3. Resume from where we left off
    // This updates the GLOBAL variable used by WRITE_HK_PACKET
    

    while(TRUE) {
        current_hk_write_addr = LOAD_LAST_ADDRESS(); 
        fprintf(EXT, "Resuming HK storage at: 0x%08lX\n", current_hk_write_addr);
        // Collect & Save
        COLLECT_HOUSEKEEPING_DATA(&hk_packet);
        WRITE_HK_PACKET(&hk_packet); // This function internally increments current_hk_write_addr

        current_hk_write_addr += HK_PACKET_SIZE;
        if (current_hk_write_addr >= HK_STORAGE_END) {
            current_hk_write_addr = HK_STORAGE_BASE;
        }
        // Save the updated GLOBAL address to Flash so we remember it on reboot
        SAVE_HK_ADDRESS_PERSISTENT(current_hk_write_addr);

        PRINT_HK_PACKET(&hk_packet);

        // // Heartbeat
        GENERATE_CW_FROM_HK(&hk_packet, &cw_packet);

        // Transmit CW Packet
        send_cw_to_com_uart(&cw_packet);
        
        // 4. Fixed Delay and Keyboard check
        int16 cw_delay = 30000; // 30 seconds
        while (cw_delay-- > 0){
            delay_ms(1);
            if(kbhit(EXT)){
                    fprintf(EXT, "Main menu activated\n");
                    main_menu();
                
            }
        }
    }
}