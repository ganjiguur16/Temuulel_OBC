#include <main.h>

char bichig[25] = "test data update of MAIN"; //test data for testing 
char bichigcom[24] = "test data update of COM"; //test data for testing 
char bichigadcs[25] = "test data update of ADCS"; //test data for testing 
char *read_data; //MAIN flash received data will be stored in here 
unsigned char buffer[40]; //secondary buffer 
char *read_data_com; //COM flash received data will be stored in here 
// unsigned int8 local_buffer[10]; 
    // unsigned int8 adcs_status = 0;
    unsigned int32 last_adcs_address = 0;
    unsigned int32 first_adcs_data_address = 0;


    //
void main() {
    //------------------------Start_Indicator-------------------------
    startup_freeze();
    //--------------------------RTC-----------------------------------
    RTC_initialize();
    //------------------------restart_indicator-----------------------
    update_shutdown_count();
    //------------------------read_chip_ID----------------------------
    fprintf(EXT, "Reading chip ID of main\n");
    delay_ms(1000);
    READ_CHIP_ID_OF();
    // READ_CHIP_ID_GENERIC(SPIPORT, CS_PIN_1, -1);  // Pass the array to be filled by the function
    fprintf(EXT, "Reading chip ID of COM\n");
    delay_ms(1000);
    READ_CHIP_ID_OF_COM();
    fprintf(EXT, "Reading chip ID of ADCS\n");
    delay_ms(1000);
    output_high(EN_SUP_3V3_DAQ);
    output_high(ADCS_PWR);
    delay_ms(5000);
    READ_CHIP_ID_OF_ADCS();
    output_low(ADCS_PWR);
    output_low(EN_SUP_3V3_DAQ);
    delay_ms(1000);
    fprintf(EXT, "Reading chip ID of IHS OVCAM\n");
    output_high(EN_SUP_3V3_2);
    output_high(OVCAM_PWR);  // Power ON OVCAM
    delay_ms(1000);
    READ_CHIP_ID_OF_IHS_OVCAM();
    fprintf(EXT, "Reading chip ID of IHS MVCAM\n");
    delay_ms(1000);
    READ_CHIP_ID_OF_IHS_MVCAM();
    delay_ms(1000);
    output_low(OVCAM_PWR);  // Power OFF OVCAM
    output_low(EN_SUP_3V3_2);
    fprintf(EXT, "Done reading chip ID\n");

    //------------------------write_flash_memory--------------------------
    // Write and read from MAIN flash memory 
    fprintf(EXT, "Starting to write data in MAIN flash memory\n");
    WRITE_DATA_NBYTES(0x00005000, bichig, sizeof(bichig));
    delay_ms(1000);
    
    read_data = READ_DATA_NBYTES(0x00005000, sizeof(bichig));
    delay_ms(1000);
    for (int i = 0; i < sizeof(bichig); i++) {
        fprintf(EXT, "%c", read_data[i]);
        delay_ms(2);
    }
    fprintf(EXT, "\n"); 

    // Write and read from COM flash memory
    fprintf(EXT, "Starting to write data in COM flash memory\n");
    WRITE_DATA_NBYTES_COM(0x00005000, bichigcom, sizeof(bichigcom));
    delay_ms(1000);
    
    read_data_com = READ_DATA_NBYTES_COM(0x00005000, sizeof(bichigcom));
    delay_ms(1000);
    for (int i = 0; i < sizeof(bichigcom); i++) {
        
        fprintf(EXT, "%c", read_data_com[i]);
        delay_ms(2);
    }
    fprintf(EXT, "\nMAIN TEST IS FINISHED!\n");
    //------------------------send_and_write_adcs_mission--------------------------
    fprintf(EXT, "Starting ADCS mission mode\n");
    ADCS_GET_RAW_HK(hk_gyro_x, hk_gyro_y, hk_gyro_z, hk_mag_x, hk_mag_y, hk_mag_z);
    delay_ms(1000);
     //------------------------send_and_plot_IHC_miss--------------------------
    eps_mission_mode();
    delay_ms(1000);
    unsigned int8 IHS_CMD[4] = {0xF0, 0x04, 0x00, 0x01}; // CAM1, 4s delay, 0s interval, 1 picture
    IHS_Test_Sequence(IHS_CMD);
    delay_ms(1000);
    fprintf(EXT, "IHS TEST IS FINISHED!\n");
    delay_ms(1000);

    //------------------------MAIN_MENU-------------------------------
    while (TRUE) {
        if (kbhit(EXT)) {
            main_menu();
            //  LOOP_DC_STATUS_ADDRESS();
            // fprintf(EXT, "%d\n",FLAG_DATA_ADDRESS_END);
        }
    }
}