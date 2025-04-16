
#include <main.h>

char bichig[25] = "test data update of MAIN"; //test data for testing 
char bichigcom[24] = "test data update of COM"; //test data for testing 
char bichigadcs[25] = "test data update of ADCS"; //test data for testing 
char *read_data; //MAIN flash received data will be stored in here 
unsigned char buffer[40]; //secondary buffer 
char *read_data_com; //COM flash received data will be stored in here 


void main() {
    //------------------------Start_Indicator-------------------------
    startup_freeze();
    //--------------------------RTC-----------------------------------
    RTC_initialize();
    //------------------------restart_indicator-----------------------
    update_shutdown_count();
    //------------------------read_chip_ID----------------------------
    fprintf(EXT, "Reading chip ID of main\n");
    // READ_CHIP_ID_GENERIC(SPIPORT, CS_PIN_1, -1);  // Pass the array to be filled by the function
    fprintf(EXT, "Reading chip ID of COM\n");
    READ_CHIP_ID_OF_COM();
    fprintf(EXT, "Reading chip ID of ADCS\n");
    READ_CHIP_ID_OF_ADCS();
    fprintf(EXT, "Done reading chip ID\n");
    delay_ms(1000);

    //------------------------write_flash_memory--------------------------
    // Write and read from MAIN flash memory 
    fprintf(EXT, "Starting to write data in MAIN flash memory\n");
    WRITE_DATA_NBYTES (0x00005000, bichig, sizeof(bichig));
    delay_ms(1000);
    
    read_data = 
    (0x00005000, sizeof(bichig));
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
    fprintf(EXT, "\n"); 
    
//    // Write and read from ADCS flash memory
//    fprintf(EXT, "Starting to write data in ADCS flash memory\n");
//    WRITE_DATA_NBYTES_ADCS(0x00005000, bichigadcs, sizeof(bichigadcs));
//    delay_ms(1000);
//    
//    read_data_adcs = READ_DATA_NBYTES_ADCS(0x00005000, sizeof(bichigadcs));
//    delay_ms(1000);
//    for (int i = 0; i < sizeof(bichigadcs); i++) {
//        fprintf(EXT, "%c", read_data_adcs[i]);
//        delay_ms(2);
//    }
//    fprintf(EXT, "\n"); // comment this line because it's not necessary for the adcs mission testing 
    fprintf(EXT, "MAIN TEST IS FINISHED!\n");

    //------------------------send_and_write_adcs_mission--------------------------
//    adcs_mission();
    //------------------------MAIN_MENU-------------------------------
    while (TRUE) {
        if (kbhit(EXT)) {
            main_menu();
        }
    }
}