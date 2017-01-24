// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------


/**
* \file         command_processing.c
*

* \brief        Command processing functions.
* \date         May 16, 2016
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <command_processing.h>
#include <interruptions.h>
#include <stdio.h>
#include <utils.h>
#include <IMU_functions.h>
#include <globals.h>

#include "commands.h"

//================================================================     variables

reg8 * EEPROM_ADDR = (reg8 *) CYDEV_EE_BASE;

//==============================================================================
//                                                            RX DATA PROCESSING
//==============================================================================
//  This function checks for the availability of a data packet and process it:
//      - Verify checksum;
//      - Process commands;
//==============================================================================

void commProcess(){

    uint8 CYDATA rx_cmd;

    rx_cmd = g_rx.buffer[0];

//==========================================================     verify checksum

    if (!(LCRChecksum(g_rx.buffer, g_rx.length - 1) == g_rx.buffer[g_rx.length - 1])){
        // Wrong checksum
        g_rx.ready = 0;
        return;
    }


    switch(rx_cmd) {

//=============================================================     CMD_ACTIVATE
        case CMD_ACTIVATE:
            cmd_activate();
            break;

//===========================================================     CMD_SET_INPUTS

        case CMD_SET_INPUTS:
            cmd_set_inputs();
            break;

//=====================================================     CMD_GET_MEASUREMENTS

        case CMD_GET_MEASUREMENTS:
            cmd_get_measurements();
            break;

//=========================================================     CMD_GET_CURRENTS

        case CMD_GET_CURRENTS:
            cmd_get_currents();
            break;


//=========================================================     CMD_GET_EMG

        case CMD_GET_EMG:
            cmd_get_emg();
            break;

//=============================================================     CMD_WATCHDOG
            
        case CMD_SET_WATCHDOG:
            cmd_set_watchdog();
            break;
            
//=========================================================     CMD_GET_ACTIVATE
            
        case CMD_GET_ACTIVATE:
            cmd_get_activate();
            break;
            
//=========================================================     CMD_SET_BAUDRATE
            
        case CMD_SET_BAUDRATE:
            cmd_set_baudrate();
            break;  
            
//============================================================     CMD_GET_INPUT

        case CMD_GET_INPUTS:
            cmd_get_inputs();
            break;

//=============================================================     CMD_GET_INFO

        case CMD_GET_INFO:
            infoGet( *((uint16 *) &g_rx.buffer[1]));
            break;

//============================================================     CMD_SET_PARAM
            
        case CMD_SET_ZEROS:
            setZeros();
            break;

//============================================================     CMD_GET_PARAM
            
        case CMD_GET_PARAM_LIST:
            get_param_list( *((uint16 *) &g_rx.buffer[1]));
            break;

//=================================================================     CMD_PING
            
        case CMD_PING:
            cmd_ping();
            break;

//=========================================================     CMD_STORE_PARAMS
            
        case CMD_STORE_PARAMS:
            cmd_store_params();
            break;

//=================================================     CMD_STORE_DEFAULT_PARAMS

        case CMD_STORE_DEFAULT_PARAMS:
            if(memStore(DEFAULT_EEPROM_DISPLACEMENT))
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=======================================================     CMD_RESTORE_PARAMS

        case CMD_RESTORE_PARAMS:
            if(memRestore())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=============================================================     CMD_INIT_MEM

        case CMD_INIT_MEM:
            if(memInit())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//===========================================================     CMD_BOOTLOADER

        case CMD_BOOTLOADER:
            //Not sure if ACK_OK is correct, should check
            sendAcknowledgment(ACK_OK);
            CyDelay(1000);
            FTDI_ENABLE_REG_Write(0x00);
            CyDelay(1000);
            Bootloadable_Load();
            break;

//============================================================     CMD_CALIBRATE

        case CMD_CALIBRATE:
            break;
            
//=====================================================     CMD_GET_IMU_READINGS

        case CMD_GET_IMU_READINGS:
            cmd_get_imu_readings();
            break;
                 
            
//=========================================================== ALL OTHER COMMANDS
        default:
            break;
            
    }
    
}


//==============================================================================
//                                                                     INFO SEND
//==============================================================================

void infoSend(){
    unsigned char packet_string[1100];
    infoPrepare(packet_string);
    UART_RS485_PutString(packet_string);
}


//==============================================================================
//                                                              COMMAND GET INFO
//==============================================================================

void infoGet(uint16 info_type) {
    static unsigned char packet_string[1100];

    //==================================     choose info type and prepare string

    switch (info_type) {
        case INFO_ALL:
            infoPrepare(packet_string);
            UART_RS485_PutString(packet_string);
            break;

        default:
            break;
    }
}

//==============================================================================
//                                                                     SET ZEROS
//==============================================================================

void setZeros()
{
    //nothing to do and nothing to say
}

//==============================================================================
//                                                            GET PARAMETER LIST
//==============================================================================

void get_param_list(uint16 index)
{
    //Package to be sent variables
    uint8 packet_data[200 + 50*N_IMU_MAX +1] = "";
    uint16 packet_lenght = 200 + (uint16)(50*N_IMU_Connected) +1;

    //Auxiliary variables
    uint16 CYDATA i, j;

    //Parameters menu string definitions
    char id_str[15]             = "1 - Device ID:";
    char n_imu_str[30]          = "2 - Number of connected IMUs:";
    char ids_str[13]            = "3 - IMUs ID:";
    char mag_param_str[24]      = "4 - Mag cal parameters:";
    char imu_table_str[42]      = "";

    //Strings lenghts
    uint8 CYDATA id_str_len = strlen(id_str);
    uint8 CYDATA n_imu_str_len = strlen(n_imu_str);
    uint8 CYDATA ids_str_len = strlen(ids_str);
    uint8 CYDATA mag_param_str_len = strlen(mag_param_str);
    uint8 CYDATA imu_table_str_len = strlen(imu_table_str);

    packet_data[0] = CMD_GET_PARAM_LIST;
    packet_data[1] = 4 + (uint8)N_IMU_Connected;        // NUM_PARAMS

    switch(index) {
        case 0:         //List of all parameters with relative types
            /*-----------------ID-----------------*/

            packet_data[2] = TYPE_UINT8;
            packet_data[3] = 1;
            packet_data[4] = c_mem.id;
            for(i = id_str_len; i != 0; i--)
                packet_data[5 + id_str_len - i] = id_str[id_str_len - i];

            /*-------------N IMU--------------*/

            packet_data[52] = TYPE_UINT8;
            packet_data[53] = 1;
            packet_data[54] = (uint8)N_IMU_Connected;
            for(i = n_imu_str_len; i != 0; i--)
                packet_data[55 + n_imu_str_len - i] = n_imu_str[n_imu_str_len - i];
                
            /*-------------IMUS ID--------------*/

            packet_data[102] = TYPE_UINT8;
            packet_data[103] = (uint8)N_IMU_Connected;
            for (i=0; i < N_IMU_Connected; i++) {
                packet_data[104 + i] = (uint8)IMU_connected[i];
            }
            for(j = ids_str_len; j != 0; j--)
                packet_data[104 + N_IMU_Connected + ids_str_len - j] = ids_str[ids_str_len - j];
                
            /*-------------GET MAG PARAM--------------*/

            packet_data[152] = TYPE_UINT8;
            packet_data[153] = (uint8)(3*N_IMU_Connected);
            for (i=0; i< N_IMU_Connected; i++) {
                packet_data[154 + 3*i] = (uint8) MagCal[IMU_connected[i]][0];
                packet_data[155 + 3*i] = (uint8) MagCal[IMU_connected[i]][1];
                packet_data[156 + 3*i] = (uint8) MagCal[IMU_connected[i]][2];
            }
            for(j = mag_param_str_len; j != 0; j--)
                packet_data[154 + 3*N_IMU_Connected + mag_param_str_len - j] = mag_param_str[mag_param_str_len - j];
            
            /*-------------GET IMUS MODE-------------*/
           for (i = 0; i < (uint8)N_IMU_Connected; i++){

                sprintf(imu_table_str, "%u - IMU %d configuration:", 5+i, (int) IMU_connected[i]);
                imu_table_str_len = strlen(imu_table_str);
            
                packet_data[(uint16)(202 + 50*i)] = TYPE_UINT8;
                packet_data[(uint16)(203 + 50*i)] = 5;
                
                packet_data[(uint16)(204 + 50*i)] = (uint8)(IMU_conf[IMU_connected[i]][0]);
                packet_data[(uint16)(205 + 50*i)] = (uint8)(IMU_conf[IMU_connected[i]][1]);
                packet_data[(uint16)(206 + 50*i)] = (uint8)(IMU_conf[IMU_connected[i]][2]);
                packet_data[(uint16)(207 + 50*i)] = (uint8)(IMU_conf[IMU_connected[i]][3]);
                packet_data[(uint16)(208 + 50*i)] = (uint8)(IMU_conf[IMU_connected[i]][4]);

                for(j = imu_table_str_len; j != 0; j--)
                    packet_data[(uint16)(209 + 50*i + imu_table_str_len - j)] = imu_table_str[imu_table_str_len - j];
            }  
          

            /*------------PARAMETERS MENU-----------*/

            packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
            commWrite(packet_data, packet_lenght);
            UART_RS485_ClearTxBuffer();
        break;

//===================================================================     set_id
        case 1:         //ID - uint8
            g_mem.id = g_rx.buffer[3];
        break;
        
//=========================================================     n_imu
        case 2:         //N imu - uint8 (read only)
        break;

//=========================================================     imus_id
        case 3:         //IMUs ID - uint8 (read only)
        break;
            
//=========================================================     get_mag_param
        case 4:         //get_mag_param - (read only)
        break;

//=========================================================     set_imu_mode
        default:         //Set Imu table (index > = 5)
            IMU_conf[IMU_connected[index-5]][0] = g_rx.buffer[3];
            IMU_conf[IMU_connected[index-5]][1] = g_rx.buffer[4];
            IMU_conf[IMU_connected[index-5]][2] = g_rx.buffer[5];
            IMU_conf[IMU_connected[index-5]][3] = g_rx.buffer[6];
            IMU_conf[IMU_connected[index-5]][4] = g_rx.buffer[7];
            
            // Recompute IMU packets dimension
            imus_data_size = 1; //header    
            for (i = 0; i < N_IMU_Connected; i++)
            {
                single_imu_size[IMU_connected[i]] = 1 + 6*IMU_conf[IMU_connected[i]][0] + 6*IMU_conf[IMU_connected[i]][1] + 6*IMU_conf[IMU_connected[i]][2] + 8*IMU_conf[IMU_connected[i]][3] + 2*IMU_conf[IMU_connected[i]][4]+ 1;
                imus_data_size = imus_data_size + single_imu_size[IMU_connected[i]];
            }
            imus_data_size = imus_data_size + 1;    //checksum
            
        break;
            
    }
}


//==============================================================================
//                                                           PREPARE DEVICE INFO
//==============================================================================

void infoPrepare(unsigned char *info_string)
{
    int i;
    if(c_mem.id != 250){                //To avoid dummy board ping
        unsigned char str[100];
        strcpy(info_string, "");
        strcat(info_string, "\r\n");
        strcat(info_string, "Firmware version: ");
        strcat(info_string, VERSION);
        strcat(info_string, "\r\n\r\n");

        strcat(info_string, "DEVICE INFO\r\n");
        sprintf(str, "ID: %d\r\n", (int) c_mem.id);
        strcat(info_string, str);
     
        sprintf(str, "IMU Connected: %d\r\n", (int) N_IMU_Connected);
        strcat(info_string, str);
        
        strcat(info_string, "\r\n");
        
        
        strcat(info_string, "IMUs CONFIGURATION\r\n");
        for (i=0; i<N_IMU_Connected; i++){
            sprintf(str, "Imu %d \r\n\tID: %d\r\n", i, (int) IMU_connected[i]);
            strcat(info_string, str);
            
            sprintf(str, "\tAccelerometers: ");
            if ((IMU_conf[IMU_connected[i]][0]))
                strcat(str, "YES\r\n");
            else
                strcat(str, "NO\r\n"); 
            strcat(str, "\tGyroscopes: ");
            if ((IMU_conf[IMU_connected[i]][1]))
                strcat(str, "YES\r\n");
            else
                strcat(str, "NO\r\n"); 
            strcat(str, "\tMagnetometers: ");
            if ((IMU_conf[IMU_connected[i]][2]))
                strcat(str, "YES\r\n");
            else
                strcat(str, "NO\r\n");
            strcat(str, "\tQuaternion: ");                
            if ((IMU_conf[IMU_connected[i]][3]))
                strcat(str, "YES\r\n");
            else
                strcat(str, "NO\r\n");
            strcat(str, "\tTemperature: ");
            if ((IMU_conf[IMU_connected[i]][4]))
                strcat(str, "YES\r\n");
            else
                strcat(str, "NO\r\n");
            
            strcat(info_string, str);
        }       
        
        strcat(info_string, "\r\n");
            
        strcat(info_string, "SENSORS INFO\r\n");
        for (i=0; i<N_IMU_Connected; i++){
            sprintf(str, "Imu %d \r\n\tID: %d\r\n", i, (int) IMU_connected[i]);
            strcat(info_string, str);
            
            if ((IMU_conf[IMU_connected[i]][0])){
                sprintf(str, "\tAcc: %d\t%d\t%d\r\n", (int16) g_imu[i].accel_value[0], (int16) g_imu[i].accel_value[1],(int16) g_imu[i].accel_value[2]);
                strcat(info_string, str);
            }

            if ((IMU_conf[IMU_connected[i]][1])){
                sprintf(str, "\tGyro: %d\t%d\t%d\r\n", (int16) g_imu[i].gyro_value[0], (int16) g_imu[i].gyro_value[1],(int16) g_imu[i].gyro_value[2]);
                strcat(info_string, str);
            }

            if ((IMU_conf[IMU_connected[i]][2])){
                sprintf(str, "\tMag: %d\t%d\t%d\r\n", (int16) g_imu[i].mag_value[0], (int16) g_imu[i].mag_value[1],(int16) g_imu[i].mag_value[2]);
                strcat(info_string, str);
            }
            
            if ((IMU_conf[IMU_connected[i]][3])){
                sprintf(str, "\tQuat: %d\t%d\t%d\t%d\r\n", (int16) g_imu[i].quat_value[0], (int16) g_imu[i].quat_value[1],(int16) g_imu[i].quat_value[2], (int16) g_imu[i].quat_value[3]);
                strcat(info_string, str);
            }
            
            if ((IMU_conf[IMU_connected[i]][4])){
                sprintf(str, "\tTemperature: %d\r\n", (int16) g_imu[i].temp_value);
                strcat(info_string, str);
            }
        
        }
        
        sprintf(str, "debug: %ld", (uint32)timer_value0 - (uint32)timer_value); //5000001
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        strcat(info_string, "\r\n");
    }
}

//==============================================================================
//                                                      WRITE FUNCTION FOR RS485
//==============================================================================

void commWrite(uint8 *packet_data, const uint16 packet_lenght)
{
    uint16 CYDATA index;

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    
    // frame - ID
    UART_RS485_PutChar(g_mem.id);
    
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index)
        UART_RS485_PutChar(packet_data[index]);
    
    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}


//==============================================================================
//                                                             CHECKSUM FUNCTION
//==============================================================================

// Performs a XOR byte by byte on the entire vector

uint8 LCRChecksum(uint8 *data_array, uint8 data_length) {
    
    uint8 CYDATA i;
    uint8 CYDATA checksum = 0x00;
    
    for(i = 0; i < data_length; ++i)
       checksum ^= data_array[i];
  
    return checksum;
}

//==============================================================================
//                                                       ACKNOWLEDGMENT FUNCTION
//==============================================================================

void sendAcknowledgment(const uint8 value) {
    int packet_lenght = 2;
    uint8 packet_data[2];

    packet_data[0] = value;
    packet_data[1] = value;

    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                                  STORE MEMORY
//==============================================================================

/**
* This function stores current memory settings on the eeprom with the specified
* displacement
**/

uint8 memStore(int displacement)
{
    int i;  // iterator
    uint8 writeStatus;
    int pages;
    uint8 ret_val = 1;

    // Disable Interrupt
    ISR_RS485_RX_Disable();


    // Retrieve temperature for better writing performance
    CySetTemp();

    memcpy( &c_mem, &g_mem, sizeof(g_mem) );

    pages = sizeof(g_mem) / 16 + (sizeof(g_mem) % 16 > 0);

    for(i = 0; i < pages; ++i) {
        writeStatus = EEPROM_Write(&g_mem.flag + 16 * i, i + displacement);
        if(writeStatus != CYRET_SUCCESS) {
            ret_val = 0;
            break;
        }
    }

    memcpy( &g_mem, &c_mem, sizeof(g_mem) );

    // Re-Enable Interrupt
    ISR_RS485_RX_Enable();

    return ret_val;
}


//==============================================================================
//                                                                 RECALL MEMORY
//==============================================================================

/**
* This function loads user settings from the eeprom.
**/

void memRecall()
{
    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        memRestore();
    } else {
        memcpy( &c_mem, &g_mem, sizeof(g_mem) );
    }
}


//==============================================================================
//                                                                RESTORE MEMORY
//==============================================================================

/**
* This function loads default settings from the eeprom.
**/

uint8 memRestore() {
    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i + (DEFAULT_EEPROM_DISPLACEMENT * 16)];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        return memInit();
    } else {
        return memStore(0);
    }
}

//==============================================================================
//                                                                   MEMORY INIT
//==============================================================================

/**
* This function initialize memory when eeprom is compromised.
**/

uint8 memInit()
{
  
    //initialize memory settings
    g_mem.id            = 1;

    
    // set the initialized flag to show EEPROM has been populated
    g_mem.flag = TRUE;
    
    //write that configuration to EEPROM
    return ( memStore(0) && memStore(DEFAULT_EEPROM_DISPLACEMENT) );
}

//==============================================================================
//                                                    ROUTINE INTERRUPT FUNCTION
//==============================================================================
/**
* Bunch of functions used on request from UART communication
**/

void cmd_get_measurements(){
   //nothing to do and nothing to say 
}

void cmd_set_inputs(){
   //nothing to do and nothing to say
}

void cmd_activate(){
    //nothing to do and nothing to say
}

void cmd_get_activate(){
    //nothing to do and nothing to say
}

void cmd_get_curr_and_meas(){
    //nothing to do and nothing to say   
}

void cmd_get_currents(){
    //nothing to do and nothing to say
}

void cmd_set_baudrate(){
    
    // Set BaudRate
    c_mem.baud_rate = g_rx.buffer[1];
    
    switch(g_rx.buffer[1]){
        case 13:
            CLOCK_UART_SetDividerValue(13);
            break;
        default:
            CLOCK_UART_SetDividerValue(3);
    }
}

void cmd_ping(){

    uint8 packet_data[2];

    // Header
    packet_data[0] = CMD_PING;
    
    // Load Payload
    packet_data[1] = CMD_PING;

    // Send Package to uart
    commWrite(packet_data, 2);
}

void cmd_set_watchdog(){
      
    if (g_rx.buffer[1] <= 0){
        // Deactivate Watchdog
        WATCHDOG_ENABLER_Write(1); 
        g_mem.watchdog_period = 0;   
    }
    else{
        // Activate Watchdog        
        if (g_rx.buffer[1] > MAX_WATCHDOG_TIMER)
            g_rx.buffer[1] = MAX_WATCHDOG_TIMER;
            
        // Period * Time_CLK = WDT
        // Period = WTD / Time_CLK =     (WTD    )  / ( ( 1 / Freq_CLK ) )
        // Set request watchdog period - (WTD * 2)  * (250 / 1024        )
        g_mem.watchdog_period = (uint8) (((uint32) g_rx.buffer[1] * 2 * 250 ) >> 10);   
        WATCHDOG_COUNTER_WritePeriod(g_mem.watchdog_period); 
        WATCHDOG_ENABLER_Write(0); 
    }
}

void cmd_get_inputs(){
    //nothing to do and nothing to say
}

void cmd_store_params(){
   
    if(memStore(0))
        sendAcknowledgment(ACK_OK);
    else
        sendAcknowledgment(ACK_ERROR);
}

void cmd_get_emg(){
    //nothing to do and nothing to say
}

void cmd_get_imu_readings(){
    //Retrieve accelerometers, gyroscopes and magnetometers readings
    
    uint8 k_imu;
    uint8 c = 1;
    uint8 k = 0;
    uint8 gl_c = 1;
    
    // Packet: header + imu id(uint8) + imu flags(uint8) + crc  
    uint8 packet_data[520];
    uint8 single_packet[30];
    
    // Packet to send is handled here (in this way you work with last consistent data)
    //isr_imu_Disable();
    
    //Header package 
    packet_data[0] = CMD_GET_IMU_READINGS;
 
    for (k_imu = 0; k_imu < N_IMU_Connected; k_imu++) 
    {	
        
        single_packet[0] = (uint8) 0x3A; //':';
        if (IMU_conf[IMU_connected[k_imu]][0]){
            *((int16 *) &single_packet[c]) = (int16) g_imu[k_imu].accel_value[0];
            *((int16 *) &single_packet[c+2]) = (int16) g_imu[k_imu].accel_value[1];
            *((int16 *) &single_packet[c+4]) = (int16) g_imu[k_imu].accel_value[2];
            c = c + 6;
        }
        if (IMU_conf[IMU_connected[k_imu]][1]){
            *((int16 *) &single_packet[c])   = (int16) g_imu[k_imu].gyro_value[0];
            *((int16 *) &single_packet[c+2]) = (int16) g_imu[k_imu].gyro_value[1];
            *((int16 *) &single_packet[c+4]) = (int16) g_imu[k_imu].gyro_value[2];
            c = c + 6;
        }
        if (IMU_conf[IMU_connected[k_imu]][2]){
            *((int16 *) &single_packet[c])   = (int16) g_imu[k_imu].mag_value[0];
            *((int16 *) &single_packet[c+2]) = (int16) g_imu[k_imu].mag_value[1];
            *((int16 *) &single_packet[c+4]) = (int16) g_imu[k_imu].mag_value[2];
            c = c + 6;
        }
        if (IMU_conf[IMU_connected[k_imu]][3]){
            *((int16 *) &single_packet[c])   = (int16) g_imu[k_imu].quat_value[0];
            *((int16 *) &single_packet[c+2]) = (int16) g_imu[k_imu].quat_value[1];
            *((int16 *) &single_packet[c+4]) = (int16) g_imu[k_imu].quat_value[2];
            *((int16 *) &single_packet[c+6]) = (int16) g_imu[k_imu].quat_value[3];
            c = c + 8;
        }
        if (IMU_conf[IMU_connected[k_imu]][4]){
            *((int16 *) &single_packet[c])   = (int16) g_imu[k_imu].temp_value;
            c = c + 2;
        }
        single_packet[single_imu_size[IMU_connected[k_imu]] - 1] = (uint8) 0x3A; //':';
        c = 1;
               
        for(k=0; k < single_imu_size[IMU_connected[k_imu]]; k++) {
            packet_data[gl_c + k] = (uint8) single_packet[k]; 
        }
        gl_c = gl_c + single_imu_size[IMU_connected[k_imu]];  
        
        memset(&single_packet, 0, sizeof(single_packet));     
        
    }

    // Calculate Checksum and send message to UART 
    packet_data[imus_data_size-1] = LCRChecksum (packet_data, imus_data_size-1);
    commWrite(packet_data, imus_data_size);
    
    //isr_imu_Enable();
}


/* [] END OF FILE */