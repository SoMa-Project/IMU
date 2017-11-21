// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2016, qbrobotics
// Copyright (c) 2017, Centro "E.Piaggio"
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------

/**
* \file         main.c
*
* \brief        Firmware main file.
* \date         October 01, 2017
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
* \mainpage     Firmware
* \brief        This is the firmware of the IMU board.
* \version      1.0
*
* \details      This is the firmware of the IMU board. It can read up to 17 IMU modules
*               connected to the PSoC microcontroller.                
*
*/


// ----------------------------------------------------------------------------
// This version changes:
//      - not reported


//=================================================================     includes

#include <device.h>
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE
#include <interruptions.h>
#include <command_processing.h>
#include <utils.h>
#include <IMU_functions.h>

//==============================================================================
//                                                                 MAIN FUNCTION
//==============================================================================

int main()
{
    // Iterator
    uint8 i;         
    
    // Variable declarations for DMA 
    
    uint8 DMA_Chan;
    uint8 DMA_TD[1];

    // Variable declarations for IMU 
    
    int k_imu;
    //================================     initializations - psoc and components

    // EEPROM

    EEPROM_Start();
    memRecall();                                        // recall configuration

    // FTDI chip enable

    CyDelay(100);
    FTDI_ENABLE_REG_Write(0x01);
    
    // RS485

    UART_RS485_Stop();                                  // stop UART
    UART_RS485_Start();                                 // start UART
    UART_RS485_Init();

    UART_RS485_ClearRxBuffer();
    UART_RS485_ClearTxBuffer();

    ISR_RS485_RX_StartEx(ISR_RS485_RX_ExInterrupt);     // RS485 isr function
    
    // WATCHDOG
    
    WATCHDOG_COUNTER_Start();
    
    ISR_WATCHDOG_StartEx(ISR_WATCHDOG_Handler);         // WATCHDOG isr function    

    RS485_CTS_Write(0);                                 // Clear To Send on RS485

    // TIMER

    MY_TIMER_Start();           
    PACER_TIMER_Start();

    CYGlobalIntEnable;                                  // enable interrupts
    
    //SPI module
	SPI_IMU_Start();
	SPI_IMU_Init();
	SPI_IMU_Enable();
	SPI_IMU_ClearRxBuffer();
	SPI_IMU_ClearTxBuffer();
	SPI_IMU_ClearFIFO();							
    CyDelay(10);
    
    Opto_Pin_Write(1); //start hw spi communication
    
    // Init MPU9250 devices
    InitIMUgeneral();
    
    
    // ISR
//    isr_imu_Start();
//    isr_imu_Enable();
    

    
    

//========================================     initializations - clean variables



    //---------------------------------------------------  Initialize referiment structure
    
                                                                                                // XXX da eliminare e riprogettare g_meas e g_pos 
    
    //---------------------------------------------------  Initialize measurement structure
    
    //---------------------------------------------------  Initialize emg structure
    dev_pwm_limit = 0;                                  // Init PWM limit
    tension_valid = FALSE;                              // Init tension_valid BIT

    reset_last_value_flag = 0;

    //------------------------------------------------- Initialize package on receive from RS485
    g_rx.length = 0;
    g_rx.ready  = 0;

    //============================================================     main loop

    for(;;)
    {
        // Put the FF reset pin to LOW
        RESET_FF_Write(0x00);

        // Call function scheduler
        function_scheduler();

        //  Wait until the FF is set to 1
        while(FF_STATUS_Read() == 0){
            // On interrupt from RS485
            if (interrupt_flag){
                // Reset WDT
                WATCHDOG_REFRESH_Write(0x01);
                // Reset flags
                interrupt_flag = FALSE;
                watchdog_flag = FALSE;
                // Manage Interrupt on rs485
                interrupt_manager();
            }
            // On interrupt from WDT
            else { 
                if (watchdog_flag){
                    // Reset WDT
                    WATCHDOG_REFRESH_Write(0x01);
                   
                }
            }
        };

        // Command a FF reset
        RESET_FF_Write(0x01);

        // Wait for FF to be reset
        while(FF_STATUS_Read() == 1);

        if(UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_SOFT_BUFF_OVER)
            UART_RS485_ClearRxBuffer();
    }
    return 0;
}



/* [] END OF FILE */