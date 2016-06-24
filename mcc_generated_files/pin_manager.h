/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB� Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC16F1827
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set I_TEST aliases
#define I_TEST_TRIS               TRISA0
#define I_TEST_LAT                LATA0
#define I_TEST_PORT               RA0
#define I_TEST_ANS                ANSA0
#define I_TEST_SetHigh()    do { LATA0 = 1; } while(0)
#define I_TEST_SetLow()   do { LATA0 = 0; } while(0)
#define I_TEST_Toggle()   do { LATA0 = ~LATA0; } while(0)
#define I_TEST_GetValue()         RA0
#define I_TEST_SetDigitalInput()    do { TRISA0 = 1; } while(0)
#define I_TEST_SetDigitalOutput()   do { TRISA0 = 0; } while(0)

#define I_TEST_SetAnalogMode()   do { ANSA0 = 1; } while(0)
#define I_TEST_SetDigitalMode()   do { ANSA0 = 0; } while(0)
// get/set O_MSG aliases
#define O_MSG_TRIS               TRISA1
#define O_MSG_LAT                LATA1
#define O_MSG_PORT               RA1
#define O_MSG_ANS                ANSA1
#define O_MSG_SetHigh()    do { LATA1 = 1; } while(0)
#define O_MSG_SetLow()   do { LATA1 = 0; } while(0)
#define O_MSG_Toggle()   do { LATA1 = ~LATA1; } while(0)
#define O_MSG_GetValue()         RA1
#define O_MSG_SetDigitalInput()    do { TRISA1 = 1; } while(0)
#define O_MSG_SetDigitalOutput()   do { TRISA1 = 0; } while(0)

#define O_MSG_SetAnalogMode()   do { ANSA1 = 1; } while(0)
#define O_MSG_SetDigitalMode()   do { ANSA1 = 0; } while(0)
// get/set O_REG_PWR aliases
#define O_REG_PWR_TRIS               TRISA2
#define O_REG_PWR_LAT                LATA2
#define O_REG_PWR_PORT               RA2
#define O_REG_PWR_ANS                ANSA2
#define O_REG_PWR_SetHigh()    do { LATA2 = 1; } while(0)
#define O_REG_PWR_SetLow()   do { LATA2 = 0; } while(0)
#define O_REG_PWR_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define O_REG_PWR_GetValue()         RA2
#define O_REG_PWR_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define O_REG_PWR_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define O_REG_PWR_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define O_REG_PWR_SetDigitalMode()   do { ANSA2 = 0; } while(0)
// get/set O_PWR_CTL aliases
#define O_PWR_CTL_TRIS               TRISA3
#define O_PWR_CTL_LAT                LATA3
#define O_PWR_CTL_PORT               RA3
#define O_PWR_CTL_ANS                ANSA3
#define O_PWR_CTL_SetHigh()    do { LATA3 = 1; } while(0)
#define O_PWR_CTL_SetLow()   do { LATA3 = 0; } while(0)
#define O_PWR_CTL_Toggle()   do { LATA3 = ~LATA3; } while(0)
#define O_PWR_CTL_GetValue()         RA3
#define O_PWR_CTL_SetDigitalInput()    do { TRISA3 = 1; } while(0)
#define O_PWR_CTL_SetDigitalOutput()   do { TRISA3 = 0; } while(0)

#define O_PWR_CTL_SetAnalogMode()   do { ANSA3 = 1; } while(0)
#define O_PWR_CTL_SetDigitalMode()   do { ANSA3 = 0; } while(0)
// get/set channel_AN4 aliases
#define channel_AN4_TRIS               TRISA4
#define channel_AN4_LAT                LATA4
#define channel_AN4_PORT               RA4
#define channel_AN4_ANS                ANSA4
#define channel_AN4_SetHigh()    do { LATA4 = 1; } while(0)
#define channel_AN4_SetLow()   do { LATA4 = 0; } while(0)
#define channel_AN4_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define channel_AN4_GetValue()         RA4
#define channel_AN4_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define channel_AN4_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define channel_AN4_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define channel_AN4_SetDigitalMode()   do { ANSA4 = 0; } while(0)
// get/set O_RED aliases
#define O_RED_TRIS               TRISA6
#define O_RED_LAT                LATA6
#define O_RED_PORT               RA6
#define O_RED_SetHigh()    do { LATA6 = 1; } while(0)
#define O_RED_SetLow()   do { LATA6 = 0; } while(0)
#define O_RED_Toggle()   do { LATA6 = ~LATA6; } while(0)
#define O_RED_GetValue()         RA6
#define O_RED_SetDigitalInput()    do { TRISA6 = 1; } while(0)
#define O_RED_SetDigitalOutput()   do { TRISA6 = 0; } while(0)

// get/set O_BLUE aliases
#define O_BLUE_TRIS               TRISA7
#define O_BLUE_LAT                LATA7
#define O_BLUE_PORT               RA7
#define O_BLUE_SetHigh()    do { LATA7 = 1; } while(0)
#define O_BLUE_SetLow()   do { LATA7 = 0; } while(0)
#define O_BLUE_Toggle()   do { LATA7 = ~LATA7; } while(0)
#define O_BLUE_GetValue()         RA7
#define O_BLUE_SetDigitalInput()    do { TRISA7 = 1; } while(0)
#define O_BLUE_SetDigitalOutput()   do { TRISA7 = 0; } while(0)

// get/set I_PWR_ON aliases
#define I_PWR_ON_TRIS               TRISB0
#define I_PWR_ON_LAT                LATB0
#define I_PWR_ON_PORT               RB0
#define I_PWR_ON_WPU                WPUB0
#define I_PWR_ON_SetHigh()    do { LATB0 = 1; } while(0)
#define I_PWR_ON_SetLow()   do { LATB0 = 0; } while(0)
#define I_PWR_ON_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define I_PWR_ON_GetValue()         RB0
#define I_PWR_ON_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define I_PWR_ON_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define I_PWR_ON_SetPullup()    do { WPUB0 = 1; } while(0)
#define I_PWR_ON_ResetPullup()   do { WPUB0 = 0; } while(0)
// get/set I_EXPWR aliases
#define I_EXPWR_TRIS               TRISB1
#define I_EXPWR_LAT                LATB1
#define I_EXPWR_PORT               RB1
#define I_EXPWR_WPU                WPUB1
#define I_EXPWR_ANS                ANSB1
#define I_EXPWR_SetHigh()    do { LATB1 = 1; } while(0)
#define I_EXPWR_SetLow()   do { LATB1 = 0; } while(0)
#define I_EXPWR_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define I_EXPWR_GetValue()         RB1
#define I_EXPWR_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define I_EXPWR_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define I_EXPWR_SetPullup()    do { WPUB1 = 1; } while(0)
#define I_EXPWR_ResetPullup()   do { WPUB1 = 0; } while(0)
#define I_EXPWR_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define I_EXPWR_SetDigitalMode()   do { ANSB1 = 0; } while(0)
// get/set SDA2 aliases
#define SDA2_TRIS               TRISB2
#define SDA2_LAT                LATB2
#define SDA2_PORT               RB2
#define SDA2_WPU                WPUB2
#define SDA2_ANS                ANSB2
#define SDA2_SetHigh()    do { LATB2 = 1; } while(0)
#define SDA2_SetLow()   do { LATB2 = 0; } while(0)
#define SDA2_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define SDA2_GetValue()         RB2
#define SDA2_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define SDA2_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define SDA2_SetPullup()    do { WPUB2 = 1; } while(0)
#define SDA2_ResetPullup()   do { WPUB2 = 0; } while(0)
#define SDA2_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define SDA2_SetDigitalMode()   do { ANSB2 = 0; } while(0)
// get/set I_CHRG aliases
#define I_CHRG_TRIS               TRISB3
#define I_CHRG_LAT                LATB3
#define I_CHRG_PORT               RB3
#define I_CHRG_WPU                WPUB3
#define I_CHRG_ANS                ANSB3
#define I_CHRG_SetHigh()    do { LATB3 = 1; } while(0)
#define I_CHRG_SetLow()   do { LATB3 = 0; } while(0)
#define I_CHRG_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define I_CHRG_GetValue()         RB3
#define I_CHRG_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define I_CHRG_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define I_CHRG_SetPullup()    do { WPUB3 = 1; } while(0)
#define I_CHRG_ResetPullup()   do { WPUB3 = 0; } while(0)
#define I_CHRG_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define I_CHRG_SetDigitalMode()   do { ANSB3 = 0; } while(0)
// get/set I_FAULT aliases
#define I_FAULT_TRIS               TRISB4
#define I_FAULT_LAT                LATB4
#define I_FAULT_PORT               RB4
#define I_FAULT_WPU                WPUB4
#define I_FAULT_ANS                ANSB4
#define I_FAULT_SetHigh()    do { LATB4 = 1; } while(0)
#define I_FAULT_SetLow()   do { LATB4 = 0; } while(0)
#define I_FAULT_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define I_FAULT_GetValue()         RB4
#define I_FAULT_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define I_FAULT_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define I_FAULT_SetPullup()    do { WPUB4 = 1; } while(0)
#define I_FAULT_ResetPullup()   do { WPUB4 = 0; } while(0)
#define I_FAULT_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define I_FAULT_SetDigitalMode()   do { ANSB4 = 0; } while(0)
// get/set SCL2 aliases
#define SCL2_TRIS               TRISB5
#define SCL2_LAT                LATB5
#define SCL2_PORT               RB5
#define SCL2_WPU                WPUB5
#define SCL2_ANS                ANSB5
#define SCL2_SetHigh()    do { LATB5 = 1; } while(0)
#define SCL2_SetLow()   do { LATB5 = 0; } while(0)
#define SCL2_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define SCL2_GetValue()         RB5
#define SCL2_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define SCL2_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define SCL2_SetPullup()    do { WPUB5 = 1; } while(0)
#define SCL2_ResetPullup()   do { WPUB5 = 0; } while(0)
#define SCL2_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define SCL2_SetDigitalMode()   do { ANSB5 = 0; } while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */