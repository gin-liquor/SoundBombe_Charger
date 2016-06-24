/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1827
        Driver Version    :  2.00
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

#include "mcc_generated_files/mcc.h"

void setLedState();
bool stPower = 0, stExPower = 0;
bool stOff = 0, stOn = 0, stCharge = 0, stFault = 0, stLowBattery = 0;
bool stPowerSaveMode = 0;
void initState();
void detectPowerOn();
void detectVBattery();
void detectCharge();
void setLedByLogical();

bool wnLowBattery0 = false;

bool fRun = false;

/*
                         Main application
 */
void main(void) {
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    while (1) {
        __delay_ms(1000);
        if (true) {
            SLEEP();
            NOP();
        }
    }
}

void initState() {
    stPower = 0, stExPower = 0;
    stOff = 0, stOn = 0, stCharge = 0, stFault = 0, stLowBattery = 0;
}

void detectPowerOn() {
    int POWER = I_PWR_ON_GetValue();
    if (!POWER) {
        stPower = true;
    } else {
        //stOff = true;
        //wnLowBattery = true;
    }
    int EXPOWER = I_EXPWR_GetValue();
    if (!EXPOWER) {
        stExPower = true;
    }
    stOff = !stPower;
}

void detectCharge() {
    int CHRG = I_CHRG_GetValue();
    int FALT = I_FAULT_GetValue();
    if (!CHRG) {
        stCharge = true;
    }
    if (!FALT) {
        stFault = true;
    }
}

float vBattery = -1;

void detectVBattery1() {
    O_REG_PWR_SetHigh();
    __delay_ms(300);
    int adcval = ADC_GetConversion(channel_AN4);
    O_REG_PWR_SetLow();
    vBattery = adcval / 1024.0 * 2.445 * 2;
    if (vBattery < 3.2) {
        stLowBattery = true;
    }
}

void detectVBattery() {
    O_REG_PWR_SetHigh();
    __delay_ms(300);
    int adcval = ADC_GetConversion(channel_AN4);
    O_REG_PWR_SetLow();
    vBattery = 2.445 / (adcval / 1023.0);
    if (vBattery < 3.17) {
        stLowBattery = true;
    }
}

void ledPower(bool on) {
    if (on) 
        O_RED_SetHigh();
    else
        O_RED_SetLow();
}

void ledCharge(bool on, bool toggle) {
    if (toggle)
        O_BLUE_Toggle();
    else if (on)
        O_BLUE_SetHigh();
    else
        O_BLUE_SetLow();
}

void psPower(bool on) {
    if (on)
        O_PWR_CTL_SetLow();
    else
        O_PWR_CTL_SetHigh();
}

void ledMessage(bool on) {
    if (on)
        O_MSG_SetHigh();
    else
        O_MSG_SetLow();
}

void setLedState() {
    ledPower(stPower && !(stCharge || stFault));
    ledCharge(stExPower && stCharge, stExPower && stFault);
    psPower(!(stOff || stLowBattery));
}

void TMR0_CallBack() {
    ledMessage(stPowerSaveMode);
    if (stPowerSaveMode) {
        ledPower(false);
        psPower(false);
        return;
    }
    initState();
    detectPowerOn();
    detectCharge();
    detectVBattery();
    setLedState();
    stPowerSaveMode = stLowBattery || stOff;
}

void delay_10ms(int d) {
    for (int i = 0; i < d; i++) {
        __delay_ms(10);
    }
}

//#include "math.h"
void putPwrMsg() {
    detectPowerOn();
    detectCharge();
    if (stPower && !(stCharge || stFault)) {
        detectVBattery();
        int lim = 3.2;
        int pardeci = (vBattery - lim) / (4.0 - lim) * 5;
        for (int i = 0; i < pardeci; i++) {
            O_BLUE_SetHigh();
            __delay_ms(40);
            O_BLUE_SetLow();
            __delay_ms(100);
        }
    }
}

void I_PWR_ON_intr() {
    stPowerSaveMode = false;
    putPwrMsg();
}

void I_EXPWR_intr() {
    stPowerSaveMode = false;
}

void I_CHRG_intr() {
    stPowerSaveMode = false;
}

void I_FAULT_intr() {
    stPowerSaveMode = false;
}

void I_TEST1_intr() {
}

void I_TEST2_intr() {
}

/**
 End of File
 */