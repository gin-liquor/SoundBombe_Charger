/**
  Generated Pin Manager File

  Company:
    Microchip Technology Inc.

  File Name:
    pin_manager.c

  Summary:
    This is the Pin Manager file generated using MPLAB� Code Configurator

  Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC16F1827
        Driver Version    :  1.02
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

#include <xc.h>
#include "pin_manager.h"

void PIN_MANAGER_Initialize(void) {
    LATA = 0x00;
    TRISA = 0x31;
    ANSELA = 0x10;
    WPUA = 0x00;

    LATB = 0x24;
    TRISB = 0xFF;
    ANSELB = 0xC0;
    WPUB = 0x1B;

    OPTION_REGbits.nWPUEN = 0x00;

    APFCON0 = 0x00;

    APFCON1 = 0x00;

    // enable interrupt-on-change individually    
    IOCBN0 = 1;
    IOCBP0 = 1;
    IOCBN1 = 1;
    IOCBP1 = 1;
    IOCBN3 = 1;
    IOCBP3 = 1;
    IOCBN4 = 1;
    IOCBP4 = 1;

    // enable interrupt-on-change globally
    INTCONbits.IOCIE = 1;

}

extern void I_PWR_ON_intr();
extern void I_EXPWR_intr();
extern void I_CHRG_intr();
extern void I_FAULT_intr();
extern void I_TEST1_intr();
extern void I_TEST2_intr();

void PIN_MANAGER_IOC(void) {
    if (((IOCBN0 == 1) || (IOCBP0 == 1)) && (IOCBF0 == 1)) {
        //@TODO Add handling code for IOC on pin RB0
        I_PWR_ON_intr();
        // clear interrupt-on-change flag
        IOCBF0 = 0;
    }
    if (((IOCBN1 == 1) || (IOCBP1 == 1)) && (IOCBF1 == 1)) {
        //@TODO Add handling code for IOC on pin RB1
        I_EXPWR_intr();
        // clear interrupt-on-change flag
        IOCBF1 = 0;
    }
    if (((IOCBN3 == 1) || (IOCBP3 == 1)) && (IOCBF3 == 1)) {
        //@TODO Add handling code for IOC on pin RB3
        I_CHRG_intr();
        // clear interrupt-on-change flag
        IOCBF3 = 0;
    }
    if (((IOCBN4 == 1) || (IOCBP4 == 1)) && (IOCBF4 == 1)) {
        //@TODO Add handling code for IOC on pin RB4
        I_FAULT_intr();
        // clear interrupt-on-change flag
        IOCBF4 = 0;
    }
}
/**
 End of File
 */