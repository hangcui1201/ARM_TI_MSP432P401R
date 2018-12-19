/******************************************************************************
* Author: Hang Cui
* Email: cuihang1201@gmail.com
* University of Illinois at Urbana-Champaign
*******************************************************************************/

#include <stdint.h>
#include "msp.h"
#include "Clock.h"

/*
 * Switch from left to right - 6, 5, 4, 3, 2, 1
 * SW6 - P4.7
 * SW5 - P4.6
 * SW4 - P4.5
 * SW3 - P4.3
 * SW2 - P4.2
 * SW1 - P4.0
 * 0b11101101 == 0xED
 */

#define Bump_SW1  0x01  //0000 0001
#define Bump_SW2  0x04  //0000 0100
#define Bump_SW3  0x08  //0000 1000
#define Bump_SW4  0x20  //0010 0000
#define Bump_SW5  0x40  //0100 0000
#define Bump_SW6  0x80  //1000 0000

#define L_SW      0x02     // On the left side of the LaunchPad board
#define R_SW      0x10     // On the right side of the LaunchPad board

// Built-in red LED connected to P2.0
// Built-in green LED connected to P2.1
// Built-in blue LED connected to P2.2

// Color    LED(s) Port2
// dark     ---    0
// red      R--    0x01
// blue     --B    0x04
// green    -G-    0x02
// yellow   RG-    0x03
// sky blue -GB    0x06
// white    RGB    0x07
// pink     R-B    0x05
#define DARK      0x00
#define RED       0x01
#define GREEN     0x02
#define BLUE      0x04


/******************** MSP432 Built-in LED and Switches ********************/

// MSP432 Launchpad LEDs P2.0 (R) P2.1 (G) P2.2 (B)
void MSP432_Launchpad_ColorLED_Init(void){
    P2->SEL0 &= ~0x07;  // Configure P2.2-P2.0 as GPIO
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x07;    // Make P2.0 P2.1 P2.2 as output
    P2->DS |= 0x07;     // Make them high current
    P2->OUT &= ~0x07;   // All three LEDs off
}

void MSP432_Launchpad_ColorLED_Write(uint8_t data){
    // Write P2.0 - P2.2 outputs
    P2->OUT = (P2->OUT&0xF8) | data;
}

void MSP432_Launchpad_RedLED_Init(void){
    P1->SEL0 &= ~0x01;
    P1->SEL1 &= ~0x01;   // Configure P1.0 as GPIO
    P1->DIR |= 0x01;     // Make P1.0 out
    P1->OUT &= ~0x01;    // Set Red LED off
}

void MSP432_Launchpad_RedLED_Write(uint8_t data){
    // Write output to P1.0
    P1->OUT = (P1->OUT&0xFE)|data;
}

void MSP432_Launchpad_SW_Init(void){
  P1->SEL0 &= ~0x12;
  P1->SEL1 &= ~0x12;   // Configure P1.4  P1.1 P1.0 as GPIO
  P1->DIR &= ~0x12;    // Make P1.4 and P1.1 in
  P1->REN |= 0x12;     // Enable pull resistors on P1.4 and P1.1
  P1->OUT |= 0x12;     // P1.4 and P1.1 are pull-up
}

uint8_t MSP432_Launchpad_SW_Read(void){
  return (P1->IN&0x12);   // Read P1.4,P1.1 inputs
}

/**************************************************************************/


/**************************** External Sensors ****************************/

void BumpSwitch_Init(void){
    // Make corresponding GPIO as input
    P4->SEL0 &= ~0xED;  // Clear bit 1 for GPIO
    P4->SEL1 &= ~0xED;  // Clear bit 1 for GPIO
    P4->DIR &= ~0xED;   // 0 for input, 1 for output
    P4->REN |= 0xED;    // Enable pull resistors on P1.4 and P1.1
    P4->OUT |= 0xED;    // Pull-up
}

uint8_t BumpSwitch_Read(void){
    return (P4->IN&0xED);
}

/**************************************************************************/


int main(void){

    uint8_t status;

    Clock_Init48MHz(); // Makes it 48 MHz

    BumpSwitch_Init();
    MSP432_Launchpad_ColorLED_Init();
    MSP432_Launchpad_SW_Init();

    while(1){

        status = BumpSwitch_Read(); // Read all the state of all bump switches

        // Pressed on Bump_SW1 - RED
        if((status&Bump_SW1) == 0x00){
            MSP432_Launchpad_ColorLED_Write(RED);
            Clock_Delay1ms(100);
            MSP432_Launchpad_ColorLED_Write(DARK);
            Clock_Delay1ms(100);
        }

        // Pressed on Bump_SW2 - GREEN
        if((status&Bump_SW2) == 0x00){
            MSP432_Launchpad_ColorLED_Write(GREEN);
            Clock_Delay1ms(100);
            MSP432_Launchpad_ColorLED_Write(DARK);
            Clock_Delay1ms(100);
        }

        // Pressed on Bump_SW3 - BLUE
        if((status&Bump_SW3) == 0x00){
            MSP432_Launchpad_ColorLED_Write(BLUE);
            Clock_Delay1ms(100);
            MSP432_Launchpad_ColorLED_Write(DARK);
            Clock_Delay1ms(100);
        }

        // Pressed on Bump_SW4 - RED+GREEN = YELLOW
        if((status&Bump_SW4) == 0x00){
            MSP432_Launchpad_ColorLED_Write(RED+GREEN);
            Clock_Delay1ms(100);
            MSP432_Launchpad_ColorLED_Write(DARK);
            Clock_Delay1ms(100);
        }

        // Pressed on Bump_SW5 - RED+GREEN+BLUE=WHITE
        if((status&Bump_SW5) == 0x00){
            MSP432_Launchpad_ColorLED_Write(RED+GREEN+BLUE);
            Clock_Delay1ms(100);
            MSP432_Launchpad_ColorLED_Write(DARK);
            Clock_Delay1ms(100);
        }

        // Pressed on Bump_SW6 - RED+BLUE=PINK
        if((status&Bump_SW6) == 0x00){
            MSP432_Launchpad_ColorLED_Write(RED+BLUE);
            Clock_Delay1ms(100);
            MSP432_Launchpad_ColorLED_Write(DARK);
            Clock_Delay1ms(100);
        }

    }

}



















