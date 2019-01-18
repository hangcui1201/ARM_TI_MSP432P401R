/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Example/Board Header files */
#include "Board.h"

#include "msp432p401r.h"
#include "CortexM.h"
#include "Clock.h"
#include "Nokia5110.h"
#include "PWM.h"
#include "Motor.h"

#include "TA3InputCapture.h"
#include "Tachometer.h"

//#include "ADC14.h"
//#include "TimerA1.h"
//#include "SysTickInts.h"


/***************************** LED Setup *****************************/

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

void MSP432_Launchpad_RedLED_Toggle(){
    // Write output to P1.0
    P1->OUT ^= 0x01;
}

/***************************** LCD Setup and Motor *****************************/

//#define DESIREDMAX 120                   // maximum rotations per minute
//#define DESIREDMIN  30                   // minimum rotations per minute (works poorly at 30 RPM due to 16-bit timer overflow)

uint16_t L_real = 0;    // actual rpm of left motor
uint16_t R_real = 0;    // actual rpm of right motor
uint16_t L_ctrl = 100;  // desired rpm of left motor
uint16_t R_ctrl = 100;  // desired rpm of right motor

//#define TACHBUFF 1                      // number of elements in tachometer array
//
//uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
//enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
//int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
//
//uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
//enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
//int32_t RightSteps;


void LCD_Clear(void){
    Nokia5110_Init();
    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("TiBot");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("Motor Speed");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("Lr="); Nokia5110_OutUDec(0); Nokia5110_OutString("rpm");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("Lc="); Nokia5110_OutUDec(0); Nokia5110_OutString("rpm");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("Rr="); Nokia5110_OutUDec(0); Nokia5110_OutString("rpm");
    Nokia5110_SetCursor(0,5); Nokia5110_OutString("Rc="); Nokia5110_OutUDec(0); Nokia5110_OutString("rpm");
}

void LCD_Out(void){
    Nokia5110_SetCursor(3,2); Nokia5110_OutUDec(L_real);
    Nokia5110_SetCursor(3,3); Nokia5110_OutUDec(L_ctrl);
    Nokia5110_SetCursor(3,4); Nokia5110_OutUDec(R_real);
    Nokia5110_SetCursor(3,5); Nokia5110_OutUDec(R_ctrl);
}

//uint16_t avg(uint16_t *array, int length){
//  int i;
//  uint32_t sum = 0;
//  for(i=0; i<length; i=i+1){
//    sum = sum + array[i];
//  }
//  return (sum/length);
//}


// run every 500ms
void Clock_500ms(void){

    //GPIO_toggle(Board_LED0);
    //GPIO_write(Board_LED0, Board_LED_ON);

    MSP432_Launchpad_RedLED_Toggle();

//    Tachometer_Get(&LeftTach[0], &LeftDir, &LeftSteps, &RightTach[0], &RightDir, &RightSteps);
//
//    L_real = 2000000/LeftTach[0];    //rpm
//    R_real = 2000000/RightTach[0];   //rpm

    LCD_Out();

    L_real += 1;
    R_real += 1;

    if(L_real == 255){
        L_real = 0;
        R_real = 0;
    }

//    Motor_Forward(L_real, R_real);

}

//// run every 5ms for PI speed control
//void Clock_5ms(void){
//
////    Motor_Forward(20, 20);   // Motor_Forward(uint16_t leftDuty, uint16_t rightDuty)
////    LCD_Out();
////    if(speed_ready == 1){
////
////        // PI control
////
////        //
////        LCD_Out();
////        count = 0;
////        speed_ready = 0;
////    }
//
//}
//
//// run every 1ms
//void Clock_1ms(void){
////    if(count < 5){
////        Tachometer_Get(&LeftTach[count], &LeftDir, &LeftSteps, &RightTach[count], &RightDir, &RightSteps);
////        count = count + 1;
////        if(count == 5){ // speed buffer is ready
////            L_real = 2000000/avg(LeftTach, TACHBUFF);    //rpm
////            R_real = 2000000/avg(RightTach, TACHBUFF);   //rpm
////            speed_ready = 1;
////        }
////    }
//
//
//
////    if(speed_ready == 1){
////
////        // PI control
////        LCD_Out();
////        count = 0;
////        speed_ready = 0;
////    }
////    Motor_Forward(20, 20);   // Motor_Forward(uint16_t leftDuty, uint16_t rightDuty)
//
//}



int main()
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
//    Board_initI2C();
//    Board_initSDSPI();
//    Board_initSPI();
//    Board_initUART();
//    Board_initWatchdog();
//    Board_initWiFi();

//    Clock_Init48MHz();                       // set system clock to 48 MHz
    MSP432_Launchpad_RedLED_Init();
//    Tachometer_Init();
//    Motor_Init();
//    EnableInterrupts();
    LCD_Clear();


    /*
     *  normal BIOS programs, would call BIOS_start() to enable interrupts
     *  and start the scheduler and kick BIOS into gear.  But, this program
     *  is a simple sanity test and calls BIOS_exit() instead.
     */
    BIOS_start();  /* terminates program and dumps SysMin output */

    return 0;
}

