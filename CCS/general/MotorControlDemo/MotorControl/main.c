/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "PWM.h"
#include "LaunchPad.h"
#include "UART0.h"
#include "Motor.h"
#include "Bump.h"
#include "ADC14.h"
#include "TimerA1.h"
#include "IRDistance.h"
#include "Nokia5110.h"
#include "LPF.h"
#include "SysTickInts.h"
#include "Tachometer.h"
#include "Reflectance.h"
#include "TA3InputCapture.h"


#define OUTUART 1

//******************************************************

volatile uint32_t nr,nc,nl;
volatile uint32_t ADCflag; // Set every 500us on ADC sample

volatile uint32_t ControllerFlag; // set every 10ms on controller execution

int32_t UR, UL;  // PWM duty 0 to 14,998
int32_t Left,Center,Right; // IR distances in mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
int32_t Ki=5;  // integral controller gain
int32_t Kp=4;  // proportional controller gain

#define TOOCLOSE 200
#define DESIRED 250
int32_t SetPoint = 250;
#define TOOFAR 400

#define PWMNOMINAL 2500
#define SWING 1000
#define PWMMIN (PWMNOMINAL-SWING)
#define PWMMAX (PWMNOMINAL+SWING)

//****************************************************************************


// incremental speed control

// ------------avg------------
// Simple math function that returns the average
// value of an array.
// Input: array is an array of 16-bit unsigned numbers
//        length is the number of elements in 'array'
// Output: the average value of the array
// Note: overflow is not considered

uint16_t avg(uint16_t *array, int length){
  int i;
  uint32_t sum = 0;
  for(i=0; i<length; i=i+1){
    sum = sum + array[i];
  }
  return (sum/length);
}

uint16_t DesiredL = 100;                  // desired rotations per minute
uint16_t DesiredR = 100;                  // desired rotations per minute
#define DESIREDMAX 120                   // maximum rotations per minute
#define DESIREDMIN  30                   // minimum rotations per minute (works poorly at 30 RPM due to 16-bit timer overflow)
uint16_t ActualL;                        // actual rotations per minute
uint16_t ActualR;                        // actual rotations per minute
uint16_t LeftDuty = 3750;                // duty cycle of left wheel (0 to 14,998)
uint16_t RightDuty = 3750;               // duty cycle of right wheel (0 to 14,998)
#define TACHBUFF 10                      // number of elements in tachometer array
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)


void main(void){  // incremental control of constant speed using tachometer

    int i = 0;
    Clock_Init48MHz();                     // set system clock to 48 MHz

    UART0_Initprintf();
    printf("\n\rSpeed Controller\n\r");
    printf("DesiredR= %d, DesiredL= %d\n\r", DesiredR, DesiredL);

    LaunchPad_Init();
    Bump_Init();
    Tachometer_Init();
    Motor_Init();
    EnableInterrupts();

    while(1)
    {

        Motor_Stop();

        while(Bump_Read() == 0)
        {

            if((LaunchPad_Input()&0x01) != 0x00)
            {
                // Button1 has been pressed
                DesiredR = DesiredR + 10;
                if(DesiredR > DESIREDMAX){
                  DesiredR = DESIREDMIN;
                }

                #if OUTUART
                    printf("DesiredR= %d, DesiredL= %d\n\r",DesiredR,DesiredL);
                #endif
            }


            if((LaunchPad_Input()&0x02) != 0x00)
            {

                // Button2 has been pressed
                DesiredL = DesiredL + 10;

                if(DesiredL > DESIREDMAX){
                    DesiredL = DESIREDMIN;
                }

                #if OUTUART
                    printf("DesiredR= %d, DesiredL= %d\n\r", DesiredR, DesiredL);
                #endif
            }

            // flash the blue LED
            i = i + 1;
            LaunchPad_Output((i&0x01)<<2);
            Clock_Delay1ms(200);               // delay ~0.2 sec at 48 MHz
        }


        for(i=0; i<10; i=i+1)
        {
          // flash the yellow LED
          LaunchPad_Output(0x03);
          Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
          LaunchPad_Output(0x00);
          Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
        }


        LaunchPad_Output(0x02);
        i = 0;


        while(Bump_Read() == 0)
        {

            Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);

            i = i + 1;

            if(i >= TACHBUFF)
            {

                i = 0;
                // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
                ActualL = 2000000/avg(LeftTach, TACHBUFF);
                ActualR = 2000000/avg(RightTach, TACHBUFF);

                // very simple, very stupid controller
                if((ActualL > (DesiredL + 3)) && (LeftDuty > 100)){
                    LeftDuty = LeftDuty - 100;
                }else if((ActualL < (DesiredL - 3)) && (LeftDuty < 14898)){
                    LeftDuty = LeftDuty + 100;
                }

                if((ActualR > (DesiredR + 3)) && (RightDuty > 100)){
                    RightDuty = RightDuty - 100;
                }else if((ActualR < (DesiredR - 3)) && (RightDuty < 14898)){
                    RightDuty = RightDuty + 100;
                }

                Motor_Forward(LeftDuty, RightDuty);


            #if OUTUART
                printf("%5d rpm, %5d rpm, %5d steps, %5d steps\n\r", ActualL, ActualR, LeftSteps, RightSteps);
            #endif

            }

            Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
        }

        Motor_Stop();

        i = 0;
        while(Bump_Read() != 0)
        {
          // flash the red LED
          i = i + 1;
          LaunchPad_Output(i&0x01);
          Clock_Delay1ms(100);               // delay ~0.1 sec at 48 MHz
        }



    } // while(1)


} //main()




