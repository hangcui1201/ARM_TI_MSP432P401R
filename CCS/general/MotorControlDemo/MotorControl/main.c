#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "PWM.h"
#include "UART0.h"
#include "Motor.h"
#include "ADC14.h"
#include "TimerA1.h"
#include "Tachometer.h"
#include "TA3InputCapture.h"

// Simple math function that returns the average value of an array.
// Input: an array of 16-bit unsigned numbers
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


#define DESIREDMAX 120                   // maximum rpm
#define DESIREDMIN  40                   // minimum rpm


int32_t Ki_L = 5;  // integral controller gain
int32_t Kp_L = 4;  // proportional controller gain

int32_t Ki_R = 5;  // integral controller gain
int32_t Kp_R = 4;  // proportional controller gain

int32_t ErrL = 0;
int32_t ErrR = 0;
int32_t ErrL_pre = 0;
int32_t ErrR_pre = 0;

int32_t ErrSumL = 0;
int32_t ErrSumR = 0;
int32_t ErrSumL_pre = 0;
int32_t ErrSumR_pre = 0;

uint16_t DesiredL = 100;                 // desired rpm
uint16_t DesiredR = 100;                 // desired rpm

uint16_t ActualL;                        // actual rpm
uint16_t ActualR;                        // actual rpm

int16_t u_L = 0;
int16_t u_R = 0;

#define TACHBUFF 10                      // number of elements in tachometer array

uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)

enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)

int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)


void main(void){  // incremental control of constant speed using tachometer

    int i = 0;

    Clock_Init48MHz();                     // Set system clock to 48 MHz

    UART0_Initprintf();
    printf("\n\rSpeed Controller\n\r");
    printf("DesiredR= %d, DesiredL= %d\n\r", DesiredR, DesiredL);

    Tachometer_Init();
    Motor_Init();
    EnableInterrupts();

    while(1)
    {

        while(i<TACHBUFF)
        {

            Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);

            i = i + 1;

            if(i >= TACHBUFF)
            {
                // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
                ActualL = 2000000/avg(LeftTach, TACHBUFF);
                ActualR = 2000000/avg(RightTach, TACHBUFF);

                ErrL = DesiredL - ActualL;
                ErrR = DesiredR - ActualR;

                ErrSumL = ErrSumL_pre + 0.0005 * (ErrL + ErrL_pre);  // called every one millisecond
                ErrSumR = ErrSumR_pre + 0.0005 * (ErrR + ErrR_pre);  // called every one millisecond

                u_L = Kp_L * ErrL + Ki_L * ErrSumL;
                u_R = Kp_R * ErrR + Ki_R * ErrSumR;

                if((u_L >= 10) || (u_L <= -10)){
                    ErrSumL = ErrSumL_pre;
                }else{
                    ErrSumL_pre = ErrSumL;
                }

                if((u_R >= 10) || (u_R <= -10)){
                    ErrSumR = ErrSumR_pre;
                }else{
                    ErrSumR_pre = ErrSumR;
                }

                ErrL_pre = ErrL;
                ErrR_pre = ErrR;

                setMotorPWM_L(u_L);
                setMotorPWN_R(u_R);

                printf("%5d rpm, %5d rpm, %5d steps, %5d steps\n\r", ActualL, ActualR, LeftSteps, RightSteps);

                i = 0;
                Clock_Delay1ms(1);

            }

        }


    } // while(1)


} //main()




