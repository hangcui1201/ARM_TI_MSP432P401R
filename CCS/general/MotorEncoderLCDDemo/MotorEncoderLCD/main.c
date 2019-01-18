/******************************************************************************
* Author: Hang Cui
* Email:cuihang1201@gmail.com
* University of Illinois at Urbana-Champaign
*******************************************************************************/

#include "msp.h"
#include "Clock.h"
#include "LS7366.h"
#include "CortexM.h"
#include "Motor.h"
#include "TimerA1.h"
#include "Nokia5110.h"

//#include "UART0.h"

#define PI 3.14159

float leftMotorSpeed = 0;
float rightMotorSpeed = 0;

int32_t encRight_pre = 0;
int32_t encRight_cur = 0;

int32_t encLeft_pre = 0;
int32_t encLeft_cur = 0;

int16_t leftDir = 0;
int16_t rightDir = 0;

uint16_t L_real = 0;    // actual rpm of left motor
uint16_t R_real = 0;    // actual rpm of right motor

// Output a 16-bit number in unsigned 3-digit fixed point, 0.1 resolution
// numbers 0 to 999 printed as " 0.0" to "99.9"
// Inputs: n  16-bit unsigned number
//void Nokia5110_OutUFix1(uint16_t n){}

void LCD_Clear(void){
    Nokia5110_Init();
    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("TiBot");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("Motor Speed");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("Lr="); Nokia5110_OutUFix1(0); Nokia5110_OutString("r/s");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("D="); Nokia5110_OutSDec(0); //Nokia5110_OutUFix1(0); Nokia5110_OutString("r/s");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("Rr="); Nokia5110_OutUFix1(0); Nokia5110_OutString("r/s");
    Nokia5110_SetCursor(0,5); Nokia5110_OutString("D="); Nokia5110_OutSDec(0); //Nokia5110_OutUFix1(0); Nokia5110_OutString("r/s");
}

void LCD_Out(void){
    Nokia5110_SetCursor(3,2); Nokia5110_OutUFix1(L_real);
    Nokia5110_SetCursor(2,3); Nokia5110_OutSDec(leftDir);
    Nokia5110_SetCursor(3,4); Nokia5110_OutUFix1(R_real);
    Nokia5110_SetCursor(2,5); Nokia5110_OutSDec(rightDir);
}


//int i = 0;

//int32_t curR[100];
//int32_t preR[100];
//float RSpeed[100];

// Call this function periodically - 20ms
void calSpeed(void){

    encRight_cur = Encoder_Read(1);
    encLeft_cur = Encoder_Read(2);

    rightMotorSpeed = (float)(encRight_cur-encRight_pre)*2*PI/(1440*0.02); //rad/s
    leftMotorSpeed = (float)(encLeft_cur-encLeft_pre)*2*PI/(1440*0.02);    //rad/s

//    if(i<100){
//        curR[i] = encRight_cur;
//        preR[i] = encRight_pre;
//        RSpeed[i] = rightMotorSpeed;
//        i++;
//    }else{
//        i = 0;
//        curR[i] = encRight_cur;
//        preR[i] = encRight_pre;
//        RSpeed[i] = rightMotorSpeed;
//    }

    encRight_pre = encRight_cur;
    encLeft_pre = encLeft_cur;

}


int main(void){

  // Use PWM to move the robot
  Clock_Init48MHz();

//  UART0_Init();
  Motor_Init(); // PWM carrier frequency - 20KHz

  Encoder_Init();
  Encoder_Count_Clear();

  TimerA1_Init(&calSpeed, 10000);  // 20ms

  EnableInterrupts();

  LCD_Clear();

  Motor_Right(10, 10);

  while(1){

      Clock_Delay1ms(500);

      if(leftMotorSpeed > 0){
          leftDir = 1;
          L_real = ((uint16_t)(leftMotorSpeed*100))/10;
      }else{
          leftDir = -1;
          L_real = ((uint16_t)(-leftMotorSpeed*100))/10;
      }

      if(rightMotorSpeed > 0){
          rightDir = 1;
          R_real = ((uint16_t)(rightMotorSpeed*100))/10;
      }else{
          rightDir = -1;
          R_real = ((uint16_t)(-rightMotorSpeed*100))/10;
      }

      LCD_Out();

  }

}































