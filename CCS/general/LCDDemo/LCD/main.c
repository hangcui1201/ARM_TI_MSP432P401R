// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// UCA3STE       (SCE, pin 3) connected to P9.4
// Reset         (RST, pin 4) connected to P9.3
// Data/Command  (D/C, pin 5) connected to P9.6
// UCA3SIMO      (DN,  pin 6) connected to P9.7
// UCA3CLK       (SCLK, pin 7) connected to P9.5
// back light    (LED, pin 8) not connected, consists of 4 3.3 V white LEDs which draw ~80mA total

#include "msp.h"
#include "Clock.h"
#include "Nokia5110.h"

#include "CortexM.h"
#include "TimerA1.h"
#include "ADC14.h"
#include "LPF.h"
#include "IRDistance.h"


volatile uint32_t ADCflag;
volatile uint32_t nr,nc,nl;
int32_t Left,Center,Right; // IR distances in mm

void LCD_IR_Clear(void){
    Nokia5110_Init();
    Nokia5110_Clear(); // erase entire display
    Nokia5110_OutString("TiBot");
    Nokia5110_SetCursor(0,1); Nokia5110_OutString("IR distance");
    Nokia5110_SetCursor(0,2); Nokia5110_OutString("L= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
    Nokia5110_SetCursor(0,3); Nokia5110_OutString("C= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
    Nokia5110_SetCursor(0,4); Nokia5110_OutString("R= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
//    Nokia5110_SetCursor(0,5); Nokia5110_OutString("E= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
}

void LCD_IR_Out(void){
  Nokia5110_SetCursor(3,2); Nokia5110_OutSDec(Left);
  Nokia5110_SetCursor(3,3); Nokia5110_OutSDec(Center);
  Nokia5110_SetCursor(3,4); Nokia5110_OutSDec(Right);
//  Nokia5110_SetCursor(3,5); Nokia5110_OutSDec(Error);
}

void IR_Sampling(void){  // runs at 2000 Hz
  uint32_t raw17,raw12,raw16;
  ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw12); // center is channel 12, P4.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  Left = LeftConvert(nl);
  Center = CenterConvert(nc);
  Right = RightConvert(nr);
  ADCflag = 1;           // semaphore
}

void main(void){

  uint32_t raw17,raw12,raw16;
  int32_t n;
  ADCflag = 0;

  Clock_Init48MHz();                       // set system clock to 48 MHz

  ADC0_InitSWTriggerCh17_12_16();          // initialize channels 17,12,16
  ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample
  LPF_Init(raw17, 64);                     // P9.0/channel 17
  LPF_Init2(raw12, 64);                    // P4.1/channel 12
  LPF_Init3(raw16, 64);                    // P9.1/channel 16

  TimerA1_Init(&IR_Sampling,250);          // 2000 Hz sampling
  LCD_IR_Clear();
  EnableInterrupts();

  while(1){

      for(n=0; n<2000; n++){
        while(ADCflag == 0){};
        ADCflag = 0; // show every 1000th point
      }

      Left = LeftConvert(nl);
      Center = CenterConvert(nc);
      Right = RightConvert(nr);

      LCD_IR_Out();
  }


}
