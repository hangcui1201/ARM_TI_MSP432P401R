/*
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

// Pololu #3543 Vreg (5V regulator output) connected to all three Pololu #136 GP2Y0A21YK0F Vcc's (+5V) and MSP432 +5V (J3.21)
// Pololu #3543 Vreg (5V regulator output) connected to positive side of three 10 uF capacitors physically near the sensors
// Pololu ground connected to all three Pololu #136 GP2Y0A21YK0F grounds and MSP432 ground (J3.22)
// Pololu ground connected to negative side of all three 10 uF capacitors
// MSP432 P9.0 (J5) (analog input to MSP432) connected to right GP2Y0A21YK0F Vout
// MSP432 P4.1 (J1.5) (analog input to MSP432) connected to center GP2Y0A21YK0F Vout
// MSP432 P9.1 (J5) (analog input to MSP432) connected to left GP2Y0A21YK0F Vout

#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "TimerA1.h"
#include "UART0.h"
#include "ADC14.h"
#include "LPF.h"
#include "IRDistance.h"

volatile uint32_t ADCflag;
volatile uint32_t nr,nc,nl;

void IR_Sensor_ISR(void){                   // runs at 2000 Hz
  uint32_t raw17, raw12, raw16;
  P1OUT ^= 0x01;                            // profile
  P1OUT ^= 0x01;                            // profile
  ADC_In17_12_16(&raw17, &raw12, &raw16);   // sample
  nr = LPF_Calc(raw17);                     // right is channel 17 P9.0
  nc = LPF_Calc2(raw12);                    // center is channel 12, P4.1
  nl = LPF_Calc3(raw16);                    // left is channel 16, P9.1
  ADCflag = 1;                              // semaphore
  P1OUT ^= 0x01;                            // profile
}

int main(void){

  uint32_t raw17,raw12,raw16;
  int32_t n; uint32_t s;

  Clock_Init48MHz();

  ADCflag = 0;

  s = 256;                                 // replace with your choice

  ADC0_InitSWTriggerCh17_12_16();          // initialize channels 17,12,16
  ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample

  LPF_Init(raw17,s);                       // P9.0/channel 17
  LPF_Init2(raw12,s);                      // P4.1/channel 12
  LPF_Init3(raw16,s);                      // P9.1/channel 16

  UART0_Init();                            // initialize UART0 115200 baud rate
  TimerA1_Init(&IR_Sensor_ISR, 250);       // 2000 Hz sampling

  UART0_OutString("GP2Y0A21YK0F connect analog signals to P9.0,P4.1,P9.1\n");

  EnableInterrupts();

  while(1){

    for(n=0; n<2000; n++){
      while(ADCflag == 0){};
      ADCflag = 0; // show every 2000th point
    }

    UART0_OutUDec5(LeftConvert(nl));
    UART0_OutString(" mm,");
    UART0_OutUDec5(CenterConvert(nc));
    UART0_OutString(" mm,");
    UART0_OutUDec5(RightConvert(nr));
    UART0_OutString(" mm\n");
  }

}

