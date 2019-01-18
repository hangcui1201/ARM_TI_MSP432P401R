#include "LS7366.h"
#include "Clock.h"
#include "msp.h"

//typedef   signed char    int8_t;
//typedef unsigned char   uint8_t;
//typedef          int    int16_t;
//typedef unsigned int   uint16_t;
//typedef          long   int32_t;
//typedef unsigned long  uint32_t;

uint8_t dummy = 0;

void SPI_Send_Command(void){

    // WR register, select MDR0
    // 10 001 000
    EUSCI_B3->TXBUF = 0x88;
    while((EUSCI_B3->IFG&0x0002)==0x0000){};     // wait until UCB3TXBUF empty

    while((EUSCI_B3->IFG&0x0001)==0x0000){};     // wait until UCB3RXBUF full
    dummy = EUSCI_B3->RXBUF;

    // Set up Mode Register 0 (MDR0)
    // B1B0 = 11 : x4 quadrature count mode (four counts per quadrature cycle)
    // B3B2 = 00 : free-running count mode
    // B5B4 = 00 : disable index
    // B6 = 0 : Negative index input
    // B7 = 1 :  Filter clock division factor = 2
    // 1000 0011
    EUSCI_B3->TXBUF = 0x83;
//    EUSCI_B3->TXBUF = 0x03;                      // Configure to 4 byte mode
    while((EUSCI_B3->IFG&0x0002)==0x0000){};     // wait until UCB3TXBUF empty

    while((EUSCI_B3->IFG&0x0001)==0x0000){};     // wait until UCB3RXBUF full
    dummy = EUSCI_B3->RXBUF;

}



void SPI_Send_Data(uint8_t data){

    EUSCI_B3->TXBUF = data;
    while((EUSCI_B3->IFG&0x0002)==0x0000){};     // wait until UCB3TXBUF empty
    while((EUSCI_B3->IFG&0x0001)==0x0000){};     // wait until UCB3RXBUF full
    dummy = EUSCI_B3->RXBUF;

}

uint8_t SPI_Read_Data(void){
    while((EUSCI_B3->IFG&0x0002)==0x0000){};     // wait until UCB3TXBUF empty
    EUSCI_B3->TXBUF = 0x00;
    while((EUSCI_B3->IFG&0x0001)==0x0000){};     // wait until UCB3RXBUF full
    return EUSCI_B3->RXBUF;
}


void Encoder_Init(void){

    // init the chip select S1 - P10.4, S2 - P10.5
    // 0011 0000
    P10->SEL0 &= ~0x31;  // Configure P10.4-P10.5, P10.0 as GPIO
    P10->SEL1 &= ~0x31;
    P10->DIR |= 0x31;    // Make P10.4 and P10.5, P10.0 as output
    P10->OUT |= 0x31;    // Make P10.4 and P10.5, P10.0 as high, the connected chip will be "deselected" by default

    // SPI init, UCB3, P10.1-CLK, P10.2-SIMO, P10.3-SOMI
    EUSCI_B3->CTLW0 = 0x0001;             // hold the eUSCI module in reset mode
    // configure UCB3CTLW0 for:
    // bit15      UCCKPH = 1; data shifts in on first edge, out on following edge
    // bit14      UCCKPL = 0; clock is low when inactive
    // bit13      UCMSB = 1; MSB first, LS7366 will only receive the MSB 8 bits
    // bit12      UC7BIT = 0; 8-bit data
    // bit11      UCMST = 1; master mode
    // bits10-9   UCMODEx = 00; 3-pin SPI
    // bit8       UCSYNC = 1; synchronous mode
    // bits7-6    UCSSELx = 10; eUSCI clock SMCLK
    // bits5-2    reserved
    // bit1       UCSTEM = 0;
    // bit0       UCSWRST = 1; reset enabled
    // 1010 1001 1000 0001
    EUSCI_B3->CTLW0 = 0xA981;
//    EUSCI_B3->CTLW0 = 0xAD83;
    // set the baud rate for the eUSCI which gets its clock from SMCLK
    // Clock_Init48MHz() from ClockSystem.c sets SMCLK = HFXTCLK/4 = 12 MHz
    // if the SMCLK is set to 12 MHz, divide by 3 for 4 MHz baud clock
    EUSCI_B3->BRW = 12;
    // modulation is not used in SPI mode, so clear UCB3MCTLW
//    EUSCI_B3->MCTLW = 0;

    P10->SEL0 |= 0x0E;
    P10->SEL1 &= ~0x0E;            // configure P10.3, P10.2, and P10.1 as primary module function

    EUSCI_B3->CTLW0 &= ~0x0001;    // enable eUSCI module
    EUSCI_B3->IE &= ~0x0003;       // disable interrupts


    // Initialize encoder 1
    // Clock division factor: 0
    // Negative index input
    // free-running count mode
    // x4 quatrature count mode (four counts per quadrature cycle)
    // P10.4
    P10->OUT &= ~0x10;      // Begin SPI conversation
    SPI_Send_Command();
    P10->OUT |= 0x10;       // Terminate SPI conversation

    // Initialize encoder 2
    // Clock division factor: 0
    // Negative index input
    // free-running count mode
    // x4 quatrature count mode (four counts per quadrature cycle)
    // P10.5
    P10->OUT &= ~0x20;      // Begin SPI conversation
    SPI_Send_Command();
    P10->OUT |= 0x20;       // Terminate SPI conversation

}

int32_t Encoder_Read(int16_t encoder){

    // Initialize temporary variables for SPI read
    uint16_t count_1, count_2, count_3, count_4;

    int32_t count_value;

    // Read encoder 1
    if (encoder == 1) {
      P10->OUT &= ~0x10;                   // Begin SPI conversation
      SPI_Send_Data(0x60);                 // Request count
      count_1 = SPI_Read_Data();           // Read highest order byte
      count_2 = SPI_Read_Data();
      count_3 = SPI_Read_Data();
      count_4 = SPI_Read_Data();           // Read lowest order byte
      P10->OUT |= 0x10;                    // Terminate SPI conversation
    }

    // Read encoder 2
    else if (encoder == 2) {
      P10->OUT &= ~0x20;                   // Begin SPI conversation
      SPI_Send_Data(0x60);                 // Request count
      count_1 = SPI_Read_Data();           // Read highest order byte
      count_2 = SPI_Read_Data();
      count_3 = SPI_Read_Data();
      count_4 = SPI_Read_Data();           // Read lowest order byte
      P10->OUT |= 0x20;                    // Terminate SPI conversation
    }

    // Calculate encoder count
    count_value = (count_1 << 8) + count_2;
    count_value = (count_value << 8) + count_3;
    count_value = (count_value << 8) + count_4;

    return count_value;
}



void Encoder_Count_Clear(void){

    // Set encoder1's data register to 0
    P10->OUT &= ~0x10;                   // Begin SPI conversation

    // Write to DTR
    SPI_Send_Data(0x98);

    // Load data
    SPI_Send_Data(0x00);   // Highest order byte
    SPI_Send_Data(0x00);
    SPI_Send_Data(0x00);
    SPI_Send_Data(0x00);   // lowest order byte

    P10->OUT |= 0x10;                    // Terminate SPI conversation

    Clock_Delay1us(100);   // provides some breathing room between SPI conversations

    // Set encoder1's current data register to center
    P10->OUT &= ~0x10;                   // Begin SPI conversation
    SPI_Send_Data(0xE0);
    P10->OUT |= 0x10;                    // Terminate SPI conversation



    // Set encoder2's data register to 0
    P10->OUT &= ~0x20;                   // Begin SPI conversation

    // Write to DTR
    SPI_Send_Data(0x98);

    // Load data
    SPI_Send_Data(0x00);   // Highest order byte
    SPI_Send_Data(0x00);
    SPI_Send_Data(0x00);
    SPI_Send_Data(0x00);   // lowest order byte

    P10->OUT |= 0x20;                    // Terminate SPI conversation

    Clock_Delay1us(100);   // provides some breathing room between SPI conversations

    // Set encoder2's current data register to center
    P10->OUT &= ~0x20;                   // Begin SPI conversation
    SPI_Send_Data(0xE0);
    P10->OUT |= 0x20;                    // Terminate SPI conversation

}








