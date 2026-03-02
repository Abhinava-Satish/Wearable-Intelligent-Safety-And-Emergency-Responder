#include "defs.h"
#include <stdint.h>

//|-> Global Variables

#define LSM_Address 0x6A
#define FullScaleSensitivity 16384

//USART Variables
#define RingBuffer_Length 32U //Always a power of 2
uint8_t RingBuffer[RingBuffer_Length];
volatile uint8_t RingHead = 0, RingTail = 0;

//I2C Variables
typedef enum{ I2C_State_Idle, I2C_State_Write, I2C_State_Reg_Write, I2C_State_Read}I2C_State_Enum;
volatile I2C_State_Enum I2C_State = I2C_State_Idle;
volatile uint8_t I2C_State_Busy = 0, I2C_TxData, I2C_RxBuffer[32], I2C_RxLength, I2C_Index, I2C_SlaveAddress, I2C_Reg_Address;

//<-|

//|-> Main Function
int main(void)
{
  
  //|-> RCC Clock Initialization
  
  //Enable Buses
  RCC_AHB2ENR  |= (1U << 0)  | (1U << 1);         //GPIOA, GPIOB
  RCC_APB1ENR1 |= (1U << 21);                     //I2C1
  RCC_APB2ENR  |= (1U << 14);                     //USART1

  //Configure Peripherals Kernel Clock as HSI16 
  RCC_CCIPR1 &= ~((3U << 0) | (3U << 10));        //USART1, I2C1
  RCC_CCIPR1 |=  (2U << 0) | (2U << 10);          //USART1, I2C1
  
  //<-|

  //|-> GPIO Initialization
  
  //Error LED
  GPIOB->MODER &= ~(3U << 16);
  GPIOB->MODER |=  (1U << 16);
  GPIOB->BSRR   =  (1U << 8);

  //USART1 

  //Set PA8 as AF mode for Rx
  GPIOA->MODER &= ~(3U << 16);
  GPIOA->MODER |=  (2U << 16);

  //Set PB12 as AF mode for Tx
  GPIOB->MODER &= ~(3U << 24);
  GPIOB->MODER |=  (2U << 24);

  //Set PA8 as AF7 as Rx
  GPIOA->AFRH &= ~( 15U << 0);
  GPIOA->AFRH |=  ( 7U << 0);

  //Set PB12 as AF7 as Tx
  GPIOB->AFRH &= ~( 15U << 16);
  GPIOB->AFRH |=  ( 7U << 16);
  
  //I2C1

  //Set PB1, PB2 as AF for SDA and SCL
  GPIOB->MODER &= ~((3U << 2) | (3U << 4));
  GPIOB->MODER |=  ( 2U << 2) | (2U << 4);

  //Set PB1, PB2 as AF4
  GPIOB->AFRL &= ~((15U << 4) | (15U << 8));
  GPIOB->AFRL |=  (4U << 4)  | (4U  << 8);

  //Set PB1, PB2 as High Speed
  GPIOB->OSPEEDR &= ~((3U << 2) | (3U << 4));
  GPIOB->OSPEEDR |= (2U << 2) | (2U << 4);

  //Set PB1, PB2 as Open Drain
  GPIOB->OTYPER &= ~((1U << 1) | (1U << 2));
  GPIOB->OTYPER |= (1U << 1) | (1U << 2);

  //Set PB1, PB2 as PullUp
  GPIOB->PUPDR &= ~((3U << 2) | (3U << 4));
  GPIOB->PUPDR |=  (1U << 2) | (1U << 4);

  //<-|

  //|-> USART1 Initialization

  //Set M Bits in USART1_CR1 for 8 Databits and Set UE = 0
  USART1->CR1 &= ~((1U << 12) | (1U << 28) | (1U << 0));
   
  //Set USART1_Presc to 0000
  USART1->PRESC = 0;

  //Set USART1_BRR as Clock Frequency / Baudrate = 1667 for 9600 Baudrate
  USART1->BRR = 1667;
  
  //Enable USART1 by setting UE to 1 + TE
  USART1->CR1 |= (1U << 0) | (1U << 3);
 
  //<-|

  //|-> I2C1 Initialization

  //Disable I2C
  I2C1->CR1 = 0;

  //Configure Timing
  I2C1->TIMINGR = (1U  << 28) | (1U  << 20) | (2U  << 16) | (31U << 8) | (39U << 0); //PRESC, SCLDEL, SDADEL, SCLH, SCLL

  // Enable I2C
  I2C1->CR1 |= (1U << 0); // PE

  //<-|

  //|-> Interrupts Initialization

  // Route IRQ to Non-secure
  NVIC_ITNS1 |= (1U << 11) | (1U << 14);    //I2C1_EV, USART1

  // Enable USART1 IRQ in NVIC
  NVIC_ISER1 |= (1U << 11) | (1U <<14);      //I2C1_EV, USART1

  // Enable global interrupts
  __asm volatile ("cpsie i");
  
  //<-|

  I2C_Read(LSM_Address, 0x0F, 1);
  char buf[3];
  Byte2String(I2C_RxBuffer[0], buf);
  USART_EnQ("Who Am I : 0x");
  USART_EnQ(buf);
  USART_EnQ("\r\r");
  
  Sensor_Configuration();
  
  while (1)
  {
    AxG_ReadnPrint();
  }

}

//<-|

//|-> USART1_IRQHandler

void USART1_IRQHandler(void)
{
    if (USART1->ISR & (1U << 7)) // TXE
    {
        if (RingHead != RingTail)
        {
            USART1->TDR = RingBuffer[RingTail];
            RingTail = (RingTail + 1) & (RingBuffer_Length - 1);
        }
        else
        {
            USART1->CR1 &= ~(1U << 7); // disable TXEIE
        }
    }
}

//<-|

//|-> I2C1_IRQHandler

void I2C1_EV_IRQHandler(void)
{
  // NACK Handling
  if(I2C1->ISR & (1U << 4))   //NACK
  {
    I2C1->ICR = 1U << 4;      //Clear NACK
    I2C_State_Busy = 0;
    I2C_State = I2C_State_Idle;
    return;
  }
  
  // TXIS Handling
  if(I2C1->ISR & (1U << 1))   //TXIS
  {
    if(I2C_State == I2C_State_Write)
    {
      if(I2C_Index == 0)
        I2C1->TXDR = I2C_Reg_Address;
      else
        I2C1->TXDR = I2C_TxData;
      I2C_Index++;
    }
    if(I2C_State == I2C_State_Reg_Write)
      I2C1->TXDR = I2C_Reg_Address;
    return;
  }

  // TC Handling
  if(I2C1->ISR & (1U << 6))   //TC
  {
    if(I2C_State == I2C_State_Reg_Write)
    {
      //Repeated Start
      I2C1->CR2 = (I2C_SlaveAddress << 1) | (1U << 10) | (1U << 13) | (I2C_RxLength << 16) | (1U << 25); //Slave Address, RD_WRN, Start, NBYTES, Autoend      
      
      I2C_State = I2C_State_Read;
      I2C_Index = 0;
    }
    return;
  }

  // RXNE Handling
  if(I2C1->ISR & (1U << 2))   //RXNE
  {
    I2C_RxBuffer[I2C_Index++] = I2C1->RXDR;
    return;
  }

  //STOPF Handling
  if(I2C1->ISR & (1U << 5)) //STOPF
  {
    I2C1->ICR = 1U << 5;    //Clear STOPF 

    I2C1->CR1 &= ~((1U << 1) | (1U << 2) | (1U << 4) | (1U << 5) | (1U << 6)); //TXIE, RXIE, NACKIE, STOPIE, TCIE

    I2C_State_Busy = 0;
    I2C_State = I2C_State_Idle;
    return;
  }
}

//<-|

//|-> Delay Function

void Delay(volatile uint32_t count) {
  count *= 1470;
  while(count--) __asm volatile ("nop");
}

//<-|

//|-> Byte2String Function

void Byte2String(uint8_t val, char *out)
{
    static const char hex[] = "0123456789ABCDEF";

    out[0] = hex[(val >> 4) & 0x0F];
    out[1] = hex[val & 0x0F];
    out[2] = '\0';
}

//<-|

//|-> Int2String Function

void Int2String(int val, char *out)
{
  if(val == 0)
  {
    *out++ = '0'  ;
    *out = '\0';
    return;
  }

  unsigned int uval;
  if(val < 0)
  {
    *out++ = '-';
    uval = -((unsigned int)val);
  }
  else
    uval = val;
 
  char *start = out, temp;

  while(uval != 0)
  {
    *out++ = (uval % 10) + '0';
    uval /= 10;
  }
  
  *out-- = '\0';

  while(start < out)
  {
    temp = *out;
    *out-- = *start;
    *start++ = temp;
  }  
}

//<-|

//|-> USART_EnQ Function

void USART_EnQ(char *data)
{
    while (*data)
    {
        // wait if buffer full
        while (RingTail == ((RingHead + 1) & (RingBuffer_Length - 1)));

        RingBuffer[RingHead] = *data++;
        RingHead = (RingHead + 1) & (RingBuffer_Length - 1);
    }

    USART1->CR1 |= (1U << 7);   // TXFNFIE
}

//<-|

//|-> I2C_Write Function

void I2C_Write(uint8_t addr, uint8_t reg, uint8_t data)
{
  while(I2C_State_Busy);
  
  I2C_State_Busy = 1;
  I2C_State = I2C_State_Write;

  I2C_SlaveAddress = addr;
  I2C_Reg_Address = reg;

  I2C_TxData = data;

  I2C_Index = 0;

  I2C1->CR2 = (I2C_SlaveAddress << 1) | (1U << 13) | (2U << 16) | (1U << 25); //Slave Address, Start, NBYTES, Autoend

  I2C1->CR1 |= (1U << 1) | (1U << 5) | (1U << 4); //TXIE, STOPIE, NACKIE

  while(I2C_State_Busy);
}

//<-|

//|-> I2C_Read Function

void I2C_Read(uint8_t addr, uint8_t reg, uint8_t len)
{
  while(I2C_State_Busy);

  I2C_State_Busy = 1;

  I2C_State = I2C_State_Reg_Write;

  I2C_SlaveAddress = addr;
  I2C_Reg_Address = reg;

  I2C_RxLength = len;
  
  I2C_Index = 0;

  I2C1->CR2 = (I2C_SlaveAddress << 1) | (1U << 13) | (1U << 16); //Slave Address, Start, NBYTES

  I2C1->CR1 |= (1U << 1) | (1U << 2) | (1U << 4) | (1U << 5) | (1U << 6); //TXIE, RXIE, NACKIE, STOPIE, TCIE
  
  while(I2C_State_Busy);
}

//<-|

//|-> Sensor Configuration Function 

void Sensor_Configuration()
{
  I2C_Write(LSM_Address, 0x12, 0x01);   //Sensor Reset (CTRL3)
  Delay(100);
  I2C_Write(LSM_Address, 0x12, 0x44);   //CTRL3 Register
  I2C_Write(LSM_Address, 0x10, 0x06);   //Accelerometer Control Register
  I2C_Write(LSM_Address, 0x11, 0x06);   //Gyroscope Control Register
  I2C_Write(LSM_Address, 0x15, 0x0C);   //Gyroscope FullScale(CTRL6)
  I2C_Write(LSM_Address, 0x17, 0x03);   //Accelerometer FullScale(CTRL8)
  I2C_Write(LSM_Address, 0x5D, 0x0F);   //Free Fall
  I2C_Write(LSM_Address, 0x5C, 0x80);   //WAKE_UP_DUR
  
  
}

//<-|

//|-> Accelerometer & Gyroscope Read & Print Function

void AxG_ReadnPrint()
{
  int32_t Out_GX, Out_GY, Out_GZ, Out_AX, Out_AY, Out_AZ;
  char buf[12];
  I2C_Read(LSM_Address, 0x22, 12);
  Delay(100);
  Out_GX = ((int16_t)(I2C_RxBuffer[0]  | I2C_RxBuffer[1]  << 8)) * 140;
  Out_GY = ((int16_t)(I2C_RxBuffer[2]  | I2C_RxBuffer[3]  << 8)) * 140;
  Out_GZ = ((int16_t)(I2C_RxBuffer[4]  | I2C_RxBuffer[5]  << 8)) * 140;
  Out_AX = ((int16_t)(I2C_RxBuffer[6]  | I2C_RxBuffer[7]  << 8)) * 488 / 1000;
  Out_AY = ((int16_t)(I2C_RxBuffer[8]  | I2C_RxBuffer[9]  << 8)) * 488 / 1000;
  Out_AZ = ((int16_t)(I2C_RxBuffer[10] | I2C_RxBuffer[11] << 8)) * 488 / 1000;

  //Scale Appropriately
    
  Int2String(Out_GX , buf);
  USART_EnQ("GX: ");
  USART_EnQ(buf);
  USART_EnQ(" mdps\t");
  Int2String(Out_GY, buf);
  USART_EnQ("GY: ");
  USART_EnQ(buf);
  USART_EnQ(" mdps\t");
  Int2String(Out_GZ, buf);
  USART_EnQ("GZ: ");
  USART_EnQ(buf);
  USART_EnQ(" mdps\r");

  Int2String(Out_AX, buf);
  USART_EnQ("AX: ");
  USART_EnQ(buf);
  USART_EnQ(" mg\t");
  Int2String(Out_AY, buf);
  USART_EnQ("AY: ");
  USART_EnQ(buf);
  USART_EnQ(" mg\t");
  Int2String(Out_AZ, buf);
  USART_EnQ("AZ: ");
  USART_EnQ(buf);
  USART_EnQ(" mg\r\r");
  
  Delay(3000);

}

//<-|

