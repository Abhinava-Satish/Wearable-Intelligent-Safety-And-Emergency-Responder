#include "defs.h"

//|-> Global Variables

#define LSM_Address 0x6A

//USART Variables
#define RingBuffer_Length 32U //Always a power of 2
uint8_t RingBuffer[RingBuffer_Length];
volatile uint8_t RingHead = 0, RingTail = 0;

//I2C Variables
typedef enum{ I2C_State_Idle, I2C_State_Write, I2C_State_Reg_Write, I2C_State_Read}I2C_State_Enum;
volatile I2C_State_Enum I2C_State = I2C_State_Idle;
volatile uint8_t I2C_State_Busy = 0, I2C_TxData, I2C_RxBuffer[12], I2C_RxLength, I2C_Index, I2C_SlaveAddress, I2C_Reg_Address;

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

  while (1);
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


