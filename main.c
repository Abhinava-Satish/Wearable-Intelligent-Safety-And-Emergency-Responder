#include "defs.h"
#include <stdint.h>

//|-> Global Variables

#define RingBuffer_Length 32U //Always a power of 2
static uint8_t RingBuffer[RingBuffer_Length];
static volatile uint8_t RingHead = 0, RingTail = 0;
static unsigned char buf[4],buffer[6];
typedef enum { I2C_JOB_NONE, I2C_JOB_READ, I2C_JOB_WRITE }i2c_job_type_t;
typedef struct { i2c_job_type_t type; uint8_t addr; uint8_t reg; uint8_t *buf; uint8_t len; }i2c_job_t;
volatile i2c_job_t i2c_job;
volatile uint8_t   i2c_busy = 0, i2c_idx  = 0;

void i2c_write_reg(uint8_t reg, uint8_t val)
{
    I2C1->CR2 =
        (0x6A << 1) |     // slave address
        (2U << 16) |      // NBYTES = 2
        (1U << 13) |      // START
        (1U << 25);       // AUTOEND

    while (!(I2C1->ISR & (1U << 1))); // TXIS
    I2C1->TXDR = reg;

    while (!(I2C1->ISR & (1U << 1))); // TXIS
    I2C1->TXDR = val;

    while (!(I2C1->ISR & (1U << 5))); // STOPF
    I2C1->ICR |= (1U << 5);            // STOPCF
}

uint8_t i2c_read_reg(uint8_t reg)
{
    uint8_t val;

    // Write register address
    I2C1->CR2 =
        (0x6A << 1) |
        (1U << 16) |      // NBYTES = 1
        (1U << 13);       // START (write)

    while (!(I2C1->ISR & (1U << 1))); // TXIS
    I2C1->TXDR = reg;

    while (!(I2C1->ISR & (1U << 6))); // TC

    // Read data
    I2C1->CR2 =
        (0x6A << 1) |
        (1U << 16) |      // NBYTES = 1
        (1U << 13) |      // START
        (1U << 10) |      // RD_WRN
        (1U << 25);       // AUTOEND

    while (!(I2C1->ISR & (1U << 2))); // RXNE
    val = I2C1->RXDR;

    while (!(I2C1->ISR & (1U << 5))); // STOPF
    I2C1->ICR |= (1U << 5);            // STOPCF

    return val;
}

void i2c_read_multi(uint8_t reg, uint8_t *buf, uint8_t len)
{
    // Write register address
    I2C1->CR2 =
        (0x6A << 1) |
        (1U << 16) |
        (1U << 13);

    while (!(I2C1->ISR & (1U << 1)));
    I2C1->TXDR = reg;

    while (!(I2C1->ISR & (1U << 6))); // TC

    // Read bytes
    I2C1->CR2 =
        (0x6A << 1) |
        (len << 16) |
        (1U << 13) |
        (1U << 10) |
        (1U << 25);

    for (uint8_t i = 0; i < len; i++) {
        while (!(I2C1->ISR & (1U << 2))); // RXNE
        buf[i] = I2C1->RXDR;
    }

    while (!(I2C1->ISR & (1U << 5)));
    I2C1->ICR |= (1U << 5);
}


//<-|

//|-> Main Function
int main(void)
{
  
  //|-> RCC Clock Initialization
  
  //Reset Peripheral Registers
  RCC_AHB2RSTR  |=  (1U << 0)  | (1U << 1);       //GPIOA, GPIOB
  RCC_APB1RSTR1 |=  (1U << 21);                   //I2C1
  RCC_APB2RSTR  |=  (1U << 14);                   //USART1
  
  //Clear Reset
  RCC_AHB2RSTR  &= ~((1U << 0)  | (1U << 1));     //GPIOA, GPIOB
  RCC_APB1RSTR1 &= ~(1U << 21);                   //I2C1
  RCC_APB2RSTR  &= ~(1U << 14);                   //USART1

  //Enable Buses
  RCC_AHB2ENR  |= (1U << 0)  | (1U << 1);         //GPIOA, GPIOB
  RCC_APB1ENR1 |= (1U << 21);                     //I2C1
  RCC_APB2ENR  |= (1U << 14);                     //USART1

  //Configure Peripherals Kernel Clock as HSI16 
  RCC_CCIPR1 &= ~((3U << 0) | (3U << 10));        //USART1, I2C1
  RCC_CCIPR1 |=  (2U << 0) | (2U << 10);          //USART1, I2C1
  
  //<-|

  //|-> GPIO Initialization

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
  
  // Set PA4 as Output for CS
  GPIOA->MODER &= ~(3U << 8);
  GPIOA->MODER |=  (1U << 8);

  //Set PA4 as High Speed
  GPIOA->OSPEEDR |= (3U << 8);

  // Set PA4 as High
  GPIOA->BSRR = (1U << 4);

  //<-|

  //|-> USART1 Initialization

  //Set M Bits in USART1_CR1 for 8 Databits and Set UE = 0
  USART1->CR1 &= ~((1U << 12) | (1U << 28) | (1U << 0));
   
  //Set USART1_Presc to 0000
  USART1->PRESC = 0;

  //Set USART1_BRR as Clock Frequency / Baudrate = 1667
  USART1->BRR = 1667;
  
  //Enable USART1 by setting UE to 1 + TE
  USART1->CR1 |= (1U << 0) | (1U << 3);
 
  //<-|

  //|-> I2C1 Initialization

  /*/Reset CR1
  I2C1->CR1 = 0;

  //TIMINGR Configuration
  I2C1->TIMINGR = 0;
  I2C1->TIMINGR |= (1U << 28) | (1U << 20) | (31U << 8) | (47U << 0); //PRESC, SCLDEL, SCLH, SCLL
  
  //Configure CR1
  I2C1->CR1 |= (1U << 5) | (1U << 4) | (1U << 2) | (1U << 1); //STOPIE, NACKIE, RXIE, TXIE 
  
  //Clear Stale Flags
  I2C1->ICR = (0x3F << 8) | (7U << 3);

  //Enable PE 
  I2C1->CR1 |= (1U << 0);

  */
  // Timing configuration (100 kHz-ish, HSI16)
I2C1->TIMINGR =
    (1U  << 28) |   // PRESC
    (1U  << 20) |   // SCLDEL
    (2U  << 16) |   // SDADEL
    (31U << 8)  |   // SCLH
    (39U << 0);     // SCLL

// Enable I2C
I2C1->CR1 = 0;
I2C1->CR1 |= (1U << 0);   // PE

  //<-|

  //|-> Interrupts Initialization

  // Route IRQ to Non-secure
  NVIC_ITNS1 |= (0U << 11) | (1U << 14);    //I2C1_EV, USART1

  // Enable USART1 IRQ in NVIC
  NVIC_ISER1 = (0U << 11) | (1U <<14);      //I2C1_EV, USART1

  // Enable global interrupts
  __asm volatile ("cpsie i");
  
  //<-|

uint8_t whoami,status;
whoami = i2c_read_reg(0x0F);
i2c_write_reg(0x10, 0x06); // accel ODR = 60 Hz
i2c_write_reg(0x12, 0x04); // IF_INC = 1
i2c_write_reg(0x17, 0x00); // ±2g
Byte2String(whoami, buf);
USART_EnQBytes((unsigned char*)"Who Am I: 0x");
Delay(100);
USART_EnQBytes((unsigned char*)buf);
Delay(100);
USART_EnQBytes((unsigned char*) "\r");
while (1)
{
    // 1. Wait for new data
    do {
        status = i2c_read_reg(0x1E);
    } while (!(status & 0x01));

    // 2. Read 6 bytes starting from OUTX_L
    i2c_read_multi(0x28, buffer, 6);

    // 3. Print AX bytes
    USART_EnQBytes((unsigned char*)"AX: ");
    Byte2String(buffer[0], buf);   // X_L
    USART_EnQBytes(buf);
    USART_EnQBytes((unsigned char*)" ");
    Byte2String(buffer[1], buf);   // X_H
    USART_EnQBytes(buf);
    USART_EnQBytes((unsigned char*)"\r");

    Delay(100);

    // 4. Print AY bytes
    USART_EnQBytes((unsigned char*)"AY: ");
    Byte2String(buffer[2], buf);   // Y_L
    USART_EnQBytes(buf);
    USART_EnQBytes((unsigned char*)" ");
    Byte2String(buffer[3], buf);   // Y_H
    USART_EnQBytes(buf);
    USART_EnQBytes((unsigned char*)"\r");

    Delay(100);

    // 5. Print AZ bytes
    USART_EnQBytes((unsigned char*)"AZ: ");
    Byte2String(buffer[4], buf);   // Z_L
    USART_EnQBytes(buf);
    USART_EnQBytes((unsigned char*)" ");
    Byte2String(buffer[5], buf);   // Z_H
    USART_EnQBytes(buf);
    USART_EnQBytes((unsigned char*)"\r");

    USART_EnQBytes((unsigned char*)"\r");
    Delay(1000);
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
    uint32_t isr = I2C1->ISR;

    // TXIS: send register address
    if (isr & (1U << 1))
    {
        I2C1->TXDR = i2c_job.reg;
        I2C1->CR1 &= ~(1U << 1);   // disable TXIE
        return;
    }

    // TC: issue repeated START for read
    if (isr & (1U << 6))
    {
        i2c_idx = 0;
        I2C1->CR2 =
            (i2c_job.addr << 1) |
            (i2c_job.len << 16) |
            (1U << 25) |          // AUTOEND
            (1U << 10) |          // RD_WRN
            (1U << 13);           // START
        return;
    }

    // RXNE: read data
    if (isr & (1U << 2))
    {
        i2c_job.buf[i2c_idx++] = I2C1->RXDR;
        return;
    }

    // STOPF: done
    if (isr & (1U << 5))
    {
        I2C1->ICR = (1U << 5);
        i2c_busy = 0;
        return;
    }

    // NACKF: error
    if (isr & (1U << 4))
    {
        I2C1->ICR = (1U << 4);
        i2c_busy = 0;
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

void Byte2String(uint8_t val, unsigned char *out)
{
    static const char hex[] = "0123456789ABCDEF";

    out[0] = hex[(val >> 4) & 0x0F];
    out[1] = hex[val & 0x0F];
    out[2] = '\0';
    out[3] = '\0';
}

//<-|

//|-> USART_EnQ Function

void USART_EnQ(const unsigned char data)
{
    // wait if buffer full
    while (RingTail == ((RingHead + 1) & (RingBuffer_Length - 1)));

    RingBuffer[RingHead] = data;
    RingHead = (RingHead + 1) & (RingBuffer_Length - 1);

    USART1->CR1 |= (1U << 7);   // TXFNFIE
}

//<-|

//|-> USART_EnQBytes Function

void USART_EnQBytes(const unsigned char *data)
{
  while(*data)
    USART_EnQ(*data++);
}

//<-|

