#ifndef DEFS_H
#define DEFS_H

#include <stdint.h>

//|->RCC Defs
#define RCC_BASE 0x46020C00U
#define RCC_AHB2RSTR  (*(volatile uint32_t *) (RCC_BASE + 0x064))
#define RCC_APB1RSTR1 (*(volatile uint32_t *) (RCC_BASE + 0x074))
#define RCC_APB2RSTR  (*(volatile uint32_t *) (RCC_BASE + 0x07C))
#define RCC_AHB2ENR   (*(volatile uint32_t *) (RCC_BASE + 0x08C))
#define RCC_APB1ENR1  (*(volatile uint32_t *) (RCC_BASE + 0x09C))
#define RCC_APB2ENR   (*(volatile uint32_t *) (RCC_BASE + 0x0A4))
#define RCC_CCIPR1    (*(volatile uint32_t *) (RCC_BASE + 0x0E0))
//<-|

//|->GPIO Defs
#define GPIO_BASE 0x42020000U

typedef struct{
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFRL, AFRH, BRR, RESERVED, SECCFGR;
}GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef *) (GPIO_BASE + (0x400 * 0)))
#define GPIOB ((GPIO_TypeDef *) (GPIO_BASE + (0x400 * 1)))
#define GPIOC ((GPIO_TypeDef *) (GPIO_BASE + (0x400 * 2)))
#define GPIOD ((GPIO_TypeDef *) (GPIO_BASE + (0x400 * 3)))
//<-|

//|->Interrtupts Defs
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104)
#define NVIC_ITNS1 (*(volatile uint32_t *)0xE000E384)
//<-|

//|->USART1 Defs
#define USART1_BASE 0x40013800U

typedef struct{
  volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR, PRESC, AUTOCR;
}USART_TypeDef;

#define USART1 ((USART_TypeDef *) USART1_BASE)
//<-|

//|->I2C1 Defs
#define I2C1_BASE 0x40005400U

typedef struct{
    volatile uint32_t CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR,  ISR, ICR, PECR, RXDR, TXDR, AUTOCR;
}I2C_TypeDef;

#define I2C1 ((I2C_TypeDef *) I2C1_BASE)
//<-|

//|->Functions
void Delay(volatile uint32_t ms);
void USART_EnQ(const unsigned char data);
void USART_EnQBytes(const unsigned char *data);
void Byte2String(uint8_t val, unsigned char *out);
int I2C_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
void i2c_write_reg(uint8_t reg, uint8_t val);
uint8_t i2c_read_reg(uint8_t reg);
void i2c_read_multi(uint8_t reg, uint8_t *buf, uint8_t len);
//<-|

#endif

