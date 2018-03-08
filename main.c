#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ext_include/stm32f7xx.h"
#include "stm32_cmsis_extension.h"

#include "own_std.h"
#include "tof_table.h"
#include "flash.h"

#define LED_ON()  HI(GPIOF, 2)
#define LED_OFF() LO(GPIOF, 2)

#define EPC01_RSTN_HIGH() HI(GPIOA,14)
#define EPC01_RSTN_LOW()  LO(GPIOA,14)
#define EPC23_RSTN_HIGH() HI(GPIOE, 2)
#define EPC23_RSTN_LOW()  LO(GPIOE, 2)

#define EPCLEDV_ON()    HI(GPIOE,3)
#define EPCLEDV_OFF()   LO(GPIOE,3)

#define EPC5V_ON()      HI(GPIOD,9)
#define EPC5V_OFF()     LO(GPIOD,9)

#define EPC10V_ON()     HI(GPIOD,8)
#define EPC10V_OFF()    LO(GPIOD,8)

#define EPCNEG10V_ON()  HI(GPIOE,4)
#define EPCNEG10V_OFF() LO(GPIOE,4)

#define EPC_DESEL() do{GPIOA->BSRR = 1UL<<15; GPIOC->BSRR = 1UL<<10; GPIOE->BSRR = 1UL<<0 | 1UL<<1;}while(0)  // Deselect all four bus buffers
// Make extra super sure you never select more than one. Deselect all first, then use a small deadtime to select one. Otherwise, short circuit and HW damage may occur.
#define EPC_SEL0() LO(GPIOA,15);
#define EPC_SEL1() LO(GPIOC,10);
#define EPC_SEL2() LO(GPIOE,0);
#define EPC_SEL3() LO(GPIOE,1);

/*
	Even though the sensors support 4 different I2C addresses, to reduce the bus capacitance, only two sensors share the I2C bus.
	Hence, two I2C peripherals are used to access the four sensors.

	DMA streams and channels are listed in ST RM0410 page 246. Errata to the text: they are not "examples", they are the actual and only mappings.
*/

#define EPC01_I2C     I2C3
#define EPC01_WR_DMA       DMA1_Stream0
#define EPC01_WR_DMA_CHAN  8
#define EPC01_RD_DMA       DMA1_Stream2
#define EPC01_RD_DMA_CHAN  3

#define EPC23_I2C     I2C4
#define EPC23_WR_DMA       DMA1_Stream5
#define EPC23_WR_DMA_CHAN  2
#define EPC23_RD_DMA       DMA1_Stream1
#define EPC23_RD_DMA_CHAN  8

#define N_SENSORS 4
#define N_I2C 2

static DMA_Stream_TypeDef * const EPC_I2C_WR_DMAS[N_I2C] = 
	{DMA1_Stream0, DMA1_Stream5};

static const int EPC_I2C_WR_DMAS_INT[N_I2C] = 
	{0, 5};

static DMA_Stream_TypeDef * const EPC_I2C_RD_DMAS[N_I2C] = 
	{DMA1_Stream2, DMA1_Stream1};

static const int EPC_I2C_RD_DMAS_INT[N_I2C] = 
	{2, 1};

static const int EPC_I2C_WR_CHANS[N_I2C] =
	{8, 2};

static const int EPC_I2C_RD_CHANS[N_I2C] =
	{3, 8};

static I2C_TypeDef * const EPC_I2CS[N_I2C] =
	{I2C3, I2C4};

static const uint8_t buses[N_SENSORS] =
{ 0, 0, 1, 1};

static const uint8_t addrs[N_SENSORS] =
{ 0b0100000, 0b0100001, 0b0100000, 0b0100001};


#define DMA_STREAM(_dma_, _stream_) (_dma_ ## _Stream ## _stream_)

#define RASPI_SPI       SPI2
#define RASPI_WR_DMA    DMA1
#define RASPI_WR_STREAM 4
//#define RASPI_WR_DMA_STREAM DMA1_Stream4 

#define RASPI_WR_DMA_CHAN   0
#define RASPI_RD_DMA    DMA1
#define RASPI_RD_STREAM 3
//#define RASPI_RD_DMA_STREAM DMA1_Stream3
#define RASPI_RD_DMA_CHAN   0

#define ROBOT_SPI     SPI5
#define ROBOT_WR_DMA       DMA2_Stream4 
#define ROBOT_WR_DMA_CHAN  2
#define ROBOT_RD_DMA       DMA2_Stream3
#define ROBOT_RD_DMA_CHAN  2



#define EPC02_ADDR 0b0100000
#define EPC13_ADDR 0b0100001



#define sq(x) ((x)*(x))

volatile int new_rx, new_rx_len;


typedef struct __attribute__((packed))
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // in mm
	int32_t y;
} pos_t;

void delay_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_us(uint32_t i)
{
	if(i==0) return;
	i *= 90;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_ms(uint32_t i) __attribute__((section(".text_itcm")));
void delay_ms(uint32_t i)
{
	while(i--)
	{
		delay_us(1000);
	}
}

static void uart_print_string_blocking(const char *buf)
{
	#ifndef USE_UART
		return;
	#endif
	while(buf[0] != 0)
	{
		while((USART3->ISR & (1UL<<7)) == 0) ;
		USART3->TDR = buf[0];
		buf++;
	}
}

void error(int code)
{
	__disable_irq();
	EPC01_RSTN_LOW();
	EPC23_RSTN_LOW();
	EPCNEG10V_OFF();
	EPC10V_OFF();
	EPC5V_OFF();
	EPCLEDV_OFF();
	EPC_DESEL();
	// Disable all other interrupts, but leave the one required for handling reflashing SPI message.
	NVIC_DisableIRQ(I2C3_EV_IRQn);
	NVIC_DisableIRQ(DMA1_Stream2_IRQn);
	NVIC_DisableIRQ(DMA2_Stream7_IRQn);
	NVIC_DisableIRQ(TIM5_IRQn);
	__enable_irq();

	int i = 0;
	while(1)
	{
		LED_ON();
		delay_ms(40);
		LED_OFF();
		delay_ms(40);
		if(++i == 10)
		{
			i = 0;
			char printbuf[32];
			uart_print_string_blocking(" E"); o_utoa32(code, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
		}
	}
}


#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

#define CALC_CRC(remainder) \
	for(int crc__bit = 8; crc__bit > 0; --crc__bit) \
	{ \
		if((remainder) & 0b10000000) \
		{ \
			(remainder) = ((remainder) << 1) ^ CRC_POLYNOMIAL; \
		} \
		else \
		{ \
			(remainder) = ((remainder) << 1); \
		} \
	}


// Things written to I2C CR1 every time:
#define I2C_CR1_BASICS (0UL<<8 /*Digital filter len 0 to 15*/)
#define I2C_CR1_BASICS_ON (I2C_CR1_BASICS | 1UL /*keep it on*/)

volatile int epc_i2c_write_busy[N_I2C], epc_i2c_read_busy[N_I2C];
volatile int epc_i2c_read_state[N_I2C];
uint8_t epc_i2c_read_slave_addr[N_I2C];
volatile uint8_t *epc_i2c_read_buf[N_I2C];
uint8_t epc_i2c_read_len[N_I2C];

void epc_i2c_write_dma(uint8_t bus, uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	if(bus >= N_I2C) return;
	// Pipeline flush is almost always needed when using this function, so easier to do it here.
	// If data cache is enabled later, remember to use non-cacheable sections (or invalidate the cache)
	__DSB(); __ISB();

	if(epc_i2c_write_busy[bus] || epc_i2c_read_busy[bus])
		error(11);

	if(EPC_I2C_WR_DMAS[bus]->CR & 1UL)
		error(12);

	epc_i2c_write_busy[bus] = 1;
	EPC_I2CS[bus]->ICR = 1UL<<5; // Clear any pending STOPF interrupt 

	if(len > 1) // Actually use DMA
	{		
		EPC_I2C_WR_DMAS[bus]->CR = EPC_I2C_WR_CHANS[bus]<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
				   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

		EPC_I2C_WR_DMAS[bus]->NDTR = len;
		EPC_I2C_WR_DMAS[bus]->M0AR = (uint32_t)buf;

		DMA_CLEAR_INTFLAGS(DMA1, EPC_I2C_WR_DMAS_INT[bus]);
		EPC_I2C_WR_DMAS[bus]->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

		EPC_I2CS[bus]->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<14 /*TX DMA*/;

	}
	else	// Just do the single write right now.
	{
		EPC_I2CS[bus]->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/; // no DMA
		EPC_I2CS[bus]->TXDR = buf[0];
	}

	EPC_I2CS[bus]->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	EPC_I2CS[bus]->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
}


#define epc_i2c_write epc_i2c_write_dma

void epc_i2c_read_dma(uint8_t bus, uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	if(bus >= N_I2C) return;
	EPC_I2C_RD_DMAS[bus]->CR = EPC_I2C_RD_CHANS[bus]<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /*transfer complete interrupt*/;

	EPC_I2C_RD_DMAS[bus]->NDTR = len;
	EPC_I2C_RD_DMAS[bus]->M0AR = (uint32_t)buf;

	EPC_I2CS[bus]->CR1 = I2C_CR1_BASICS_ON | 0UL<<5 /*OFF FOR READ: STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<15 /*RX DMA*/ ;

	EPC_I2CS[bus]->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 1UL<<10 /*read*/ | slave_addr_7b<<1;

	DMA_CLEAR_INTFLAGS(DMA1, EPC_I2C_RD_DMAS_INT[bus]);
	EPC_I2C_RD_DMAS[bus]->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

	EPC_I2CS[bus]->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
}

/*
	In I2C, a million things can go wrong. STM32 I2C implementation is notoriously buggy, device implementations often are, too.
	It makes little sense to try to detect every possible error condition, since they most often end up in total bus lockup
	anyway, and can only be reliably solved by device&bus reset. Since this is time-consuming anyway, and always leads to loss
	of data and hangup in execution time, we can as well solve all error checking using a simple watchdog that does the same,
	with much less code footprint (good for reliability, execution speed, flash usage, and code maintainability) compared to trying
	to catch all unexpected things in actual interrupt services or API functions.
	TODO: actually implement said watchdog
*/

void epc_i2c_read(uint8_t bus, uint8_t slave_addr_7b, uint8_t reg_addr, volatile uint8_t *buf, uint8_t len)
{
	if(bus >= N_I2C) return;
	if(epc_i2c_write_busy[bus] || epc_i2c_read_busy[bus])
		error(12);
	/*
		Full I2C read cycle consists of a write cycle (payload = reg_addr), without stop condition,
		then the actual read cycle starting with the repeated start condition.

		Since the write is always one byte of payload, we run it without DMA.

		After the write is complete, there is no AUTOEND generated, instead we get an interrupt, allowing
		us to go on with the read.
	*/
	epc_i2c_read_busy[bus] = 1;

	EPC_I2CS[bus]->CR1 = I2C_CR1_BASICS_ON | 1UL<<6 /*Transfer Complete interrupt - happens because AUTOEND=0 and RELOAD=0*/; // no DMA
	EPC_I2CS[bus]->CR2 = 0UL<<25 /*AUTOEND off*/ | 1UL<<16 /*len=1*/ | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	EPC_I2CS[bus]->TXDR = reg_addr;
	EPC_I2CS[bus]->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
	epc_i2c_read_state[bus] = 1;
	epc_i2c_read_slave_addr[bus] = slave_addr_7b;
	epc_i2c_read_buf[bus] = buf;
	epc_i2c_read_len[bus] = len;

}

// Returns true whenever read or write operation is going on.
int epc_i2c_is_busy(uint8_t bus)
{
	if(bus >= N_I2C) return 1;
	return epc_i2c_write_busy[bus] || epc_i2c_read_busy[bus];
}


void epc01_rx_dma_inthandler() // read complete.
{
	DMA_CLEAR_INTFLAGS(DMA1, 2);
	epc_i2c_read_busy[0] = 0;
}

void epc23_rx_dma_inthandler() // read complete.
{
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	epc_i2c_read_busy[1] = 0;
}

/*
	For some reason, I can't get repeated start (Sr) condition out of the new STM32 I2C - it generates a stop-start sequence even when only asked for a START.
	Luckily, the EPC chip still works correctly even though this violates the spec.
*/
void epc01_i2c_inthandler()
{
	I2C3->ICR = 1UL<<5; // Clear the STOPF flag.
	if(epc_i2c_read_state[0])
	{
		// Writing START to 1 (in epc_i2c_read_dma()) clears the interrupt flag
		epc_i2c_read_dma(0, epc_i2c_read_slave_addr[0], epc_i2c_read_buf[0], epc_i2c_read_len[0]);
		epc_i2c_read_state[0]=0;

	}
	else // STOPF interrupt - this was a write.
	{
		if(I2C3->ISR & (1UL<<15)) // busy shouldn't be high - at least fail instead of doing random shit
		{
			error(22);
		}

		// Write is now finished.
		epc_i2c_write_busy[0] = 0;
		I2C3->CR1 = I2C_CR1_BASICS_ON;
	}
}

void epc23_i2c_inthandler()
{
	I2C4->ICR = 1UL<<5; // Clear the STOPF flag.
	if(epc_i2c_read_state[1])
	{
		// Writing START to 1 (in epc_i2c_read_dma()) clears the interrupt flag
		epc_i2c_read_dma(1, epc_i2c_read_slave_addr[1], epc_i2c_read_buf[1], epc_i2c_read_len[1]);
		epc_i2c_read_state[1]=0;

	}
	else // STOPF interrupt - this was a write.
	{
		if(I2C4->ISR & (1UL<<15)) // busy shouldn't be high - at least fail instead of doing random shit
		{
			error(22);
		}

		// Write is now finished.
		epc_i2c_write_busy[1] = 0;
		I2C4->CR1 = I2C_CR1_BASICS_ON;
	}
}

void epc01_i2c_init()
{
	// DMA1 Stream0 = tx
	// DMA1 Stream2 = rx
	// For now, init in Fast mode 400 kHz
	// Use the datasheet example of 48MHz input clock, with prescaler 1.125 times bigger for 54 MHz. Sync delay calculations are near enough.
	// Silicon bug errata: tSU;DAT (do they mean SDADEL register??) must be at least one I2CCLK period
	// Silicon bug errata: bus errors are spuriously generated for no reason, the bug is not said to affect the transfer: just ignore BERR.
 
	DMA1_Stream0->PAR = (uint32_t)&(I2C3->TXDR);
	DMA1_Stream2->PAR = (uint32_t)&(I2C3->RXDR);

	// open drain:
	GPIOC->OTYPER  |= 1UL<<9;
	GPIOA->OTYPER  |= 1UL<<8;

	IO_TO_ALTFUNC(GPIOC, 9);
	IO_TO_ALTFUNC(GPIOA, 8);
	IO_SET_ALTFUNC(GPIOC, 9, 4);
	IO_SET_ALTFUNC(GPIOA, 8, 4);

//	I2C3->TIMINGR = 6UL<<28/*PRESCALER*/  | 9UL<<0/*SCLL*/  | 3UL<<8/*SCLH*/  | 3UL<<16 /*SDADEL*/  | 3UL<<20 /*SCLDEL*/;
	I2C3->TIMINGR = 14UL<<28/*PRESCALER*/  | 9UL<<0/*SCLL*/  | 3UL<<8/*SCLH*/  | 3UL<<16 /*SDADEL*/  | 3UL<<20 /*SCLDEL*/;
	I2C3->CR1 = I2C_CR1_BASICS;

	I2C3->CR1 |= 1UL; // Enable

	NVIC_SetPriority(I2C3_EV_IRQn, 0b0101);
	NVIC_EnableIRQ(I2C3_EV_IRQn);
	NVIC_SetPriority(DMA1_Stream2_IRQn, 0b0101);
	NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

void epc23_i2c_init()
{
	DMA1_Stream5->PAR = (uint32_t)&(I2C4->TXDR);
	DMA1_Stream1->PAR = (uint32_t)&(I2C4->RXDR);

	// open drain:
	GPIOB->OTYPER |= 1UL<<6;
	GPIOB->OTYPER |= 1UL<<7;

	IO_TO_ALTFUNC(GPIOB, 6);
	IO_TO_ALTFUNC(GPIOB, 7);
	IO_SET_ALTFUNC(GPIOB, 6, 11);
	IO_SET_ALTFUNC(GPIOB, 7, 11);

//	I2C4->TIMINGR = 6UL<<28/*PRESCALER*/  | 9UL<<0/*SCLL*/  | 3UL<<8/*SCLH*/  | 3UL<<16 /*SDADEL*/  | 3UL<<20 /*SCLDEL*/;
	I2C4->TIMINGR = 14UL<<28/*PRESCALER*/  | 9UL<<0/*SCLL*/  | 3UL<<8/*SCLH*/  | 3UL<<16 /*SDADEL*/  | 3UL<<20 /*SCLDEL*/;
	I2C4->CR1 = I2C_CR1_BASICS;

	I2C4->CR1 |= 1UL; // Enable

	NVIC_SetPriority(I2C4_EV_IRQn, 0b0101);
	NVIC_EnableIRQ(I2C4_EV_IRQn);
	NVIC_SetPriority(DMA1_Stream1_IRQn, 0b0101);
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}


#define EPC_XS 160
#define EPC_YS 60

typedef struct __attribute__((packed))
{
	uint16_t start_pad[2];
	uint16_t img[EPC_XS*EPC_YS];
} epc_img_t;

typedef struct __attribute__((packed))
{
	epc_img_t dcs[4];
} epc_4dcs_t;

typedef struct __attribute__((packed))
{
	epc_img_t dcs[2];
} epc_2dcs_t;

#define SIZEOF_MONO (sizeof(epc_img_t))  //(EPC_XS*EPC_YS*2)
#define SIZEOF_2DCS (sizeof(epc_2dcs_t)) //(EPC_XS*EPC_YS*2*2)
#define SIZEOF_4DCS (sizeof(epc_4dcs_t)) //(EPC_XS*EPC_YS*2*4)


volatile int epc_capture_finished = 0;
void epc_dcmi_dma_inthandler()
{
	// DMA finished
	DMA_CLEAR_INTFLAGS(DMA2, 7);
	epc_capture_finished = 1;
}

void epc_shutdown_inthandler()
{
	/*
		Proper power-down sequence to prevent damaging the sensors.
		This is quickly triggered from the 3V3 line dropping.
		The idea is to turn the reset on and disable all the other power
		supplies in the correct order before the 3V3 line goes completely down.
	*/
	EPC01_RSTN_LOW();
	EPC23_RSTN_LOW();
	EPCNEG10V_OFF();
	EPC10V_OFF();
	EPC5V_OFF();
	EPCLEDV_OFF();
	EPC_DESEL();
	LED_ON();
	while(1);
}

void epc_dcmi_init()
{
	RCC->AHB2ENR |= 1UL;
	IO_TO_ALTFUNC(GPIOA,  6);
	IO_TO_ALTFUNC(GPIOA,  9);
	IO_TO_ALTFUNC(GPIOA, 10);
	IO_TO_ALTFUNC(GPIOG, 10);
	IO_TO_ALTFUNC(GPIOG, 11);
	IO_TO_ALTFUNC(GPIOC, 11);
	IO_TO_ALTFUNC(GPIOD,  3);
	IO_TO_ALTFUNC(GPIOB,  8);
	IO_TO_ALTFUNC(GPIOB,  9);

	IO_SET_ALTFUNC(GPIOA,  6, 13);
	IO_SET_ALTFUNC(GPIOA,  9, 13);
	IO_SET_ALTFUNC(GPIOA, 10, 13);
	IO_SET_ALTFUNC(GPIOG, 10, 13);
	IO_SET_ALTFUNC(GPIOG, 11, 13);
	IO_SET_ALTFUNC(GPIOC, 11, 13);
	IO_SET_ALTFUNC(GPIOD,  3, 13);
	IO_SET_ALTFUNC(GPIOB,  8, 13);
	IO_SET_ALTFUNC(GPIOB,  9, 13);

	DCMI->CR = 0b00UL<<10 /*8-bit data*/ | 0UL<<5 /* CLK falling edge*/ | 1UL<<4 /*Embedded sync*/ | 0UL<<1 /*continuous grab*/;

	// Program the default synchronization codes used in the epc635
	DCMI->ESCR = 0x1eUL<<0 /*frame start*/ | 0xffUL<<24 /*frame end*/ | 0xaaUL<<8 /*line start*/ | 0x55<<16 /*line end*/;
	DCMI->ESUR = 0xffffffffUL; // mask for the previous: all bits compared

	DCMI->CR |= 1UL<<14; // Enable

	DMA2_Stream7->PAR = (uint32_t)&(DCMI->DR);
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 0UL<<8 /*circular OFF*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	// NDTR is not set yet because it varies. Stream is not enabled yet.

	NVIC_SetPriority(DMA2_Stream7_IRQn, 0b1111);
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	DCMI->CR |= 1UL<<0; // Start CAPTURE

}

void dcmi_start_dma(void *data, int size)
{
	// Disable the stream first
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	while(DMA2_Stream7->CR & 1UL) ;

	DMA_CLEAR_INTFLAGS(DMA2, 7);

	DMA2_Stream7->M0AR = (uint32_t)data;

	DMA2_Stream7->NDTR = size/4; // Num of 32-bit transfers
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	DMA_CLEAR_INTFLAGS(DMA2, 7);
	DMA2_Stream7->CR |= 1; // Enable DMA
	__DSB(); __ISB();
}

void trig(int idx)
{
	static volatile uint8_t b[2] = {0xa4, 1};
	epc_i2c_write(buses[idx], addrs[idx], b, 2);
	while(epc_i2c_is_busy(buses[idx]));
}

volatile uint8_t epc_wrbuf[16];// __attribute__((section(".data_dtcm"))); // to skip cache
volatile uint8_t epc_rdbuf[16];// __attribute__((section(".data_dtcm"))); // to skip cache

volatile int timer_10k;
void timebase_handler()
{
	TIM5->SR = 0; // Clear interrupt flag
	timer_10k++;
}

/*
Process four DCS images, with offset_mm in millimeters. With clk_div=1, does the calculation at fled=20MHz (unamb range = 7.5m). Larger clk_div
multiplies the result.
*/

void tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int i=0; i < 160*60; i++)
	{
		uint16_t dist;
		uint8_t ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
		{
			ampl = 255;
			dist = 0;
		}
		else if((in->dcs[0].img[i]&1) || (in->dcs[1].img[i]&1) ||
		   (in->dcs[2].img[i]&1) || (in->dcs[3].img[i]&1))
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			// Use the lookup table to perform atan:

			int16_t dcs31_mod, dcs20_mod;

			if(dcs31<0)
				dcs31_mod = -dcs31;
			else
				dcs31_mod = dcs31;

			if(dcs20<0)
				dcs20_mod = -dcs20;
			else
				dcs20_mod = dcs20;

			int swapped = 0;
			if(dcs20_mod<dcs31_mod)
			{
				swapped = 1;
				int16_t tmp = dcs20_mod;
				dcs20_mod = dcs31_mod;
				dcs31_mod = tmp;
			}

			if(dcs20_mod == 0)
			{
				dist = 65535;
				ampl = 0;
			}
			else
			{

				int idx = (dcs31_mod*(TOF_TBL_LEN-1))/dcs20_mod;

				int32_t dist_i = tof_tbl[idx];
				if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
				if(dcs20<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
				if(dcs31<0) dist_i = -dist_i;

				dist_i += offset;

//				if(dist_i < 0) dist_i += TOF_TBL_PERIOD;
//				else if(dist_i > TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;
				
				dist_i *= clk_div;


				if(dist_i < 1) dist_i = 1; else if(dist_i>6000) dist_i=6000;

				ampl = sqrt(sq(dcs20)+sq(dcs31))/23;

				dist = dist_i;

			}

		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}


void epc_clk_div(int idx, int div)
{
	epc_wrbuf[0] = 0x85;
	epc_wrbuf[1] = div-1;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

void epc_dis_leds(int idx)
{
	epc_wrbuf[0] = 0x90;
	epc_wrbuf[1] = 0b11001000; // leds disabled
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

void epc_ena_leds(int idx)
{
	epc_wrbuf[0] = 0x90;
	epc_wrbuf[1] = 0b11101000; // LED2 output on
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

void epc_greyscale(int idx)
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b11000100; // greyscale modulation
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

void epc_2dcs(int idx)
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00010100; // 2dcs modulation
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

void epc_4dcs(int idx)
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00110100; // 4dcs modulation
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

void epc_intlen(int idx, uint8_t multiplier, uint16_t time)
{
	int intlen = ((int)time<<2)-1;
	epc_wrbuf[0] = 0xA1;
	epc_wrbuf[1] = multiplier;
	epc_wrbuf[2] = (intlen&0xff00)>>8;
	epc_wrbuf[3] = intlen&0xff;

	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 4);
}

void calc_toofar_ignore_from_2dcs(uint8_t *ignore_out, epc_4dcs_t *in, int threshold /*mm*/, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int yy = 0; yy < EPC_YS; yy++)
	{
		for(int xx = 0; xx < EPC_XS; xx++)
		{
			int i = yy*EPC_XS+xx;
			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

			if((in->dcs[0].img[i]&1) || (in->dcs[1].img[i]&1) || dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
			{
				// Overexp - it's ok, probably not too far.
			}
			else
			{
				// for 2dcs mode to get the quadrant correct:
				dcs0 *= -1;
				dcs1 *= -1;

				// Use the lookup table to perform atan:

				int16_t dcs1_mod, dcs0_mod;

				if(dcs1<0)
					dcs1_mod = -dcs1;
				else
					dcs1_mod = dcs1;

				if(dcs0<0)
					dcs0_mod = -dcs0;
				else
					dcs0_mod = dcs0;

				int swapped = 0;
				if(dcs0_mod<dcs1_mod)
				{
					swapped = 1;
					int16_t tmp = dcs0_mod;
					dcs0_mod = dcs1_mod;
					dcs1_mod = tmp;
				}

				if(sq(dcs0)+sq(dcs1) < sq(75) || dcs0_mod == 0 /* should always be true if the first one is: todo: prove*/)
				{
					// amplitude too low
					ignore_out[yy*EPC_XS+xx] = 1;
				}
				else
				{
					int idx = (dcs1_mod*(TOF_TBL_LEN-1))/dcs0_mod;

					int32_t dist_i = tof_tbl[idx];
					if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
					if(dcs0<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
					if(dcs1<0) dist_i = -dist_i;

					dist_i += offset;

					//if(dist_i < 0) dist_i += TOF_TBL_PERIOD;
					//else if(dist_i > TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;
				
					dist_i *= clk_div;

					if(dist_i > threshold)
						ignore_out[yy*EPC_XS+xx] = 1;
				}

			}



		}


	}
}



void calc_interference_ignore_from_2dcs(uint8_t *ignore_out, epc_4dcs_t *in, int threshold)
{
	for(int yy = 0; yy < EPC_YS; yy++)
	{
		for(int xx = 0; xx < EPC_XS; xx++)
		{
			int16_t dcs0 = ((in->dcs[0].img[yy*EPC_XS+xx]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[yy*EPC_XS+xx]&0b0011111111111100)>>2)-2048;

			if( (in->dcs[0].img[yy*EPC_XS+xx]&1) || (in->dcs[1].img[yy*EPC_XS+xx]&1) ||
			    dcs0 < -1*threshold || dcs0 > threshold || dcs1 < -1*threshold || dcs1 > threshold)
			{
				ignore_out[yy*EPC_XS+xx] = 1;
			}
		}
	}
}


/*
	Suitability vector.

	We know that low amplitudes aren't good: there is more sensor noise - also small
	amplitudes are swept over with modulated stray light in the lens.

	In reality, high amplitudes aren't that good either: they suggest excessive exposure,
	which increases multipath and stray light!

	When combining HDR images, we want to prevent abrupt changes. So we are going to take
	the combination of suitable exposures and combine them based on how "good" we think
	they are based on amplitude.

*/
static uint8_t ampl_suitability[256] __attribute__((section(".dtcm_data"))) =
{
	0,	0,	0,	5,	10,	15,	20,	25,	30,	35,
	40,	50,	60,	80,	100,	120,	140,	160,	180,	200,
	220,	240,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	249,	248,	247,	246,	245,	244,	243,	242,	241,	240,	
	239,	238,	237,	236,	235,	234,	233,	232,	231,	230,	


	229,	228,	227,	226,	225,	224,	223,	222,	221,	220,	
	219,	218,	217,	216,	215,	214,	213,	212,	211,	210,	
	209,	208,	207,	206,	205,	204,	203,	202,	201,	200,	
	199,	198,	197,	196,	195,	194,	193,	192,	191,	190,	
	189,	188,	187,	186,	185,	184,	183,	182,	181,	180,	
	179,	178,	177,	176,	175,	174,	173,	172,	171,	170,	
	169,	168,	167,	166,	165,	164,	163,	162,	161,	160,	
	159,	158,	157,	156,	155,	154,	153,	152,	151,	150,	
	149,	148,	147,	146,	145,	144,	143,	142,	141,	140,	
	139,	138,	137,	136,	135,	134,	133,	132,	131,	130,	

	129,	128,	127,	126,	125,	124,	123,	122,	121,	120,	
	119,	118,	117,	116,	115,	114,	113,	112,	111,	110,	
	109,	108,	107,	106,	105,	104,	103,	102,	101,	100,	
	99,	98,	97,	96,	95,	94,	93,	92,	91,	90,	
	89,	88,	87,	86,	85,	84,	83,	82,	81,	80,	
	79,	78,	77,	76,	75,	0 /*overexp*/
};


/*
	ampl and dist are expected to contain 3*EPC_XS*EPC_YS elements.
*/
void tof_calc_dist_3hdr_with_ignore(uint16_t* dist_out, uint8_t* ampl, uint16_t* dist, uint8_t* ignore_in)
{
	for(int yy=0; yy < EPC_YS; yy++)
	{
		for(int xx=0; xx < EPC_XS; xx++)
		{
			int pxidx = yy*EPC_XS+xx;
			if(     // On ignore list: either directly, or on any 8 neighbors.
				ignore_in[pxidx] ||
				( yy>0        &&    ( ignore_in[(yy-1)*EPC_XS+xx] || (xx>0 && ignore_in[(yy-1)*EPC_XS+xx-1]) || (xx<EPC_XS-1 && ignore_in[(yy-1)*EPC_XS+xx+1]) ) ) ||
				( yy<EPC_YS-1 &&    ( ignore_in[(yy+1)*EPC_XS+xx] || (xx>0 && ignore_in[(yy+1)*EPC_XS+xx-1]) || (xx<EPC_XS-1 && ignore_in[(yy+1)*EPC_XS+xx+1]) ) ) ||
				( xx>0        &&    ignore_in[yy*EPC_XS+xx-1] ) ||
				( xx<EPC_XS-1 &&    ignore_in[yy*EPC_XS+xx+1] )
			  )
			{
				dist_out[pxidx] = 0;
			}
			else if(ampl[0*EPC_XS*EPC_YS+pxidx] == 255) // Shortest exposure overexposed
			{
				dist_out[pxidx] = 1;
			}
			else
			{
				int32_t suit0 = ampl_suitability[ampl[0*EPC_XS*EPC_YS+pxidx]];
				int32_t suit1 = ampl_suitability[ampl[1*EPC_XS*EPC_YS+pxidx]];
				int32_t suit2 = ampl_suitability[ampl[2*EPC_XS*EPC_YS+pxidx]];

				int32_t dist0 = dist[0*EPC_XS*EPC_YS+pxidx];
				int32_t dist1 = dist[1*EPC_XS*EPC_YS+pxidx];
				int32_t dist2 = dist[2*EPC_XS*EPC_YS+pxidx];

				int32_t suit_sum = suit0 + suit1 + suit2;

				if(suit_sum < 10)
					dist_out[pxidx] = 0;
				else
				{
					dist_out[pxidx] = (dist0*suit0 + dist1*suit1 + dist2*suit2)/suit_sum;
				}				

			}

		}
	}
}

#define HDR_EXP_MULTIPLIER 5 // integration time multiplier when going from shot 0 to shot1, or from shot1 to shot2

#define STRAY_CORR_LEVEL 2000 // smaller -> more correction
#define STRAY_BLANKING_LVL 40 // smaller -> more easily ignored

void tof_calc_dist_3hdr_with_ignore_with_straycomp(uint16_t* dist_out, uint8_t* ampl, uint16_t* dist, uint8_t* ignore_in, uint16_t stray_ampl, uint16_t stray_dist)
{
	int stray_ignore = stray_ampl/STRAY_BLANKING_LVL;
	for(int yy=0; yy < EPC_YS; yy++)
	{
		for(int xx=0; xx < EPC_XS; xx++)
		{
			int pxidx = yy*EPC_XS+xx;
			if(     // On ignore list: either directly, or on any 8 neighbors.
				ignore_in[pxidx] ||
				( yy>0        &&    ( ignore_in[(yy-1)*EPC_XS+xx] || (xx>0 && ignore_in[(yy-1)*EPC_XS+xx-1]) || (xx<EPC_XS-1 && ignore_in[(yy-1)*EPC_XS+xx+1]) ) ) ||
				( yy<EPC_YS-1 &&    ( ignore_in[(yy+1)*EPC_XS+xx] || (xx>0 && ignore_in[(yy+1)*EPC_XS+xx-1]) || (xx<EPC_XS-1 && ignore_in[(yy+1)*EPC_XS+xx+1]) ) ) ||
				( xx>0        &&    ignore_in[yy*EPC_XS+xx-1] ) ||
				( xx<EPC_XS-1 &&    ignore_in[yy*EPC_XS+xx+1] )
			  )
			{
				dist_out[pxidx] = 0;
			}
			else if(ampl[0*EPC_XS*EPC_YS+pxidx] == 255) // Shortest exposure overexposed
			{
				dist_out[pxidx] = 1;
			}
			else
			{
				int32_t suit0 = ampl_suitability[ampl[0*EPC_XS*EPC_YS+pxidx]];
				int32_t suit1 = ampl_suitability[ampl[1*EPC_XS*EPC_YS+pxidx]];
				int32_t suit2 = ampl_suitability[ampl[2*EPC_XS*EPC_YS+pxidx]];

				int32_t dist0 = dist[0*EPC_XS*EPC_YS+pxidx];
				int32_t dist1 = dist[1*EPC_XS*EPC_YS+pxidx];
				int32_t dist2 = dist[2*EPC_XS*EPC_YS+pxidx];

				int32_t ampl0 = ampl[0*EPC_XS*EPC_YS+pxidx]*HDR_EXP_MULTIPLIER*HDR_EXP_MULTIPLIER;
				int32_t ampl1 = ampl[1*EPC_XS*EPC_YS+pxidx]*HDR_EXP_MULTIPLIER;
				int32_t ampl2 = ampl[2*EPC_XS*EPC_YS+pxidx];

				int32_t suit_sum = suit0 + suit1 + suit2;

				if(suit_sum < 10)
					dist_out[pxidx] = 0;
				else
				{
					int32_t combined_ampl = (ampl0*suit0 + ampl1*suit1 + ampl2*suit2)/suit_sum;
					int32_t combined_dist = (dist0*suit0 + dist1*suit1 + dist2*suit2)/suit_sum;

					int32_t corr_amount = ((16*(int32_t)stray_ampl)/(int32_t)combined_ampl);

					// Expect higher amplitude from "close" pixels. If the amplitude is low, they are artefacts most likely.
					int32_t expected_ampl;
					if(combined_dist > 2000)
						expected_ampl = stray_ignore;
					else
						expected_ampl = (2000*stray_ignore)/combined_dist;

					if(corr_amount > 1000 || corr_amount < -100 || combined_ampl < expected_ampl)
					{
						dist_out[pxidx] = 0;
					}
					else
					{
						combined_dist -= stray_dist;

						combined_dist *= STRAY_CORR_LEVEL + corr_amount;
						combined_dist /= STRAY_CORR_LEVEL;

						combined_dist += stray_dist;

						if(combined_dist < 1) combined_dist = 1;
						else if(combined_dist > 6000) combined_dist = 6000;

						dist_out[pxidx] = combined_dist;
					}
				}				

			}

		}
	}
}


void process_bw(uint8_t *out, epc_img_t *in)
{
	for(int i=0; i<EPC_XS*EPC_YS; i++)
	{
		int lum = ((in->img[i]&0b0011111111111100)>>2)-2048;
		if(lum < 0) lum = 0; else if(lum > 2047) lum = 2047;
		out[i] = lum>>3;
	}
}

void process_dcs(uint8_t *out, epc_img_t *in)
{
	for(int i=0; i<EPC_XS*EPC_YS; i++)
	{
		uint16_t lum = ((in->img[i]&0b0011111111111100)>>2);
		out[i] = lum>>4;
	}
}

typedef struct __attribute__((packed)) __attribute__((aligned(4)))
{
	uint32_t header;
	uint8_t status; // Only read this far and deassert chip select for polling the status.
	uint8_t dummy1;
	uint8_t dummy2;
	uint8_t sensor_idx;

	pos_t robot_pos; // Robot pose during the acquisition

	uint16_t depth[EPC_XS*EPC_YS];
//	uint8_t  ampl[EPC_XS*EPC_YS];
//	uint8_t  ambient[EPC_XS*EPC_YS];
	uint16_t uncorrected_depth[EPC_XS*EPC_YS];

	uint8_t dbg_id;
	uint8_t dbg[2*EPC_XS*EPC_YS];

	uint16_t timestamps[24]; // 0.1ms unit timestamps of various steps for analyzing the timing of low-level processing
	int32_t  dbg_i32[8];

} pulutof_frame_t;

volatile pulutof_frame_t raspi_tx __attribute__((aligned(4)));

#define RASPI_RX_LEN 1000
volatile uint8_t raspi_rx[RASPI_RX_LEN] __attribute__((aligned(4)));


void init_raspi_tx()
{
	raspi_tx.header = 0x11223344;
	raspi_tx.status = 0;
	raspi_tx.dummy1 = 111;
	raspi_tx.dummy2 = 222;
	raspi_tx.sensor_idx = 0;
}

static uint8_t ignore[EPC_XS*EPC_YS] __attribute__((section(".dtcm_bss")));
static epc_4dcs_t dcsa, dcsb __attribute__((aligned(4)));
static epc_img_t mono_long, mono_short __attribute__((aligned(4)));

void top_init()
{
	int idx = 0;
	delay_ms(300);
	EPC10V_ON();
	EPC5V_ON();
	delay_ms(100);
	EPCNEG10V_ON();
	delay_ms(100);
	EPC01_RSTN_HIGH();
	delay_ms(300);


	epc01_i2c_init();

	/*
		Even with 40cm cable, 40MHz (div 2) works well!

	*/

	{
		epc_wrbuf[0] = 0x89; 
		epc_wrbuf[1] = (3 /*TCMI clock div 2..16, default 4*/ -1) | 0<<7 /*add clock delay?*/;
		epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
		while(epc_i2c_is_busy(buses[idx]));
	}

	{
		epc_wrbuf[0] = 0xcb; // i2c&tcmi control
		epc_wrbuf[1] = 0b01101111; // saturation bit, split mode, gated dclk, ESM
		epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
		while(epc_i2c_is_busy(buses[idx]));
	}

	{
		epc_wrbuf[0] = 0x90; // led driver control
		epc_wrbuf[1] = 0b11001000;
		epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
		while(epc_i2c_is_busy(buses[idx]));
	}

	{
		epc_wrbuf[0] = 0x92; // modulation select
		epc_wrbuf[1] = 0b11000100; // grayscale
		epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
		while(epc_i2c_is_busy(buses[idx]));
	}

/*
	{
		epc_wrbuf[0] = 0xcc; // tcmi polarity settings
		epc_wrbuf[1] = 1<<7; //saturate data;
		epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
		while(epc_i2c_is_busy(buses[idx]));
	}
*/

	EPCLEDV_ON();

	delay_ms(100);

	EPC_SEL0();

	epc_dcmi_init();

	// Take a dummy frame, which will eventually output the end-of-frame sync marker, to get the DCMI sync marker parser in the right state
	{
		dcmi_start_dma(&mono_short, SIZEOF_MONO);
		trig(idx);
		delay_ms(100);
	}


}

#define OFFSET_CALC_Y_IGNORE (15)
#define OFFSET_CALC_X_IGNORE (50)
#define OFFSET_CALC_NUM_PX   ((EPC_YS-2*OFFSET_CALC_Y_IGNORE)*(EPC_XS-2*OFFSET_CALC_X_IGNORE))

void tof_calc_offset(epc_4dcs_t *in, int clk_div, int *n_overs, int *n_unders, int *n_valids, int *dist_sum)
{
	int overexps = 0, underexps = 0, valids = 0, dist_accum = 0;

	for(int yy=OFFSET_CALC_Y_IGNORE; yy < 60-OFFSET_CALC_Y_IGNORE; yy++)
	{
		for(int xx=OFFSET_CALC_X_IGNORE; xx<160-OFFSET_CALC_X_IGNORE; xx++)
		{
			int i = yy*EPC_XS+xx;

			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

			if((in->dcs[0].img[i]&1) || (in->dcs[1].img[i]&1) ||
			   (in->dcs[2].img[i]&1) || (in->dcs[3].img[i]&1) ||
			   dcs0 < -1800 || dcs0 > 1800 || dcs1 < -1800 || dcs1 > 1800 || dcs2 < -1800 || dcs2 > 1800 || dcs3 < -1800 || dcs3 > 1800)
			{
				overexps++;
			}
			else
			{
				int16_t dcs31 = dcs3-dcs1;
				int16_t dcs20 = dcs2-dcs0;

				// Use the lookup table to perform atan:

				int16_t dcs31_mod, dcs20_mod;

				if(dcs31<0)
					dcs31_mod = -dcs31;
				else
					dcs31_mod = dcs31;

				if(dcs20<0)
					dcs20_mod = -dcs20;
				else
					dcs20_mod = dcs20;

				int swapped = 0;
				if(dcs20_mod<dcs31_mod)
				{
					swapped = 1;
					int16_t tmp = dcs20_mod;
					dcs20_mod = dcs31_mod;
					dcs31_mod = tmp;
				}

				if(dcs20_mod == 0 || sq(dcs20)+sq(dcs31) < sq(400))
				{
					underexps++;
				}
				else
				{

					int idx = (dcs31_mod*(TOF_TBL_LEN-1))/dcs20_mod;

					int32_t dist_i = tof_tbl[idx];
					if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
					if(dcs20<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
					if(dcs31<0) dist_i = -dist_i;

//					if(dist_i < 0) dist_i += TOF_TBL_PERIOD;
//					else if(dist_i > TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;
				
					dist_i *= clk_div;

					valids++;
					dist_accum += dist_i;
				}

			}
		}
	}

	*dist_sum = dist_accum;
	*n_valids = valids;
	*n_unders = underexps;
	*n_overs = overexps;
}



int run_offset_cal()
{
	int idx = 0;
	epc_ena_leds(idx);
	while(epc_i2c_is_busy(buses[idx]));

	epc_4dcs(idx);
	while(epc_i2c_is_busy(buses[idx]));

	for(int clkdiv=1; clkdiv < 4; clkdiv++)
	{
		epc_clk_div(idx, clkdiv);
		while(epc_i2c_is_busy(buses[idx]));

		int int_time = 5;
		while(1)
		{
			epc_intlen(idx, 1, int_time);
			while(epc_i2c_is_busy(buses[idx]));

			dcmi_start_dma(&dcsa, SIZEOF_4DCS);
			trig(idx);
			LED_ON();
			while(!epc_capture_finished) ;
			epc_capture_finished = 0;
			LED_OFF();

			int n_overs, n_unders, n_valids, dist_sum;

			tof_calc_offset(&dcsa, clkdiv, &n_overs, &n_unders, &n_valids, &dist_sum);

			if(n_overs > 20)
				return 100+clkdiv;

			if(int_time > 500)
				return 200+clkdiv;


			if(n_valids < (OFFSET_CALC_NUM_PX/8))
			{
				int_time = (int_time*3)/2;

			}
			else if(n_valids < (OFFSET_CALC_NUM_PX*9/10))
			{
				int_time = (int_time*6)/5;
			}
			else
			{
				settings.offsets[clkdiv] = -1*(dist_sum/n_valids) + 50;
				break;
			}

			delay_ms(100); // Let the LED dies cool down a bit - we never do super-long bursts on a single camera in the actual usage, either.
		}
		delay_ms(200);
	} 

	save_flash_settings();
	refresh_settings();
	return 0;
}

/*
	Estimate the amount of stray light interference in the optical path (internal reflections in the lens mostly)

	Use the shortest actual 4dcs exposure for this.

	While all light contributes to stray light, only that of very high intensity does matter - the stray light error can be seen
	when there are reflective objects very close to the camera (less than 0.5m, usually it gets really bad below 200 mm).

	Objects on the LED side cause more problems.

	Objects just outside the imageable area cause stray light as well. Plastic lens hoods take care of most, but we cannot completely
	solve the issue using them, there always is a transition area where we can't see using the sensor, but which still hits the lens.

	We only use the shortest HDR shot for this. It's important that:

		* There is not too much overexposure - very close (like 5cm) objects of course get overexposed. But the amount of stray light
		generated doesn't get saturated, it increases even further, so we'll lost track of the actual amount of exposure.
		We just assume overexposed pixel to have somewhat (50%?) higher amplitude than the biggest legal value. But if we have a lot
		of overexposed stuff, we can't know if it's really +20% or +500% overexposed, which will affect the quality of stray light estimate.

	The function estimates the amount of "stray light amplitude" - this can be later compared with amplitudes at other pixel sites.
	The function also calculates a weighed average of the distance this stray light is at. This can be used when correcting other pixels.

	The function gives extra weigh for areas where objects cause higher levels of stray light. For example, the pixel edge on the LED side
	is given a lot of weight, since it indicates that the object most likely continues over the edge. Still, let's not overdo the corrections.

	Weight table is reduced in resolution by 4*4 (i.e., it's 40*15 instead of 160*60)

	From octave/Matlab:
	mid=[4, 4, 6, 8,10,13,15,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,15,13,10, 8, 6, 4, 4]
	c=[floor(mid*6.2);floor(mid*2.5);floor(mid*2.3);floor(mid*2.0);floor(mid*1.7);floor(mid*1.5);floor(mid*1.4);floor(mid*1.3);floor(mid*1.2);floor(mid*1.1);floor(mid*1.05);floor(mid*1.0);floor(mid*0.95);floor(mid*0.9);floor(mid*0.9)]

*/

static uint8_t stray_weight[15*40] __attribute__((section(".dtcm_data"))) =
{                                             /* LEDS ON THIS SIDE */
 24,24,37,49,62,80,93,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,93,80,62,49,37,24,24,
 10,10,15,20,25,32,37,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,37,32,25,20,15,10,10,
 9, 9,13,18,23,29,34,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,34,29,23,18,13, 9, 9,
 8, 8,12,16,20,26,30,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,30,26,20,16,12, 8, 8,
 6, 6,10,13,17,22,25,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,25,22,17,13,10, 6, 6,
 6, 6, 9,12,15,19,22,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,22,19,15,12, 9, 6, 6,
 5, 5, 8,11,14,18,21,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,21,18,14,11, 8, 5, 5,
 5, 5, 7,10,13,16,19,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,19,16,13,10, 7, 5, 5,
 4, 4, 7, 9,12,15,18,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,18,15,12, 9, 7, 4, 4,
 4, 4, 6, 8,11,14,16,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,16,14,11, 8, 6, 4, 4,
 4, 4, 6, 8,10,13,15,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,15,13,10, 8, 6, 4, 4,
 4, 4, 6, 8,10,13,15,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,15,13,10, 8, 6, 4, 4,
 3, 3, 5, 7, 9,12,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,14,12, 9, 7, 5, 3, 3,
 3, 3, 5, 7, 9,11,13,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,11, 9, 7, 5, 3, 3,
 3, 3, 5, 7, 9,11,13,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,11, 9, 7, 5, 3, 3
};                            /* NON-LED SIDE: items here don't seem to cause any issue */

static void calc_stray_estimate(uint8_t *ampl_in, uint16_t *dist_in, uint16_t *stray_ampl, uint16_t *stray_dist)
{
	int32_t ampl_acc = 0;
	int64_t dist_acc = 0;
	// To save time, we only process 1/16 of the pixels
	for(int yy=0; yy<15; yy++)
	{
		for(int xx=0; xx<40; xx++)
		{
			int weigh_i = yy*40+xx;
			int pix_i = (yy<<2)*160+(xx<<2);

			int32_t ampl_causing_stray = (int32_t)ampl_in[pix_i] * (int32_t)stray_weight[weigh_i]; // Biggest = 255*99 = 25245

			ampl_acc += ampl_causing_stray;
			dist_acc += dist_in[pix_i] * ampl_causing_stray; // Weigh the distances based on the amplitude. Biggest: 65535 * 25245
		}
	}

	*stray_dist = dist_acc / ampl_acc; // Average distance of weighed stray-causing light (short distances dominate)
	*stray_ampl = ampl_acc / (15*40); // Biggest = 25245
}

/*
	The previous stray light estimator works very well as long as the object causing stray light is within the imaged
	area. Unfortunately, objects outside the imaged area cause stray light as well, as long as they hit the lens surface.
	Good lens hoods almost solve the issue, but it's impossible to completely solve in a passive way.

	The hint is: *real* near objects have very high amplitudes. If calculated distance is low, with low amplitude, it's
	likely a miscalculation from the stray light.


*/
static int32_t calc_outside_stray_estimate(uint8_t* ampl_in, uint16_t* dist_in, uint16_t *stray_ampl)
{
	// To save time, we only process 1/4 of the pixels

	int32_t n_acc = 0, acc = 0;
	for(int yy=0; yy<EPC_YS; yy+=2)
	{
		for(int xx=0; xx<EPC_XS; xx+=2)
		{
			int pix_i = yy*EPC_XS+xx;

			if(ampl_in[pix_i] > 2) // reliable distance calculation available
			{

				int64_t dist = dist_in[pix_i]; if(dist < 100) dist = 100;
				int64_t ampl = ampl_in[pix_i]; if(ampl == 255) ampl = 10000;

				int estimate = (int64_t)10000000/( ampl * sq(dist) );

				/*
					Examples how the equation works out:
					ampl=255->10000, dist<=100->100:     0
					ampl=200         dist = 100:         5
					ampl=200         dist = 200:         1
					ampl=50          dist = 100:         20
					ampl=50          dist = 200:         5
					ampl=10          dist = 100:         100 (should really ring a bell)
					ampl=10          dist = 200:         25  (not impossible, but shady)
					ampl=10          dist = 500:         4   (only slighly weird)
					ampl=10          dist = 1000:        1   (completely plausible)
					ampl=3           dist<=100->100:     333 (biggest value)
				*/

				acc += estimate;
				n_acc++;
			}
		}
	}

	if(n_acc>200)
	{
		int amnt = 10*acc/n_acc;
		if(amnt > 65535) amnt = 65535;
		*stray_ampl = amnt;
	}
	else
		*stray_ampl = 0;

	return n_acc;

}

#define SHORTEST_INTEGRATION 125

void epc_test()
{
	int idx = 0;
	int cnt = 0;
	while(1)
	{
		raspi_tx.dbg_id = raspi_rx[4];

		raspi_tx.timestamps[21] = settings.offsets[1];
		raspi_tx.timestamps[22] = settings.offsets[2];
		raspi_tx.timestamps[23] = settings.offsets[3];


		timer_10k = 0;

		raspi_tx.status = 100;

		memset(ignore, 0, sizeof(ignore));

		/*
			6.66 MHz
			LEDS OFF
			2DCS
			Short exp
			Purpose: to find other HF AC sources or other interferences to be ignored
		*/

								raspi_tx.timestamps[0] = timer_10k;
		epc_clk_div(idx, 3);
		while(epc_i2c_is_busy(buses[idx]));

		epc_dis_leds(idx);
		while(epc_i2c_is_busy(buses[idx]));

		epc_2dcs(idx);
		while(epc_i2c_is_busy(buses[idx]));

		epc_intlen(idx, 8, 200);
		while(epc_i2c_is_busy(buses[idx]));
		;

								raspi_tx.timestamps[1] = timer_10k;


		dcmi_start_dma(&dcsa, SIZEOF_2DCS);
		trig(idx);
		LED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();

								raspi_tx.timestamps[2] = timer_10k;


		/*
			20 MHz
			LEDS OFF
			2DCS
			Short exp
			Purpose: Same as the previous, at different freq
		*/
		epc_clk_div(idx, 1);
		while(epc_i2c_is_busy(buses[idx]));
		epc_intlen(idx, 24, 200);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&dcsb, SIZEOF_2DCS);
		trig(idx);
		LED_ON();

								raspi_tx.timestamps[3] = timer_10k;

		// Calculate the previous
		calc_interference_ignore_from_2dcs(ignore, &dcsa, 60);

		if(raspi_rx[4] == 1) memcpy(raspi_tx.dbg, ignore, sizeof(ignore));




								raspi_tx.timestamps[4] = timer_10k;


		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


								raspi_tx.timestamps[5] = timer_10k;



		/*
			20 MHz
			LEDS OFF
			MONOCHROME
			Long exp
			Purpose: General purpose monochrome, HDR long
		*/

		epc_greyscale(idx);
		while(epc_i2c_is_busy(buses[idx]));

		epc_intlen(idx, 120, 1000);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&mono_long, SIZEOF_MONO);

		trig(idx);
		LED_ON();

		// Calculate the previous
		calc_interference_ignore_from_2dcs(ignore, &dcsb, 60);

		if(raspi_rx[4] == 2) memcpy(raspi_tx.dbg, ignore, sizeof(ignore));


		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


								raspi_tx.timestamps[6] = timer_10k;


		/*
			20 MHz
			LEDS OFF
			MONOCHROME
			Short exp
			Purpose: General purpose monochrome, HDR short
		*/

		epc_intlen(idx, 120, 250);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&mono_short, SIZEOF_MONO);

		trig(idx);
		LED_ON();

		// do something useful:

		//process_bw(raspi_tx.ambient, &mono_long);


		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


								raspi_tx.timestamps[7] = timer_10k;



		/*
			6.66 MHz (wrap at 22.5m)
			LEDS ON
			2DCS
			Long exp
			Purpose: Long-distance approximate readings for ignoring too-far points.
		*/

		epc_clk_div(idx, 3);
		while(epc_i2c_is_busy(buses[idx]));

		epc_ena_leds(idx);
		while(epc_i2c_is_busy(buses[idx]));

		epc_2dcs(idx);
		while(epc_i2c_is_busy(buses[idx]));

		epc_intlen(idx, 8, SHORTEST_INTEGRATION*HDR_EXP_MULTIPLIER*HDR_EXP_MULTIPLIER*3/2);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&dcsa, SIZEOF_2DCS);
		trig(idx);
		LED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


								raspi_tx.timestamps[8] = timer_10k;



		/*
			10 MHz (wrap at 15m)
			LEDS ON
			2DCS
			Mid exp
			Purpose: Same as previous
		*/


		epc_clk_div(idx, 2);
		while(epc_i2c_is_busy(buses[idx]));

		epc_intlen(idx, 8, SHORTEST_INTEGRATION*HDR_EXP_MULTIPLIER*HDR_EXP_MULTIPLIER*3/2);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&dcsb, SIZEOF_2DCS);
		trig(idx);
		LED_ON();


								raspi_tx.timestamps[9] = timer_10k;


		// Calculate the previous
		// We don't need fancy HDR combining here, since both exposures simply update the ignore list.
		calc_toofar_ignore_from_2dcs(ignore, &dcsa, 6000, settings.offsets[3], 3);

		if(raspi_rx[4] == 3) memcpy(raspi_tx.dbg, ignore, sizeof(ignore));

								raspi_tx.timestamps[10] = timer_10k;


		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


								raspi_tx.timestamps[11] = timer_10k;




		/*
			20 MHz (wrap at 7.5m)
			LEDS ON
			4DCS
			Short exp
			Purpose: The real thing starts here! Shortest of the three HDR aqcuisition. Shortest first, because
			it'll include the closest objects, and the closest are likely to move most during the
			whole process -> minimize their motion blur, i.e., the ignore list is fairly recent now.
		*/


		static uint8_t  actual_ampl[3*EPC_XS*EPC_YS];
		static uint16_t actual_dist[3*EPC_XS*EPC_YS];


		epc_clk_div(idx, 1);
		while(epc_i2c_is_busy(buses[idx]));

		epc_4dcs(idx);
		while(epc_i2c_is_busy(buses[idx]));

		epc_intlen(idx, 8, SHORTEST_INTEGRATION);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&dcsa, SIZEOF_4DCS);
		trig(idx);
		LED_ON();

		// Calculate the previous
		calc_toofar_ignore_from_2dcs(ignore, &dcsb, 6000, settings.offsets[2], 2);

		if(raspi_rx[4] == 4) memcpy(raspi_tx.dbg, ignore, sizeof(ignore));

		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();

								raspi_tx.timestamps[12] = timer_10k;




		/*
			20 MHz (wrap at 7.5m)
			LEDS ON
			4DCS
			Mid exp
			Purpose: The real thing continues.
		*/

		epc_intlen(idx, 8, SHORTEST_INTEGRATION*HDR_EXP_MULTIPLIER);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&dcsb, SIZEOF_4DCS);
		trig(idx);
		LED_ON();


								raspi_tx.timestamps[13] = timer_10k;


		// Calculate the previous
		tof_calc_dist_ampl(&actual_ampl[0], &actual_dist[0], &dcsa, settings.offsets[1], 1);

		if(raspi_rx[4] == 5) memcpy(raspi_tx.dbg, &actual_ampl[0], 1*160*60);
		if(raspi_rx[4] == 6) memcpy(raspi_tx.dbg, &actual_dist[0], 2*160*60);


								raspi_tx.timestamps[14] = timer_10k;



		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();



								raspi_tx.timestamps[15] = timer_10k;


		raspi_tx.status = 80;


		/*
			20 MHz (wrap at 7.5m)
			LEDS ON
			4DCS
			Long exp
			Purpose: The real thing continues.
		*/

		epc_intlen(idx, 8, SHORTEST_INTEGRATION*HDR_EXP_MULTIPLIER*HDR_EXP_MULTIPLIER);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&dcsa, SIZEOF_4DCS);
		trig(idx);
		LED_ON();

		// Calculate the previous
		tof_calc_dist_ampl(&actual_ampl[1*EPC_XS*EPC_YS], &actual_dist[1*EPC_XS*EPC_YS], &dcsb, settings.offsets[1], 1);

		if(raspi_rx[4] == 7) memcpy(raspi_tx.dbg, &actual_ampl[1*EPC_XS*EPC_YS], 1*160*60);
		if(raspi_rx[4] == 8) memcpy(raspi_tx.dbg, &actual_dist[1*EPC_XS*EPC_YS], 2*160*60);


		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


								raspi_tx.timestamps[16] = timer_10k;


		// All captures done.

		// Calculate the last measurement:
		tof_calc_dist_ampl(&actual_ampl[2*EPC_XS*EPC_YS], &actual_dist[2*EPC_XS*EPC_YS], &dcsa, settings.offsets[1], 1);
		if(raspi_rx[4] == 9)  memcpy(raspi_tx.dbg, &actual_ampl[2*EPC_XS*EPC_YS], 1*160*60);
		if(raspi_rx[4] == 10) memcpy(raspi_tx.dbg, &actual_dist[2*EPC_XS*EPC_YS], 2*160*60);


								raspi_tx.timestamps[17] = timer_10k;

		uint16_t stray_ampl, stray_dist, outstray_n, outstray_ampl;

		calc_stray_estimate(&actual_ampl[0*EPC_XS*EPC_YS], &actual_dist[0*EPC_XS*EPC_YS], &stray_ampl, &stray_dist);
		outstray_n = calc_outside_stray_estimate(&actual_ampl[2*EPC_XS*EPC_YS], &actual_dist[2*EPC_XS*EPC_YS], &outstray_ampl);

		raspi_tx.dbg_i32[0] = stray_ampl;
		raspi_tx.dbg_i32[1] = stray_dist;

		raspi_tx.dbg_i32[2] = outstray_ampl;
		raspi_tx.dbg_i32[3] = outstray_n;
								raspi_tx.timestamps[18] = timer_10k;


		// Then combine everything:
		

		raspi_tx.status = 50;

//		if(cnt&1)
		{
			uint16_t combined_stray_ampl, combined_stray_dist;
			if(outstray_ampl > stray_ampl)
			{
				combined_stray_ampl = outstray_ampl;
				combined_stray_dist = 100;
			}
			else
			{
				combined_stray_ampl = stray_ampl;
				combined_stray_dist = stray_dist;
			}
			tof_calc_dist_3hdr_with_ignore_with_straycomp(raspi_tx.depth, actual_ampl, actual_dist, ignore, combined_stray_ampl, combined_stray_dist);

			if(outstray_ampl > stray_ampl)
				raspi_tx.depth[0] = 1;
			else
				raspi_tx.depth[0] = 3500;
			raspi_tx.depth[1] = 2000;
			raspi_tx.depth[2] = 1;
		}
//		else
			tof_calc_dist_3hdr_with_ignore(raspi_tx.uncorrected_depth, actual_ampl, actual_dist, ignore);


								raspi_tx.timestamps[19] = timer_10k;
		raspi_tx.status = 255;

		while(timer_10k < 7500)
		{
			if(new_rx)
			{
				if(*((volatile uint32_t*)&raspi_rx[0]) == 0xca0ff5e7) // offset calibration cmd
				{
					*((volatile uint32_t*)&raspi_rx[0]) = 0; // zero it out so that we don't do it again
					__DSB(); __ISB();
					int ret = run_offset_cal();
					raspi_tx.dbg_i32[7] = ret;
				}

				new_rx = 0;
				if(new_rx_len > 8)
					raspi_tx.status = 150;
			}
		}

		cnt++;
	}


}

#define NUM_ADC_DATA 1

typedef struct __attribute__((packed))
{
	uint16_t vref;
	uint16_t tcpu;
} adc_datum_t;

volatile adc_datum_t adc_data[NUM_ADC_DATA];

volatile int spi_test_cnt;

/*
	STM32 SPI doesn't _actually_ support any kind of nSS pin hardware management in slave mode, even when they are talking about it like that.
	The hint is that while they talk about nSS slave HW management, they don't tell you what it means in _practice_.

	So, when the SPI is enabled with DMA, it always generates 3 DMA requests right away to fill its TX fifo with 3 bytes. When the
	nSS is asserted half a year later, the first 3 bytes are from half a year ago, rest is DMA'ed from the memory at that point.

	To fix this, we would need interrupt logic also from the falling nSS edge, quickly setting up the SPI and DMA. But maybe we don't need to
	do that:

	The type of the data packet we are going to send has been decided even before the nSS edge - so we'll apply a 4-byte header telling
	about the data type (or anything else that doesn't need to be recent).

	This has the advantage that there is one urgent ISR less to handle very quickly. DMA and SPI will happily crunch 4 extra bytes without
	wasting any CPU time. The same works on the RASPI side - we don't even need to look at those 4 bytes if we don't have time to do that.
*/

void raspi_spi_xfer_end_inthandler()
{
	// Triggered when cs goes high - switch DMA rx buffers, zero the tx buffer loc

	EXTI->PR = 1UL<<12; // Clear "pending bit".

	spi_test_cnt++;

	// Disable the DMAs:
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	while(DMA1_Stream4->CR & 1UL) ;
	while(DMA1_Stream3->CR & 1UL) ;

	new_rx = 1;
	new_rx_len = sizeof(raspi_rx) - DMA1_Stream3->NDTR;


	// Check if there is the maintenance magic code:

	if(*((volatile uint32_t*)&raspi_rx[0]) == 0x9876fedb)
	{
		EPC01_RSTN_LOW();
		EPC23_RSTN_LOW();
		EPCNEG10V_OFF();
		EPC10V_OFF();
		EPC5V_OFF();
		EPCLEDV_OFF();
		EPC_DESEL();
		run_flasher();
	}

	// Hard-reset SPI - the only way to empty TXFIFO! (Go figure.)

	RCC->APB1RSTR = 1UL<<14;
	__asm__ __volatile__ ("nop");
	RCC->APB1RSTR = 0;
			
	// Re-enable:

	SPI2->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/ | 1UL<<12 /*Don't Reject The Last Byte*/;

	// TX DMA
	DMA1_Stream4->NDTR = sizeof(raspi_tx);
	DMA_CLEAR_INTFLAGS(DMA1, 4);
	DMA1_Stream4->CR |= 1; // Enable DMA

	// RX DMA

	DMA1_Stream3->NDTR = sizeof(raspi_rx);
	DMA_CLEAR_INTFLAGS(DMA1, 3);
	DMA1_Stream3->CR |= 1; // Enable DMA

	SPI2->CR2 |= 1UL<<1 /*TX DMA ena*/; // not earlier!

	SPI2->CR1 = 1UL<<6; // Enable in slave mode

}

/*
	Between the RobotBoard and the PULUTOF1 MASTER,
	there's a continuous SPI transfer going on. With fixed length and no gaps,
	it can fully run on DMA.

	PULUTOF1 MASTER is the SPI MASTER, RobotBoard is slave.
*/

typedef struct __attribute__((packed))
{
	uint8_t dummy[12];
} master_to_robot_t;

typedef struct __attribute__((packed))
{
	pos_t robot_pos;
} robot_to_master_t;


void main()
{

	/*
	For 216MHz operation, "overdrive" must be on, device must have -6 suffix, and max Ta is 80 degC.
	Entering the overdrive requires a specific sequence, explained in the refman on page 123.

	XTAL = HSE = 8 MHz
	PLLCLK = SYSCLK = 216 MHz (max)
	AHB = HCLK = 216 MHz (max) --> AHB prescaler = 1
	APB2 = high-speed APB = 108 MHz (108 MHz max) --> APB2 prescaler = 2
	APB1 = PCLK1 = low-speed APB  = 54 MHz (54 MHz max) --> APB1 prescaler = 4
	APB2 timers x2 = 216 MHz
	APB1 timers x2 = 108 MHz

	PLL CONFIG:
	Input: 8 MHz
	M (for PLL input)   =  4 -> 2 MHz (must be between 1..2MHz)
	N (PLL multiplier)  = 216 -> 432 MHz
	P (for main system) = 2  -> 216 MHz
	Q (for USB) = 9 -> 48 MHZ
	*/

	RCC->AHB1ENR = 0xff /*GPIOA to H*/ | 1UL<<21 /*DMA1*/ | 1UL<<22 /*DMA2*/ | 1UL<<20 /*DTCM RAM*/;
	IO_TO_GPO(GPIOF, 2);

	LED_ON();
	delay_ms(5); // longer actually
	LED_OFF();


	RCC->APB1ENR |= 1UL<<28; // Power interface clock enable - needed to configure PWR registers
	RCC->PLLCFGR = 9UL<<24 /*Q*/ | 1UL<<22 /*HSE as source*/ | 0b00UL<<16 /*P=2*/ | 216UL<<6 /*N*/ | 4UL /*M*/;

	RCC->CR |= 1UL<<16; // HSE clock on
	RCC->CR |= 1UL<<24; // PLL on

	PWR->CR1 |= 1UL<<16; // Overdrive on
	while(!(PWR->CSR1 & (1UL<<16))) ; // Wait for overdrive ready
	PWR->CR1 |= 1UL<<17; // Switch to overdrive
	while(!(PWR->CSR1 & (1UL<<17))) ; // Wait for overdrive switching ready


	FLASH->ACR = 1UL<<9 /* ART accelerator enable (caches) */ | 1UL<<8 /*prefetch enable*/ | 7UL /*7 wait states*/;
	RCC->CFGR = 0b100UL<<13 /*APB2 div 2*/ | 0b101UL<<10 /*APB1 div 4*/;
	RCC->DCKCFGR2 = 0b01UL<<4 /*USART3 = sysclk*/ | 0b00UL<<22 /*I2C4 = APB1clk*/ | 0b00UL<<20 /*I2C3 = APB1clk*/;

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL
	RCC->CFGR |= 0b10; // Change PLL to system clock
	while((RCC->CFGR & (0b11UL<<2)) != (0b10UL<<2)) ; // Wait for switchover to PLL.

	// Enable FPU

	SCB->CPACR |= 0b1111UL<<20;
	__DSB();
	__ISB();

	RCC->AHB2ENR = 1UL<<0 /*DCMI*/;
	RCC->APB1ENR |= 1UL<<18 /*USART3*/ | 1UL<<24 /*I2C4*/ | 1UL<<23 /*I2C3*/ | 1UL<<3 /*TIM5*/ | 1UL<<14 /*SPI2*/;
	RCC->APB2ENR = 1UL<<14 /*SYSCFG*/ | 1UL<<8 /*ADC1*/ | 1UL<<20 /*SPI5*/;

	IO_TO_GPO(GPIOA,14);
	IO_TO_GPO(GPIOE, 2);
	IO_TO_GPO(GPIOE, 3);
	IO_TO_GPO(GPIOD, 9);
	IO_TO_GPO(GPIOD, 8);
	IO_TO_GPO(GPIOE, 4);

	// Before configuring the buffer output enable pins as outputs,
	// the pins must have inactive levels (high):
	EPC_DESEL();
	IO_TO_GPO(GPIOA, 15);
	IO_TO_GPO(GPIOC, 10);
	IO_TO_GPO(GPIOE, 0);
	IO_TO_GPO(GPIOE, 1);


	LED_ON();
	delay_ms(50);
	LED_OFF();
	delay_ms(200);
	LED_ON();
	delay_ms(50);
	LED_OFF();

	/*
		USART3 = the raspi USART @ APB1 = 54 MHz
		fck=216MHz
	*/


#ifdef USE_UART
	IO_TO_ALTFUNC(GPIOB, 10);
	IO_TO_ALTFUNC(GPIOB, 11);
	IO_SET_ALTFUNC(GPIOB, 10, 7);
	IO_SET_ALTFUNC(GPIOB, 11, 7);
	USART3->BRR = 1875; // 115200
//	USART3->CR2 = 0b10UL<<12 /* 2 stop bits*/;
//	USART3->CR3 = 1UL<<6 /*RX DMA*/;

	USART3->CR1 = 0UL<<15 /*Oversamp:16*/ | 0UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/ |  1UL /*USART ENA*/;
#endif

	/*	
		ADC  @ APB2 (108 MHz)
		ADC clock max 36MHz ("typ: 30 MHz")
		prescaler = 4 -> 27MHz

		Measured Vrefint values at:
		3.3V -> 1505
		3.0V -> 1656
		2.7V -> 1840
	*/
	ADC->CCR = 1UL<<23 /*temp sensor & Vref ena*/ | 0b01UL<<16 /*prescaler=4*/;

	ADC1->CR1 = 1UL<<23 /*AWD ena*/ | 1UL<<9 /*AWD only on a single channel*/ | 1UL<<8 /*scan mode*/ | 1UL<<6 /*AWD interrupt*/ | 10UL /*AWD channel*/;
	ADC1->CR1 = 1UL<<8 /*scan mode*/;
	ADC1->CR2 = 1UL<<8 /*DMA mode*/ | 1UL<<9 /* The magical DDS bit to actually make DMA work*/ | 1UL<<1 /*continuous conversion*/;

	ADC1->SQR1 = (/*sequence length:*/ 2   -1)<<20;
	ADC1->SQR3 = 
		17UL<<0 /*1st conversion: Vrefint*/ |
		18UL<<5 /*2nd conversion: Vtempsense*/;

	ADC1->SMPR1 = 0b111UL<<24 /*Ch18:tempsense*/ | 0b110UL<<21 /*Ch17: Vrefint*/;

	DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);
	DMA2_Stream0->M0AR = (uint32_t)&(adc_data[0]);
	DMA2_Stream0->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 1UL<<8 /*circular*/ |
			   0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;
	DMA2_Stream0->NDTR = sizeof(adc_datum_t)*NUM_ADC_DATA/2; /*/2 for 16-bit transfers*/
	DMA_CLEAR_INTFLAGS(DMA2, 0);
	DMA2_Stream0->CR |= 1; // Enable DMA
	ADC1->CR2 |= 1UL; // Enable ADC // OPT_TODO: try combining with the previous
	ADC1->CR2 |= 1UL<<30; // Start conversion   OPT_TODO: try removing this


	PWR->CR1 |= 0b111UL<<5 /*Power Voltage Detector level: 2.9V*/ | 1UL<<4 /*PVD on*/;

	EXTI->IMR |= 1UL<<16; // enable interurpt mask for PVD
	// Either rising or falling PVD edge generates the interrupt:
	EXTI->FTSR |= 1UL<<16;
	EXTI->RTSR |= 1UL<<16;

	/*
		Interrupts will have 4 levels of pre-emptive priority, and 4 levels of sub-priority.

		Interrupt with more urgent (lower number) pre-emptive priority will interrupt another interrupt running.

		If interrupts with similar or lower urgency happen during another ISR, the subpriority level will decide
		the order they will be run after finishing the more urgent ISR first.
	*/
	NVIC_SetPriorityGrouping(2);
	NVIC_SetPriority(PVD_IRQn, 0b0000);
	NVIC_EnableIRQ(PVD_IRQn);


	/*
		TIM5 @ APB1, the counter runs at 108MHz
		Create 10 kHz timebase interrupt
	*/

	TIM5->DIER |= 1UL; // Update interrupt
	TIM5->ARR = 10799; // 108MHz -> 10 kHz
	TIM5->CR1 |= 1UL; // Enable

	NVIC_SetPriority(TIM5_IRQn, 0b1010);
	NVIC_EnableIRQ(TIM5_IRQn);

	LED_OFF();

/*
	delay_ms(10);
	EPC10V_ON();
	EPC5V_ON();
	delay_ms(10);
	EPCNEG10V_ON();
	delay_ms(100);
	EPC01_RSTN_HIGH();
	EPC23_RSTN_HIGH();
	delay_ms(100);
*/

	// Raspi SPI:

	IO_ALTFUNC(GPIOB, 12, 5);
	IO_ALTFUNC(GPIOB, 13, 5);
	IO_ALTFUNC(GPIOB, 14, 5);
	IO_ALTFUNC(GPIOB, 15, 5);

	IO_SPEED(GPIOB, 14, 3); // MISO pin gets the highest speed.


	// Initialization order from reference manual:

	/*
		STM32 TRAP WARNING:
		FRXTH (bit 12) in SPI->CR2 needs to be set for the SPI to work in a sane way. Otherwise, DMA
		requests for the last transfer never come, and the last data cannot be never read in the specified way!
		I can't figure out any reason to use such mode, and it's the default. So remember to set this bit.
	*/

	init_raspi_tx();

//	uint8_t c = 0;
//	for(int i=0; i<160*60; i++)
//		raspi_tx.ambient[i] = c++;

	__DSB(); __ISB();
	SPI2->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/ | 1UL<<12 /*Don't Reject The Last Byte*/;

	// TX DMA
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = (uint32_t)&raspi_tx;
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream4->NDTR = sizeof(raspi_tx);
	DMA_CLEAR_INTFLAGS(DMA1, 4);
	DMA1_Stream4->CR |= 1; // Enable TX DMA

	// RX DMA

	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream3->M0AR = (uint32_t)(raspi_rx);
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;
	DMA1_Stream3->NDTR = sizeof(raspi_rx);
	DMA_CLEAR_INTFLAGS(DMA1, 3);
	DMA1_Stream3->CR |= 1; // Enable RX DMA

	SPI2->CR2 |= 1UL<<1 /*TX DMA ena*/; // not earlier!

	SPI2->CR1 = 1UL<<6; // Enable in slave mode

	// Chip select is hardware managed for rx start - but ending must be handled by software. We use EXTI for that.

	// nCS is PB12, so EXTI12 must be used.
	SYSCFG->EXTICR[/*refman idx:*/4   -1] = 0b0001UL<<0; // PORTB used for EXTI12.
	IO_PULLUP_ON(GPIOB, 12); // Pull the nCS up to avoid glitches before Raspi initialization
	EXTI->IMR |= 1UL<<12;
	EXTI->RTSR |= 1UL<<12; // Get the rising edge interrupt
	// The interrupt priority must be fairly high, to quickly reconfigure the DMA so that if the master pulls CSn low
	// again very quickly to make a new transaction, we have the DMA up and running for that before the super small 4-byte
	// FIFO in the SPI is exhausted.
	NVIC_SetPriority(EXTI15_10_IRQn, 0b0101);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	/*
		MOSI = green
		MISO = blue
		SCK  = violet
		CS0  = grey
	*/

	top_init();
	epc_test();
}
