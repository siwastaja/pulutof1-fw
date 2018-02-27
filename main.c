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



#define EPC02_ADDR 0b0100001
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

void uart_print_string_blocking(const char *buf)
{
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


void uart_send_blocking(const uint8_t *buf, int len)
{
	while(len--)
	{
		while((USART3->ISR & (1UL<<7)) == 0) ;
		USART3->TDR = buf[0];
		buf++;
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

void uart_send_blocking_crc(const uint8_t *buf, uint8_t id, int len)
{
	uint8_t chk = CRC_INITIAL_REMAINDER;

	while((USART3->ISR & (1UL<<7)) == 0) ;
	USART3->TDR = id;
	while((USART3->ISR & (1UL<<7)) == 0) ;
	USART3->TDR = len & 0xff;
	while((USART3->ISR & (1UL<<7)) == 0) ;
	USART3->TDR = (len & 0xff00)>>8;

	while(len--)
	{
		chk ^= buf[0];
		CALC_CRC(chk);
		while((USART3->ISR & (1UL<<7)) == 0) ;
		USART3->TDR = buf[0];
		buf++;
	}
	while((USART3->ISR & (1UL<<7)) == 0) ;
	USART3->TDR = chk;
}


// Things written to I2C CR1 every time:
#define I2C_CR1_BASICS (0UL<<8 /*Digital filter len 0 to 15*/)
#define I2C_CR1_BASICS_ON (I2C_CR1_BASICS | 1UL /*keep it on*/)

volatile int epc_i2c_write_busy, epc_i2c_read_busy;

volatile int epc_i2c_read_state;
uint8_t epc_i2c_read_slave_addr;
volatile uint8_t *epc_i2c_read_buf;
uint8_t epc_i2c_read_len;

void epc_i2c_write_dma(uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	/*
		Was chasing for a really strange data coherency issue for two days: sometimes, buf[0] is randomly replaced
		by a former value. Input buf is in non-cached section (core-coupled ram), but it still happens!

		I don't know what the hell is going on, but for some reason I can't understand, invalidation of data cache here
		seemed to fix the issue.
	*/
	__DSB(); __ISB();

	if(epc_i2c_write_busy || epc_i2c_read_busy)
		error(11);

	if(DMA1_Stream0->CR & 1UL)
		error(12);

	epc_i2c_write_busy = 1;
	I2C3->ICR = 1UL<<5; // Clear any pending STOPF interrupt 

	if(len > 1) // Actually use DMA
	{		
		DMA1_Stream0->CR = 8UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
				   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

		DMA1_Stream0->NDTR = len;
		DMA1_Stream0->M0AR = (uint32_t)buf;

		DMA_CLEAR_INTFLAGS(DMA1, 0);
		DMA1_Stream0->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

		I2C3->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<14 /*TX DMA*/;

	}
	else	// Just do the single write right now.
	{
		I2C3->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/; // no DMA
		I2C3->TXDR = buf[0];
	}

	I2C3->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
#if 0

	I2C3->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/; // no DMA
	I2C3->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
	for(int i=0; i<len; i++)
	{
		while(!(I2C3->ISR & (1UL<<1)));
		I2C3->TXDR = buf[i];
	}
#endif


}

#define epc_i2c_write epc_i2c_write_dma

void epc_i2c_read_dma(uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	DMA1_Stream2->CR = 3UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /*transfer complete interrupt*/;

	DMA1_Stream2->NDTR = len;
	DMA1_Stream2->M0AR = (uint32_t)buf;

	I2C3->CR1 = I2C_CR1_BASICS_ON | 0UL<<5 /*OFF FOR READ: STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<15 /*RX DMA*/ ;

	I2C3->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 1UL<<10 /*read*/ | slave_addr_7b<<1;

	DMA_CLEAR_INTFLAGS(DMA1, 2);
	DMA1_Stream2->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
}

/*
	In I2C, a million things can go wrong. STM32 I2C implementation is notoriously buggy, device implementations often are, too.
	It makes little sense to try to detect every possible error condition, since they most often end up in total bus lockup
	anyway, and can only be reliably solved by device&bus reset. Since this is time-consuming anyway, and always leads to loss
	of data and hangup in execution time, we can as well solve all error checking using a simple watchdog that does the same,
	with much less code footprint (good for reliability, execution speed, flash usage, and code maintainability) compared to trying
	to catch all unexpected things in actual interrupt services or API functions.
*/

void epc_i2c_read(uint8_t slave_addr_7b, uint8_t reg_addr, volatile uint8_t *buf, uint8_t len)
{
	if(epc_i2c_write_busy || epc_i2c_read_busy)
		error(12);
	/*
		Full I2C read cycle consists of a write cycle (payload = reg_addr), without stop condition,
		then the actual read cycle starting with the repeated start condition.

		Since the write is always one byte of payload, we run it without DMA.

		After the write is complete, there is no AUTOEND generated, instead we get an interrupt, allowing
		us to go on with the read.
	*/
	epc_i2c_read_busy = 1;

	I2C3->CR1 = I2C_CR1_BASICS_ON | 1UL<<6 /*Transfer Complete interrupt - happens because AUTOEND=0 and RELOAD=0*/; // no DMA
	I2C3->CR2 = 0UL<<25 /*AUTOEND off*/ | 1UL<<16 /*len=1*/ | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C3->TXDR = reg_addr;
	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
	epc_i2c_read_state = 1;
	epc_i2c_read_slave_addr = slave_addr_7b;
	epc_i2c_read_buf = buf;
	epc_i2c_read_len = len;

}

// Returns true whenever read or write operation is going on.
int epc_i2c_is_busy()
{
	return epc_i2c_write_busy || epc_i2c_read_busy;
}


void epc_rx_dma_inthandler() // read complete.
{
	DMA_CLEAR_INTFLAGS(DMA1, 2);
	epc_i2c_read_busy = 0;
}

/*
	For some reason, I can't get repeated start (Sr) condition out of the new STM32 I2C - it generates a stop-start sequence even when only asked for a START.
	Luckily, the EPC chip still works correctly even though this violates the spec.
*/
void epc_i2c_inthandler()
{
	I2C3->ICR = 1UL<<5; // Clear the STOPF flag.
	if(epc_i2c_read_state)
	{
		// Writing START to 1 (in epc_i2c_read_dma()) clears the interrupt flag
		epc_i2c_read_dma(epc_i2c_read_slave_addr, epc_i2c_read_buf, epc_i2c_read_len);
		epc_i2c_read_state=0;

	}
	else // STOPF interrupt - this was a write.
	{
		if(I2C3->ISR & (1UL<<15)) // busy shouldn't be high - at least fail instead of doing random shit
		{
			error(22);
		}

		// Write is now finished.
		epc_i2c_write_busy = 0;
		I2C3->CR1 = I2C_CR1_BASICS_ON;
	}
}


void epc_i2c_init()
{
	// DMA1 Stream0 = tx
	// DMA1 Stream2 = rx
	// For now, init in Fast mode 400 kHz
	// Use the datasheet example of 48MHz input clock, with prescaler 1.125 times bigger for 54 MHz. Sync delay calculations are near enough.
	// Errata: tSU;DAT (do they mean SDADEL register??) must be at least one I2CCLK period
	// Errata: bus errors are spuriously generated for no reason, the bug is not said to affect the transfer: just ignore BERR.
 
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


epc_img_t img_mono;

epc_img_t img_compmono;

//epc_dcs_t img_dcs[2];

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

	//DMA2_Stream7->NDTR = sizeof(epc_frame_t)/4;

	//DMA_CLEAR_INTFLAGS(DMA2, 7);
	//DMA2_Stream7->CR |= 1; // Enable DMA

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

	DMA2_Stream7->NDTR = size/4+1; // Num of 32-bit transfers
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	DMA_CLEAR_INTFLAGS(DMA2, 7);
	DMA2_Stream7->CR |= 1; // Enable DMA
	__DSB(); __ISB();
}


typedef struct __attribute__((packed))
{
	// *4, then -1 to form the register values:
	uint16_t bw_int_len; 
	uint16_t dist_int_len;

	uint8_t clk_div; // 1 = 20MHz LED, 2 = 10 MHz...
	uint8_t pll_shift; // delay in 12.5 ns/step , 0-12
	uint8_t dll_shift; // delay in approx 2ns/step, 0-49

	int16_t offsets[7];

} config_t;

config_t config =  // active configuration
{
	.bw_int_len = 500,
	.dist_int_len = 200,
	.clk_div = 1,
	.pll_shift = 0,
	.dll_shift = 0,
	.offsets = {0,0,0,0,0,0,0}
};


void apply_config(config_t *conf)
{
	if(conf->clk_div < 1 || conf->clk_div > 6 || conf->clk_div == 5)
	{
		error(55);
	}

	if(conf->bw_int_len*4-1 > 65535 || conf->dist_int_len*4-1 > 65535)
	{
		error(55);
	} 

	if(conf->pll_shift > 12 || conf->dll_shift > 49)
	{
		error(55);
	} 

	memcpy(&config, conf, sizeof(config_t));
}


void trig()
{
	static volatile uint8_t b[2] = {0xa4, 1};
	epc_i2c_write(EPC02_ADDR, b, 2);
	while(epc_i2c_is_busy());
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
		uint16_t dist = 0;
		uint8_t ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
		{
			//dist = 65535;
			ampl = 0;
		}
		else if((in->dcs[0].img[i]&1) || (in->dcs[1].img[i]&1) ||
		   (in->dcs[2].img[i]&1) || (in->dcs[3].img[i]&1))
		{
			//dist = 65535;
			ampl = 0;
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
				//dist = 65534;
				ampl = 1;
			}
			else
			{

				int idx = (dcs31_mod*(TOF_TBL_LEN-1))/dcs20_mod;

				int32_t dist_i = tof_tbl[idx];
				if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
				if(dcs20<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
				if(dcs31<0) dist_i = -dist_i;

				dist_i += offset;

				if(dist_i < 0) dist_i += TOF_TBL_PERIOD;
				else if(dist_i > TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;
				
				dist_i *= clk_div;

				ampl = 1+sqrt(sq(dcs20)+sq(dcs31))/23;

			//	if(ampl < sq(75)*2)
			//		dist = 65534;
			//	else
					dist = dist_i;

			}

		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}


/*
Process eight DCS images, with offset_mm in millimeters. With clk_div=1, does the calculation at fled=20MHz (unamb range = 7.5m). Larger clk_div
multiplies the result.
*/

#define HDR_SWITCHOVER 1600
void tof_calc_dist_hdr(uint16_t *dist_out, epc_4dcs_t *in_s, epc_4dcs_t *in_l, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int i=0; i < 160*60; i++)
	{
		uint16_t dist = 0;

		int16_t s_dcs0 = ((in_s->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t s_dcs1 = ((in_s->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t s_dcs2 = ((in_s->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t s_dcs3 = ((in_s->dcs[3].img[i]&0b0011111111111100)>>2)-2048;
		int16_t l_dcs0 = ((in_l->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t l_dcs1 = ((in_l->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t l_dcs2 = ((in_l->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t l_dcs3 = ((in_l->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(s_dcs0 < -2047 || s_dcs0 > 2046 || s_dcs1 < -2047 || s_dcs1 > 2046 || s_dcs2 < -2047 || s_dcs2 > 2046 || s_dcs3 < -2047 || s_dcs3 > 2046 || 
		  (in_s->dcs[0].img[i]&1) || (in_s->dcs[1].img[i]&1) || (in_s->dcs[2].img[i]&1) || (in_s->dcs[3].img[i]&1))
		{
			dist = 65535;
		}
		else
		{
			int16_t dcs31, dcs20;
			if(l_dcs0 < -HDR_SWITCHOVER || l_dcs0 > HDR_SWITCHOVER || l_dcs1 < -HDR_SWITCHOVER || l_dcs1 > HDR_SWITCHOVER || 
			   l_dcs2 < -HDR_SWITCHOVER || l_dcs2 > HDR_SWITCHOVER || l_dcs3 < -HDR_SWITCHOVER || l_dcs3 > HDR_SWITCHOVER ||
			   (in_l->dcs[0].img[i]&1) || (in_l->dcs[1].img[i]&1) || (in_l->dcs[2].img[i]&1) || (in_l->dcs[3].img[i]&1))
			{
				dcs31 = s_dcs3-s_dcs1;
				dcs20 = s_dcs2-s_dcs0;
			}
			else
			{
				dcs31 = l_dcs3-l_dcs1;
				dcs20 = l_dcs2-l_dcs0;
			}

			// Use the lookup table to perform atan2:

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

			if(dcs20_mod == 0 || sq(dcs20)+sq(dcs31) < sq(75)*2)
			{
				dist = 65534;
			}
			else
			{
				int idx = (dcs31_mod*(TOF_TBL_LEN-1))/dcs20_mod;

				int32_t dist_i = tof_tbl[idx];
				if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
				if(dcs20<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
				if(dcs31<0) dist_i = -dist_i;

				dist_i += offset;

				if(dist_i < 0) dist_i += TOF_TBL_PERIOD;
				else if(dist_i > TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;
				
				dist = dist_i*clk_div;
			}

		}
		dist_out[i] = dist;
	}
}

typedef struct __attribute__((packed))
{
	uint8_t bw[EPC_XS*EPC_YS];
	uint16_t dist[EPC_XS*EPC_YS];
	uint8_t ampl[EPC_XS*EPC_YS];
	uint16_t calc_time; // 0.1ms units
} spi_tof_frame_t;

volatile spi_tof_frame_t spitof;

#define SIZEOF_MONO (EPC_XS*EPC_YS*2)
#define SIZEOF_2DCS (EPC_XS*EPC_YS*2*2)
#define SIZEOF_4DCS (EPC_XS*EPC_YS*2*4)


void epc_clk_div(int div)
{
	epc_wrbuf[0] = 0x85;
	epc_wrbuf[1] = div-1;
	epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
}

void epc_dis_leds()
{
	epc_wrbuf[0] = 0x90;
	epc_wrbuf[1] = 0b11001000; // leds disabled
	epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
}

void epc_ena_leds()
{
	epc_wrbuf[0] = 0x90;
	epc_wrbuf[1] = 0b11101000; // LED2 output on
	epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
}

void epc_greyscale()
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b11000100; // greyscale modulation
	epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
}

void epc_2dcs()
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00010100; // 2dcs modulation
	epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
}

void epc_4dcs()
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00110100; // 4dcs modulation
	epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
}

void epc_intlen(uint8_t multiplier, uint16_t time)
{
	int intlen = ((int)time<<2)-1;
	epc_wrbuf[0] = 0xA1;
	epc_wrbuf[1] = multiplier;
	epc_wrbuf[2] = (intlen&0xff00)>>8;
	epc_wrbuf[3] = intlen&0xff;

	epc_i2c_write(EPC02_ADDR, epc_wrbuf, 4);
}

void calc_toofar_ignore_from_2dcs(uint8_t *ignore_out, epc_4dcs_t *in, int threshold /*mm*/, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int yy = 0; yy < EPC_YS; yy++)
	{
		for(int xx = 0; xx < EPC_XS; xx++)
		{
			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

			if((in->dcs[0].img[i]&1) || (in->dcs[1].img[i]&1) || dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
			{
				// Overexp - it's ok, probably not too far.
			}
			else
			{
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

				int32_t dist_i;
				int ampl_low = 0;
				if(sqrt(sq(dcs0)+sq(dcs1)) < 100 || dcs0_mod == 0 /* should always be true if the first one is: todo: prove*/)
				{
					// amplitude too low
					ampl_low = 1;
				}
				else
				{
					int idx = (dcs1_mod*(TOF_TBL_LEN-1))/dcs0_mod;

					dist_i = tof_tbl[idx];
					if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
					if(dcs0<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
					if(dcs1<0) dist_i = -dist_i;

					dist_i += offset;

					if(dist_i < 0) dist_i += TOF_TBL_PERIOD;
					else if(dist_i > TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;
				
					dist_i *= clk_div;

				}

				if(ampl_low || dist_i > threshold)
				{
					//  ooo
					//  oxo
					//  ooo
					out[yy*EPC_XS+xx] = 1;
					if(xx > 0)
					{
						//  ooo
						//  xoo
						//  ooo
						out[yy*EPC_XS+xx-1] = 1;
						//  xoo
						//  ooo
						//  ooo
						if(yy > 0) out[(yy-1)*EPC_XS+xx-1] = 1;
						//  ooo
						//  ooo
						//  xoo
						if(yy < EPC_YS-1) out[(yy+1)*EPC_XS+xx-1] = 1;
					}
					if(xx < EPC_XS-1)
					{
						//  ooo
						//  oox
						//  ooo
						out[yy*EPC_XS+xx+1] = 1;
						//  oox
						//  ooo
						//  ooo
						if(yy > 0) out[(yy-1)*EPC_XS+xx+1] = 1;
						//  ooo
						//  ooo
						//  oox
						if(yy < EPC_YS-1) out[(yy+1)*EPC_XS+xx+1] = 1;
					}

					//  oxo
					//  ooo
					//  ooo
					if(yy > 0) out[(yy-1)*EPC_XS+xx] = 1;

					//  ooo
					//  ooo
					//  oxo
					if(yy < EPC_YS-1) out[(yy+1)*EPC_XS+xx] = 1;
				}

			}



		}


	}
}







void calc_interference_ignore_from_2dcs(uint8_t *out, epc_4dcs_t *in, int threshold)
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
				//  ooo
				//  oxo
				//  ooo
				out[yy*EPC_XS+xx] = 1;
				if(xx > 0)
				{
					//  ooo
					//  xoo
					//  ooo
					out[yy*EPC_XS+xx-1] = 1;
					//  xoo
					//  ooo
					//  ooo
					if(yy > 0) out[(yy-1)*EPC_XS+xx-1] = 1;
					//  ooo
					//  ooo
					//  xoo
					if(yy < EPC_YS-1) out[(yy+1)*EPC_XS+xx-1] = 1;
				}
				if(xx < EPC_XS-1)
				{
					//  ooo
					//  oox
					//  ooo
					out[yy*EPC_XS+xx+1] = 1;
					//  oox
					//  ooo
					//  ooo
					if(yy > 0) out[(yy-1)*EPC_XS+xx+1] = 1;
					//  ooo
					//  ooo
					//  oox
					if(yy < EPC_YS-1) out[(yy+1)*EPC_XS+xx+1] = 1;
				}

				//  oxo
				//  ooo
				//  ooo
				if(yy > 0) out[(yy-1)*EPC_XS+xx] = 1;

				//  ooo
				//  ooo
				//  oxo
				if(yy < EPC_YS-1) out[(yy+1)*EPC_XS+xx] = 1;
			}
		}
	}
}


void epc_test()
{
	char printbuf[64];

	delay_ms(300);
	EPC10V_ON();
	EPC5V_ON();
	delay_ms(100);
	EPCNEG10V_ON();
	delay_ms(100);
	EPC01_RSTN_HIGH();
	delay_ms(300);


	epc_i2c_init();

#if 0
	{
		epc_wrbuf[0] = 0x89; 
		epc_wrbuf[1] = (6 /*TCMI clock div 2..16*/ -1) | 0<<7 /*add clock delay*/;
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}
#endif

	{
		epc_wrbuf[0] = 0xcb; // i2c&tcmi control
		epc_wrbuf[1] = 0b01101111; // saturation bit, split mode, gated dclk, ESM
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}

	{
		epc_wrbuf[0] = 0x90; // led driver control
		epc_wrbuf[1] = 0b11001000;
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}

	{
		epc_wrbuf[0] = 0x92; // modulation select
		epc_wrbuf[1] = 0b11000100; // grayscale
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}

/*
	{
		epc_wrbuf[0] = 0xcc; // tcmi polarity settings
		epc_wrbuf[1] = 1<<7; //saturate data;
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}
*/

	EPCLEDV_ON();

	delay_ms(100);

	EPC_SEL0();

	epc_dcmi_init();

	// Take a dummy frame, which will eventually output the end-of-frame sync marker, to get the DCMI sync marker parser in the right state
	{
		dcmi_start_dma(&img_mono, SIZEOF_MONO);
		trig();
		delay_ms(100);
	}

	while(1)
	{
		while(!new_rx)
		{
			LED_ON();
			delay_us(1);
			LED_OFF();
			delay_us(20);
		}
		new_rx = 0;

		static uint8_t ignore[EPC_XS*EPC_YS];
		static epc_4dcs_t dcsa, dcsb;

		memset(ignore, 0, sizeof(ignore));

		/*
			6.66 MHz
			LEDS OFF
			2DCS
			Short exp
			Purpose: to find other HF AC sources or other interferences to be ignored
		*/
		epc_clk_div(3);
		while(epc_i2c_is_busy());
		epc_dis_leds();
		while(epc_i2c_is_busy());
		epc_intlen(8, 200);
		while(epc_i2c_is_busy());


		dcmi_start_dma(dcsa, SIZEOF_2DCS);
		trig();
		LED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


		/*
			20 MHz
			LEDS OFF
			2DCS
			Short exp
			Purpose: Same as the previous, at different freq
		*/
		epc_clk_div(1);
		while(epc_i2c_is_busy());
		epc_intlen(24, 200);
		while(epc_i2c_is_busy());

		dcmi_start_dma(dcsb, SIZEOF_2DCS);
		trig();
		LED_ON();

		// Calculate the previous
		calc_interference_ignore_from_2dcs(ignore, dcsa, 10);

		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();



		/*
			20 MHz
			LEDS OFF
			MONOCHROME
			Long exp
			Purpose: General purpose monochrome, HDR long
		*/

		epc_greyscale();
		while(epc_i2c_is_busy());

		epc_intlen(120, 1000);
		while(epc_i2c_is_busy());

		dcmi_start_dma(&img_mono, SIZEOF_MONO);

		trig();
		LED_ON();

		// Calculate the previous
		calc_interference_ignore_from_2dcs(ignore, dcsb, 10);

		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


		/*
			20 MHz
			LEDS OFF
			MONOCHROME
			Short exp
			Purpose: General purpose monochrome, HDR short
		*/

		epc_intlen(120, 250);
		while(epc_i2c_is_busy());

		dcmi_start_dma(&img_mono, SIZEOF_MONO);

		trig();
		LED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


		/*
			6.66 MHz (wrap at 22.5m)
			LEDS ON
			2DCS
			Long exp
			Purpose: Long-distance approximate readings for ignoring too-far points.
		*/

		epc_clk_div(3);
		while(epc_i2c_is_busy());

		epc_ena_leds();
		while(epc_i2c_is_busy());

		epc_2dcs();
		while(epc_i2c_is_busy());

		epc_intlen(8, 2000);
		while(epc_i2c_is_busy());

		dcmi_start_dma(dcsa, SIZEOF_4DCS);
		trig();
		LED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();


		/*
			6.66 MHz (wrap at 22.5m)
			LEDS ON
			2DCS
			Short exp
			Purpose: Same as previous, but if there are super-reflective things like 
				traffic signals, they may overexpose the previous one.
		*/


		epc_intlen(8, 500);
		while(epc_i2c_is_busy());

		dcmi_start_dma(dcsb, SIZEOF_4DCS);
		trig();
		LED_ON();

		// Calculate the previous
		// We don't need fancy HDR combining here, since both exposures simply update the ignore list.
		calc_toofar_ignore_from_2dcs(ignore, dcsa, 6000);

		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		LED_OFF();





		__DSB(); __ISB();



		{
			for(int i=0; i < 160*60; i++)
			{
				uint16_t val = img_mono.img[i];
				uint16_t lum = ((val&0b0011111111111100)>>2);
				spitof.bw[i] = lum>>4;
			}
		}




		{
			timer_10k = 0;


			tof_calc_dist_ampl(spitof.ampl, spitof.dist, &img_dcs[0], config.offsets[config.clk_div], config.clk_div);

			int tooktime = timer_10k;
			spitof.calc_time = tooktime;
		}




	}


}


#define NUM_ADC_DATA 1

typedef struct __attribute__((packed))
{
	uint16_t vref;
	uint16_t tcpu;
} adc_datum_t;


volatile adc_datum_t adc_data[NUM_ADC_DATA];

#define SPI_TEST_LEN 10000
volatile uint8_t spi_test_rx[SPI_TEST_LEN];

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
	new_rx_len = sizeof(spitof) - DMA1_Stream3->NDTR;


	// Check if there is the maintenance magic code:

	if(*((volatile uint32_t*)&spi_test_rx[0]) == 0x9876fedb)
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
	DMA1_Stream4->NDTR = sizeof(spitof);
	DMA_CLEAR_INTFLAGS(DMA1, 4);
	DMA1_Stream4->CR |= 1; // Enable DMA

	// RX DMA

	DMA1_Stream3->NDTR = sizeof(spitof);
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


typedef struct __attribute__((packed))
{
	uint32_t header;
	uint8_t status; // Only read this and deassert chip select for polling the status
	uint8_t dummy1;
	uint8_t dummy2;
	uint8_t sensor_idx;

	pos_t robot_pos; // Robot pose during the acquisition

	uint16_t depth[EPC_XS*EPC_YS];
	uint8_t  ampl[EPC_XS*EPC_YS];
	uint8_t  ambient[EPC_XS*EPC_YS];

	uint16_t timestamps[10]; // 0.1ms unit timestamps of various steps for analyzing the timing of low-level processing

} pulutof_frame_t;

pulutof_frame_t raspi_tx;

void init_raspi_tx()
{
	raspi_tx.header = 0x11223344;

}

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

	RCC->AHB1ENR = 0xff /*GPIOA to H*/ | 1UL<<21 /*DMA1*/ | 1UL<<22 /*DMA2*/;
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
	RCC->DCKCFGR2 = 0b01UL<<4 /*USART3 = sysclk*/ | 0b00UL<<20 /*I2C3 = APB1clk*/;

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL
	RCC->CFGR |= 0b10; // Change PLL to system clock
	while((RCC->CFGR & (0b11UL<<2)) != (0b10UL<<2)) ; // Wait for switchover to PLL.

	// Enable FPU

	SCB->CPACR |= 0b1111UL<<20;
	__DSB();
	__ISB();

	RCC->AHB2ENR = 1UL<<0 /*DCMI*/;
	RCC->APB1ENR |= 1UL<<18 /*USART3*/ | 1UL<<23 /*I2C3*/ | 1UL<<3 /*TIM5*/ | 1UL<<14 /*SPI2*/;
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


	IO_TO_ALTFUNC(GPIOB, 10);
	IO_TO_ALTFUNC(GPIOB, 11);
	IO_SET_ALTFUNC(GPIOB, 10, 7);
	IO_SET_ALTFUNC(GPIOB, 11, 7);
	USART3->BRR = 1875; // 115200
//	USART3->CR2 = 0b10UL<<12 /* 2 stop bits*/;
//	USART3->CR3 = 1UL<<6 /*RX DMA*/;

	USART3->CR1 = 0UL<<15 /*Oversamp:16*/ | 0UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/ |  1UL /*USART ENA*/;


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

	SPI2->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/ | 1UL<<12 /*Don't Reject The Last Byte*/;

	// TX DMA
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = (uint32_t)&spitof;
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream4->NDTR = sizeof(spitof);
	DMA_CLEAR_INTFLAGS(DMA1, 4);
	DMA1_Stream4->CR |= 1; // Enable TX DMA

	// RX DMA

	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream3->M0AR = (uint32_t)&(spi_test_rx[0]);
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;
	DMA1_Stream3->NDTR = sizeof(spitof);
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

	epc_test();
}
