#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ext_include/stm32f7xx.h"
#include "stm32_cmsis_extension.h"

#include "own_std.h"
#include "tof_table.h"

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



#define EPC02_ADDR 0b0100000
#define EPC13_ADDR 0b0100001



#define sq(x) ((x)*(x))


void delay_us(uint32_t i)
{
	if(i==0) return;
	i *= 90;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_ms(uint32_t i)
{
	while(i--)
	{
		delay_us(1000);
	}
}

void error()
{
	__disable_irq();
	EPC01_RSTN_LOW();
	EPC23_RSTN_LOW();
	EPCNEG10V_OFF();
	EPC10V_OFF();
	EPC5V_OFF();
	EPCLEDV_OFF();
	while(1)
	{
		LED_ON();
		delay_ms(40);
		LED_OFF();
		delay_ms(40);
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
	SCB_InvalidateDCache();

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
			error();
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
} epc_dcs_t;

epc_img_t img_mono;

epc_img_t img_compmono;

epc_dcs_t img_dcs[2];

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
	IO_TO_ALTFUNC(GPIOB,  6);
	IO_TO_ALTFUNC(GPIOB,  8);
	IO_TO_ALTFUNC(GPIOB,  9);

	IO_SET_ALTFUNC(GPIOA,  6, 13);
	IO_SET_ALTFUNC(GPIOA,  9, 13);
	IO_SET_ALTFUNC(GPIOA, 10, 13);
	IO_SET_ALTFUNC(GPIOG, 10, 13);
	IO_SET_ALTFUNC(GPIOG, 11, 13);
	IO_SET_ALTFUNC(GPIOC, 11, 13);
	IO_SET_ALTFUNC(GPIOB,  6, 13);
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

	DMA2_Stream7->NDTR = size;
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	DMA_CLEAR_INTFLAGS(DMA2, 7);
	DMA2_Stream7->CR |= 1; // Enable DMA

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
	.bw_int_len = 1000,
	.dist_int_len = 1000,
	.clk_div = 1,
	.pll_shift = 0
};

config_t rx_config = // modified config
{
	.bw_int_len = 1000,
	.dist_int_len = 1000,
	.clk_div = 1,
	.pll_shift = 0
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

int16_t get_tof_tbl(int16_t y, int16_t x)
{
	int yy, xx;

	if(y<0)
		yy = -y;
	else
		yy = y;

	if(x<0)
		xx = -x;
	else
		xx = x;

	int swapped = 0;
	if(xx<yy)
	{
		swapped = 1;
		int16_t tmp = xx;
		xx = yy;
		yy = tmp;
	}

	if(xx == 0) return 0;

	int idx = (yy*(TOF_TBL_LEN-1))/xx;

	int16_t res = tof_tbl[idx];
	if(swapped) res = TOF_TBL_QUART_PERIOD - res;
	if(x<0) res = TOF_TBL_HALF_PERIOD - res;
	if(y<0) res = -res;

	return res;
}


/*
Process four DCS images, with offset_mm in millimeters. With clk_div=1, does the calculation at fled=20MHz (unamb range = 7.5m). Larger clk_div
multiplies the result.
*/

void tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, epc_dcs_t *in, int offset_mm, int clk_div)
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
				
				dist_i *= config.clk_div;

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

void tof_calc_dist(uint16_t *dist_out, epc_dcs_t *in, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int i=0; i < 160*60; i++)
	{
		uint16_t dist = 0;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
		{
			dist = 65535;
		}
		else if((in->dcs[0].img[i]&1) || (in->dcs[1].img[i]&1) ||
		   (in->dcs[2].img[i]&1) || (in->dcs[3].img[i]&1))
		{
			dist = 65535;
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
				
				dist_i *= config.clk_div;

				int ampl = sq(dcs20)+sq(dcs31);

				if(ampl < sq(75)*2)
					dist = 65534;
				else
					dist = dist_i;

			}

		}
		dist_out[i] = dist;
	}
}


/*
Process eight DCS images, with offset_mm in millimeters. With clk_div=1, does the calculation at fled=20MHz (unamb range = 7.5m). Larger clk_div
multiplies the result.
*/

#define HDR_SWITCHOVER 1600
void tof_calc_dist_hdr(uint16_t *dist_out, epc_dcs_t *in_s, epc_dcs_t *in_l, int offset_mm, int clk_div)
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

int auto_bw_exp = 1000;
int auto_dcs_exp = 1000;

#if 0
void epc_automatic()
{

	epc_i2c_init();

	{
		epc_wrbuf[0] = 0xcb;
		epc_wrbuf[1] = 0b01101111; // saturation bit, split mode, gated dclk, ESM
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}

	{
		epc_wrbuf[0] = 0x90;
		epc_wrbuf[1] = 0b11001000;
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}

	{
		epc_wrbuf[0] = 0x92;
		epc_wrbuf[1] = 0b11000100;
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}

/*
	{
		epc_wrbuf[0] = 0xcc;
		epc_wrbuf[1] = 1<<7; //saturate data;
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}
*/

	EPCLEDV_ON();

	delay_ms(100);

	epc_dcmi_init();

	// dummy grayscale frame
	{
		dcmi_start_dma(&img_mono, (EPC_XS*EPC_YS*2)/4+1);
		trig();
		delay_ms(100);
	}


	while(1)
	{

/*
		{
			epc_wrbuf[0] = 0x8b;
			epc_wrbuf[1] = config.pll_shift;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}
		{
			epc_wrbuf[0] = 0x73;
			epc_wrbuf[1] = config.dll_shift;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}
		{
			epc_wrbuf[0] = 0xae;
			epc_wrbuf[1] = (config.dll_shift>0)?0x04:0x01;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}
*/

		/*

			Dealiasing happens at 37.5m, 15.0m and 7.5m ranges. Using three frequencies (instead of two) furher helps eliminate HF modulated light sources such as some CCFL and LED lamps,
			and further confirm aliasing never happens. Shortest beat distance of the three is at 75.0m, which shouldn't be reached by the LEDs, even with highly reflective targets.
			(Maximum design range = 15m with 50% reflective targets -> 1250% reflectance would be needed for interference at 75 meters!)

		*/

		/*
			Instead of simple single register for the integration time with sufficient resolution, the sensor
			implements a 16-bit value with a 10-bit multiplier for said value.

			Further, the value is specified valid only when (val+1)%4 == 0.

			We utilize this register in HDR modes by keeping the integration time register but changing the multiplier.
		*/


/*
	Sequence:

	There is (barely) not enough memory to keep two sets of complete HDR acquisition (16 DCS frames) in memory to process the first set
	while the second is being acquired. If we calculate the HDR set (8 frames) after it's acquired, we are ok.

	We really want to minimize any delay between the aqcuisitions, so the calculation can start even before the DMA is finished.

	Also, we can start aqcuiring a bit in advance as we know the exposure takes some time, and even when the data starts coming, it's
	the head, and we are processing the tail.

	s = short exposure time
	l = long exposure time
	ambient comp = frame with ambient light purely for the compensation algorithm, same exposure than with short dcs


Imaging:     (ambient comp s)    (37.5m s) (37.5m l)          (7.5m s) (7.5m l)          (15.0m s) (15.0m l)  (ambient loooong)
Processing:                                 (dist 37.5m composite)       (dist 7.5m composite)      (dist 15.0m composite) (dealiasing and corrections, final composite, no hurry..........)
*/


		// First img is ambient light, clkdiv for 37.5m is used here.
		{
			epc_wrbuf[0] = 0x85;
			epc_wrbuf[1] = 1-1; //5-1; // 37.5m (37.470m) unambiguity range
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x90;
			epc_wrbuf[1] = 0b11001000; // leds disabled
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x92;
			epc_wrbuf[1] = 0b11000100; // greyscale modulation
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			int intlen = (auto_bw_exp<<2)-1;
			epc_wrbuf[0] = 0xA1;
			epc_wrbuf[1] = 20;
			epc_wrbuf[2] = (intlen&0xff00)>>8;
			epc_wrbuf[3] = intlen&0xff;

			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 4);
			while(epc_i2c_is_busy());
		}

		dcmi_start_dma(&img_mono, (EPC_XS*EPC_YS*2)/4+1);

		trig();

		GLED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		GLED_OFF();

		{
			epc_wrbuf[0] = 0x90;
			epc_wrbuf[1] = 0b11101000; // LED2 output on
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x92;
			epc_wrbuf[1] = 0b00110100; // Sinusoidal 4DCS modulation
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}


		// Short exp
		{
			int intlen = (auto_dcs_exp<<2)-1;
			epc_wrbuf[0] = 0xA1;
			epc_wrbuf[1] = 2;
			epc_wrbuf[2] = (intlen&0xff00)>>8;
			epc_wrbuf[3] = intlen&0xff;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 4);
			while(epc_i2c_is_busy());
		}

		dcmi_start_dma(&img_dcs[0].dcs[0], 4*((EPC_XS*EPC_YS*2)/4+1));

		trig();

		GLED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		GLED_OFF();


		// Long exp
		{
			int intlen = (auto_dcs_exp<<2)-1;
			epc_wrbuf[0] = 0xA1;
			epc_wrbuf[1] = 16*2;
			epc_wrbuf[2] = (intlen&0xff00)>>8;
			epc_wrbuf[3] = intlen&0xff;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 4);
			while(epc_i2c_is_busy());
		}

		dcmi_start_dma(&img_dcs[1].dcs[0], 4*((EPC_XS*EPC_YS*2)/4+1));

		trig();

		GLED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		GLED_OFF();


		static uint8_t sync_packet[8] = {0xff,0xff,0xff,0xff,  0xff,0xff,0x12,0xab};
		uart_send_blocking_crc(sync_packet, 0xaa, 8);
		delay_ms(5);

		// image data DMA is done at this point, data is not going to change anymore, so dcache is okay to use, just invalidate it first:
		SCB_InvalidateDCache();

		static uint8_t txbuf[EPC_XS*EPC_YS*2+2];

		{
			int i = 0;
			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx+=2)
				{
					uint16_t val1 = img_mono.img[yy*EPC_XS+xx];
					uint16_t lum1 = ((val1&0b0011111111111100)>>2);
					uint16_t val2 = img_mono.img[yy*EPC_XS+xx+1];
					uint16_t lum2 = ((val2&0b0011111111111100)>>2);
					txbuf[i++] = (lum1&0xff0)>>4;
					txbuf[i++] = (lum1&0xf)<<4 | (lum2&0xf00)>>8;
					txbuf[i++] = (lum2&0xff);
				}
			}
			uart_send_blocking_crc(txbuf, 0x01, i);
			delay_ms(5);
		}

		{
			timer_10k = 0;

			static uint16_t calc_dist[EPC_XS*EPC_YS+1]; // +1 for calc time

			tof_calc_dist_hdr(calc_dist, &img_dcs[0], &img_dcs[1], -5100, 1);

			int tooktime = timer_10k;
			calc_dist[EPC_YS*EPC_XS] = (uint16_t)tooktime;

			uart_send_blocking_crc((uint8_t*)calc_dist, 0x80, 2*EPC_YS*EPC_XS+2);
			delay_ms(5);
		}


	}

}

#ifdef COMPILE_TEST
void epc_test()
{
	char printbuf[64];

	delay_ms(300);
	EPC10V_ON();
	EPC5V_ON();
	delay_ms(100);
	EPCNEG10V_ON();
	delay_ms(100);
	EPC_RSTN_HIGH();
	delay_ms(300);


	uart_print_string_blocking("epc_i2c_init...");
	epc_i2c_init();
	uart_print_string_blocking("ok\r\n");

	uart_print_string_blocking("read IC type & version...");
	epc_i2c_read(EPC02_ADDR, 0x00, epc_rdbuf, 2);

	while(epc_i2c_is_busy());
	uart_print_string_blocking("type=");
	o_utoa8_fixed(epc_rdbuf[0], printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking(" version=");
	o_utoa8_fixed(epc_rdbuf[1], printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("read embedded sync data labels... ");
	epc_i2c_read(EPC02_ADDR, 0x1c, epc_rdbuf, 4);
	while(epc_i2c_is_busy());
	for(int i=0; i<4; i++)
	{
		o_utoa8_fixed(epc_rdbuf[i], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" ");
	}

	uart_print_string_blocking("\r\n");

	//while(DBG_BUT()) ;


	{
		epc_wrbuf[0] = 0xcb;
		epc_wrbuf[1] = 0b01101111; // saturation bit, split mode, gated dclk, ESM
		uart_print_string_blocking("write i2c&tcmi control reg...");
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

	{
		epc_wrbuf[0] = 0x90;
		epc_wrbuf[1] = 0b11001000;
		uart_print_string_blocking("write led driver control...");
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

	{
		epc_wrbuf[0] = 0x92;
		epc_wrbuf[1] = 0b11000100;
		uart_print_string_blocking("write modulation select (greyscale)...");
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

/*
	{
		epc_wrbuf[0] = 0xcc;
		epc_wrbuf[1] = 1<<7; //saturate data;
		uart_print_string_blocking("write tcmi polarity settings...");
		epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}
*/

	EPCLEDV_ON();

	delay_ms(100);


	uart_print_string_blocking("epc_dcmi_init...");
	epc_dcmi_init();
	uart_print_string_blocking("ok\r\n");

	{
		uart_print_string_blocking("write shutter (for dummy frame)...");
		dcmi_start_dma(&img_mono, (EPC_XS*EPC_YS*2)/4+1);
		trig();
		delay_ms(100);
		uart_print_string_blocking("ok\r\n");
	}


//	uart_print_string_blocking("Press button S1 to start (with binary communication)");
	uart_print_string_blocking("\r\n");


//	delay_ms(300);

	while(1)
	{

		// fetch config:

		static int conf_errors = 0;
		if(DMA1_Stream1->NDTR == sizeof(config_t))
		{
			config_t pending;
			memcpy(&pending, &rx_config, sizeof(config_t));
			if(DMA1_Stream1->NDTR == sizeof(config_t))
			{
				// config was unaccessed before and after copy - not being modified during copy.
				apply_config(&pending);
				conf_errors=0;
			}
			else
			{
				conf_errors++;
			}
		}
		else
		{
			conf_errors++;
		}
		epc_wrbuf[0] = 0x99;

		if(conf_errors > 2)
		{
			conf_errors = 0;
			// Very improbable that conf rx happens during the short unallowed period three times in a row - it's just stuck midway transfer (due to missed or excess bytes) - reset DMA.

			DMA1_Stream1->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 1UL<<8 /*circular ON*/ |
					   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
					   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 0UL<<4 /* Transfer complete interrupt*/;
			while(DMA1_Stream1->CR & 1UL) ;
			DMA1_Stream1->NDTR = sizeof(config_t);
			DMA_CLEAR_INTFLAGS(DMA1, 1);
			DMA1_Stream1->CR |= 1; // Enable DMA
		}



		//uart_print_string_blocking("RISR="); o_utoa32(DCMI->RISR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
		//uart_print_string_blocking("NDTR="); o_utoa32(DMA2_Stream7->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");

		{
			epc_wrbuf[0] = 0x8b;
			epc_wrbuf[1] = config.pll_shift;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}


		{
			epc_wrbuf[0] = 0x73;
			epc_wrbuf[1] = config.dll_shift;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0xae;
			epc_wrbuf[1] = (config.dll_shift>0)?0x04:0x01;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}


		{
			epc_wrbuf[0] = 0x85;
			epc_wrbuf[1] = config.clk_div-1;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x90;
			epc_wrbuf[1] = 0b11001000; // leds disabled
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x92;
			epc_wrbuf[1] = 0b11000100; // greyscale modulation
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			int intlen = ((int)config.bw_int_len<<2)-1;
			epc_wrbuf[0] = 0xA1;
			epc_wrbuf[1] = 120/config.clk_div;
			epc_wrbuf[2] = (intlen&0xff00)>>8;
			epc_wrbuf[3] = intlen&0xff;

			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 4);
			while(epc_i2c_is_busy());
		}



		dcmi_start_dma(&img_mono, (EPC_XS*EPC_YS*2)/4+1);

		trig();

		GLED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		GLED_OFF();


		{
			epc_wrbuf[0] = 0x90;
			epc_wrbuf[1] = 0b11101000; // LED2 output on
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x92;
			epc_wrbuf[1] = 0b00110100; // Sinusoidal 4DCS modulation
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			int intlen = ((int)config.dist_int_len<<2)-1;
			epc_wrbuf[0] = 0xA1;
			epc_wrbuf[1] = 24/config.clk_div;
			epc_wrbuf[2] = (intlen&0xff00)>>8;
			epc_wrbuf[3] = intlen&0xff;
			epc_i2c_write(EPC02_ADDR, epc_wrbuf, 4);
			while(epc_i2c_is_busy());
		}

		dcmi_start_dma(&img_dcs[0].dcs[0], 4*((EPC_XS*EPC_YS*2)/4+1));

		trig();

		GLED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		GLED_OFF();



		static uint8_t sync_packet[8] = {0xff,0xff,0xff,0xff,  0xff,0xff,0x12,0xab};
		uart_send_blocking_crc(sync_packet, 0xaa, 8);
		delay_ms(5);

		// image data DMA is done at this point, data is not going to change anymore, so dcache is okay to use, just invalidate it first:
		SCB_InvalidateDCache();

		static uint8_t txbuf[EPC_XS*EPC_YS*2+2];

#define SEND_BW
//#define SEND_DCS
//#define CALC_DIST_FLOAT
#define CALC_DIST_TABLE

#ifdef SEND_BW
		{
			int i = 0;
			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx+=2)
				{
					uint16_t val1 = img_mono.img[yy*EPC_XS+xx];
					uint16_t lum1 = ((val1&0b0011111111111100)>>2);
					uint16_t val2 = img_mono.img[yy*EPC_XS+xx+1];
					uint16_t lum2 = ((val2&0b0011111111111100)>>2);
					txbuf[i++] = (lum1&0xff0)>>4;
					txbuf[i++] = (lum1&0xf)<<4 | (lum2&0xf00)>>8;
					txbuf[i++] = (lum2&0xff);
				}
			}
			uart_send_blocking_crc(txbuf, 0x01, i);
			delay_ms(5);
		}
#endif

#ifdef SEND_DCS
		for(int dcs=0; dcs<4; dcs++)
		{
			int i = 0;
			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx+=2)
				{
					uint16_t val1 = img_dcs[0].dcs[dcs].img[yy*EPC_XS+xx];
					uint16_t lum1 = ((val1&0b0011111111111100)>>2);
					uint16_t val2 = img_dcs[0].dcs[dcs].img[yy*EPC_XS+xx+1];
					uint16_t lum2 = ((val2&0b0011111111111100)>>2);
					txbuf[i++] = (lum1&0xff0)>>4;
					txbuf[i++] = (lum1&0xf)<<4 | (lum2&0xf00)>>8;
					txbuf[i++] = (lum2&0xff);
				}
			}
			uart_send_blocking_crc(txbuf, 0x10+dcs, i);
			delay_ms(5);
		}
#endif

	// 260ms with software FP
	// 28ms with HW FP
	// 23ms with amplitude calculation optimized
	// 6.5ms with fixed point atan table


#ifdef CALC_DIST_FLOAT


		{
			int i = 0;

			timer_10k = 0;

			float fled = 20000000.0 / config.clk_div;
			float mult = 299792458.0/2.0*1.0/(2.0*M_PI*fled); 
			float unamb = 299792458.0/2.0*1.0/(fled); 

			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx++)
				{
					uint16_t dist;
//					if((img_dcs[0].dcs[0].img[yy*EPC_XS+xx]&1) || (img_dcs[0].dcs[1].img[yy*EPC_XS+xx]&1) ||
//					   (img_dcs[0].dcs[2].img[yy*EPC_XS+xx]&1) || (img_dcs[0].dcs[3].img[yy*EPC_XS+xx]&1))
//					{
//						dist = 65535;
//					}

					int16_t dcs0 = ((img_dcs[0].dcs[0].img[yy*EPC_XS+xx]&0b0011111111111100)>>2)-2048;
					int16_t dcs1 = ((img_dcs[0].dcs[1].img[yy*EPC_XS+xx]&0b0011111111111100)>>2)-2048;
					int16_t dcs2 = ((img_dcs[0].dcs[2].img[yy*EPC_XS+xx]&0b0011111111111100)>>2)-2048;
					int16_t dcs3 = ((img_dcs[0].dcs[3].img[yy*EPC_XS+xx]&0b0011111111111100)>>2)-2048;

					if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
					{
						dist = 65535;
					}
					else if((img_dcs[0].dcs[0].img[yy*EPC_XS+xx]&1) || (img_dcs[0].dcs[1].img[yy*EPC_XS+xx]&1) ||
					   (img_dcs[0].dcs[2].img[yy*EPC_XS+xx]&1) || (img_dcs[0].dcs[3].img[yy*EPC_XS+xx]&1))
					{
						dist = 65535;
					}
					else
					{
						float dcs31 = dcs3-dcs1;
						float dcs20 = dcs2-dcs0;

						float dist_f = (mult * (/*M_PI +*/ atan2(dcs31,dcs20)))+((float)config.offsets[config.clk_div]/1000.0);

						if(dist_f < 0.0) dist_f += unamb;
						else if(dist_f > unamb) dist_f -= unamb;

						//float ampl = sqrt(sq(dcs20)+sq(dcs31))/2.0;
						float ampl = sq(dcs20)+sq(dcs31);
						if(ampl < sq(75.0)*2.0)
							dist = 65534;
						else
							dist = dist_f*1000.0;
					}

					txbuf[i++] = (dist&0xff00)>>8;
					txbuf[i++] = dist&0xff;
				}
			}
			int tooktime = timer_10k;
			txbuf[i++] = (tooktime&0xff00)>>8;
			txbuf[i++] = tooktime&0xff;

			uart_send_blocking_crc(txbuf, 0x80, i);
			delay_ms(5);
		}



#endif

#ifdef CALC_DIST_TABLE


		{
			timer_10k = 0;

			static uint16_t calc_dist[EPC_XS*EPC_YS+1]; // +1 for calc time
			static uint8_t  calc_ampl[EPC_XS*EPC_YS];

			tof_calc_dist_ampl(calc_ampl, calc_dist, &img_dcs[0], config.offsets[config.clk_div], config.clk_div);

			int tooktime = timer_10k;
			calc_dist[EPC_YS*EPC_XS] = (uint16_t)tooktime;

			uart_send_blocking_crc((uint8_t*)calc_dist, 0x80, 2*EPC_YS*EPC_XS+2);
			uart_send_blocking_crc((uint8_t*)calc_ampl, 0x81, EPC_YS*EPC_XS);
			delay_ms(5);
		}


#endif


	}


}
#endif

#endif

#define NUM_ADC_DATA 1

typedef struct __attribute__((packed))
{
	uint16_t vref;
	uint16_t tcpu;
} adc_datum_t;


volatile adc_datum_t adc_data[NUM_ADC_DATA];

void uart_dbg()
{
	static uint8_t test[10000];
	uint8_t c = 0;
	for(int i=0; i<10000; i++)
	{
		test[i] = c++;
	}

	while(1)
	{
		static uint8_t sync_packet[8] = {0xff,0xff,0xff,0xff,  0xff,0xff,0x12,0xab};
		uart_send_blocking_crc(sync_packet, 0xaa, 8);
		uart_send_blocking_crc(test, 0x30, 10000);

	}
}

#define SPI_TEST_LEN 1024
volatile uint8_t spi_test_tx[SPI_TEST_LEN];
volatile uint8_t spi_test_rx[SPI_TEST_LEN];

volatile int spi_test_cnt;

void raspi_spi_xfer_end_inthandler()
{
	EXTI->PR = 1UL<<12; // Clear "pending bit".

	// Triggered when cs goes high - switch DMA rx buffers, zero the tx buffer loc

	spi_test_cnt++;

	while(DMA2_Stream7->CR & 1UL) ;


	// Disable the DMAs:
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;


	while(DMA1_Stream4->CR & 1UL) ;
	while(DMA1_Stream3->CR & 1UL) ;

	// Hard-reset SPI - the only way to empty TXFIFO! (Go figure.)

	RCC->APB1RSTR = 1UL<<14;
	__asm__ __volatile__ ("nop");
	RCC->APB1RSTR = 0;
			
	// Re-enable:

	SPI2->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/;

	// TX DMA
	DMA1_Stream4->NDTR = SPI_TEST_LEN;
	DMA_CLEAR_INTFLAGS(DMA1, 4);
	DMA1_Stream4->CR |= 1; // Enable DMA

	// RX DMA

	DMA1_Stream3->NDTR = SPI_TEST_LEN;
	DMA_CLEAR_INTFLAGS(DMA1, 3);
	DMA1_Stream3->CR |= 1; // Enable DMA

	SPI2->CR2 |= 1UL<<1 /*TX DMA ena*/; // not earlier!

	SPI2->CR1 = 1UL<<6; // Enable in slave mode

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

//	SCB->CPACR |= 0b1111UL<<20;
//	__DSB();
//	__ISB();

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

//	TIM5->DIER |= 1UL; // Update interrupt
//	TIM5->ARR = 10799; // 108MHz -> 10 kHz
//	TIM5->CR1 |= 1UL; // Enable

//	NVIC_SetPriority(TIM5_IRQn, 0b1010);
//	NVIC_EnableIRQ(TIM5_IRQn);

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

	uint8_t c = 0x42;
	for(int i = 0; i < SPI_TEST_LEN; i++)
	{
		spi_test_tx[i] = c++;
	}


/*
#define RASPI_SPI       SPI2
#define RASPI_WR_DMA    DMA1
#define RASPI_WR_STREAM 4
//#define RASPI_WR_DMA_STREAM DMA1_Stream4 
#define RASPI_WR_DMA_STREAM (RASPI_WR_DMA ## _Stream ## RASPI_WR_STREAM)

#define RASPI_WR_DMA_CHAN   0
#define RASPI_RD_DMA    DMA1
#define RASPI_RD_STREAM 3
//#define RASPI_RD_DMA_STREAM DMA1_Stream3
#define RASPI_RD_DMA_STREAM (RASPI_RD_DMA ## _Stream ## RASPI_RD_STREAM)
#define RASPI_RD_DMA_CHAN   0
*/

	IO_ALTFUNC(GPIOB, 12, 5);
	IO_ALTFUNC(GPIOB, 13, 5);
	IO_ALTFUNC(GPIOB, 14, 5);
	IO_ALTFUNC(GPIOB, 15, 5);

	// Max freq - tested with 1024 long sequence, contains all byte values, test ran a few times.
	// Remember to leave safety margin - these are the highest speeds which showed no bit errors, but
	// the test is short and at room temp only, on one device only.
	// 0 --> 17MHz
	// 1 --> 31MHz
	// 2 --> 31MHz, but fewer errors at 32MHz than at speed1, so there is a miniscule difference
	// 3 --> 41MHz
	IO_SPEED(GPIOB, 14, 3);

/*
	spi_test_tx[0] = 0xaa;
	spi_test_tx[1] = 0xbb;
	spi_test_tx[2] = 0xcc;
	spi_test_tx[3] = 0xdd;
	spi_test_tx[4] = 0xee;
*/
//	delay_us(100);



	// Initialization order from reference manual:

	SPI2->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/;

	// TX DMA
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = (uint32_t)&(spi_test_tx[0]);
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream4->NDTR = SPI_TEST_LEN;
	DMA_CLEAR_INTFLAGS(DMA1, 4);
	DMA1_Stream4->CR |= 1; // Enable TX DMA

	// RX DMA

	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream3->M0AR = (uint32_t)&(spi_test_rx[0]);
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;
	DMA1_Stream3->NDTR = SPI_TEST_LEN;
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

	while(1)
	{
		char printbuf[64];
		uart_print_string_blocking("SPI SR = "); o_utoa16(SPI2->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("tx NDTR = "); o_utoa16(DMA1_Stream4->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("rx NDTR = "); o_utoa16(DMA1_Stream3->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("spi_test_cnt = "); o_utoa16(spi_test_cnt, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		/*
		uart_print_string_blocking(" "); o_utoa8_fixed(spi_test_tx[0], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" "); o_utoa8_fixed(spi_test_tx[1], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" "); o_utoa8_fixed(spi_test_tx[2], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" "); o_utoa8_fixed(spi_test_tx[3], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" "); o_utoa8_fixed(spi_test_tx[4], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" "); o_utoa8_fixed(spi_test_tx[5], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking("\r\n");
		*/

		uart_print_string_blocking("\r\n\r\n");
		delay_ms(100);

//		LED_ON();
//		delay_ms(500);
//		LED_OFF();
//		delay_ms(500);
	}

//	__enable_irq();


	//uart_dbg();
}
