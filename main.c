//#define SEND_EXTRA
#define FAST_APPROX_AMPLITUDE


#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ext_include/stm32f7xx.h"
#include "stm32_cmsis_extension.h"

#include "own_std.h"
#include "tof_table.h"
#include "flash.h"

#include "epc635_seq_v10.c"

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

// Temperature sensor readout procedure is weird, and requires storing and restoring some undocumented internal registers to the chip:
static uint8_t epc_tempsens_regx[N_SENSORS], epc_tempsens_regy[N_SENSORS];
static float epc_tempsens_factory_offset[N_SENSORS]; // in some intermediate format, as specified in datasheet parameter "z"
int temperatures[N_SENSORS]; // in degs/10

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
#define abso(x) ((x<0)?(-x):(x))

volatile int new_rx, new_rx_len;
volatile int robot_new_rx, robot_new_rx_len;


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

#ifdef USE_UART
static void uart_print_string_blocking(const char *buf)
{
	while(buf[0] != 0)
	{
		while((USART3->ISR & (1UL<<7)) == 0) ;
		USART3->TDR = buf[0];
		buf++;
	}
}
#endif

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
	int o = 0;
	while(1)
	{
		LED_ON();
		delay_ms(120);
		LED_OFF();
		delay_ms(120);

		i++;
		if(i >= code)
		{
			i = 0;
			delay_ms(600);
			o++;

			if(o > 5)
			{
				NVIC_SystemReset();
				while(1);
			}
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
volatile int init_err_cnt = 0;
void epc_i2c_write_dma(uint8_t bus, uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	if(bus >= N_I2C) return;
	// Pipeline flush is almost always needed when using this function, so easier to do it here.
	// If data cache is enabled later, remember to use non-cacheable sections (or invalidate the cache)
	__DSB(); __ISB();

	if(epc_i2c_write_busy[bus] || epc_i2c_read_busy[bus] || epc_i2c_read_state[bus])
		error(4);

	if(DMA1->LISR & 1UL<<0)
		error(5);
	if(DMA1->LISR & 1UL<<2)
		error(6);
	if(DMA1->LISR & 1UL<<3)
		error(7);

	if(EPC_I2C_WR_DMAS[bus]->CR & 1UL)
	{
		if(DMA1->LISR & 1UL<<5)
			error(8);

		error(15+init_err_cnt);
	}

	epc_i2c_write_busy[bus] = 1;
	EPC_I2CS[bus]->ICR = 1UL<<5; // Clear any pending STOPF interrupt 
	if(bus == 0)
		NVIC_ClearPendingIRQ(I2C3_EV_IRQn);
	else if(bus==1)
		NVIC_ClearPendingIRQ(I2C4_EV_IRQn);


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
	// Pipeline flush is almost always needed when using this function, so easier to do it here.
	// If data cache is enabled later, remember to use non-cacheable sections (or invalidate the cache)
	__DSB(); __ISB();

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

	// Pipeline flush is almost always needed when using this function, so easier to do it here.
	// If data cache is enabled later, remember to use non-cacheable sections (or invalidate the cache)
	__DSB(); __ISB();

	if(epc_i2c_write_busy[bus] || epc_i2c_read_busy[bus])
		error(10);
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
	else  // STOPF interrupt - this was a write.
	{
		if(I2C3->ISR & (1UL<<15)) // busy shouldn't be high - at least fail instead of doing random shit
		{
			error(11);
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
			error(12);
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

	NVIC_SetPriority(I2C3_EV_IRQn, 0b1010);
	NVIC_EnableIRQ(I2C3_EV_IRQn);
	NVIC_SetPriority(DMA1_Stream2_IRQn, 0b1010);
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

	NVIC_SetPriority(I2C4_EV_IRQn, 0b1010);
	NVIC_EnableIRQ(I2C4_EV_IRQn);
	NVIC_SetPriority(DMA1_Stream1_IRQn, 0b1010);
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

int poll_capt_with_timeout();
int poll_capt_with_timeout_complete();

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

void epc_select(uint8_t idx)
{
	__DSB(); __ISB();
	EPC_DESEL();
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	switch(idx)
	{
		case 0: EPC_SEL0(); break;
		case 1: EPC_SEL1(); break;
		case 2: EPC_SEL2(); break;
		case 3: EPC_SEL3(); break;
		default: break;
	}
}


volatile uint8_t epc_wrbuf[16];// __attribute__((section(".data_dtcm"))); // to skip cache
volatile uint8_t epc_rdbuf[16];// __attribute__((section(".data_dtcm"))); // to skip cache

volatile int timer_10k;
void timebase_handler()
{
	TIM5->SR = 0; // Clear interrupt flag
	timer_10k++;
}

void tof_calc_ampl_hdr(uint8_t *ampl_out, uint8_t* long_in, uint8_t* short_in)
{
	for(int i=0; i<160*60; i++)
	{
		uint8_t out;
		if(long_in[i] == 255)
			out = 128+(short_in[i]>>1);
		else
			out = long_in[i]>>1;
		ampl_out[i] = out;
	}
}


/*
Process four DCS images, with offset_mm in millimeters. With clk_div=1, does the calculation at fled=20MHz (unamb range = 7.5m). Larger clk_div
multiplies the result.
*/

void tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, int offset_mm, int clk_div) __attribute__((section(".text_itcm")));
void tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int i=0; i < 160*60; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
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
				
				dist_i *= clk_div;

				if(dist_i < 1) dist_i = 1; else if(dist_i>6000) dist_i=6000;
//				if(dist_i < 1) dist_i += 3000; 
//				if(dist_i < 1) dist_i = 1;

				if(dist_i>6000) dist_i=6000;

#ifdef FAST_APPROX_AMPLITUDE
				ampl = (abso(dcs20)+abso(dcs31))/30; if(ampl > 255) ampl = 255;
#else
				ampl = sqrt(sq(dcs20)+sq(dcs31))/23;
#endif

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

void epc_greyscale(int idx) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b11000100; // greyscale modulation
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

void epc_2dcs(int idx) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00010100; // 2dcs modulation
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

void epc_4dcs(int idx) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00110100; // 4dcs modulation
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}


void epc_4dcs_dualint(int idx) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00111100;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

// Required for dual integration time or dual phase mode:
void epc_dualphase_or_int(int idx) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x94;
	epc_wrbuf[1] = 0x80;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}

// Back to normal mode (non-dual int.time or non-dual phase)
void epc_normalphase_or_int(int idx) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x94;
	epc_wrbuf[1] = 0x00;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
}


void epc_intlen(int idx, uint8_t multiplier, uint16_t time) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	int intlen = ((int)time<<2)-1;
	epc_wrbuf[0] = 0xA1;
	epc_wrbuf[1] = multiplier;
	epc_wrbuf[2] = (intlen&0xff00)>>8;
	epc_wrbuf[3] = intlen&0xff;

	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 4);
}

// time1 = odd rows, time2 = even rows
// epc_4dcs_dualint() and epc_dualphase_or_int() must be called first
void epc_intlen_dual(int idx, uint8_t multiplier, uint16_t time1, uint16_t time2) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	int intlen1 = ((int)time1<<2)-1;
	int intlen2 = ((int)time2<<2)-1;
	epc_wrbuf[0] = 0x9E;
	epc_wrbuf[1] = (intlen2&0xff00)>>8;
	epc_wrbuf[2] = intlen2&0xff;
	epc_wrbuf[3] = 0;
	epc_wrbuf[4] = multiplier;
	epc_wrbuf[5] = (intlen1&0xff00)>>8;
	epc_wrbuf[6] = intlen1&0xff;

	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 7);
}

// Do the magic stuff specified in the datasheet to enable temperature sensor conversion:
void epc_temperature_magic_mode(int idx)
{
	epc_wrbuf[0] = 0xD3;
	epc_wrbuf[1] = epc_tempsens_regx[idx] | 0x60;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
	while(epc_i2c_is_busy(buses[idx]));

	epc_wrbuf[0] = 0xD5;
	epc_wrbuf[1] = epc_tempsens_regy[idx] & 0x0f;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
	while(epc_i2c_is_busy(buses[idx]));
}


// Do the magic stuff specified in the datasheet to disable temperature sensor conversion, back to normal operation
void epc_temperature_magic_mode_off(int idx)
{
	epc_wrbuf[0] = 0xD3;
	epc_wrbuf[1] = epc_tempsens_regx[idx];
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
	while(epc_i2c_is_busy(buses[idx]));

	epc_wrbuf[0] = 0xD5;
	epc_wrbuf[1] = epc_tempsens_regy[idx];
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
	while(epc_i2c_is_busy(buses[idx]));
}

uint16_t epc_read_temperature_regs(int idx)
{
	uint8_t hi, lo;
	epc_i2c_read(buses[idx], addrs[idx], 0x60, &hi, 1);
	while(epc_i2c_is_busy(buses[idx]));
	epc_i2c_read(buses[idx], addrs[idx], 0x61, &lo, 1);
	while(epc_i2c_is_busy(buses[idx]));

	delay_us(50); // see the comment about i2c read bug in top_init

	return ((uint16_t)hi<<8) | ((uint16_t)lo);
}


void epc_fix_modulation_table_defaults(int idx)
{
	epc_wrbuf[0] = 0x22;
	epc_wrbuf[1] = 0x30;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
	while(epc_i2c_is_busy(buses[idx]));
	epc_wrbuf[0] = 0x25;
	epc_wrbuf[1] = 0x35;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
	while(epc_i2c_is_busy(buses[idx]));
	epc_wrbuf[0] = 0x28;
	epc_wrbuf[1] = 0x3A;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
	while(epc_i2c_is_busy(buses[idx]));
	epc_wrbuf[0] = 0x2B;
	epc_wrbuf[1] = 0x3F;
	epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
	while(epc_i2c_is_busy(buses[idx]));
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

			if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
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

			if( dcs0 < -1*threshold || dcs0 > threshold || dcs1 < -1*threshold || dcs1 > threshold)
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


// has been: 100, 600, 3600
// now is  : 80, 560, 3920

#define SHORTEST_INTEGRATION 80
#define HDR_EXP_MULTIPLIER 7 // integration time multiplier when going from shot 0 to shot1, or from shot1 to shot2

#define STRAY_CORR_LEVEL 2000 // smaller -> more correction
#define STRAY_BLANKING_LVL 40 // smaller -> more easily ignored

#define STRAY_CORR_FACTOR 16 // bigger -> do more correction

/*
	ampl and dist are expected to contain 3*EPC_XS*EPC_YS elements: three images at different exposures that vary by HDR_EXP_MULTIPLIER
*/
void tof_calc_dist_3hdr_with_ignore_with_straycomp(uint16_t* dist_out, uint8_t* ampl, uint16_t* dist, uint8_t* ignore_in, uint16_t stray_ampl, uint16_t stray_dist)
{
	int stray_ignore = stray_ampl/STRAY_BLANKING_LVL;
	if(stray_ignore < 5) stray_ignore = 5;
	for(int yy=0; yy < EPC_YS; yy++)
	{
		for(int xx=0; xx < EPC_XS; xx++)
		{
			int pxidx          = yy*EPC_XS+xx;
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
				dist_out[pxidx] = 3;
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

					int32_t corr_amount = ((STRAY_CORR_FACTOR*(int32_t)stray_ampl)/(int32_t)combined_ampl);

					// Expect higher amplitude from "close" pixels. If the amplitude is low, they are artefacts most likely.
					int32_t expected_ampl;
					if(combined_dist > 2000)
						expected_ampl = stray_ignore;
					else
						expected_ampl = (2000*stray_ignore)/combined_dist;

					if(corr_amount > 1000 || corr_amount < -300 || combined_ampl < expected_ampl)
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

/*
	threshold = 250mm
	Starts to eat some areas away, but would be perfectly usable if good filtration is important

	threshold = 500mm
	Doesn't filter all real-world midliers
*/
#define MIDLIER_THRESHOLD 300
void tof_remove_midliers(uint16_t* out, uint16_t* in)
{
	memset(out, 0, 2*EPC_XS*EPC_YS);
	for(int xx=0; xx<EPC_XS; xx++)
	{
		for(int yy=0; yy<EPC_YS; yy++)
		{
			int pxval = in[yy*EPC_XS+xx];

			int min = pxval-MIDLIER_THRESHOLD;
			int max = pxval+MIDLIER_THRESHOLD;

			for(int xxx=xx-1; xxx<=xx+1; xxx++)
			{
				for(int yyy=yy-1; yyy<=yy+1; yyy++)
				{
					if((xxx>=0 && xxx<EPC_XS && yyy>=0 && yyy<EPC_YS) && (in[yyy*EPC_XS+xxx] < min || in[yyy*EPC_XS+xxx] > max))
						goto MIDLIER_FOUND;
				}
			}

			out[yy*EPC_XS+xx] = pxval;
			MIDLIER_FOUND: continue;
		}
	}
}


/*
	ampl and dist are expected to contain 2*EPC_XS*EPC_YS elements.
	The first 1*EPC_XS*EPC_YS is a dual int mode set, even lines at shortest integration, odd lines at mid integration
	The second is a single full image.
*/
void tof_calc_dist_3hdr_dualint_and_normal_with_ignore_with_straycomp(uint16_t* dist_out, uint8_t* ampl, uint16_t* dist, uint8_t* ignore_in, uint16_t stray_ampl, uint16_t stray_dist)
{
	int stray_ignore = stray_ampl/STRAY_BLANKING_LVL;
	if(stray_ignore < 2) stray_ignore = 2;
	for(int yy=0; yy < EPC_YS; yy++)
	{
		for(int xx=0; xx < EPC_XS; xx++)
		{
			int pxidx          = yy*EPC_XS+xx;
			int pxidx_shortint = (yy&(~1UL))*EPC_XS+xx;
			int pxidx_midint   = (yy | 1UL) *EPC_XS+xx;
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
			else if(ampl[0*EPC_XS*EPC_YS+pxidx_shortint] == 255) // Shortest exposure overexposed
			{
				dist_out[pxidx] = 3;
			}
			else
			{
				int32_t suit0 = ampl_suitability[ampl[0*EPC_XS*EPC_YS+pxidx_shortint]];
				int32_t suit1 = ampl_suitability[ampl[0*EPC_XS*EPC_YS+pxidx_midint]];
				int32_t suit2 = ampl_suitability[ampl[1*EPC_XS*EPC_YS+pxidx]];

				int32_t dist0 = dist[0*EPC_XS*EPC_YS+pxidx_shortint];
				int32_t dist1 = dist[0*EPC_XS*EPC_YS+pxidx_midint];
				int32_t dist2 = dist[1*EPC_XS*EPC_YS+pxidx];

				int32_t ampl0 = ampl[0*EPC_XS*EPC_YS+pxidx_shortint]*HDR_EXP_MULTIPLIER*HDR_EXP_MULTIPLIER;
				int32_t ampl1 = ampl[0*EPC_XS*EPC_YS+pxidx_midint]*HDR_EXP_MULTIPLIER;
				int32_t ampl2 = ampl[1*EPC_XS*EPC_YS+pxidx];

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

					if(corr_amount > 1000 || corr_amount < -300 || combined_ampl < expected_ampl)
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

	pos_t robot_pos;

} robot_to_pulutof_t;

typedef struct __attribute__((packed)) __attribute__((aligned(4)))
{
	uint32_t header;

	pos_t dummy;

} pulutof_to_robot_t;

volatile robot_to_pulutof_t robot_to_pulutof;
volatile pulutof_to_robot_t pulutof_to_robot;

volatile pos_t latest_pos;

typedef struct __attribute__((packed)) __attribute__((aligned(4)))
{
	uint32_t header;
	uint8_t status; // Only read this far and deassert chip select for polling the status.
	uint8_t dummy1;
	uint8_t dummy2;
	uint8_t sensor_idx;

	pos_t robot_pos; // Robot pose during the acquisition

	uint16_t depth[EPC_XS*EPC_YS];
	uint8_t  ampl[EPC_XS*EPC_YS];
//	uint8_t  ambient[EPC_XS*EPC_YS];
#ifdef SEND_EXTRA
	uint16_t uncorrected_depth[EPC_XS*EPC_YS];

	uint8_t dbg_id;
	uint8_t dbg[2*EPC_XS*EPC_YS];
#endif

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
static epc_img_t mono_comp __attribute__((aligned(4)));

void top_init()
{
	delay_ms(300);
	EPC10V_ON();
	EPC5V_ON();
	delay_ms(100);
	EPCNEG10V_ON();
	delay_ms(100);
	EPC01_RSTN_HIGH();
	delay_ms(5);
	EPC23_RSTN_HIGH();
	delay_ms(300);


	epc01_i2c_init();
	epc23_i2c_init();

	/*
		Even with 40cm cable, 40MHz (div 2) works well!

	*/

	for(int idx = 0; idx < N_SENSORS; idx++)
//	for(int idx = 0; idx < 1; idx++)
	{
		init_err_cnt = idx;
		delay_ms(10);

		// Reprogram the sequencer binary:
		for(int seq=0; seq<SEQ_LEN; seq++)
		{
			epc_i2c_write(buses[idx], addrs[idx], epc_seq[seq].d, epc_seq[seq].len);
			while(epc_i2c_is_busy(buses[idx]));
		}

		delay_ms(10);


		{
			epc_wrbuf[0] = 0xaa; 
			epc_wrbuf[1] = 4;
			epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
			while(epc_i2c_is_busy(buses[idx]));
		}

		{
			epc_wrbuf[0] = 0xab; 
			epc_wrbuf[1] = 4;
			epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
			while(epc_i2c_is_busy(buses[idx]));
		}


// Temperature sensor readout procedure is weird, and requires storing and restoring some undocumented internal registers to the chip:

		{

			epc_i2c_read(buses[idx], addrs[idx], 0xd3, &epc_tempsens_regx[idx], 1);
			while(epc_i2c_is_busy(buses[idx]));

			epc_i2c_read(buses[idx], addrs[idx], 0xd5, &epc_tempsens_regy[idx], 1);
			while(epc_i2c_is_busy(buses[idx]));

			uint8_t temp_offset;
			epc_i2c_read(buses[idx], addrs[idx], 0xe8, &temp_offset, 1);
			while(epc_i2c_is_busy(buses[idx]));
			__DSB(); __ISB();

			epc_tempsens_factory_offset[idx] = (float)temp_offset/4.7 - (float)0x12b;

		}
			/*
				Weird i2c bug: if epc_i2c_read is used, then write operations, _two_ write operations later the write fails
				because the previous write DMA is unfinished. It seems this is because of spurious STOPF interrupt happening
				somewhere between the first (succesful) epc_i2c_write and while(epc_i2c_is_busy()).  I can't get the interrupt
				away, no matter what I try. A small delay after the i2c read operation fixes the issue temporarily. 
				Empirically found out that delay_us(8) is not enough while delay_us(10) is (of course add a safety margin there).

			*/
			delay_us(50);


		{
			epc_wrbuf[0] = 0xcc; // tcmi polarity settings
			epc_wrbuf[1] = 1<<0 /*dclk rising edge*/ | 1<<7 /*saturate data*/; // bit 0 actually defaults to 1, at least on stock sequencer
			epc_i2c_write(buses[idx], addrs[idx], epc_wrbuf, 2);
			while(epc_i2c_is_busy(buses[idx]));
			__DSB(); __ISB();
		}


		{
			epc_wrbuf[0] = 0x89; 
			epc_wrbuf[1] = (2 /*TCMI clock div 2..16, default 4*/ -1) | 0<<7 /*add clock delay?*/;
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


	
	}

	EPCLEDV_ON();

	delay_ms(100);

	EPC_DESEL(); // just in case... To avoid mistakes
	EPC_SEL0();

	epc_dcmi_init();

	// Take a dummy frame, which will eventually output the end-of-frame sync marker, to get the DCMI sync marker parser in the right state
	// Any camera works for this, let's use 0.
	{
		dcmi_start_dma(&mono_comp, SIZEOF_MONO);
		trig(0);
		delay_ms(100);
	}

	EPC_DESEL();


}

#define OFFSET_CALC_Y_IGNORE (15)
#define OFFSET_CALC_X_IGNORE (60)
#define OFFSET_CALC_NUM_PX   ((EPC_YS-2*OFFSET_CALC_Y_IGNORE)*(EPC_XS-2*OFFSET_CALC_X_IGNORE))

void tof_calc_offset(epc_4dcs_t *in, int clk_div, int *n_overs, int *n_unders, int *n_valids, int *dist_sum)
{
	int overexps = 0, underexps = 0, valids = 0, dist_accum = 0;

	int smallest = 99999;
	int biggest = -99999;

	for(int yy=OFFSET_CALC_Y_IGNORE; yy < 60-OFFSET_CALC_Y_IGNORE; yy++)
	{
		for(int xx=OFFSET_CALC_X_IGNORE; xx<160-OFFSET_CALC_X_IGNORE; xx++)
		{
			int i = yy*EPC_XS+xx;

			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

			if(dcs0 < -1900 || dcs0 > 1900 || dcs1 < -1900 || dcs1 > 1900 || dcs2 < -1900 || dcs2 > 1900 || dcs3 < -1900 || dcs3 > 1900)
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

				if(dcs20_mod == 0 || sq(dcs20)+sq(dcs31) < sq(350))
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

					if(dist_i < smallest) smallest = dist_i;
					if(dist_i > biggest) biggest = dist_i;

					valids++;
					dist_accum += dist_i;
				}

			}
		}
	}

	raspi_tx.dbg_i32[4] = smallest;
	raspi_tx.dbg_i32[5] = biggest;
	raspi_tx.dbg_i32[6] = valids;

	*dist_sum = dist_accum;
	*n_valids = valids;
	*n_unders = underexps;
	*n_overs = overexps;
}



int run_offset_cal(uint8_t idx)
{
	if(idx > N_SENSORS-1)
		return 1;

	__DSB(); __ISB();

	epc_select(idx);

	epc_ena_leds(idx);
	while(epc_i2c_is_busy(buses[idx]));

	epc_4dcs(idx);
	while(epc_i2c_is_busy(buses[idx]));

	for(int clkdiv=1; clkdiv < 4; clkdiv++)
	{
		epc_clk_div(idx, clkdiv);
		while(epc_i2c_is_busy(buses[idx]));

		int int_time = 10;
		while(1)
		{
			epc_intlen(idx, 8, int_time);
			while(epc_i2c_is_busy(buses[idx]));

			dcmi_start_dma(&dcsa, SIZEOF_4DCS);
			trig(idx);
			LED_ON();
			if(poll_capt_with_timeout()) return 99;
			LED_OFF();

			int n_overs, n_unders, n_valids, dist_sum;

			tof_calc_offset(&dcsa, clkdiv, &n_overs, &n_unders, &n_valids, &dist_sum);

			if(n_overs > 30)
			{
				raspi_tx.dbg_i32[4] = n_overs;
				raspi_tx.dbg_i32[5] = n_unders;
				raspi_tx.dbg_i32[6] = n_valids;
				return 100+clkdiv;
			}
			if(int_time > 500)
				return 200+clkdiv;


			if(n_valids < (OFFSET_CALC_NUM_PX/8))
			{
				int_time = (int_time*3)/2;

			}
			else if(n_valids < (OFFSET_CALC_NUM_PX*9/10))
			{
				int_time = (int_time*7)/6;
			}
			else
			{
				settings.offsets[idx][clkdiv] = -1*(dist_sum/n_valids) + 75;
				__DSB(); __ISB();
				break;
			}

			delay_ms(70); // Let the LED dies cool down a bit - we never do super-long bursts on a single camera in the actual usage, either.
		}
		delay_ms(250);
	} 

	settings.offsets_at_temps[idx] = temperatures[idx];

	__DSB(); __ISB();
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


	This function works only on even rows. As it's usually used with the shortest HDR integration, when using dual integration mode,
	use the shorter time on even rows.

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
				int64_t ampl = ampl_in[pix_i]; if(ampl == 255) ampl = 20000;

				int estimate = (int64_t)100000000/( ampl * sq(dist) );

				/*
					Examples how the equation works out:
					ampl=255->20000, dist<=100->100:     0
					ampl=200         dist = 100:         50
					ampl=200         dist = 200:         12
					ampl=50          dist = 100:         200 (should really ring a bell)
					ampl=50          dist = 200:         50
					ampl=10          dist = 100:         1000 
					ampl=10          dist = 200:         250
					ampl=10          dist = 500:         40   (only slighly weird)
					ampl=10          dist = 1000:        10   (plausible)
					ampl=3           dist<=100->100:     3333 (biggest value)
				*/

				if(estimate > 500) estimate = 500;

				acc += estimate;
				n_acc++;
			}
		}
	}

	if(n_acc>200)
	{
		int amnt = acc/n_acc;
		if(amnt > 65535) amnt = 65535;
		*stray_ampl = amnt;
	}
	else
		*stray_ampl = 0;

	return n_acc;

}

static int err_cnt = 0;

int poll_capt_with_timeout()
{
	int timeout = 1600000; // 40000 tested to be barely ok with exposure time 125*5*5

	while(DMA2_Stream7->NDTR > 1 && timeout>0) timeout--;

	if(timeout == 0)
	{
		// Disable the stream
		DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ |
				   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

		DMA_CLEAR_INTFLAGS(DMA2, 7);

		err_cnt++;
		raspi_tx.dbg_i32[6] = err_cnt;
		if(err_cnt > 200)
		{
			error(13);
		}
		return 1;
	}

	return 0;
}


int poll_capt_with_timeout_complete()
{
	int timeout = 1600000;

	while(DMA2_Stream7->NDTR > 0 && timeout>0) timeout--;

	if(timeout == 0)
	{
		// Disable the stream
		DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ |
				   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

		DMA_CLEAR_INTFLAGS(DMA2, 7);

		err_cnt++;
		raspi_tx.dbg_i32[6] = err_cnt;
		if(err_cnt > 200)
		{
			error(14);
		}
		return 1;
	}

	return 0;
}


/*

	Optimization effort:

	STARTING POINT
0:0.0 1:0.6 2:5.3 3:5.7 4:6.9 5:10.2 6:18.7 7:22.8 8:38.8 9:39.2 10:42.2 11:50.8 12:59.4 13:59.8 14:71.4 15:71.4 16:84.6 17:96.1 18:97.2 19:103.6 20:0.0 21:148.4 22:480.8 23:827.0 
Time deltas to:
>1:0.6 >2:4.7 >3:0.4 >4:1.2 >5:3.3 >6:8.5 >7:4.1 >8:16.0 >9:0.4 >10:3.0 >11:8.6 >12:8.6 >13:0.4 >14:11.6 >15:0.0 >16:13.2 >17:11.5 >18:1.1 >19:6.4

	With tof_calc_dist_ampl removed (nonfunctional)

Frame read ok, timing:
0:0.0 1:0.6 2:5.3 3:5.7 4:6.9 5:10.2 6:18.7 7:22.7 8:38.7 9:39.1 10:42.3 11:50.7 12:59.3 13:59.7 14:59.7 15:68.6 16:81.8 17:81.8 18:82.0 19:85.5 20:0.0 21:148.4 22:480.8 23:827.0 
Time deltas to:
>1:0.6 >2:4.7 >3:0.4 >4:1.2 >5:3.3 >6:8.5 >7:4.0 >8:16.0 >9:0.4 >10:3.2 >11:8.4 >12:8.6 >13:0.4 >14:0.0 >15:8.9 >16:13.2 >17:0.0 >18:0.2 >19:3.5

	STARTING POINT
0:0.0 1:0.6 2:5.3 3:5.7 4:6.9 5:10.2 6:18.7 7:22.8 8:38.8 9:39.2 10:42.2 11:50.8 12:59.4 13:59.8 14:71.4 15:71.4 16:84.6 17:96.1 18:97.2 19:103.6 20:0.0 21:148.4 22:480.8 23:827.0 
Time deltas to:
>1:0.6 >2:4.7 >3:0.4 >4:1.2 >5:3.3 >6:8.5 >7:4.1 >8:16.0 >9:0.4 >10:3.0 >11:8.6 >12:8.6 >13:0.4 >14:11.6 >15:0.0 >16:13.2 >17:11.5 >18:1.1 >19:6.4

	DCMI freq up from 26.6 to 40 MHz (max)


Frame read ok, timing:
0:0.0 1:0.6 2:5.3 3:5.7 4:6.9 5:10.2 6:18.7 7:22.6 8:38.7 9:39.1 10:42.0 11:50.6 12:59.3 13:59.6 14:71.3 15:71.3 16:84.5 17:95.6 18:96.9 19:102.5 20:0.0 21:148.4 22:480.8 23:827.0 
Time deltas to:
>1:0.6 >2:4.7 >3:0.4 >4:1.2 >5:3.3 >6:8.5 >7:3.9 >8:16.1 >9:0.4 >10:2.9 >11:8.6 >12:8.7 >13:0.3 >14:11.7 >15:0.0 >16:13.2 >17:11.1 >18:1.3 >19:5.6

	No effect


	Analysis:

	calc_interference_ignore_from_2dcs (>4 1.2ms) is not a bottleneck - polling takes (>5 3.3ms).
	i2c writes for the next acquisition are nonoptimally placed, EPC supports some commands to be applied while acquiring.
	Not expecting a big effect (for example, >3:0.4 ms), but let's do that anyway


Frame read ok, timing:
0:0.0 1:0.6 2:5.3 3:5.6 4:7.0 5:10.0 6:18.2 7:21.9 8:37.7 9:37.9 10:41.2 11:49.4 12:57.8 13:57.9 14:69.9 15:69.9 16:82.9 17:94.2 18:95.1 19:100.8 20:0.0 21:148.4 22:480.8 23:827.0 
Time deltas to:
>1:0.6 >2:4.7 >3:0.3 >4:1.4 >5:3.0 >6:8.2 >7:3.7 >8:15.8 >9:0.2 >10:3.3 >11:8.2 >12:8.4 >13:0.1 >14:12.0 >15:0.0 >16:13.0 >17:11.3 >18:0.9 >19:5.7 


	Got about 3ms of savings at this point.

	Moving timestamps to polls to find bottlenecks

Frame read ok, timing:
0:0.9 1:5.3 2:7.0 3:10.0 4:11.5 5:18.2 6:18.7 7:21.9 8:22.5 9:37.7 10:40.3 11:49.4 12:52.3 13:57.8 14:69.7 15:69.7 16:81.4 17:82.8 18:0.0 19:0.0 20:0.0 21:148.4 22:480.8 23:827.0 
Time deltas to:
>1:4.4 >2:1.7 >3:3.0 >4:1.5 >5:6.7 >6:0.5 >7:3.2 >8:0.6 >9:15.2 >10:2.6 >11:9.1 >12:2.9 >13:5.5 >14:11.9 >15:0.0 >16:11.7 >17:1.4 
   """           """           """           """           """"             """             """              """              """


Conclusion: >15 is the only place that can be optimized (tof_calc_dist_ampl, while the mid-length exposure is being taken. The potential is only a few ms.)


	tof_calc_dist_ampl takes up to 11.5ms (moving the camera trying to find maximum)

	-> try optimizing it a bit

	EPC is configured to saturate the data, so that we don't need to look at the saturation flags anymore:

	Down to about 11.1ms

	atan LUT moved to dtcm -> no difference, not even when measured during acquisition (so DMA to main SRAM is not a big penalty after all)


	Moved tof_calc_dist_ampl to itcm -> no difference (meaning that ART accelerator just works)

	amplitude calculation is taking the time: 5.1ms without!
	With just sqrt removed (kept (x^2+y^2)/n): 5.5ms
	sqrt is the problem
	-> Fixed by approximation, seems good enough, really
	This approximation slightly overestimates values where dcs31 and dcs20 are in the midrange - these values are good, so this overestimation plays for us.


0:0.0 1:79.8 2:91.4

79.8 ms to the end of the last capture,
11.6ms processing after that to combine everything.

	Nonmeasurable improvement by stopping polling when DMA data left count is 1.

	Upping the readout freq to 80MHz didn't improve anything, still 79.8 ms, this is confusing.
	Now there was some random data corruption visible on the 40cm FFC.


	Upgrading the sequencer took from 79.8ms to 79.5ms

0:0.1 1:79.6 2:93.0
	--> new, shorter sequence:
0:0.0 1:53.1 2:64.3

	--> could make the actual exposure longer
	(was considering an additional 10MHz set, because lower freq gives more usable LED power, but it makes more sense to just extend the existing exposure)

0:0.1 1:58.0 2:73.5


*/

void epc_run()
{
	int idx = 0;
	int cnt = 0;

	while(1)
	{
#ifdef SEND_EXTRA
		raspi_tx.dbg_id = raspi_rx[4];
#endif
		raspi_tx.sensor_idx = idx;

		raspi_tx.timestamps[10] = settings.offsets_at_temps[idx];
		raspi_tx.timestamps[11] = settings.offsets[idx][1];
		raspi_tx.timestamps[12] = settings.offsets[idx][2];
		raspi_tx.timestamps[13] = settings.offsets[idx][3];


		timer_10k = 0;

		raspi_tx.status = 60;

		memset(ignore, 0, sizeof(ignore));
		epc_select(idx);

		/*
			20.0 MHz
			LEDS OFF
			2DCS
			Short exp
			Purpose: to find other HF AC sources or other interferences to be ignored
		*/

								raspi_tx.timestamps[0] = timer_10k;
		epc_clk_div(idx, 1);
		while(epc_i2c_is_busy(buses[idx]));

		epc_dis_leds(idx);
		while(epc_i2c_is_busy(buses[idx]));

		epc_2dcs(idx);
		while(epc_i2c_is_busy(buses[idx]));

		epc_intlen(idx, 8, 600);
		while(epc_i2c_is_busy(buses[idx]));
		;


		dcmi_start_dma(&dcsa, SIZEOF_2DCS);
		trig(idx);
		LED_ON();
		epc_intlen(idx, 3, 200);      // FOR THE NEXT, SEE BELOW
		while(epc_i2c_is_busy(buses[idx]));
		if(poll_capt_with_timeout()) continue;
		LED_OFF();


		/*
			6.66 MHz
			LEDS OFF
			2DCS
			Short exp
			Purpose: Same as the previous, at different freq
		*/
		epc_clk_div(idx, 3);
		while(epc_i2c_is_busy(buses[idx]));

		dcmi_start_dma(&dcsb, SIZEOF_2DCS);
		trig(idx);
		LED_ON();


		// Calculate the previous
		calc_interference_ignore_from_2dcs(ignore, &dcsa, 60);

#ifdef SEND_EXTRA
		if(raspi_rx[4] == 1) memcpy(raspi_tx.dbg, ignore, sizeof(ignore));
#endif

		epc_intlen(idx, 8, SHORTEST_INTEGRATION*HDR_EXP_MULTIPLIER*HDR_EXP_MULTIPLIER/2);  // For the next
		while(epc_i2c_is_busy(buses[idx]));

		if(poll_capt_with_timeout()) continue;
		LED_OFF();

		raspi_tx.status = 45;



		/*
			6.66 MHz (wrap at 22.5m)
			LEDS ON
			2DCS
			Long exp
			Purpose: Long-distance approximate readings for ignoring too-far points.
		*/

		epc_ena_leds(idx);
		while(epc_i2c_is_busy(buses[idx]));


		dcmi_start_dma(&dcsa, SIZEOF_2DCS);
		trig(idx);
		LED_ON();

		// Calculate the previous
		calc_interference_ignore_from_2dcs(ignore, &dcsb, 60);

#ifdef SEND_EXTRA
		if(raspi_rx[4] == 2) memcpy(raspi_tx.dbg, ignore, sizeof(ignore));
#endif


		epc_greyscale(idx);	// FOR THE NEXT
		while(epc_i2c_is_busy(buses[idx]));
		epc_intlen(idx, 8, SHORTEST_INTEGRATION*HDR_EXP_MULTIPLIER); // FOR THE NEXT
		while(epc_i2c_is_busy(buses[idx]));

		if(poll_capt_with_timeout()) continue;
		LED_OFF();

		raspi_tx.status = 37;


		/*
			20 MHz
			LEDS OFF
			MONOCHROME
			Mid exp
			Purpose: Ambient light compensation monochrome; also converts the temperature sensor reading

			Note: The temperature readout has a weird implementation: first we do some bit magic on two otherwise unspecified registers,
			then we need to actually capture a B&W image (which is needed anyway, luckily), and as a side effect, this image will have 2.5 times
			less sensitivity than if it was taken without the temperature measurement active. Again, luckily, this image is for BG illumination
			compensation only, which is rather inaccurate anyway, so reduced sensitivity / increased SNR is not an issue.
		*/

		epc_clk_div(idx, 1);
		while(epc_i2c_is_busy(buses[idx]));

		epc_temperature_magic_mode(idx);

		dcmi_start_dma(&mono_comp, SIZEOF_MONO);

		trig(idx);
		LED_ON();

		// Calculate the previous
		calc_toofar_ignore_from_2dcs(ignore, &dcsa, 5500, settings.offsets[idx][3], 3);

#ifdef SEND_EXTRA
		if(raspi_rx[4] == 3) memcpy(raspi_tx.dbg, ignore, sizeof(ignore));
#endif



		epc_4dcs(idx); // for the next
		while(epc_i2c_is_busy(buses[idx]));
		epc_intlen(idx, 8, SHORTEST_INTEGRATION); // for the next
		while(epc_i2c_is_busy(buses[idx]));

		if(poll_capt_with_timeout_complete()) continue;
		LED_OFF();

		int32_t temp = epc_read_temperature_regs(idx);

		temperatures[idx] = ((float)(temp-0x2000)*0.134 + epc_tempsens_factory_offset[idx])*10.0;
		raspi_tx.timestamps[20] = temperatures[idx];

		epc_temperature_magic_mode_off(idx);

		int tempcomp = ((float)(temperatures[idx] - settings.offsets_at_temps[idx])/10.0)  * 17.0 /*mm per 1 degC*/;




		/*
			20 MHz (wrap at 7.5m)
			LEDS ON
			4DCS
			Short exp, Mid exp in dual integration mode; or just the short exp
			Purpose: The real thing starts here! Shortest of the three HDR aqcuisition. Shortest first, because
			it'll include the closest objects, and the closest are likely to move most during the
			whole process -> minimize their motion blur, i.e., the ignore list is fairly recent now.
		*/

		static uint8_t  actual_ampl[3*EPC_XS*EPC_YS];
		static uint16_t actual_dist[3*EPC_XS*EPC_YS];

		dcmi_start_dma(&dcsa, SIZEOF_4DCS);
		trig(idx);
		LED_ON();

#ifdef SEND_EXTRA
		if(raspi_rx[4] == 4) memcpy(raspi_tx.dbg, mono_comp.img, sizeof(ignore));
#endif


		epc_intlen(idx, 8, SHORTEST_INTEGRATION*HDR_EXP_MULTIPLIER); // for the next
		while(epc_i2c_is_busy(buses[idx]));

		if(poll_capt_with_timeout()) continue;

		raspi_tx.status = 30;

		LED_OFF();


		/*
			20 MHz (wrap at 7.5m)
			LEDS ON
			4DCS
			Mid exp (if not in dual int mode)
			Purpose: The real thing continues.
		*/


		dcmi_start_dma(&dcsb, SIZEOF_4DCS);
		trig(idx);
		LED_ON();

		// Calc the previous:
		tof_calc_dist_ampl(&actual_ampl[0*EPC_XS*EPC_YS], &actual_dist[0*EPC_XS*EPC_YS], &dcsa, settings.offsets[idx][1]-tempcomp, 1);
		epc_intlen(idx, 8, SHORTEST_INTEGRATION*HDR_EXP_MULTIPLIER*HDR_EXP_MULTIPLIER); // for the next
		while(epc_i2c_is_busy(buses[idx]));

		if(poll_capt_with_timeout()) continue;

		LED_OFF();

		/*
			20 MHz (wrap at 7.5m)
			LEDS ON
			4DCS
			Long exp
			Purpose: The real thing continues: the last shot.
		*/


		dcmi_start_dma(&dcsa, SIZEOF_4DCS);

		trig(idx);
		LED_ON();

		// This is a great time to copy the latest robot pose.
		__disable_irq();
		__DSB(); __ISB();
		raspi_tx.robot_pos = latest_pos;
		__DSB(); __ISB();
		__enable_irq();


		// Calculate the previous
		// tof_calc_dist_ampl works with the dual-integration data as well, it processes pixel-by-pixel

		tof_calc_dist_ampl(&actual_ampl[1*EPC_XS*EPC_YS], &actual_dist[1*EPC_XS*EPC_YS], &dcsb, settings.offsets[idx][1]-tempcomp, 1);

		if(poll_capt_with_timeout()) continue;

		LED_OFF();

		raspi_tx.status = 15;
		raspi_tx.timestamps[1] = timer_10k;


		// All captures done.

		// Calculate the last measurement:

		tof_calc_dist_ampl(&actual_ampl[2*EPC_XS*EPC_YS], &actual_dist[2*EPC_XS*EPC_YS], &dcsa, settings.offsets[idx][1]-tempcomp, 1);


#ifdef SEND_EXTRA
		if(raspi_rx[4] == 5)  memcpy(raspi_tx.dbg, &actual_ampl[0], 1*160*60);
		if(raspi_rx[4] == 6)  memcpy(raspi_tx.dbg, &actual_dist[0], 2*160*60);
		if(raspi_rx[4] == 7)  memcpy(raspi_tx.dbg, &actual_ampl[1*EPC_XS*EPC_YS], 1*160*60);
		if(raspi_rx[4] == 8)  memcpy(raspi_tx.dbg, &actual_dist[1*EPC_XS*EPC_YS], 2*160*60);
		if(raspi_rx[4] == 9)  memcpy(raspi_tx.dbg, &actual_ampl[2*EPC_XS*EPC_YS], 1*160*60);
		if(raspi_rx[4] == 10) memcpy(raspi_tx.dbg, &actual_dist[2*EPC_XS*EPC_YS], 2*160*60);
#endif

		raspi_tx.status = 10;


		uint16_t stray_ampl, stray_dist, outstray_n, outstray_ampl;

		calc_stray_estimate(&actual_ampl[0*EPC_XS*EPC_YS], &actual_dist[0*EPC_XS*EPC_YS], &stray_ampl, &stray_dist);

		outstray_n = calc_outside_stray_estimate(&actual_ampl[2*EPC_XS*EPC_YS], &actual_dist[2*EPC_XS*EPC_YS], &outstray_ampl);
		
		raspi_tx.dbg_i32[0] = stray_ampl;
		raspi_tx.dbg_i32[1] = stray_dist;

		raspi_tx.dbg_i32[2] = outstray_ampl;
		raspi_tx.dbg_i32[3] = outstray_n;


		// Then combine everything:
		

		raspi_tx.status = 7;

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

			static uint16_t combined_depth[EPC_XS*EPC_YS];

			tof_calc_dist_3hdr_with_ignore_with_straycomp(combined_depth, actual_ampl, actual_dist, ignore, combined_stray_ampl, combined_stray_dist);
			tof_remove_midliers(raspi_tx.depth, combined_depth);
		}

								raspi_tx.timestamps[2] = timer_10k;

		raspi_tx.status = 255;
		// Bravely calculate the HDR amplitude image while DMA transfers the result
		tof_calc_ampl_hdr(raspi_tx.ampl, &actual_ampl[2*EPC_XS*EPC_YS], &actual_ampl[1*EPC_XS*EPC_YS]);

#ifdef SEND_EXTRA
		while(timer_10k < ((idx==N_SENSORS-1)?3000:1500)) // 1.33 FPS
#else
//		while(timer_10k < ((idx==N_SENSORS-1)?1500:800)) // 2.6 FPS
//		while(timer_10k < ((idx==N_SENSORS-1)?2500:800)) // 2.04 FPS
		while(timer_10k < ((idx==N_SENSORS-1)?2000:900))
#endif
		{
			if(new_rx)
			{
				new_rx = 0;
				if(new_rx_len > 8)
					raspi_tx.status = 80;

				if(*((volatile uint32_t*)&raspi_rx[0]) == 0xca0ff5e7) // offset calibration cmd
				{
					raspi_tx.status = 100;
					raspi_tx.dbg_i32[7] = 99999;

//					*((volatile uint32_t*)&raspi_rx[0]) = 0; // zero it out so that we don't do it again
					uint8_t sensor_idx = raspi_rx[4];
					__DSB(); __ISB();
					int ret = 999;
					if(sensor_idx < N_SENSORS)
						ret = run_offset_cal(sensor_idx);
					raspi_tx.dbg_i32[7] = ret;
				}

			}
		}

		cnt++;

		idx++;
		if(idx >= N_SENSORS) idx = 0;
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
	// Triggered when cs goes high

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

	PULUTOF1 MASTER is the SPI Slave, RobotBoard is Master.
*/

void robot_spi_xfer_end_inthandler()
{
	// Triggered when cs goes high
	EXTI->PR = 1UL<<6; // Clear "pending bit".

	__DSB(); __ISB();
	latest_pos = robot_to_pulutof.robot_pos;
	__DSB(); __ISB();


	// Disable the DMAs:
	DMA2_Stream4->CR = 2UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA2_Stream3->CR = 2UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	while(DMA2_Stream4->CR & 1UL) ;
	while(DMA2_Stream3->CR & 1UL) ;

	robot_new_rx = 1;
	robot_new_rx_len = sizeof(robot_to_pulutof) - DMA2_Stream3->NDTR;

	// Hard-reset SPI - the only way to empty TXFIFO! (Go figure.)

	RCC->APB2RSTR = 1UL<<20;
	__asm__ __volatile__ ("nop");
	RCC->APB2RSTR = 0;
			
	// Re-enable:

	SPI5->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/ | 1UL<<12 /*Don't Reject The Last Byte*/;

	// TX DMA
	DMA2_Stream4->NDTR = sizeof(pulutof_to_robot);
	DMA_CLEAR_INTFLAGS(DMA2, 4);
	DMA2_Stream4->CR |= 1; // Enable DMA

	// RX DMA

	DMA2_Stream3->NDTR = sizeof(robot_to_pulutof);
	DMA_CLEAR_INTFLAGS(DMA2, 3);
	DMA2_Stream3->CR |= 1; // Enable DMA

	SPI5->CR2 |= 1UL<<1 /*TX DMA ena*/; // not earlier!

	SPI5->CR1 = 1UL<<6; // Enable in slave mode

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

	NVIC_SetPriority(TIM5_IRQn, 0b1111);
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

	// Raspi SPI: SLAVE

	IO_ALTFUNC(GPIOB, 12, 5);
	IO_ALTFUNC(GPIOB, 13, 5);
	IO_ALTFUNC(GPIOB, 14, 5);
	IO_ALTFUNC(GPIOB, 15, 5);

	IO_SPEED(GPIOB, 14, 3); // MISO pin gets the highest speed.



	init_raspi_tx();
	__DSB(); __ISB();

	/*
		STM32 TRAP WARNING:
		FRXTH (bit 12) in SPI->CR2 needs to be set for the SPI to work in a sane way. Otherwise, DMA
		requests for the last transfer never come, and the last data cannot be never read in the specified way!
		I can't figure out any reason to use such mode, and it's the default. So remember to set this bit.
	*/

	// Initialization order from reference manual:
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


	// Robot SPI: SLAVE

	IO_ALTFUNC(GPIOF, 6, 5);
	IO_ALTFUNC(GPIOF, 7, 5);
	IO_ALTFUNC(GPIOF, 8, 5);
	IO_ALTFUNC(GPIOF, 9, 5);

	// Use the second-highest speed
	IO_SPEED(GPIOF, 8, 2); // MISO


	// Initialization order from reference manual:
	SPI5->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/ | 1UL<<12 /*Don't Reject The Last Byte*/;

	// TX DMA
	DMA2_Stream4->PAR = (uint32_t)&(SPI5->DR);
	DMA2_Stream4->M0AR = (uint32_t)&pulutof_to_robot;
	DMA2_Stream4->CR = 2UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA2_Stream4->NDTR = sizeof(raspi_tx);
	DMA_CLEAR_INTFLAGS(DMA2, 4);
	DMA2_Stream4->CR |= 1; // Enable TX DMA

	// RX DMA

	DMA2_Stream3->PAR = (uint32_t)&(SPI5->DR);
	DMA2_Stream3->M0AR = (uint32_t)&robot_to_pulutof;
	DMA2_Stream3->CR = 2UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;
	DMA2_Stream3->NDTR = sizeof(raspi_rx);
	DMA_CLEAR_INTFLAGS(DMA2, 3);
	DMA2_Stream3->CR |= 1; // Enable RX DMA

	SPI5->CR2 |= 1UL<<1 /*TX DMA ena*/; // not earlier!

	SPI5->CR1 = 1UL<<6; // Enable in slave mode

	// Chip select is hardware managed for rx start - but ending must be handled by software. We use EXTI for that.

	// nCS is PF6, so EXTI6 must be used.
	SYSCFG->EXTICR[/*refman idx:*/2   -1] = 0b0101UL<<8; // PORTF used for EXTI6.
	IO_PULLUP_ON(GPIOF, 6); // Pull the nCS up to avoid glitches
	EXTI->IMR |= 1UL<<6;
	EXTI->RTSR |= 1UL<<6; // Get the rising edge interrupt
	// The interrupt priority must be fairly high, to quickly reconfigure the DMA so that if the master pulls CSn low
	// again very quickly to make a new transaction, we have the DMA up and running for that before the super small 4-byte
	// FIFO in the SPI is exhausted.
	NVIC_SetPriority(EXTI9_5_IRQn, 0b0101);
	NVIC_EnableIRQ(EXTI9_5_IRQn);


	top_init();
	epc_run();
}
