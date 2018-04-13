#include <stdint.h>

#include "ext_include/stm32f7xx.h"

void stm32init();
void nmi_handler();
void invalid_handler();
void hardfault_handler();

extern void error(int code);
extern void main();

extern void epc01_i2c_inthandler();
extern void epc23_i2c_inthandler();
extern void epc_dcmi_dma_inthandler();
extern void epc_shutdown_inthandler();
extern void epc01_rx_dma_inthandler();
extern void epc23_rx_dma_inthandler();
extern void raspi_spi_xfer_end_inthandler();
extern void robot_spi_xfer_end_inthandler();
extern void timebase_handler();
extern unsigned int _STACKTOP;

// Vector table on page 311 on the Reference Manual RM0410
unsigned int * the_nvic_vector[126] __attribute__ ((section(".nvic_vector"))) =
{
/* 0x0000                    */ (unsigned int *) &_STACKTOP,
/* 0x0004 RESET              */ (unsigned int *) stm32init,
/* 0x0008 NMI                */ (unsigned int *) nmi_handler,
/* 0x000C HARDFAULT          */ (unsigned int *) hardfault_handler,
/* 0x0010                    */ (unsigned int *) invalid_handler,
/* 0x0014                    */ (unsigned int *) invalid_handler,
/* 0x0018                    */ (unsigned int *) invalid_handler,
/* 0x001C                    */ (unsigned int *) invalid_handler,
/* 0x0020                    */ (unsigned int *) invalid_handler,
/* 0x0024                    */ (unsigned int *) invalid_handler,
/* 0x0028                    */ (unsigned int *) invalid_handler,
/* 0x002C                    */ (unsigned int *) invalid_handler,
/* 0x0030                    */ (unsigned int *) invalid_handler,
/* 0x0034                    */ (unsigned int *) invalid_handler,
/* 0x0038                    */ (unsigned int *) invalid_handler,
/* 0x003C                    */ (unsigned int *) invalid_handler,
/* 0x0040                    */ (unsigned int *) invalid_handler,
/* 0x0044 PVD (volt detector)*/ (unsigned int *) epc_shutdown_inthandler,
/* 0x0048                    */ (unsigned int *) invalid_handler,
/* 0x004C                    */ (unsigned int *) invalid_handler,
/* 0x0050                    */ (unsigned int *) invalid_handler,
/* 0x0054                    */ (unsigned int *) invalid_handler,
/* 0x0058                    */ (unsigned int *) invalid_handler,
/* 0x005C                    */ (unsigned int *) invalid_handler,
/* 0x0060                    */ (unsigned int *) invalid_handler,
/* 0x0064                    */ (unsigned int *) invalid_handler,
/* 0x0068                    */ (unsigned int *) invalid_handler,
/* 0x006C DMA1_Stream0       */ (unsigned int *) invalid_handler,
/* 0x0070 DMA1_Stream1       */ (unsigned int *) epc23_rx_dma_inthandler,
/* 0x0074 DMA1_Stream2       */ (unsigned int *) epc01_rx_dma_inthandler,
/* 0x0078 DMA1_Stream3       */ (unsigned int *) invalid_handler,
/* 0x007C DMA1_Stream4       */ (unsigned int *) invalid_handler,
/* 0x0080 DMA1_Stream5       */ (unsigned int *) invalid_handler,
/* 0x0084 DMA1_Stream6       */ (unsigned int *) invalid_handler,
/* 0x0088 ADC                */ (unsigned int *) invalid_handler,
/* 0x008C                    */ (unsigned int *) invalid_handler,
/* 0x0090                    */ (unsigned int *) invalid_handler,
/* 0x0094                    */ (unsigned int *) invalid_handler,
/* 0x0098                    */ (unsigned int *) invalid_handler,
/* 0x009C EXTI5..9           */ (unsigned int *) robot_spi_xfer_end_inthandler,
/* 0x00A0                    */ (unsigned int *) invalid_handler,
/* 0x00A4 TIM1_UP_TIM10      */ (unsigned int *) invalid_handler,
/* 0x00A8                    */ (unsigned int *) invalid_handler,
/* 0x00AC                    */ (unsigned int *) invalid_handler,
/* 0x00B0                    */ (unsigned int *) invalid_handler,
/* 0x00B4                    */ (unsigned int *) invalid_handler,
/* 0x00B8                    */ (unsigned int *) invalid_handler,
/* 0x00BC I2C1 event         */ (unsigned int *) invalid_handler,
/* 0x00C0 I2C1 error         */ (unsigned int *) invalid_handler,
/* 0x00C4 I2C2 event         */ (unsigned int *) invalid_handler,
/* 0x00C8 I2C2 error         */ (unsigned int *) invalid_handler,
/* 0x00CC                    */ (unsigned int *) invalid_handler,
/* 0x00D0                    */ (unsigned int *) invalid_handler,
/* 0x00D4                    */ (unsigned int *) invalid_handler,
/* 0x00D8                    */ (unsigned int *) invalid_handler,
/* 0x00DC                    */ (unsigned int *) invalid_handler,
/* 0x00E0 EXTI 10..15        */ (unsigned int *) raspi_spi_xfer_end_inthandler,
/* 0x00E4                    */ (unsigned int *) invalid_handler,
/* 0x00E8                    */ (unsigned int *) invalid_handler,
/* 0x00EC                    */ (unsigned int *) invalid_handler,
/* 0x00F0                    */ (unsigned int *) invalid_handler,
/* 0x00F4                    */ (unsigned int *) invalid_handler,
/* 0x00F8                    */ (unsigned int *) invalid_handler,
/* 0x00FC                    */ (unsigned int *) invalid_handler,
/* 0x0100                    */ (unsigned int *) invalid_handler,
/* 0x0104                    */ (unsigned int *) invalid_handler,
/* 0x0108 TIM5               */ (unsigned int *) timebase_handler,
/* 0x010C                    */ (unsigned int *) invalid_handler,
/* 0x0110                    */ (unsigned int *) invalid_handler,
/* 0x0114                    */ (unsigned int *) invalid_handler,
/* 0x0118                    */ (unsigned int *) invalid_handler,
/* 0x011C                    */ (unsigned int *) invalid_handler,
/* 0x0120 DMA2_Stream0       */ (unsigned int *) invalid_handler,
/* 0x0124 DMA2_Stream1       */ (unsigned int *) invalid_handler,
/* 0x0128 DMA2_Stream2       */ (unsigned int *) invalid_handler,
/* 0x012C DMA2_Stream3       */ (unsigned int *) invalid_handler,
/* 0x0130 DMA2_Stream4       */ (unsigned int *) invalid_handler,
/* 0x0134                    */ (unsigned int *) invalid_handler,
/* 0x0138                    */ (unsigned int *) invalid_handler,
/* 0x013C                    */ (unsigned int *) invalid_handler,
/* 0x0140                    */ (unsigned int *) invalid_handler,
/* 0x0144                    */ (unsigned int *) invalid_handler,
/* 0x0148                    */ (unsigned int *) invalid_handler,
/* 0x014C                    */ (unsigned int *) invalid_handler,
/* 0x0150 DMA2_Stream5       */ (unsigned int *) invalid_handler,
/* 0x0154 DMA2_Stream6       */ (unsigned int *) invalid_handler,
/* 0x0158 DMA2_Stream7       */ (unsigned int *) epc_dcmi_dma_inthandler,
/* 0x015C                    */ (unsigned int *) invalid_handler,
/* 0x0160 I2C3 event         */ (unsigned int *) epc01_i2c_inthandler,
/* 0x0164 I2C3 error         */ (unsigned int *) invalid_handler,
/* 0x0168                    */ (unsigned int *) invalid_handler,
/* 0x016C                    */ (unsigned int *) invalid_handler,
/* 0x0170                    */ (unsigned int *) invalid_handler,
/* 0x0174                    */ (unsigned int *) invalid_handler,
/* 0x0178                    */ (unsigned int *) invalid_handler,
/* 0x017C                    */ (unsigned int *) invalid_handler,
/* 0x0180                    */ (unsigned int *) invalid_handler,
/* 0x0184                    */ (unsigned int *) invalid_handler,
/* 0x0188                    */ (unsigned int *) invalid_handler,
/* 0x018C                    */ (unsigned int *) invalid_handler,
/* 0x0190                    */ (unsigned int *) invalid_handler,
/* 0x0194                    */ (unsigned int *) invalid_handler,
/* 0x0198                    */ (unsigned int *) invalid_handler,
/* 0x019C                    */ (unsigned int *) invalid_handler,
/* 0x01A0                    */ (unsigned int *) invalid_handler,
/* 0x01A4                    */ (unsigned int *) invalid_handler,
/* 0x01A8                    */ (unsigned int *) invalid_handler,
/* 0x01AC                    */ (unsigned int *) invalid_handler,
/* 0x01B0                    */ (unsigned int *) invalid_handler,
/* 0x01B4                    */ (unsigned int *) invalid_handler,
/* 0x01B8                    */ (unsigned int *) invalid_handler,
/* 0x01BC I2C4 event         */ (unsigned int *) epc23_i2c_inthandler,
/* 0x01C0 I2C4 error         */ (unsigned int *) invalid_handler,
/* 0x01C4                    */ (unsigned int *) invalid_handler,
/* 0x01C8                    */ (unsigned int *) invalid_handler,
/* 0x01CC                    */ (unsigned int *) invalid_handler,
/* 0x01D0                    */ (unsigned int *) invalid_handler,
/* 0x01D4                    */ (unsigned int *) invalid_handler,
/* 0x01D8                    */ (unsigned int *) invalid_handler,
/* 0x01DC                    */ (unsigned int *) invalid_handler,
/* 0x01E0                    */ (unsigned int *) invalid_handler,
/* 0x01E4                    */ (unsigned int *) invalid_handler,
/* 0x01E8                    */ (unsigned int *) invalid_handler,
/* 0x01EC                    */ (unsigned int *) invalid_handler,
/* 0x01F0                    */ (unsigned int *) invalid_handler,
/* 0x01F4                    */ (unsigned int *) invalid_handler
};

extern unsigned int _BSS_BEGIN;
extern unsigned int _BSS_END;

extern unsigned int _DATA_BEGIN;
extern unsigned int _DATA_END;
extern unsigned int _DATAI_BEGIN;

extern unsigned int _DTCM_DATA_BEGIN;
extern unsigned int _DTCM_DATA_END;
extern unsigned int _DTCM_DATA_I_BEGIN;

extern unsigned int _SETTINGS_BEGIN;
extern unsigned int _SETTINGS_END;
extern unsigned int _SETTINGS_I_BEGIN;

extern unsigned int _DTCM_BSS_BEGIN;
extern unsigned int _DTCM_BSS_END;

extern unsigned int _TEXT_ITCM_BEGIN;
extern unsigned int _TEXT_ITCM_END;
extern unsigned int _TEXT_ITCM_I_BEGIN;


extern void hwtest_main();

void refresh_settings()
{
	volatile uint32_t* settings_begin  = (volatile uint32_t*)&_SETTINGS_BEGIN;
	volatile uint32_t* settings_end    = (volatile uint32_t*)&_SETTINGS_END;
	volatile uint32_t* settings_i_begin = (volatile uint32_t*)&_SETTINGS_I_BEGIN;

	while(settings_begin < settings_end)
	{
		*settings_begin = *settings_i_begin;
		settings_begin++;
		settings_i_begin++;
	}
	__DSB(); __ISB();
}

void stm32init(void)
{
//	static volatile int i = 5000;
//	while(i--)
//		__asm__ __volatile__ ("nop");


	RCC->AHB1ENR |= 1UL<<20; // Enable DTCM RAM...

	uint32_t* bss_begin = (uint32_t*)&_BSS_BEGIN;
	uint32_t* bss_end   = (uint32_t*)&_BSS_END;
	while(bss_begin < bss_end)
	{
		*bss_begin = 0;
		bss_begin++;
	}

	uint32_t* dtcm_bss_begin = (uint32_t*)&_DTCM_BSS_BEGIN;
	uint32_t* dtcm_bss_end   = (uint32_t*)&_DTCM_BSS_END;
	while(dtcm_bss_begin < dtcm_bss_end)
	{
		*dtcm_bss_begin = 0;
		dtcm_bss_begin++;
	}


	uint32_t* data_begin  = (uint32_t*)&_DATA_BEGIN;
	uint32_t* data_end    = (uint32_t*)&_DATA_END;
	uint32_t* datai_begin = (uint32_t*)&_DATAI_BEGIN;

	while(data_begin < data_end)
	{
		*data_begin = *datai_begin;
		data_begin++;
		datai_begin++;
	}

	uint32_t* dtcm_data_begin  = (uint32_t*)&_DTCM_DATA_BEGIN;
	uint32_t* dtcm_data_end    = (uint32_t*)&_DTCM_DATA_END;
	uint32_t* dtcm_data_i_begin = (uint32_t*)&_DTCM_DATA_I_BEGIN;

	while(dtcm_data_begin < dtcm_data_end)
	{
		*dtcm_data_begin = *dtcm_data_i_begin;
		dtcm_data_begin++;
		dtcm_data_i_begin++;
	}

	refresh_settings();

	uint32_t* text_itcm_begin  = (uint32_t*)&_TEXT_ITCM_BEGIN;
	uint32_t* text_itcm_end    = (uint32_t*)&_TEXT_ITCM_END;
	uint32_t* text_itcm_i_begin = (uint32_t*)&_TEXT_ITCM_I_BEGIN;

	while(text_itcm_begin < text_itcm_end)
	{
		*text_itcm_begin = *text_itcm_i_begin;
		text_itcm_begin++;
		text_itcm_i_begin++;
	}

	main();
}


void nmi_handler(void)
{
	error(1);
}

void hardfault_handler(void)
{
	error(2);
}

void invalid_handler(void)
{
	error(3);
}
