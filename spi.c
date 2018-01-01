/*****************************************************************************
 *                                                                            *
 * DFU/SD/SDHC Bootloader for LPC17xx                                         *
 *                                                                            *
 * by Triffid Hunter                                                          *
 *                                                                            *
 *                                                                            *
 * This firmware is Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the GNU General Public License as published by       *
 * the Free Software Foundation; either version 2 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * GNU General Public License for more details.                               *
 *                                                                            *
 * You should have received a copy of the GNU General Public License          *
 * along with this program; if not, write to the Free Software                *
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA *
 *                                                                            *
 *****************************************************************************/

#define DBwrite(x)

#define SOFT_SPI

#define FORCE_SOFT_SPI      0
#define USE_HG42_freq       1
#define USE_LPCxLIB_freq    1
#define USE_LPCxLIB_init    1
#define USE_LPCxLIB_write   1
#define USE_LPCxLIB_write__read_until_rx_empty  1
#define USE_LPCxLIB_write__wait_tx_not_full     1
#define USE_LPCxLIB_write__clear_status         1
#define USE_LPCxLIB_write__overrun_exit         1

#define CLK_MAX_PERCENT     5

#if FORCE_SOFT_SPI && ! defined(SOFTSPI)
#define SOFT_SPI
#endif

#include "spi.h"

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_ssp.h"
//#include "lpc17xx_gpio.h"
#include "gpio.h"

#include "min-printf.h"

//#include "lpc17xx_spi.h"
#include "lpc17xx_clkpwr.h"
//#include "lpc17xx_libcfg_default.h"


#ifdef SOFT_SPI

// measure at 25000Hz (slow spi init frequence)
// change here to fit 40us/cycle, higher number -> longer delay
// calibration may depend on compiler options
// this is for -Os -g
#define SPI_SOFT_DELAY_CALIBRATION          6500000

#endif


static PinName spi_miso;
static PinName spi_mosi;
static PinName spi_sclk;
static uint32_t spi_frequency = 25000;  // default

static SPI_REG * ssp_device;
static uint32_t ssp_clk;


// SPI interface reimplemented by hg42

#if USE_HG42_freq && ! defined(IMPL_SPI_frequency) && ! FORCE_SOFT_SPI
#define IMPL_SPI_frequency HG42_SPI_frequency

static void HG42_SPI_frequency (uint32_t target_clock)
{
    uint32_t prescale, cr0_div, current_clock;

    printf("HG42_SPI_frequency\n");

    //CHECK_PARAM(PARAM_SSPx(SSPx));

    /* The SSP clock is derived from the (main system oscillator / 2),
       so compute the best divider from that clock */
    if (ssp_device == LPC_SSP0)
    	ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP0);
    else if (ssp_device == LPC_SSP1)
    	ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP1);
    else
    	return;

    /* Find closest divider to get at or under the target frequency.
       Use smallest prescale possible and rely on the divider to get
       the closest target frequency */
    cr0_div = 0;
    current_clock = 0xFFFFFFFF;
    prescale = 2;
    int32_t best_diff = target_clock + 1;
    uint32_t best_cr0_div, best_prescale;
    uint32_t max_clock = (target_clock / 100) * (100 + CLK_MAX_PERCENT);
    while (1)
    {
        current_clock = ssp_clk / ((cr0_div + 1) * prescale);
        int32_t diff = current_clock - target_clock;
        if(current_clock < max_clock)
        {
            if(diff < 0)
                diff = -diff;
            if(diff < best_diff)
            {
                best_prescale = prescale;
                best_cr0_div  = cr0_div;
                best_diff = diff;
                //printf("***** f=%lu sspclk=%lu prescale=%lu div=%lu -> %lu %ld (%ld)\n", target_clock, ssp_clk, prescale, cr0_div + 1, current_clock, diff, best_diff);
                if(!diff)
                    break;
            }
        }
        //printf("find: f=%lu sspclk=%lu prescale=%lu div=%lu -> %lu %ld (%ld)\n", target_clock, ssp_clk, prescale, cr0_div + 1, current_clock, diff, best_diff);
        cr0_div++;
        if (cr0_div > 0xFF)
        {
            cr0_div = 0;
            prescale += 2;
            if(prescale > SSP_CPSR_BITMASK)
                break;
        }
    }

    printf("freq: f=%lu sspclk=%lu prescale=%lu div=%lu -> %lu\n", target_clock, ssp_clk, prescale, cr0_div + 1, ssp_clk / ((best_cr0_div + 1) * best_prescale));

    /* Write computed prescaler and divider back to register */
    ssp_device->CR0  &= (~SSP_CR0_SCR(0xFF)) & SSP_CR0_BITMASK;
    ssp_device->CR0  |= (SSP_CR0_SCR(best_cr0_div)) & SSP_CR0_BITMASK;
    ssp_device->CPSR &= ~SSP_CPSR_BITMASK;
    ssp_device->CPSR |= best_prescale & SSP_CPSR_BITMASK;
}
#endif

// SPI interface imported from LPC17xx_ssp.c

#if USE_LPCxLIB_freq && ! defined(IMPL_SPI_frequency) && ! FORCE_SOFT_SPI
#define IMPL_SPI_frequency LPCxLIB_SPI_frequency

static void LPCxLIB_SPI_frequency (uint32_t target_clock)
{
    uint32_t prescale, cr0_div, cmp_clk;

    printf("LPCxLIB_SPI_frequency\n");

    //CHECK_PARAM(PARAM_SSPx(SSPx));

    /* The SSP clock is derived from the (main system oscillator / 2),
       so compute the best divider from that clock */
    if (ssp_device == LPC_SSP0)
    	ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP0);
    else if (ssp_device == LPC_SSP1)
    	ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP1);
    else
    	return;

    /* Find closest divider to get at or under the target frequency.
       Use smallest prescale possible and rely on the divider to get
       the closest target frequency */
    cr0_div = 0;
    cmp_clk = 0xFFFFFFFF;
    prescale = 2;
    int32_t best_diff = target_clock;
    uint32_t best_cr0_div, best_prescale;
    while (1)
    {
        cmp_clk = ssp_clk / ((cr0_div + 1) * prescale);
        int32_t diff = cmp_clk - target_clock;
        if (cmp_clk * 100 > target_clock * (100 + CLK_MAX_PERCENT))
        {
            cr0_div++;
            if (cr0_div > 0xFF)
            {
                cr0_div = 0;
                prescale += 2;
                if(prescale > SSP_CPSR_BITMASK)
                    break;
            }
        }
        else
        {
            best_prescale = prescale;
            best_cr0_div  = cr0_div;
            break;
        }
        //printf("find: f=%lu sspclk=%lu prescale=%lu div=%lu -> %lu %ld (%ld)\n", target_clock, ssp_clk, prescale, cr0_div + 1, cmp_clk, diff, best_diff);
    }

    printf("find: f=%lu sspclk=%lu prescale=%lu div=%lu -> %lu\n", target_clock, ssp_clk, prescale, cr0_div + 1, ssp_clk / ((best_cr0_div + 1) * best_prescale));

    /* Write computed prescaler and divider back to register */
    ssp_device->CR0  &= (~SSP_CR0_SCR(0xFF)) & SSP_CR0_BITMASK;
    ssp_device->CR0  |= (SSP_CR0_SCR(best_cr0_div)) & SSP_CR0_BITMASK;
    ssp_device->CPSR &= ~SSP_CPSR_BITMASK;
    ssp_device->CPSR |= best_prescale & SSP_CPSR_BITMASK;
}
#endif

#if USE_LPCxLIB_init && ! defined(IMPL_SPI_init) && ! FORCE_SOFT_SPI
#define IMPL_SPI_init LPCxLIB_SPI_init

static void LPCxLIB_SPI_init(void)
{
    uint32_t tmp;
    PINSEL_CFG_Type PinCfg;

    printf("LPCxLIB_SPI_init\n");

    // PinCfg.Funcnum = 3;
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;

    PinCfg.Portnum = PORT(spi_sclk);
    PinCfg.Pinnum  =  PIN(spi_sclk);
    PINSEL_ConfigPin(&PinCfg);

    PinCfg.Portnum = PORT(spi_miso);
    PinCfg.Pinnum  =  PIN(spi_miso);
    PINSEL_ConfigPin(&PinCfg);

    PinCfg.Portnum = PORT(spi_mosi);
    PinCfg.Pinnum  =  PIN(spi_mosi);
    PINSEL_ConfigPin(&PinCfg);

    SSP_CFG_Type cfg;
    cfg.CPHA        = SSP_CPHA_FIRST;
    cfg.CPOL        = SSP_CPOL_HI;
    cfg.ClockRate   = spi_frequency;
    cfg.Databit     = SSP_DATABIT_8;
    cfg.Mode        = SSP_MASTER_MODE;
    cfg.FrameFormat = SSP_FRAME_SPI;

    /* Configure SSP, interrupt is disable, LoopBack mode is disable,
     * SSP is disable, Slave output is disable as default
     */
    tmp = ((cfg.CPHA) | (cfg.CPOL) | (cfg.FrameFormat) | (cfg.Databit))
            & SSP_CR0_BITMASK;
    // write back to SSP control register
    ssp_device->CR0 &= ~SSP_CR0_BITMASK;
    ssp_device->CR0 |= tmp;

    tmp = cfg.Mode & SSP_CR1_BITMASK;
    // Write back to CR1
    ssp_device->CR1 &= ~SSP_CR1_BITMASK;
    ssp_device->CR1 |= tmp;

    // Set clock rate for SSP peripheral
    //LPCxLIB_SPI_frequency(ssp_device, cfg.ClockRate);
    SPI_frequency(cfg.ClockRate);   // if mixed, to work with all variants

    ssp_device->CR1 |= SSP_CR1_SSP_EN;
}
#endif

#if USE_LPCxLIB_write && ! defined(IMPL_SPI_write) && ! FORCE_SOFT_SPI
#define IMPL_SPI_write LPCxLIB_SPI_write

static inline void LPCxLIB_SPI_SendData(uint8_t data)
{
    ssp_device->DR = SSP_DR_BITMASK(data);
}

static inline uint8_t LPCxLIB_SPI_ReceiveData(void)
{
    return ((uint8_t) (SSP_DR_BITMASK(ssp_device->DR)));
}

static uint8_t LPCxLIB_SPI_write(uint8_t data)
{
    uint32_t stat;
    uint32_t tmp;

    #if USE_LPCxLIB_write__read_until_rx_empty
    /* Clear all remaining data in RX FIFO */
    while (ssp_device->SR & SSP_SR_RNE){
        tmp = (uint32_t) LPCxLIB_SPI_ReceiveData();
        printf("drop 0x%lx\n", tmp);
    }
    #endif

    #if USE_LPCxLIB_write__clear_status
    // Clear status
    ssp_device->ICR = SSP_ICR_BITMASK;
    #endif

    #if USE_LPCxLIB_write__wait_tx_not_full
    while (!(ssp_device->SR & SSP_SR_TNF)) ;
    #endif

    LPCxLIB_SPI_SendData(data);

    #if USE_LPCxLIB_write__overrun_exit
    // Check overrun error
    if ((stat = ssp_device->RIS) & SSP_RIS_ROR)
        return (-1);
    #endif

    // Check for any data available in RX FIFO
    while (!(ssp_device->SR & SSP_SR_RNE)) ;

    // Read data from SSP data
    return LPCxLIB_SPI_ReceiveData();
}

#endif


// SPI by hardware, minimal version ("classic")

#if ! defined(IMPL_SPI_frequency) && ! FORCE_SOFT_SPI
#define IMPL_SPI_frequency MINI_SPI_frequency

static void MINI_SPI_frequency(uint32_t f)
{
    printf("MINI_SPI_frequency\n");

    //ssp_clk = 25000000;
    ssp_clk = SystemCoreClock/4;
    uint32_t clk_per_cycle = ssp_clk / f;
    // f = 25MHz / (CPSR . [SCR + 1])
    // CPSR . (SCR + 1) = 25MHz / f
    // min freq is 25MHz / (254 * 256)
    if (ssp_device) {
        if (f < 385) {
            ssp_device->CPSR = 254;
            ssp_device->CR0 &= 0x00FF;
            ssp_device->CR0 |= 255 << 8;
        }
        // max freq is 25MHz / (2 * 1)
        else if (f > 12500000) {
            ssp_device->CPSR = 2;
            ssp_device->CR0 &= 0x00FF;
        }
        else {
            ssp_device->CPSR = clk_per_cycle & 0xFE;
            // CPSR . (SCR + 1) = f;
            // (SCR + 1) = f / CPSR;
            // SCR = (f / CPSR) - 1
            ssp_device->CR0 &= 0x00FF;
            ssp_device->CR0 |= (((clk_per_cycle / ssp_device->CPSR) - 1) & 0xFF) << 8;
        }
    }
}
#endif

#if ! defined(IMPL_SPI_init) && ! FORCE_SOFT_SPI
#define IMPL_SPI_init MINI_SPI_init

static void MINI_SPI_init(void)
{
    printf("MINI_SPI_init\n");

    GPIO_setup(spi_sclk); GPIO_output(spi_sclk);
    GPIO_setup(spi_mosi); GPIO_output(spi_mosi);
    GPIO_setup(spi_miso); GPIO_input(spi_miso);
    // FIO_SetDir((spi_mosi >> 5) & 7, 1UL << (spi_mosi & 0x1F), 1);
    // FIO_SetDir((spi_miso >> 5) & 7, 1UL << (spi_miso & 0x1F), 0);
    // FIO_SetDir((spi_sclk >> 5) & 7, 1UL << (spi_sclk & 0x1F), 1);
    if (ssp_device == LPC_SSP1)
    {
        LPC_PINCON->PINSEL0 &= ~((3 << (7*2)) | (3 << (8*2)) | (3 << (9*2)));
        LPC_PINCON->PINSEL0 |=  ((2 << (7*2)) | (2 << (8*2)) | (2 << (9*2)));

        LPC_SC->PCLKSEL0 &= 0xFFCFFFFF;
        LPC_SC->PCLKSEL0 |= 0x00100000;

        LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP1;
    }
    else if (ssp_device == LPC_SSP0)
    {
        if (spi_sclk == P0_15)
        {
            LPC_PINCON->PINSEL0 &= ~(3 << (15*2));
            LPC_PINCON->PINSEL0 |=  (2 << (15*2));
            LPC_PINCON->PINSEL1 &= ~( (3 << ((17*2)&30)) | (3 << ((18*2)&30)) );
            LPC_PINCON->PINSEL1 |=  ( (2 << ((17*2)&30)) | (2 << ((18*2)&30)) );

            LPC_SC->PCLKSEL1 &= 0xFFFFF3FF;
            LPC_SC->PCLKSEL1 |= 0x00000400;

            LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP0;
        }
        else if (spi_sclk == P1_20)
        {
            //         isr_dispatch[0] = this;

            // //         LPC_PINCON->PINSEL3 &= 0xFFFC3CFF;
            //         LPC_PINCON->PINSEL3 |= 0x0003C300;

            //         LPC_PINCON->PINSEL3 &= ~( (3 << ((20*2)&30)) | (3 << ((23*2)&30)) | (3 << ((24*2)&30)) );
            LPC_PINCON->PINSEL3 |=  ( (3 << ((20*2)&30)) | (3 << ((23*2)&30)) | (3 << ((24*2)&30)) );

            LPC_SC->PCLKSEL1 &= 0xFFFFF3FF;
            LPC_SC->PCLKSEL1 |= 0x00000400;

            LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP0;
        }
    }
    ssp_device->CR0 &= ~0xFFFF;
    ssp_device->CR0 |= SSP_DATABIT_8 | SSP_FRAME_SPI | SSP_CPHA_FIRST | SSP_CPOL_HI;
    ssp_device->CR1 = SSP_MASTER_MODE;
    SPI_frequency(spi_frequency);
    ssp_device->CR1 |= SSP_CR1_SSP_EN;
}
#endif

#if ! defined(IMPL_SPI_write) && ! FORCE_SOFT_SPI
#define IMPL_SPI_write MINI_SPI_write

static uint8_t MINI_SPI_write(uint8_t data)
{
    while ((ssp_device->SR & SSP_SR_TNF) == 0);

    ssp_device->DR = data;

    while ((ssp_device->SR & SSP_SR_RNE) == 0);

    return ssp_device->DR & 0xFF;
}
#endif


// SPI by software, works for normal digital pins
//   used as fallback, may be used via FORCE_SOFT_SPI to completely ignore hardware SPI

#ifdef SOFT_SPI

static uint32_t spi_soft_delay = 0;

static void spi__delay(uint32_t ticks) {
    for (;ticks;ticks--)
        asm volatile("nop\n\t");
}

void SOFT_SPI_frequency(uint32_t f)
{
    printf("SOFT_SPI_frequency\n");

    // -0.5 to eventually force the result to 0 if near
    spi_soft_delay = (2 * SPI_SOFT_DELAY_CALIBRATION / f - 1) / 2;
    //if (!ssp_device)
      printf("spi: f=%lu soft_delay=%lx\n", f, spi_soft_delay);
}

static void SOFT_SPI_init(void)
{
    printf("SOFT_SPI_init\n");

    GPIO_setup(spi_sclk); GPIO_output(spi_sclk);
    GPIO_setup(spi_mosi); GPIO_output(spi_mosi);
    GPIO_setup(spi_miso); GPIO_input(spi_miso);
    SPI_frequency(spi_frequency);
}

static uint8_t SOFT_SPI_write(uint8_t data)
{
    uint8_t r = 0;

    for (int bit = 7; bit >= 0; bit--) {

        GPIO_clear(spi_sclk);               // clock LOW

        if (data & (1 << bit))                              // WRITE
            GPIO_set(spi_mosi);
        else
            GPIO_clear(spi_mosi);

        spi__delay(spi_soft_delay);                 // DELAY

        GPIO_set(spi_sclk);                 // clock HIGH

        spi__delay(spi_soft_delay);                 // DELAY

        if (GPIO_get(spi_miso))                             // READ
            r |= (1 << bit);
    }
    GPIO_clear(spi_sclk);                   // clock LOW

    return r;
}

#endif


// public interface

void SPI_frequency(uint32_t f)
{
    // CCLK = 25MHz
    // CPSR = 2 to 254, even only
    // CR0[8:15] (SCR, 0..255) is a further prescale

    spi_frequency = f;

    #if ! FORCE_SOFT_SPI
        if(ssp_device)
        {
            IMPL_SPI_frequency(f);
            printf("spi: f=%lu sspclk=%lu CPSR=%lu CRf=%lu -> %lu\n", f, ssp_clk, ssp_device->CPSR, ((ssp_device->CR0 >> 8) & 0xFF) + 1, ssp_clk / ssp_device->CPSR / (((ssp_device->CR0 >> 8) & 0xFF) + 1));
        }
    #endif

    #ifdef SOFT_SPI
        SOFT_SPI_frequency(f);
    #endif
}

void SPI_init(PinName mosi, PinName miso, PinName sclk)
{
    spi_sclk = sclk;
    spi_mosi = mosi;
    spi_miso = miso;

    ssp_device = (SPI_REG *) 0;
    #if ! FORCE_SOFT_SPI
        if (spi_mosi == P0_9 && spi_miso == P0_8 && spi_sclk == P0_7)
            ssp_device = LPC_SSP1;
        else
        if (spi_mosi == P0_18 && spi_miso == P0_17 && spi_sclk == P0_15)
            ssp_device = LPC_SSP0;
        else
        if (spi_mosi == P1_24 && spi_miso == P1_23 && spi_sclk == P1_20)
            ssp_device = LPC_SSP0;
    #endif

    #ifdef SOFT_SPI
        SOFT_SPI_init();
    #endif

    #if ! FORCE_SOFT_SPI
        if(ssp_device)
        {
            IMPL_SPI_init();
            printf("PINSEL0=0x%lx PCLKSEL0=0x%lx PCONP=0x%lx CPSR=0x%lx CR0=0x%lx CR1=0x%lx\n", LPC_PINCON->PINSEL0, LPC_SC->PCLKSEL0, LPC_SC->PCONP, ssp_device->CPSR, ssp_device->CR0, ssp_device->CR1);
        }
    #endif
}

uint8_t SPI_write(uint8_t data)
{
    uint8_t r = 0;
    #if ! FORCE_SOFT_SPI
        if (ssp_device) {
            DBwrite(printf("SPI: >0x%x ", data);)
            r = IMPL_SPI_write(data);
            DBwrite(printf("= 0x%x\n", r);)
        }
    #else
        if(0) ;
    #endif
    #ifdef SOFT_SPI
        else {
            DBwrite(printf("SPISOFT: >0x%x ", data);)
            r = SOFT_SPI_write(data);
            DBwrite(printf("= 0x%x\n", r);)
        }
    #endif
    //if(r == 0xFF) printf(" <0x%x\b\b\b\b\b\b", r); else printf(" <0x%x\n", r);
    //if(data != 0xFF || r != 0xFF) printf("SPI%s: >0x%x <0x%x\n", ssp_device?"":"soft", data, r);
    return r;
}

// TODO: timer feeds DMA feeds 0xFFs to card then we listen for responses using our interrupt
// allow me to do something like:
// disk.start_multi_write(int blocks, int blocksize, void *buffer);
// enable_usb_isr();
// [...]
// usb_isr() {
//    if (disk.buffer_in_use(void *buffer))
//        return;
//    usb_ep_read(buffer);
//    if (buffer_full)
//        disk.validate_buffer(buffer);
//    if (disk.finished_transfer())
//        disk.end_multi_write();
// };

// bool SPI_can_DMA()
// {
//     return (ssp_device != NULL);
// }

// int SPI::setup_DMA_rx(DMA_REG *dma)
// {
//     if (!ssp_device)
//         return -1;
//
//     dma->DMACCControl = 0;
//     dma->DMACCConfiguration = 0;
//     if (ssp_device == LPC_SSP0)
//         dma->DMACCConfiguration |= (GPDMA_CONN_SSP0_Rx << 6);
//     if (ssp_device == LPC_SSP1)
//         dma->DMACCConfiguration |= (GPDMA_CONN_SSP1_Rx << 6);
//
//     dma->DMACCConfiguration |= GPDMA_TRANSFERTYPE_M2P << 11;
//     return 0;
// }
//
// int SPI::start_DMA_rx(DMA_REG *dma)
// {
//     dma->DMACCConfiguration |=
// }

// int SPI_writeblock(uint8_t *block, int blocklen)
// {
//     static DMA *d = new DMA();
//     d.sourceaddr(block);
//     d.transferlength(blocklen);
//     d.destinationperipheral(ssp_device);
//     d.start();
//     while (d.active());
//     return blocklen;
//     return 0;
// }

// void SPI_irq()
// {
// }

// void SSP0_IRQHandler(void) {
//     if (isr_dispatch[0])
//         isr_dispatch[0]();
// }

// void SSP1_IRQHandler(void) {
//     if (isr_dispatch[1])
//         isr_dispatch[1]();
// }
