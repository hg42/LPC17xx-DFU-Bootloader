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

#define USE_NEW_freq    0
#define USE_NEW_init    0
#define USE_NEW_write   0
#define USE_NEW_write_read_until_rx_empty  1
#define USE_NEW_write_wait_tx_not_full     1
#define USE_NEW_write_overrun_exit         1

// measure at 25000Hz (slow spi init frequence)
// change here to fit 40us/cycle, higher number -> longer delay
// calibration may depend on compiler options
// this is for -Os -g
#define SPI_SOFT_DELAY_CALIBRATION          6500000

#include "spi.h"

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_ssp.h"
//#include "lpc17xx_gpio.h"
#include "gpio.h"

#include "min-printf.h"

#include "lpc17xx_spi.h"
#include "lpc17xx_clkpwr.h"
//#include "lpc17xx_libcfg_default.h"

static uint32_t spi_soft_delay = 0;
static PinName spi_miso;
static PinName spi_mosi;
static PinName spi_sclk;
static uint32_t spi_frequency = 25000;  // default
static SPI_REG * sspr;
static uint32_t ssp_clk = 0;

static void spi__delay(uint32_t ticks) {
    for (;ticks;ticks--)
        asm volatile("nop\n\t");
}



#if USE_NEW_freq
/*********************************************************************//**
 * @brief 		Setup clock rate for SSP device
 * @param[in] 	SSPx	SSP peripheral definition, should be:
 * 						- LPC_SSP0: SSP0 peripheral
 * 						- LPC_SSP1: SSP1 peripheral
 * @param[in]	target_clock : clock of SSP (Hz)
 * @return 		None
 ***********************************************************************/
static void SSP_SetClock (LPC_SSP_TypeDef *SSPx, uint32_t target_clock)
{
    uint32_t prescale, cr0_div, cmp_clk; //, ssp_clk;

    //CHECK_PARAM(PARAM_SSPx(SSPx));

    /* The SSP clock is derived from the (main system oscillator / 2),
       so compute the best divider from that clock */
    //if (SSPx == LPC_SSP0){
    //	ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP0);
    //} else if (SSPx == LPC_SSP1) {
    	ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP1);

    //} else {
    //	return;
    //}

    /* Find closest divider to get at or under the target frequency.
       Use smallest prescale possible and rely on the divider to get
       the closest target frequency */
    cr0_div = 0;
    cmp_clk = 0xFFFFFFFF;
    prescale = 2;
    while (cmp_clk > target_clock)
    {
        cmp_clk = ssp_clk / ((cr0_div + 1) * prescale);
        if (cmp_clk > target_clock)
        {
            cr0_div++;
            if (cr0_div > 0xFF)
            {
                cr0_div = 0;
                prescale += 2;
            }
        }
    }

    /* Write computed prescaler and divider back to register */
    SSPx->CR0 &= (~SSP_CR0_SCR(0xFF)) & SSP_CR0_BITMASK;
    SSPx->CR0 |= (SSP_CR0_SCR(cr0_div)) & SSP_CR0_BITMASK;
    SSPx->CPSR = prescale & SSP_CPSR_BITMASK;
}
#endif

#if USE_NEW_init
/********************************************************************//**
 * @brief		Initializes the SSPx peripheral according to the specified
*               parameters in the SSP_ConfigStruct.
 * @param[in]	SSPx	SSP peripheral selected, should be:
 * 				 		- LPC_SSP0: SSP0 peripheral
 * 						- LPC_SSP1: SSP1 peripheral
 * @param[in]	SSP_ConfigStruct Pointer to a SSP_CFG_Type structure
*                    that contains the configuration information for the
*                    specified SSP peripheral.
 * @return 		None
 *********************************************************************/
void SSP_Init(LPC_SSP_TypeDef *SSPx, SSP_CFG_Type *SSP_ConfigStruct)
{
    uint32_t tmp;

    //CHECK_PARAM(PARAM_SSPx(SSPx));

    //if(SSPx == LPC_SSP0) {
    //    /* Set up clock and power for SSP0 module */
    //    CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSSP0, ENABLE);
    //} else if(SSPx == LPC_SSP1) {
        /* Set up clock and power for SSP1 module */
        CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSSP1, ENABLE);
    //} else {
    //    return;
    //}

    /* Configure SSP, interrupt is disable, LoopBack mode is disable,
     * SSP is disable, Slave output is disable as default
     */
    tmp = ((SSP_ConfigStruct->CPHA) | (SSP_ConfigStruct->CPOL) \
            | (SSP_ConfigStruct->FrameFormat) | (SSP_ConfigStruct->Databit))
            & SSP_CR0_BITMASK;
    // write back to SSP control register
    SSPx->CR0 = tmp;

    tmp = SSP_ConfigStruct->Mode & SSP_CR1_BITMASK;
    // Write back to CR1
    SSPx->CR1 = tmp;

    // Set clock rate for SSP peripheral
    //SSP_SetClock(SSPx, SSP_ConfigStruct->ClockRate);
    SPI_frequency(SSP_ConfigStruct->ClockRate);   // to work with all variants
}
#endif

#if USE_NEW_write
/*********************************************************************//**
 * @brief		Transmit a single data through SSPx peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be:
 * 						- LPC_SSP0: SSP0 peripheral
 * 						- LPC_SSP1: SSP1 peripheral
 * @param[in]	Data	Data to transmit (must be 16 or 8-bit long,
 * 						this depend on SSP data bit number configured)
 * @return 		none
 **********************************************************************/
inline void SSP_SendData(LPC_SSP_TypeDef* SSPx, uint16_t Data)
{
    //CHECK_PARAM(PARAM_SSPx(SSPx));

    SSPx->DR = SSP_DR_BITMASK(Data);
}



/*********************************************************************//**
 * @brief		Receive a single data from SSPx peripheral
 * @param[in]	SSPx	SSP peripheral selected, should be
 * 						- LPC_SSP0: SSP0 peripheral
 * 						- LPC_SSP1: SSP1 peripheral
 * @return 		Data received (16-bit long)
 **********************************************************************/
inline uint16_t SSP_ReceiveData(LPC_SSP_TypeDef* SSPx)
{
    //CHECK_PARAM(PARAM_SSPx(SSPx));

    return ((uint16_t) (SSP_DR_BITMASK(SSPx->DR)));
}

uint8_t SSP_Poll8 (LPC_SSP_TypeDef *SSPx, uint8_t wdata8)
{
    uint32_t stat;
    uint32_t tmp;

    #if USE_NEW_write_read_until_rx_empty
    /* Clear all remaining data in RX FIFO */
    while (SSPx->SR & SSP_SR_RNE){
        tmp = (uint32_t) SSP_ReceiveData(SSPx);
        printf("drop 0x%lx\n", tmp);
    }
    #endif

    // Clear status
    SSPx->ICR = SSP_ICR_BITMASK;

    #if USE_NEW_write_wait_tx_not_full
    //while (!(SSPx->SR & SSP_SR_TNF)) ;
    while(1)
    {
        uint32_t SR = SSPx->SR;
        //printf("/ 0x%lx\n", SR);
        if ((SR & SSP_SR_TNF) != 0)
            break;
    }
    #endif

    SSP_SendData(SSPx, wdata8);

    #if USE_NEW_write_overrun_exit
    // Check overrun error
    if ((stat = SSPx->RIS) & SSP_RIS_ROR){
        return (-1);
    }
    #endif

    // Check for any data available in RX FIFO
    //while (!(SSPx->SR & SSP_SR_RNE)) ;
    while(1)
    {
        uint32_t SR = SSPx->SR;
        //printf("\\ 0x%lx\n", SR);
        if ((SR & SSP_SR_RNE) != 0)
            break;
    }

    // Read data from SSP data
    return SSP_ReceiveData(SSPx);
}
#endif


void SPI_reinit(int force_soft_spi)
{
    sspr = (SPI_REG *) 0;

    if(force_soft_spi)
    {
        GPIO_setup(spi_sclk); GPIO_output(spi_sclk);
        GPIO_setup(spi_mosi); GPIO_output(spi_mosi);
        GPIO_setup(spi_miso); GPIO_input(spi_miso);
    }
    else
    {

#if USE_NEW_init
        PINSEL_CFG_Type PinCfg;

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

        //	uint32_t Databit; 		/** Databit number, should be SPI_DATABIT_x,
        //							where x is in range from 8 - 16 */
        //	uint32_t CPHA;			/** Clock phase, should be:
        //							- SPI_CPHA_FIRST: first clock edge
        //							- SPI_CPHA_SECOND: second clock edge */
        //	uint32_t CPOL;			/** Clock polarity, should be:
        //							- SPI_CPOL_HI: high level
        //							- SPI_CPOL_LO: low level */
        //	uint32_t Mode;			/** SPI mode, should be:
        //							- SPI_MASTER_MODE: Master mode
        //							- SPI_SLAVE_MODE: Slave mode */
        //	uint32_t DataOrder;		/** Data order, should be:
        //							- SPI_DATA_MSB_FIRST: MSB first
        //							- SPI_DATA_LSB_FIRST: LSB first */
        //	uint32_t ClockRate;		/** Clock rate,in Hz, should not exceed
        //							(SPI peripheral clock)/8 */
    	SSP_CFG_Type cfg;
        cfg.CPHA        = SSP_CPHA_FIRST;
    	cfg.CPOL        = SSP_CPOL_HI;
    	cfg.ClockRate   = spi_frequency;
    	cfg.Databit     = SSP_DATABIT_8;
    	cfg.Mode        = SSP_MASTER_MODE;
    	cfg.FrameFormat = SSP_FRAME_SPI;
        sspr = LPC_SSP1;
        SSP_Init(sspr, &cfg);
    	sspr->CR1 |= SSP_CR1_SSP_EN;

#else

        GPIO_setup(spi_sclk); GPIO_output(spi_sclk);
        GPIO_setup(spi_mosi); GPIO_output(spi_mosi);
        GPIO_setup(spi_miso); GPIO_input(spi_miso);
        // FIO_SetDir((spi_mosi >> 5) & 7, 1UL << (spi_mosi & 0x1F), 1);
        // FIO_SetDir((spi_miso >> 5) & 7, 1UL << (spi_miso & 0x1F), 0);
        // FIO_SetDir((spi_sclk >> 5) & 7, 1UL << (spi_sclk & 0x1F), 1);
        if (spi_mosi == P0_9 && spi_miso == P0_8 && spi_sclk == P0_7)
        {
            // SSP1 on 0.7,0.8,0.9
            sspr = LPC_SSP1;
            //         isr_dispatch[1] = this;
            //printf("SPI:Using SSP1\n");

            LPC_PINCON->PINSEL0 &= ~((3 << (7*2)) | (3 << (8*2)) | (3 << (9*2)));
            LPC_PINCON->PINSEL0 |=  ((2 << (7*2)) | (2 << (8*2)) | (2 << (9*2)));

            LPC_SC->PCLKSEL0 &= 0xFFCFFFFF;
            LPC_SC->PCLKSEL0 |= 0x00100000;

            LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP1;
        }
        else if (spi_mosi == P0_18 && spi_miso == P0_17 && spi_sclk == P0_15)
        {
            // SSP0 on 0.15,0.16,0.17,0.18
            sspr = LPC_SSP0;
            //         isr_dispatch[0] = this;

            LPC_PINCON->PINSEL0 &= ~(3 << (15*2));
            LPC_PINCON->PINSEL0 |=  (2 << (15*2));
            LPC_PINCON->PINSEL1 &= ~( (3 << ((17*2)&30)) | (3 << ((18*2)&30)) );
            LPC_PINCON->PINSEL1 |=  ( (2 << ((17*2)&30)) | (2 << ((18*2)&30)) );

            LPC_SC->PCLKSEL1 &= 0xFFFFF3FF;
            LPC_SC->PCLKSEL1 |= 0x00000400;

            LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP0;
        }
        else if (spi_mosi == P1_24 && spi_miso == P1_23 && spi_sclk == P1_20)
        {
            // SSP0 on 1.20,1.23,1.24
            sspr = LPC_SSP0;
            //         isr_dispatch[0] = this;

            // //         LPC_PINCON->PINSEL3 &= 0xFFFC3CFF;
            //         LPC_PINCON->PINSEL3 |= 0x0003C300;

            //         LPC_PINCON->PINSEL3 &= ~( (3 << ((20*2)&30)) | (3 << ((23*2)&30)) | (3 << ((24*2)&30)) );
            LPC_PINCON->PINSEL3 |=  ( (3 << ((20*2)&30)) | (3 << ((23*2)&30)) | (3 << ((24*2)&30)) );

            LPC_SC->PCLKSEL1 &= 0xFFFFF3FF;
            LPC_SC->PCLKSEL1 |= 0x00000400;

            LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP0;
        }

        if (sspr) {
            sspr->CR0 &= ~0xFFFF;
            sspr->CR0 |= SSP_DATABIT_8 | SSP_FRAME_SPI | SSP_CPHA_FIRST | SSP_CPOL_HI;
            //printf("CR0=0x%lx\n", sspr->CR0);
            SPI_frequency(spi_frequency);
            //printf("CR0=0x%lx\n", sspr->CR0);
            sspr->CR1 = SSP_MASTER_MODE;
            sspr->CR1 |= SSP_CR1_SSP_EN;
        }
#endif
    }

}

void SPI_init(PinName mosi, PinName miso, PinName sclk, int force_soft_spi)
{
    spi_sclk = sclk;
    spi_mosi = mosi;
    spi_miso = miso;
    SPI_reinit(force_soft_spi);

    if(sspr) printf("PINSEL0=0x%lx PCLKSEL0=0x%lx PCONP=0x%lx CPSR=0x%lx CR0=0x%lx CR1=0x%lx\n", LPC_PINCON->PINSEL0, LPC_SC->PCLKSEL0, LPC_SC->PCONP, sspr->CPSR, sspr->CR0, sspr->CR1);
}

void SPI_frequency(uint32_t f)
{
    // CCLK = 25MHz
    // CPSR = 2 to 254, even only
    // CR0[8:15] (SCR, 0..255) is a further prescale

    spi_frequency = f;

#if USE_NEW_freq
    SSP_SetClock(sspr, f);
    uint32_t clk_per_cycle = ssp_clk / f;
#else
    ssp_clk = 25000000;
    uint32_t clk_per_cycle = ssp_clk / f;
    // f = 25MHz / (CPSR . [SCR + 1])
    // CPSR . (SCR + 1) = 25MHz / f
    // min freq is 25MHz / (254 * 256)
    if (sspr) {
        if (f < 385) {
            sspr->CPSR = 254;
            sspr->CR0 &= 0x00FF;
            sspr->CR0 |= 255 << 8;
        }
        // max freq is 25MHz / (2 * 1)
        else if (f > 12500000) {
            sspr->CPSR = 2;
            sspr->CR0 &= 0x00FF;
        }
        else {
            sspr->CPSR = clk_per_cycle & 0xFE;
            // CPSR . (SCR + 1) = f;
            // (SCR + 1) = f / CPSR;
            // SCR = (f / CPSR) - 1
            sspr->CR0 &= 0x00FF;
            sspr->CR0 |= (((clk_per_cycle / sspr->CPSR) - 1) & 0xFF) << 8;
        }
    }
#endif
    spi_soft_delay = (2*SPI_SOFT_DELAY_CALIBRATION/spi_frequency-1)/2;
    if (sspr)  printf("spi: f=%lu sspclk=%lu clk_per_cycle=%lx CPSR=%lu CRf=%lu -> %lu\n", f, ssp_clk, clk_per_cycle, sspr->CPSR, ((sspr->CR0 >> 8) & 0xFF) + 1, ssp_clk / sspr->CPSR / (((sspr->CR0 >> 8) & 0xFF) + 1));
    if (!sspr) printf("spi: f=%lu soft_delay=%lx\n", f, spi_soft_delay);
}

uint8_t SPI_write(uint8_t data)
{
    uint8_t r = 0;
#if USE_NEW_write
    if (sspr) {
        r = SSP_Poll8(sspr, data);
    }
#else
    // _cs = 1;
    if (sspr) {
        DBwrite(printf("SPI: >0x%x ", data);)

        while ((sspr->SR & SSP_SR_TNF) == 0);

        sspr->DR = data;

        while ((sspr->SR & SSP_SR_RNE) == 0);

        r = sspr->DR & 0xFF;

        DBwrite(printf("= 0x%x\n", r);)
    }
#endif
    #ifdef SOFT_SPI
    else {
        DBwrite(printf("SPISOFT: >0x%x ", data);)
        uint8_t bits = data;
        for (int i = 0; i < 8; i++) {
            GPIO_clear(spi_sclk);               // clock LOW

            if (bits & 0x80)                            // WRITE
                GPIO_set(spi_mosi);
            else
                GPIO_clear(spi_mosi);
            bits <<= 1;

            spi__delay(spi_soft_delay);                         // DELAY

            GPIO_set(spi_sclk);                 // clock HIGH

            spi__delay(spi_soft_delay);                         // DELAY

            r <<= 1;
            if (GPIO_get(spi_miso))                     // READ
                r |= 1;
        }
        GPIO_clear(spi_sclk);                   // clock LOW
        DBwrite(printf("= 0x%x\n", r);)
    }
    #endif
    //if(r == 0xFF) printf(" <0x%x\b\b\b\b\b\b", r); else printf(" <0x%x\n", r);
    //if(data != 0xFF || r != 0xFF) printf("SPI%s: >0x%x <0x%x\n", sspr?"":"soft", data, r);
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
//     return (sspr != NULL);
// }

// int SPI::setup_DMA_rx(DMA_REG *dma)
// {
//     if (!sspr)
//         return -1;
//
//     dma->DMACCControl = 0;
//     dma->DMACCConfiguration = 0;
//     if (sspr == LPC_SSP0)
//         dma->DMACCConfiguration |= (GPDMA_CONN_SSP0_Rx << 6);
//     if (sspr == LPC_SSP1)
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
//     d.destinationperipheral(sspr);
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
