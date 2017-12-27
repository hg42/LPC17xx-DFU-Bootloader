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

#include "spi.h"

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_gpio.h"
#include "gpio.h"

//#include <stdio.h>
#include "min-printf.h"

//#define FORCE_SOFT_SPI
//#define SOFT_SPI

#ifdef FORCE_SOFT_SPI
  #ifndef SOFT_SPI
    #define SOFT_SPI
  #endif
#endif

uint32_t delay;
PinName spi_miso;
PinName spi_mosi;
PinName spi_sclk;
SPI_REG volatile * volatile sspr;


static void spi__delay(uint32_t ticks) {
    for (;ticks;ticks--)
        asm volatile("nop\n\t");
}

void SPI_init(PinName mosi, PinName miso, PinName sclk)
{
#ifdef FORCE_SOFT_SPI
    GPIO_setup(sclk); GPIO_output(sclk); spi_sclk = sclk;
    GPIO_setup(mosi); GPIO_output(mosi); spi_mosi = mosi;
    GPIO_setup(miso); GPIO_input(miso);  spi_miso = miso;
#else
    GPIO_setup(sclk); GPIO_output(sclk); spi_sclk = sclk;
    GPIO_setup(mosi); GPIO_output(mosi); spi_mosi = mosi;
    GPIO_setup(miso); GPIO_input(miso);  spi_miso = miso;
	FIO_SetDir((mosi >> 5) & 7, 1UL << (mosi & 0x1F), 1);
	FIO_SetDir((miso >> 5) & 7, 1UL << (miso & 0x1F), 0);
	FIO_SetDir((sclk >> 5) & 7, 1UL << (sclk & 0x1F), 1);
    if (mosi == P0_9 && miso == P0_8 && sclk == P0_7)
    {
        // SSP1 on 0.7,0.8,0.9
        sspr = LPC_SSP1;
//         isr_dispatch[1] = this;
 		printf("SPI:Using SSP1\n");
spi__delay(delay);

        LPC_PINCON->PINSEL0 &= ~((3 << (7*2)) | (3 << (8*2)) | (3 << (9*2)));
        LPC_PINCON->PINSEL0 |=  ((2 << (7*2)) | (2 << (8*2)) | (2 << (9*2)));

spi__delay(delay);
        LPC_SC->PCLKSEL0 &= 0xFFCFFFFF;
        LPC_SC->PCLKSEL0 |= 0x00100000;
spi__delay(delay);

        LPC_SC->PCONP |= CLKPWR_PCONP_PCSSP1;
spi__delay(delay);
    }
    else if (mosi == P0_18 && miso == P0_17 && sclk == P0_15)
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
    else if (mosi == P1_24 && miso == P1_23 && sclk == P1_20)
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
    else
#endif
    {
        sspr = (LPC_SSP_TypeDef *) 0;
    }

    if (sspr) {
spi__delay(delay);
        sspr->CR0 = SSP_DATABIT_8 |
                    SSP_FRAME_SPI;
spi__delay(delay);
        sspr->CR1 = SSP_MASTER_MODE;
spi__delay(delay);
        SPI_frequency(10000);
spi__delay(delay);
        sspr->CR1 |= SSP_CR1_SSP_EN;
spi__delay(delay);
    }
}

void SPI_frequency(uint32_t f)
{
    // CCLK = 25MHz
    // CPSR = 2 to 254, even only
    // CR0[8:15] (SCR, 0..255) is a further prescale

printf("SPI: frequency %lu:", f);
    delay = 25000000 / f;
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
            sspr->CPSR = delay & 0xFE;
            // CPSR . (SCR + 1) = f;
            // (SCR + 1) = f / CPSR;
            // SCR = (f / CPSR) - 1
            sspr->CR0 &= 0x00FF;
            sspr->CR0 |= (((delay / sspr->CPSR) - 1) & 0xFF) << 8;
        }
printf(" CPSR=%lu, CR0=%lu", sspr->CPSR, sspr->CR0);
    }
printf("\n");
}

uint8_t SPI_write(uint8_t data)
{
//     _cs = 1;
    uint8_t r = 0;
    if (sspr) {
//     printf("SPI: >0x%x ", data);
spi__delay(100*delay);
        while ((sspr->SR & SSP_SR_TNF) == 0);
spi__delay(100*delay);
        sspr->DR = data;
spi__delay(100*delay);
        while ((sspr->SR & SSP_SR_RNE) == 0);
spi__delay(100*delay);
        r = sspr->DR & 255;
spi__delay(100*delay);
    }
#ifdef SOFT_SPI
    else {
//     printf("SPISOFT: >0x%x", data);
        uint8_t bits = data;
#define delay1  (delay >> 1)
        for (int i = 0; i < 8; i++) {
//spi__delay(delay1);
            GPIO_clear(spi_sclk);               // clock LOW

//spi__delay(delay1);
            if (bits & 0x80)                            // WRITE
                GPIO_set(spi_mosi);
            else
                GPIO_clear(spi_mosi);
            bits <<= 1;

            spi__delay(delay1);                                // DELAY

            GPIO_set(spi_sclk);                 // clock HIGH

            spi__delay(delay1);                                // DELAY

            r <<= 1;
            if (GPIO_get(spi_miso))                     // READ
                r |= 1;
        }
//spi__delay(delay1);
        GPIO_clear(spi_sclk);                   // clock LOW
//spi__delay(delay1);
    }
#endif
//     if(r == 0xFF) printf(" <0x%x\b\b\b\b\b\b", r); else printf(" <0x%x\n", r);
//     if(data != 0xFF || r != 0xFF) printf("SPI%s: >0x%x <0x%x\n", sspr?"":"soft", data, r);
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
