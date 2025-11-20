/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * This file provides functions for configuring the clocks.
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Robert Hammelrath
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "samd_soc.h"

static uint32_t cpu_freq = CPU_FREQ;
static uint32_t peripheral_freq = DFLL48M_FREQ;
static uint32_t dfll48m_calibration;
// SAM L21 Family Data Sheet p.142
int sercom_gclk_id[] = {
    SERCOM0_GCLK_ID_CORE, SERCOM1_GCLK_ID_CORE,
    SERCOM2_GCLK_ID_CORE, SERCOM3_GCLK_ID_CORE,
    SERCOM4_GCLK_ID_CORE, SERCOM5_GCLK_ID_CORE};

uint32_t get_cpu_freq(void)
{

    return cpu_freq;
}

uint32_t get_peripheral_freq(void)
{
    return peripheral_freq;
}
/*SAM L21 Family Data Sheet p.123*/
void set_cpu_freq(uint32_t cpu_freq_arg)
{
    // Set 2 wait state to be safe
    __disable_irq();
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_MANW | NVMCTRL_CTRLB_RWS(2);
    __enable_irq();
    int div = MAX(DFLL48M_FREQ / cpu_freq_arg, 1);
    peripheral_freq = DFLL48M_FREQ / div;

    // Use GCLK1 as source of GCLK0(MAIN CLOCK) temporarily before disable DFLL48M
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC_GCLKGEN1 | GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN;
    while ((GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0) == GCLK_SYNCBUSY_GENCTRL0)
    {
    }

    // Enable GCLK output: 48MHz from DFLL48M on both GCLK0 and GCLK2

    // DFFL48M disable
    OSCCTRL->DFLLCTRL.reg = 0U;
    while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) != OSCCTRL_STATUS_DFLLRDY)
    {
    }
    // DFLL48M CALIBRATION VALUES WRITTEN INTO THEIR PLACES
    uint8_t calibCoarse = (uint8_t)(((*(uint32_t *)0x00806020U) >> 26U) & 0x3fU);
    OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE((uint32_t)calibCoarse) | OSCCTRL_DFLLVAL_FINE(512U);
    while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) != OSCCTRL_STATUS_DFLLRDY)
    {
    }
    // DFLL48M ENABLE
    OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;
    while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) != OSCCTRL_STATUS_DFLLRDY)
    {
    }
    // DFLL48M output to GCLK0 (main clock)
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_DIV(div) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN;
    while ((GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0) == GCLK_SYNCBUSY_GENCTRL0)
    {
    }

    GCLK->GENCTRL[2].reg = GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_GENEN;
    while ((GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL2) == GCLK_SYNCBUSY_GENCTRL2)
    {
        // Sync
    }

    // The comparison is >=, such that for 48MHz still the FDPLL96 is used for the CPU clock.
    if (cpu_freq_arg >= 48000000)
    {
        cpu_freq = cpu_freq_arg;
        // Connect GCLK1 to the FDPLL96 input.
        GCLK->PCHCTRL[1].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
        while (GCLK->SYNCBUSY.bit.GENCTRL1)
        {
        }
        // configure the FDPLL96
        // CtrlB: Set the ref source to GCLK, set the Wakeup-Fast Flag.
        OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(0x2) | OSCCTRL_DPLLCTRLB_WUF;
        // Set the FDPLL ratio and enable the DPLL.
        int ldr = cpu_freq / FDPLL_REF_FREQ;
        int frac = ((cpu_freq - ldr * FDPLL_REF_FREQ) / (FDPLL_REF_FREQ / 16)) & 0x0f;
        OSCCTRL->DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDR((frac << 16 | ldr) - 1);
        OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
        // Wait for the DPLL lock.
        while (!OSCCTRL->DPLLSTATUS.bit.LOCK)
        {
        }
        // Finally switch GCLK0 to FDPLL96M.
        GCLK->GENCTRL[0].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_OE;
        while (GCLK->SYNCBUSY.bit.GENCTRL0)
        {
        }
    }
    else
    {
        cpu_freq = peripheral_freq;
        // Disable the FDPLL96M in case it was enabled.
        OSCCTRL->DPLLCTRLA.reg = 0;
    }
    if (cpu_freq >= 8000000)
    {

        // Enable GCLK output: 48MHz on GCLK5 for USB
        GCLK->GENCTRL[5].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M;
        while (GCLK->SYNCBUSY.bit.GENCTRL5)
        {
        }
        GCLK->PCHCTRL[USB_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK5 | GCLK_PCHCTRL_CHEN;
    }
    else
    {
        // Disable GCLK output on GCLK5 for USB, since USB is not reliable below 8 Mhz.
        GCLK->PCHCTRL[USB_GCLK_ID].bit.CHEN = 0;
        while (GCLK->SYNCBUSY.bit.GENCTRL5)
        {
        }
    }

    // Set 1 wait state to be safe
    __disable_irq();
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS(cpu_freq > 24000000 ? 2 : 0);
    __enable_irq();

    SysTick_Config(cpu_freq / 1000);
}

#if !MICROPY_HW_XOSC32K || MICROPY_HW_DFLL_USB_SYNC
static void sync_dfll48_with_xosc32kulp(void)
{

    OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;
    while (!OSCCTRL->STATUS.bit.DFLLRDY)
    {
    }
    // Connect GCLK4 to the DFLL input.
    GCLK->PCHCTRL[0].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
    while (GCLK->SYNCBUSY.bit.GENCTRL4)
    {
    }
    // Set the multiplication values. The offset of 16384 to the freq is for rounding.
    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_MUL((CPU_FREQ + 16384) / 32768) |
                           OSCCTRL_DFLLMUL_FSTEP(1) | OSCCTRL_DFLLMUL_CSTEP(1);
    while (OSCCTRL->STATUS.bit.DFLLRDY == 0)
    {
    }
    // Start the DFLL and wait for the PLL lock. We just wait for the fine lock, since
    // coarse adjusting is bypassed.
    OSCCTRL->DFLLCTRL.reg |= OSCCTRL_DFLLCTRL_MODE | OSCCTRL_DFLLCTRL_WAITLOCK | OSCCTRL_DFLLCTRL_STABLE |
                             OSCCTRL_DFLLCTRL_BPLCKC | OSCCTRL_DFLLCTRL_ENABLE;
    while (!OSCCTRL->STATUS.bit.DFLLLCKF)
    {
    }
}
#endif

void check_usb_clock_recovery_mode(void)
{
#if MICROPY_HW_DFLL_USB_SYNC
    // Check USB status for up to 1 second. If not connected,
    // switch DFLL48M back to open loop
    for (int i = 0; i < 100; i++)
    {
        if (USB->DEVICE.DeviceEndpoint[0].EPCFG.reg != 0)
        {
            return;
        }

        mp_hal_delay_ms(10);
    }
    // No USB sync. Use XOSC32KULP as clock reference for DFLL48M
    sync_dfll48_with_xosc32kulp();
#endif
}

// Purpose of the #defines for the clock configuration.
//
// The CPU is either driven by the FDPLL96 oscillator for f >= 48MHz,
// or by the DFLL48M for lower frequencies. The FDPLL96 takes 32768 Hz
// as reference frequency, supplied through GCLK1.
//
// DFLL48M is used for the peripheral clock, e.g. for PWM, UART, SPI, I2C.
// DFLL48M is either free running, or controlled by the 32kHz crystal, or
// Synchronized with the USB clock.
// Both CPU and peripheral devices are clocked by the DFLL48M clock.
// DFLL48M is either free running, or controlled by the 32kHz crystal, or
// Synchronized with the USB clock.
//
// #define MICROPY_HW_XOSC32K (0 | 1)
//
// If MICROPY_HW_XOSC32K = 1, the 32kHz crystal is used as input for GCLK 1, which
// serves as reference clock source for the DFLL48M oscillator,
// The crystal is used, unless MICROPY_HW_MCU_OSC32KULP is set.
// In that case GCLK1 (and the CPU clock) is driven by the 32K Low power oscillator.
// The reason for offering this option is a design flaw of the Adafruit
// Feather boards, where the RGB LED and Debug signals interfere with the
// crystal, causing the CPU to fail if it is driven by the crystal.
//
// If MICROPY_HW_XOSC32K = 0, the 32kHz signal for GCLK1 (and the CPU) is
// created by dividing the 48MHz clock of DFLL48M, but not used otherwise.
//
// If MICROPY_HW_DFLL_USB_SYNC = 0, the DFLL48M oscillator is free running using
// the pre-configured trim values. In that mode, the peripheral clock is
// not exactly 48Mhz and has a substitutional temperature drift.
//
// If MICROPY_HW_DFLL_USB_SYNC = 1, the DFLL48 is synchronized with the 1 kHz USB sync
// signal. If after boot there is no USB sync within 1000 ms, the configuration falls
// back to a free running 48Mhz oscillator.
//
// In all modes, the 48MHz signal has a substantial jitter, largest when
// MICROPY_HW_DFLL_USB_SYNC is active. That is caused by the respective
// reference frequencies of 32kHz or 1 kHz being low. That affects most
// PWM. Std Dev at 1kHz 0.156Hz (w. Crystal) up to 0.4 Hz (with USB sync).
//
// If none of the mentioned defines is set, the device uses the internal oscillators.

void init_clocks(uint32_t cpu_freq)
{

    dfll48m_calibration = 0; // Please the compiler

    // SAML21 Clock settings
    //
    // GCLK0: 48MHz, source: DFLL48M or FDPLL96M, usage: CPU
    // GCLK1: 32kHz, source: XOSC32K or OSCULP32K, usage: FDPLL96M reference
    // GCLK2: 1-48MHz, source: DFLL48M, usage: Peripherals
    // GCLK3: 2Mhz,  source: DFLL48M, usage: us-counter (TC0/TC1)
    // GCLK4: 32kHz, source: XOSC32K or OSCULP32K, usage: DFLL48M reference
    // GCLK5: 48MHz, source: DFLL48M, usage: USB
    // GCLK8: 1kHz,  source: XOSC32K or OSCULP32K, usage: WDT and RTC
    // DFLL48M: Reference sources:
    //          - in closed loop mode: either XOSC32K or OSCULP32K from GCLK4
    //            or USB clock.
    //          - in open loop mode: None
    // FDPLL96M: Reference source GCLK1
    //           Used for the CPU clock for freq >= 48Mhz

    NVMCTRL->CTRLB.bit.MANW = 1; // errata "Spurious Writes"
    NVMCTRL->CTRLB.bit.RWS = 2;  // 1 read wait state for 48MHz

#if MICROPY_HW_XOSC32K
    // Set up OSC32K according data sheet 17.6.3
    OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_STARTUP(0x3) | OSC32KCTRL_XOSC32K_EN32K |
                              OSC32KCTRL_XOSC32K_XTALEN;
    OSC32KCTRL->XOSC32K.bit.ENABLE = 1;
    while (OSC32KCTRL->STATUS.bit.XOSC32KRDY == 0)
    {
    }
    /*SAM L21 Family Data Sheet p.208
    21. OSCCTRL – Oscillators Controller*/
    // Step 1: Set up the reference clock

#if MICROPY_HW_MCU_OSC32KULP
    // Connect the GCLK1 to the XOSC32KULP
    GCLK->GENCTRL[1].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_OE;
#else
    // Connect the GCLK1 to OSC32K
    GCLK->GENCTRL[1].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_OE;
#endif

    while (GCLK->SYNCBUSY.bit.GENCTRL1)
    {
    }

    // Connect the GCLK4 to OSC32K
    GCLK->GENCTRL[4].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K;
    // Connect GCLK4 to the DFLL input.GCLK_DFLL48M_REF
    GCLK->PCHCTRL[0].reg = GCLK_PCHCTRL_GEN_GCLK4 | GCLK_PCHCTRL_CHEN;
    while (GCLK->SYNCBUSY.bit.GENCTRL4)
    {
    }

    // Enable access to the DFLLCTRL register according to Errata 1.2.1
    OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;
    while (OSCCTRL->STATUS.bit.DFLLRDY == 0)
    {
    }
    // Set coarse and fine values
    uint8_t calibCoarse = (uint8_t)(((*(uint32_t *)0x00806020U) >> 26U) & 0x3fU);

    if (calibCoarse == 0x3f)
    {
        calibCoarse = 0x1f;
    }
    OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE(calibCoarse) | OSCCTRL_DFLLVAL_FINE(512);
    while (OSCCTRL->STATUS.bit.DFLLRDY == 0)
    {
    }
    // Step 3: Set the multiplication values. The offset of 16384 to the freq is for rounding.
    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_MUL((CPU_FREQ + 16384) / 32768) |
                           OSCCTRL_DFLLMUL_FSTEP(1) | OSCCTRL_DFLLMUL_CSTEP(1);
    while (OSCCTRL->STATUS.bit.DFLLRDY == 0)
    {
    }
    // Step 4: Start the DFLL and wait for the PLL lock. We just wait for the fine lock, since
    OSCCTRL->DFLLCTRL.reg |= OSCCTRL_DFLLCTRL_MODE | OSCCTRL_DFLLCTRL_WAITLOCK | OSCCTRL_DFLLCTRL_STABLE |
                             OSCCTRL_DFLLCTRL_BPLCKC | OSCCTRL_DFLLCTRL_ENABLE;
    while (OSCCTRL->STATUS.bit.DFLLLCKF == 0)
    {
    }
    // Set GCLK8 to 1 kHz.
    GCLK->GENCTRL[8].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_DIV(32);
    while (GCLK->SYNCBUSY.bit.GENCTRL8)
    {
    }

#else // MICROPY_HW_XOSC32K

    // Connect the GCLK1 to the XOSC32KULP
    GCLK->GENCTRL[1].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K;
    while (GCLK->SYNCBUSY.bit.GENCTRL1)
    {
    }
    // Connect the GCLK4 to the XOSC32KULP
    GCLK->GENCTRL[4].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K;
    while (GCLK->SYNCBUSY.bit.GENCTRL4)
    {
    }

    // Enable DFLL48M
    OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;
    while (!OSCCTRL->STATUS.bit.DFLLRDY)
    {
    }

    uint32_t coarse = (uint32_t)(((*(uint32_t *)0x00806020U) >> 26U) & 0x3fU); // calib data p46
    uint32_t fine = 512U;

    if (coarse == 0x3F)
    {
        coarse = 0x1F; // Adjust invalid calibration
    }

    OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE(coarse) | OSCCTRL_DFLLVAL_FINE(fine);

#if MICROPY_HW_DFLL_USB_SYNC
    // Set the Multiplication factor.
    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP(1) | OSCCTRL_DFLLMUL_FSTEP(1) | OSCCTRL_DFLLMUL_MUL(48000);
    // Set the mode to closed loop USB Recovery mode
    OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_USBCRM | OSCCTRL_DFLLCTRL_CCDIS | OSCCTRL_DFLLCTRL_MODE | OSCCTRL_DFLLCTRL_ENABLE;
    while (!OSCCTRL->STATUS.bit.DFLLRDY)
    {
    }

#else // MICROPY_HW_DFLL_USB_SYNC

    sync_dfll48_with_xosc32kulp();

#endif // MICROPY_HW_DFLL_USB_SYNC

    // Set GCLK8 to 1 kHz.
    GCLK->GENCTRL[8].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIV(32);
    while (GCLK->SYNCBUSY.bit.GENCTRL8)
    {
    }

#endif // MICROPY_HW_XOSC32K

    set_cpu_freq(cpu_freq);

    // Enable GCLK output: 2MHz on GCLK3 for TC0 slave-TC0 master, see samd_soc.c
    GCLK->GENCTRL[3].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_DIV(24);
    while (GCLK->SYNCBUSY.bit.GENCTRL3)
    {
    }
}

void enable_sercom_clock(int id)
{
    // Select multiplexer generic clock source and enable.
    // SAML21 DS. p.141
    GCLK->PCHCTRL[sercom_gclk_id[id]].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK2;
    // Wait while it updates sync
    while (GCLK->SYNCBUSY.bit.GENCTRL2)
    {
    }
}
