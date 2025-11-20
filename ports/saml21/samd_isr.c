/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Damien P. George
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

#include "py/runtime.h"
#include "py/mphal.h"
#include "samd_soc.h"

#include "pendsv.h"
#include "shared/runtime/softtimer.h"

typedef void (*ISR)(void);

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern void Default_Handler(void);
extern void SysTick_Handler(void);
extern void PendSV_Handler(void);
extern void EIC_Handler(void);

const ISR isr_vector[];
volatile uint32_t systick_ms;
volatile uint32_t ticks_us64_upper;
#if defined(MCU_SAML21)
volatile uint32_t rng_state;
#endif
void Reset_Handler(void) __attribute__((naked));
void Reset_Handler(void)
{
// Set stack pointer
#if __CORTEX_M >= 0x03
    __asm volatile("ldr sp, =_estack");
#else
    __asm volatile("ldr r0, =_estack");
    __asm volatile("mov sp, r0");
#endif
    // Copy .data section from flash to RAM
    for (uint32_t *src = &_sidata, *dest = &_sdata; dest < &_edata;)
    {
        *dest++ = *src++;
    }
    // Zero out .bss section
    for (uint32_t *dest = &_sbss; dest < &_ebss;)
    {
        *dest++ = 0;
    }

    // When we get here: stack is initialised, bss is clear, data is copied

#if __FPU_PRESENT == 1 && __FPU_USED == 1
    // Set CP10 and CP11 Full Access
    SCB->CPACR |= (3UL << 10 * 2) | (3UL << 11 * 2);
#endif

    // SCB->VTOR
    *((volatile uint32_t *)0xe000ed08) = (uint32_t)&isr_vector;

    // SCB->CCR: enable 8-byte stack alignment for IRQ handlers, in accord with EABI
    *((volatile uint32_t *)0xe000ed14) |= 1 << 9;

#if defined(MCU_SAML21)
    // CHOOSE POWER LEVEL 2  needed for USB
    PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2;
#endif

    // Initialise the cpu and peripherals
    samd_init();

    // Now that we have a basic system up and running we can call main
    samd_main();

    // we must not return
    for (;;)
    {
    }
}
void Default_Handler(void)
{
    for (;;)
    {
    }
}

void SysTick_Handler(void)
{
#if defined(MCU_SAML21)
    // Use the phase jitter between the clocks to get some entropy
    // and accumulate the random number register with a "spiced" LCG.
    rng_state = (rng_state * 32310901 + 1) ^ (REG_TC0_COUNT32_COUNT >> 1);
#endif
    uint32_t next_tick = systick_ms + 1;
    systick_ms = next_tick;

    if (soft_timer_next == next_tick)
    {
        pendsv_schedule_dispatch(PENDSV_DISPATCH_SOFT_TIMER, soft_timer_handler);
    }
}

void us_timer_IRQ(void)
{
#if defined(MCU_SAML21)
    if (TC0->COUNT32.INTFLAG.reg & TC_INTFLAG_OVF)
    {
        ticks_us64_upper++;
    }
    TC0->COUNT32.INTFLAG.reg = TC_INTFLAG_OVF;
#endif
}

// Sercom IRQ handler support
void (*sercom_irq_handler_table[SERCOM_INST_NUM])(int num) = {};

void sercom_register_irq(int sercom_id, void(*sercom_irq_handler))
{
    if (sercom_id < SERCOM_INST_NUM)
    {
        sercom_irq_handler_table[sercom_id] = sercom_irq_handler;
    }
}

static inline void common_sercom_irq_handler(int sercom_id)
{
    if (sercom_irq_handler_table[sercom_id])
    {
        sercom_irq_handler_table[sercom_id](sercom_id);
    }
}

void Sercom0_Handler(void)
{
    common_sercom_irq_handler(0);
}
void Sercom1_Handler(void)
{
    common_sercom_irq_handler(1);
}
void Sercom2_Handler(void)
{
    common_sercom_irq_handler(2);
}
void Sercom3_Handler(void)
{
    common_sercom_irq_handler(3);
}
void Sercom4_Handler(void)
{
    common_sercom_irq_handler(4);
}
void Sercom5_Handler(void)
{
    common_sercom_irq_handler(5);
}

#if defined(MCU_SAML21)
const ISR isr_vector[] __attribute__((section(".isr_vector"))) = {
    (ISR)&_estack,
    &Reset_Handler,
    &Default_Handler, // NMI_Handler
    &Default_Handler, // HardFault_Handler
    &Default_Handler, // MemManage_Handler
    &Default_Handler, // BusFault_Handler
    &Default_Handler, // UsageFault_Handler
    0,
    0,
    0,
    0,
    &Default_Handler, //  SVC_Handler
    &Default_Handler, //  DebugMon_Handler
    0,
    &PendSV_Handler,     //  PendSV_Handler
    &SysTick_Handler,    //  SysTick_Handler
    0,                   //  0 shared (MCLK)(OSC32KCTRL)  (OSCCTRL) (PAC) (PM) (SUPC)
    0,                   //  1  Watchdog Timer (WDT)
    0,                   //  2 Real-Time Counter (RTC)
    &EIC_Handler,        //  3  External Interrupt Controller (EIC)
    0,                   //  4 Non-Volatile Memory Controller (NVMCTRL)
    0,                   //  5 Direct Memory Access Controller (DMAC)
    USB_Handler_wrapper, //  6 Universal Serial Bus (USB)
    0,                   //  7 Event System Interface (EVSYS)
    &Sercom0_Handler,    //  8 Serial Communication Interface (SERCOM0)
    &Sercom1_Handler,    //  9 Serial Communication Interface (SERCOM1)
    &Sercom2_Handler,    // 10 Serial Communication Interface (SERCOM2)
    &Sercom3_Handler,    // 11 Serial Communication Interface (SERCOM3)
    &Sercom4_Handler,    // 12 Serial Communication Interface (SERCOM4)
    &Sercom5_Handler,    // 13 Serial Communication Interface (SERCOM5)
    0,                   // 14 Timer Counter Control (TCC0)
    0,                   // 15 Timer Counter Control (TCC1)
    0,                   // 16 Timer Counter Control (TCC2)
    &us_timer_IRQ,       // 17 Basic Timer Counter (TC0)
    0,                   // 18 Basic Timer Counter (TC1)
    0,                   // 19 Basic Timer Counter (TC2)
    0,                   // 20 Basic Timer Counter (TC3)
    0,                   // 21 Basic Timer Counter (TC4)
    0,                   // 22 Analog Digital Converter (ADC)
    0,                   // 23 Analog Comparators (AC)
    0,                   // 24 Digital-to-Analog Converter (DAC) NO DAC For WLR089U0
    0,                   // 25 Peripheral Touch Controller (PTC)
    0,                   // 26 Advanced Encryption Standard (AES)
    0,                   // 27 True Random Generator (TRNG)
};
#endif
