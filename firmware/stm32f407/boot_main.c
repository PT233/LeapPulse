#include "flash_slots.h"
#include "stm32f4xx_hal.h"

static __attribute__((naked, noreturn)) void jump_to_app(uint32_t sp, uint32_t reset)
{
    __asm volatile ("msr msp, r0\n"
                    "cpsie i\n"
                    "bx r1\n");
}

int main(void)
{
    HAL_Init();
    if (lp_flash_install_pending() && lp_flash_application_valid()) {
        const uint32_t *vectors = (const void *)LP_APPLICATION_BASE;
        __disable_irq();
        SysTick->CTRL = 0;
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
        SCB->VTOR = LP_APPLICATION_BASE;
        __DSB();
        __ISB();
        jump_to_app(vectors[0], vectors[1]);
    }

    for (;;) {
        __WFI();
    }
}
