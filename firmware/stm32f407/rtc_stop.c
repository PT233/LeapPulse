#include "board_port.h"
#include "FreeRTOS.h"
#include "task.h"

#if configUSE_TICKLESS_IDLE == 2

#include "rtc_stop.h"

#define RTC_WAKE_HZ 2048u
#define MAX_SLEEP_MS 1000u

static RTC_HandleTypeDef *clock_rtc;
static void (*restore_clock_tree)(void);

void lp_stm32_stop_init(RTC_HandleTypeDef *rtc, void (*restore_clock)(void))
{
    clock_rtc = rtc;
    restore_clock_tree = restore_clock;
    __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_IT();
    __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_RISING_EDGE();
}

static uint32_t rtc_ticks(uint32_t per_second)
{
    RTC_TypeDef *rtc = clock_rtc->Instance;
    uint32_t subsecond = rtc->SSR;
    uint32_t time = rtc->TR;
    /* Reading DR releases the RTC's shadow snapshot. */
    (void)rtc->DR;
    uint32_t hours = ((time >> 20) & 3u) * 10u + ((time >> 16) & 15u);
    uint32_t minutes = ((time >> 12) & 7u) * 10u + ((time >> 8) & 15u);
    uint32_t seconds = ((time >> 4) & 7u) * 10u + (time & 15u);
    return (hours * 3600u + minutes * 60u + seconds) * per_second +
           per_second - 1u - subsecond;
}

static bool arm_timer(uint32_t ticks)
{
    RTC_TypeDef *rtc = clock_rtc->Instance;
    __HAL_RTC_WRITEPROTECTION_DISABLE(clock_rtc);
    CLEAR_BIT(rtc->CR, RTC_CR_WUTE);

    /* Bound WUTWF waiting because the HAL timebase is masked here. */
    uint32_t attempts = SystemCoreClock / 1000u;
    while (!(rtc->ISR & RTC_ISR_WUTWF) && attempts) {
        --attempts;
    }
    if (!attempts) {
        __HAL_RTC_WRITEPROTECTION_ENABLE(clock_rtc);
        return false;
    }

    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(clock_rtc, RTC_FLAG_WUTF);
    __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
    rtc->WUTR = ticks - 1u;
    MODIFY_REG(rtc->CR, RTC_CR_WUCKSEL, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    SET_BIT(rtc->CR, RTC_CR_WUTIE | RTC_CR_WUTE);
    __HAL_RTC_WRITEPROTECTION_ENABLE(clock_rtc);
    return true;
}

static void cancel_timer(void)
{
    __HAL_RTC_WRITEPROTECTION_DISABLE(clock_rtc);
    CLEAR_BIT(clock_rtc->Instance->CR, RTC_CR_WUTE | RTC_CR_WUTIE);
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(clock_rtc, RTC_FLAG_WUTF);
    __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    __HAL_RTC_WRITEPROTECTION_ENABLE(clock_rtc);
}

void vPortSuppressTicksAndSleep(TickType_t expected_ticks)
{
    if (expected_ticks < 3u || !clock_rtc || !restore_clock_tree || lp_board_hc05_connected()) {
        __DSB();
        __WFI();
        return;
    }

    __disable_irq();
    __DSB();
    __ISB();
    if (eTaskConfirmSleepModeStatus() == eAbortSleep ||
        (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)) {
        __enable_irq();
        return;
    }

    if (expected_ticks > MAX_SLEEP_MS) {
        expected_ticks = MAX_SLEEP_MS;
    }
    uint32_t wake_ticks = ((expected_ticks - 1u) * RTC_WAKE_HZ) / 1000u;
    uint32_t per_second = (clock_rtc->Instance->PRER & RTC_PRER_PREDIV_S) + 1u;
    if (per_second < 1000u || !arm_timer(wake_ticks)) {
        __enable_irq();
        return;
    }

    uint32_t control = SysTick->CTRL;
    uint32_t reload = SysTick->LOAD;
    SysTick->CTRL = 0;
    HAL_SuspendTick();
    uint32_t before = rtc_ticks(per_second);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    restore_clock_tree();
    uint32_t after = rtc_ticks(per_second);
    cancel_timer();
    HAL_ResumeTick();

    uint32_t day = per_second * 86400u;
    uint32_t elapsed = after >= before ? after - before : after + day - before;
    uint32_t ms = (uint32_t)(((uint64_t)elapsed * 1000u + per_second / 2u) / per_second);
    if (ms >= expected_ticks) {
        ms = expected_ticks - 1u;
    }
    vTaskStepTick(ms);
    SysTick->LOAD = reload;
    SysTick->VAL = 0;
    SysTick->CTRL = control;
    __enable_irq();
}
#endif
