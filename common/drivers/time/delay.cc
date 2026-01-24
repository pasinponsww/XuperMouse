#include "delay.h"

#ifdef STM32L476xx
#include "stm32l476xx.h"
#endif

namespace MM::Utils
{

void DelayMs(uint32_t ms)
{
#ifdef STM32F4xx
    // Embedded target: busy-wait loop
    for (uint32_t i = 0; i < ms * 4000; i++)
    {
        __NOP();
    }
#else
    (void)ms;
#endif
}

void DelayUs(uint32_t us)
{
#ifdef STM32F4xx
    for (uint32_t i = 0; i < us * 4; i++)
    {
        __NOP();
    }
#else
    (void)us;
#endif
}

}  // namespace MM::Utils
