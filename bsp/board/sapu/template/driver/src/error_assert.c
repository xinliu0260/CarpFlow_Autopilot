#include "rthw.h"
#include "rtthread.h"

#define DBG_TAG "error_assert"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

void _Error_Handler(char *s, int num)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    LOG_E("Error_Handler at file:%s num:%d", s, num);
    while (1)
    {
    }
    /* USER CODE END Error_Handler */
}


void assert_failed(uint8_t* file, uint32_t line)
{
    rt_hw_interrupt_disable();
// TODO:添加使用cmbacktrace
#ifdef USING_CM_BACKTRACE
    cm_backtrace_assert(cmb_get_sp());
#endif

    while (1)
        ;
}

void assert_hook(const char* ex, const char* func, rt_size_t line)
{
    // TODO: 修改诊断构造函数
    //console_printf("(%s) assertion failed at function:%s, line number:%ld \n", ex, func, line);
    // TODO:添加宏USING_CHECKED
#ifdef USING_CHECKED
    assert_failed((uint8_t*)func, (uint32_t)line);
#endif
}