#include "board.h"
#include "stm32h7xx.h"
#include <rtthread.h>
#include <rthw.h>

static rt_thread_t tid0;



static void CFA_Init_thread_entry(void *parameter){

    bsp_initialize();
    // todo : 添加任务管理模块以解除注释
    //task_manager_init();
    bsp_post_initialize();
    // task_manager_start();
}


int rt_application_init(){
    tid0 = rt_thread_create("CFA Init", 
                            CFA_Init_thread_entry, 
                            RT_NULL, 
                            8092, 
                            RT_THREAD_PRIORITY_MAX/2, 
                            20);
    if(tid0 != RT_NULL)
        rt_thread_startup(tid0);
    return 0;
}

void rtthread_startup(void)
{
    /* disable interrupt first */
    rt_hw_interrupt_disable();

    // rt_assert_set_hook(assert_hook);

    /* board level initialization
     * NOTE: please initialize heap inside board initialization.
     */
    rt_hw_board_init();

    /* init timer system */
    rt_system_timer_init();

    /* init scheduler system */
    rt_system_scheduler_init();

    /* init application */
    rt_application_init();

    /* init timer thread */
    rt_system_timer_thread_init();

    /* init idle thread */
    rt_thread_idle_init();

#ifdef RT_USING_SMP
    rt_hw_spin_lock(&_cpus_lock);
#endif /*RT_USING_SMP*/

    /* start scheduler */
    rt_system_scheduler_start();

    /* never reach here */
    return;
}

int entry(void){
    rtthread_startup();
    return 0;
}
