#include "board.h"
#include "error_assert.h"
static void SystemClock_Config(void);
static void PeriphCommonClock_Config(void);
void bsp_early_initialize(void){
    /* enable cache ----------------------------------------------------------------------*/
	#ifdef BSP_SCB_ENABLE_I_CACHE
	SCB_EnableICache();
	#endif
	#ifdef BSP_SCB_ENABLE_D_CACHE
	SCB_EnableDCache();
	#endif
    	/* initialize the Dynamic memory heap ----------------------------------------------- */
	#if defined(RT_USING_HEAP)
	/** 
	 * Dynamic memory heap initialization. When the memory management algorithm is
	 * set to memheap, the rt_system_heap_init function is called once 
	 * to initialize the system_heap in kservice.c for dynamic heap allocation. 
	 * Additional memory blocks for dynamic allocation can then be added by 
	 * repeatedly calling rt_memheap_init.
	 */
	static struct rt_memheap sram1_heap;
	static struct rt_memheap sram2_heap;
	static struct rt_memheap sram3_heap;
	rt_system_heap_init((void *)HEAP_BEGIN, (void *)STM32_AXI_SRAM_END);
	rt_memheap_init(&sram1_heap, "sram1", (void *)STM32_SRAM1_BEGIN, STM32_SRAM1_SIZE*1024);
	rt_memheap_init(&sram2_heap, "sram2", (void *)STM32_SRAM2_BEGIN, STM32_SRAM2_SIZE*1024);
	rt_memheap_init(&sram3_heap, "sram3", (void *)STM32_SRAM3_BEGIN, STM32_SRAM3_SIZE*1024);
	#endif

    /* HAL library initialization */
    HAL_Init();

    /* System clock initialization */
    SystemClock_Config();
	PeriphCommonClock_Config();
}

void bsp_initialize(void){



}

void bsp_post_initialize(void){


}

void rt_hw_board_init()
{
    bsp_early_initialize();
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 400;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
    Error_Handler();
    }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
static void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Initializes the peripherals clock
    */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 2;
    PeriphClkInitStruct.PLL2.PLL2N = 40;
    PeriphClkInitStruct.PLL2.PLL2P = 8;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}