#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif


// Board Information
#define TARGET_NAME  "SAPU"

// RAM definition
#define STM32_FLASH_START_ADRESS     ((uint32_t)0x08000000)
#define STM32_FLASH_SIZE             (2048 * 1024)
#define STM32_FLASH_END_ADDRESS      ((uint32_t)(STM32_FLASH_START_ADRESS + STM32_FLASH_SIZE))

#define STM32_AXI_SRAM_SIZE           (512)
#define STM32_AXI_SRAM_END            (0x24000000 + STM32_AXI_SRAM_SIZE * 1024)

#define STM32_SRAM1_SIZE              (128)
#define STM32_SRAM1_BEGIN             (0x30000000)
#define STM32_SRAM1_END               (0x30000000 + STM32_SRAM1_SIZE * 1024)

#define STM32_SRAM2_SIZE              (128)
#define STM32_SRAM2_BEGIN             (0x30020000)
#define STM32_SRAM2_END               (0x30020000 + STM32_SRAM2_SIZE * 1024)

#define STM32_SRAM3_SIZE              (32)
#define STM32_SRAM3_BEGIN             (0x30040000)
#define STM32_SRAM3_END               (0x30040000 + STM32_SRAM3_SIZE * 1024)

#define STM32_SRAM4_SIZE              (64)
#define STM32_SRAM4_BEGIN             (0x38000000)
#define STM32_SRAM4_END               (0x38000000 + STM32_SRAM4_SIZE * 1024)

#define STM32_BACKUP_SRAM_SIZE        (4)
#define STM32_BACKUP_SRAM_BEGIN       (0x38800000)
#define STM32_BACKUP_SRAM_END         (0x38800000 + STM32_BACKUP_SRAM_SIZE * 1024)

// 定义堆起始地址
#if defined(__ARMCC_VERSION)
extern int Image$$AIX_SRAM$$ZI$$Limit;
#define HEAP_BEGIN      (&Image$$AIX_SRAM$$ZI$$Limit) // end of section ZI in AIXSARM 
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#else
extern int __bss_end;
#define HEAP_BEGIN      (&__bss_end)
#endif

#define HEAP_END        STM32_AXI_SRAM_END

// 中断优先级分组
#ifndef NVIC_PRIORITYGROUP_0
    #define NVIC_PRIORITYGROUP_0 ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority, \
                                                             4 bits for subpriority */
    #define NVIC_PRIORITYGROUP_1 ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority, \
                                                             3 bits for subpriority */
    #define NVIC_PRIORITYGROUP_2 ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority, \
                                                             2 bits for subpriority */
    #define NVIC_PRIORITYGROUP_3 ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority, \
                                                             1 bit  for subpriority */
    #define NVIC_PRIORITYGROUP_4 ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority, \
                                                             0 bit  for subpriority */
#endif

// 引脚定义
#define GREEN_LED_PIN       GET_PIN(A, 1)
#define RED_LED_PIN         GET_PIN(A, 2)
#define BLUE_LED_PIN        GET_PIN(A, 3)


void rt_hw_board_init(void);
void bsp_early_initialize(void);
void bsp_initialize(void);
void bsp_post_initialize(void);
void board_show_version(void);


#ifdef __cplusplus
}
#endif

#endif