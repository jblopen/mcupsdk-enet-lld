#include "ti_enet_config.h"

/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=16384
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=32768
-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */

/* This is the size of stack when R5 is in IRQ mode
 * In NORTOS,
 * - Here interrupt nesting is enabled
 * - This is the stack used by ISRs registered as type IRQ
 * In FreeRTOS,
 * - Here interrupt nesting is disabled
 * - This is stack that is used initally when a IRQ is received
 * - But then the mode is switched to SVC mode and SVC stack is used for all user ISR callbacks
 * - Hence in FreeRTOS, IRQ stack size is less and SVC stack size is more
 */
__IRQ_STACK_SIZE = 256;
/* This is the size of stack when R5 is in IRQ mode
 * - In both NORTOS and FreeRTOS nesting is disabled for FIQ
 */
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 4096; /* This is the size of stack when R5 is in SVC mode */
__ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */
__UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */

SECTIONS
{
    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > R5F_VECS

    /* This has the R5F boot code until MPU is enabled,  this MUST be at a address < 0x80000000
     * i.e this cannot be placed in R50_0_OCRAM
     */
    GROUP {
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)
        .text.boot: palign(8)
        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
    } > R5F_TCMA

    /* This is rest of code. This can be placed in R50_0_OCRAM if R50_0_OCRAM is available and needed */
    GROUP {
        .text:   {} palign(8)   /* This is where code resides */
        .rodata: {} palign(8)   /* This is where const's go */
    } > R50_0_OCRAM

    /* This is rest of initialized data. This can be placed in R50_0_OCRAM if R50_0_OCRAM is available and needed */
    GROUP {
        .data:   {} palign(8)   /* This is where initialized globals and static go */
    } > R50_0_OCRAM

    /* This is rest of uninitialized data. This can be placed in R50_0_OCRAM if R50_0_OCRAM is available and needed */
    GROUP {
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
        .sysmem: {} palign(8)   /* This is where the malloc heap goes */
        .stack:  {} palign(8)   /* This is where the main() stack goes */
    } > R50_0_OCRAM

    /* This is where the stacks for different R5F modes go */
    GROUP {
        .irqstack: {. = . + __IRQ_STACK_SIZE;} align(8)
        RUN_START(__IRQ_STACK_START)
        RUN_END(__IRQ_STACK_END)
        .fiqstack: {. = . + __FIQ_STACK_SIZE;} align(8)
        RUN_START(__FIQ_STACK_START)
        RUN_END(__FIQ_STACK_END)
        .svcstack: {. = . + __SVC_STACK_SIZE;} align(8)
        RUN_START(__SVC_STACK_START)
        RUN_END(__SVC_STACK_END)
        .abortstack: {. = . + __ABORT_STACK_SIZE;} align(8)
        RUN_START(__ABORT_STACK_START)
        RUN_END(__ABORT_STACK_END)
        .undefinedstack: {. = . + __UNDEFINED_STACK_SIZE;} align(8)
        RUN_START(__UNDEFINED_STACK_START)
        RUN_END(__UNDEFINED_STACK_END)
    } > R50_0_OCRAM

    /* Sections needed for C++ projects */
    GROUP {
        .ARM.exidx:  {} palign(8)   /* Needed for C++ exception handling */
        .init_array: {} palign(8)   /* Contains function pointers called before main */
        .fini_array: {} palign(8)   /* Contains function pointers called after main */
    } > R50_0_OCRAM

        .enet_dma_mem (NOLOAD) : {
        *(*ENET_DMA_DESC_MEMPOOL)
        *(*ENET_DMA_RING_MEMPOOL)
#if (ENET_SYSCFG_PKT_POOL_ENABLE == 1)
        *(*ENET_DMA_PKT_MEMPOOL)
#endif
    } > R50_0_OCRAM

}

MEMORY
{
    R5F_VECS  : ORIGIN = 0x00000000 , LENGTH = 0x00000040
    R5F_TCMA  : ORIGIN = 0x00000040 , LENGTH = 0x00007FC0
    R5F_TCMB0 : ORIGIN = 0x41010000 , LENGTH = 0x00004000

    WKUP_R5_OCRAM (RWIX)         : ORIGIN = 0x72000000 LENGTH = 0x00080000 // 512 KB for wakeup core

    R50_0_OCRAM   (RWIX)         : ORIGIN = 0x72080000 LENGTH = 0x00100000 // 1 MB for r5fss0-0 core
    R50_1_OCRAM   (RWIX)         : ORIGIN = 0x72180000 LENGTH = 0x00080000 // 512 KB for r5fss0-1 core
    R51_0_OCRAM   (RWIX)         : ORIGIN = 0x72280000 LENGTH = 0x00080000 // 512 KB for r5fss1-0 core
    R51_1_OCRAM   (RWIX)         : ORIGIN = 0x72300000 LENGTH = 0x00080000 // 512 KB for r5fss1-1 core

    C75_0_OCRAM   (RWIX)         : ORIGIN = 0x72200000 LENGTH = 0x00080000 // 512 KB for c75ss0-0 core
    C75_1_OCRAM   (RWIX)         : ORIGIN = 0x72400000 LENGTH = 0x00080000 // 512 KB for c75ss1-0 core

    LOG_SHM_MEM                  : ORIGIN = 0x72380000, LENGTH = 0x40000

}
