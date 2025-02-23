#include "generated/ti_enet_config.h"
/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
__TI_STACK_SIZE = 8192;
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
__TI_HEAP_SIZE = 34000;
ENTRY(_vectors)  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */

/* This is the size of stack when R5 is in IRQ mode
 * In NORTOS,
 * - Here interrupt nesting is disabled as of now
 * - This is the stack used by ISRs registered as type IRQ
 * In FreeRTOS,
 * - Here interrupt nesting is enabled
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



/*
NOTE: Below memory is reserved for DMSC usage
 - During Boot till security handoff is complete
   0x701E0000 - 0x701FFFFF (128KB)
 - After "Security Handoff" is complete (i.e at run time)
   0x701FC000 - 0x701FFFFF (16KB)

 Security handoff is complete when this message is sent to the DMSC,
   TISCI_MSG_SEC_HANDOVER

 This should be sent once all cores are loaded and all application
 specific firewall calls are setup.
*/

MEMORY
{
    R5F_VECS  : ORIGIN = 0x00000000 , LENGTH = 0x00000040
    R5F_TCMA  : ORIGIN = 0x00000040 , LENGTH = 0x00007FC0
    R5F_TCMB0 : ORIGIN = 0x41010000 , LENGTH = 0x00008000

    /* when using multi-core application's i.e more than one R5F/M4F active, make sure
     * this memory does not overlap with other R5F's
     */
    MSRAM     : ORIGIN = 0x70080000 , LENGTH = 0x160000

    /* This section can be used to put XIP section of the application in flash, make sure this does not overlap with
     * other CPUs. Also make sure to add a MPU entry for this section and mark it as cached and code executable
     */
    FLASH   : ORIGIN = 0x60200000 , LENGTH = 0x100000 
    DDR     : ORIGIN = 0x80000000 , LENGTH = 0x120000 


}

SECTIONS
{
    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors : ALIGN (8) {}  > R5F_VECS

    /* This has the R5F boot code until MPU is enabled,  this MUST be at a address < 0x80000000
     * i.e this cannot be placed in DDR
     */
    .text.hwi   : ALIGN (8) {} > MSRAM
    .text.cache : ALIGN (8) {} > MSRAM
    .text.mpu   : ALIGN (8) {} > MSRAM
    .text.boot  : ALIGN (8) {} > MSRAM
    .text:abort : ALIGN (8) {} > MSRAM  /* this helps in loading symbols when using XIP mode */

    OVERLAY : NOCROSSREFS
    {
        .icssfw {
        }

        .icss_mem {
#if (ENET_SYSCFG_ICSSG0_ENABLED == 1)
    #if(ENET_SYSCFG_DUAL_MAC == 1)
        #if(ENET_SYSCFG_DUALMAC_PORT1_ENABLED == 1)
            *(*gEnetSoc_icssg0HostPoolMem_0)
            *(*gEnetSoc_icssg0HostQueueMem_0)
            *(*gEnetSoc_icssg0ScratchMem_0)
            #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
                *(*gEnetSoc_icssg0HostPreQueueMem_0)
            #endif
        #else
            *(*gEnetSoc_icssg0HostPoolMem_1)
            *(*gEnetSoc_icssg0HostQueueMem_1)
            *(*gEnetSoc_icssg0ScratchMem_1)
            #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
                    *(*gEnetSoc_icssg0HostPreQueueMem_1)
            #endif
        #endif
    #endif
#endif
#if (ENET_SYSCFG_ICSSG1_ENABLED == 1)
    #if(ENET_SYSCFG_DUAL_MAC == 1)
        #if(ENET_SYSCFG_DUALMAC_PORT1_ENABLED == 1)
                *(*gEnetSoc_icssg1HostPoolMem_0)
                *(*gEnetSoc_icssg1HostQueueMem_0)
                *(*gEnetSoc_icssg1ScratchMem_0)
            #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
                *(*gEnetSoc_icssg1HostPreQueueMem_0)
            #endif
        #else
                *(*gEnetSoc_icssg1HostPoolMem_1)
                *(*gEnetSoc_icssg1HostQueueMem_1)
                *(*gEnetSoc_icssg1ScratchMem_1)
            #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
                *(*gEnetSoc_icssg1HostPreQueueMem_1)
            #endif
        #endif
    #endif
#endif
#if (ENET_SYSCFG_ICSSG0_ENABLED == 1)
    #if(ENET_SYSCFG_DUAL_MAC == 0)
        *(*gEnetSoc_icssg0PortPoolMem_0)
        *(*gEnetSoc_icssg0PortPoolMem_1)
        *(*gEnetSoc_icssg0HostPoolMem_0)
        *(*gEnetSoc_icssg0HostPoolMem_1)
        *(*gEnetSoc_icssg0HostQueueMem_0)
        *(*gEnetSoc_icssg0HostQueueMem_1)
        *(*gEnetSoc_icssg0ScratchMem_0)
        *(*gEnetSoc_icssg0ScratchMem_1)
        #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
            *(*gEnetSoc_icssg0HostPreQueueMem_0)
            *(*gEnetSoc_icssg0HostPreQueueMem_1)
        #endif
    #endif
#endif
#if (ENET_SYSCFG_ICSSG1_ENABLED == 1)
    #if(ENET_SYSCFG_DUAL_MAC == 0)
        *(*gEnetSoc_icssg1PortPoolMem_0)
        *(*gEnetSoc_icssg1PortPoolMem_1)
        *(*gEnetSoc_icssg1HostPoolMem_0)
        *(*gEnetSoc_icssg1HostPoolMem_1)
        *(*gEnetSoc_icssg1HostQueueMem_0)
        *(*gEnetSoc_icssg1HostQueueMem_1)
        *(*gEnetSoc_icssg1ScratchMem_0)
        *(*gEnetSoc_icssg1ScratchMem_1)
        #if (ENET_SYSCFG_PREMPTION_ENABLE == 1)
            *(*gEnetSoc_icssg1HostPreQueueMem_0)
            *(*gEnetSoc_icssg1HostPreQueueMem_1)
        #endif
    #endif
#endif
        }
    } > MSRAM


    /* This is rest of code. This can be placed in DDR if DDR is available and needed */
    .text       : ALIGN (8) {} > DDR  /* This is where code resides */
    .rodata     : ALIGN (8) {} > DDR  /* This is where const's go */

    /* This is rest of initialized data. This can be placed in DDR if DDR is available and needed */
    .data           : ALIGN (8)    {} > DDR   /* This is where initialized globals and static go */

    /* This is rest of uninitialized data. This can be placed in DDR if DDR is available and needed */
    .bss : {
        __bss_start__ = .;
        __BSS_START = .;
        *(.bss)
        *(.bss.*)
        . = ALIGN (8);
        __BSS_END = .;
        __bss_end__ = .;
        . = ALIGN (8);
    } > DDR

    /* This is where the stacks for different R5F modes go */
    .irqstack : ALIGN(16) {
        __IRQ_STACK_START = .;
        . = . + __IRQ_STACK_SIZE;
        __IRQ_STACK_END = .;
        . = ALIGN (8);
    } > MSRAM
    .fiqstack : ALIGN(16) {
        __FIQ_STACK_START = .;
        . = . + __FIQ_STACK_SIZE;
        __FIQ_STACK_END = .;
    } > MSRAM
    .svcstack : ALIGN(16) {
        __SVC_STACK_START = .;
        . = . + __SVC_STACK_SIZE;
        __SVC_STACK_END = .;
    } > MSRAM
    .abortstack : ALIGN(16) {
        __ABORT_STACK_START = .;
        . = . + __ABORT_STACK_SIZE;
        __ABORT_STACK_END = .;
    } > MSRAM
    .undefinedstack : ALIGN(16) {
        __UNDEFINED_STACK_START = .;
        . = . + __UNDEFINED_STACK_SIZE;
        __UNDEFINED_STACK_END = .;
    } > MSRAM

    .heap (NOLOAD) : {
        end = .;
        KEEP(*(.heap))
        . = . + __TI_HEAP_SIZE;
    } > MSRAM

    .stack (NOLOAD) : ALIGN(16) {
        __TI_STACK_BASE = .;
        KEEP(*(.stack))
        . = . + __TI_STACK_SIZE;
        __STACK_END = .;
    } > DDR

    .enet_dma_mem (NOLOAD) : ALIGN(128) {
        *(.bss:ENET_DMA_DESC_MEMPOOL)
        *(.bss:ENET_DMA_RING_MEMPOOL)
#if (ENET_SYSCFG_PKT_POOL_ENABLE == 1)
        *(.bss:ENET_DMA_PKT_MEMPOOL)
#endif
    } > DDR

    .bss:ENET_DMA_OBJ_MEM (NOLOAD) : ALIGN (128) {} > MSRAM
    .bss:ENET_DMA_PKT_INFO_MEMPOOL (NOLOAD) : ALIGN (128) {} > MSRAM
    .bss:ENET_ICSSG_OCMC_MEM (NOLOAD) : ALIGN (128) {} > MSRAM
}
