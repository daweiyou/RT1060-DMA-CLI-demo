#include "fsl_gpt.h"

uint32_t McuRTOS_RunTimeCounter; /* runtime counter, used for configGENERATE_RUNTIME_STATS */

void GPT2_IRQHandler(void) {
  /* Clear interrupt flag.*/
  GPT_ClearStatusFlags(GPT2, kGPT_OutputCompare1Flag);
  McuRTOS_RunTimeCounter++; /* increment runtime counter */
#if defined __CORTEX_M && (__CORTEX_M == 4U || __CORTEX_M == 7U)
  __DSB();
#endif
}
uint32_t AppGetRuntimeCounterValueFromISR()
{
    return McuRTOS_RunTimeCounter;

}

void AppConfigureTimerForRuntimeStats(void) {
  uint32_t gptFreq;
  gpt_config_t gptConfig;
 
  GPT_GetDefaultConfig(&gptConfig);
 
  /* Initialize GPT module */
  GPT_Init(GPT2, &gptConfig);
 
  /* Divide GPT clock source frequency by 3 inside GPT module */
  GPT_SetClockDivider(GPT2, 3);
 
  /* Get GPT clock frequency */
  gptFreq = CLOCK_GetFreq(kCLOCK_PerClk);
 
  /* GPT frequency is divided by 3 inside module */
  gptFreq /= 3;
 
  /* Set GPT module to 10x of the FreeRTOS tick counter */
  gptFreq = USEC_TO_COUNT(100, gptFreq); /* FreeRTOS tick is 1 kHz */
  GPT_SetOutputCompareValue(GPT2, kGPT_OutputCompare_Channel1, gptFreq);
 
  /* Enable GPT Output Compare1 interrupt */
  GPT_EnableInterrupts(GPT2, kGPT_OutputCompare1InterruptEnable);
 
  /* Enable at the Interrupt and start timer */
  EnableIRQ(GPT2_IRQn);
  GPT_StartTimer(GPT2);
}



