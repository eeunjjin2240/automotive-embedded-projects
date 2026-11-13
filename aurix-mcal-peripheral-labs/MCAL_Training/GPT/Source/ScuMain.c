#include "Gpt.h"

static uint32 timeEcuAlive1ms = 0u;
static uint32 systemTimerLatest;
static uint32 systemTimerPrevious = 0u;
static uint32 timerDiffUs = 0u;

#define GptChannelId_0 0u

void GptNotification_1ms(void)
{
    systemTimerLatest = *((uint32 *)(0xF0000010u)); /* 80MHz */
    timeEcuAlive1ms++;
    timerDiffUs = (systemTimerLatest - systemTimerPrevious) / 80;
    systemTimerPrevious = systemTimerLatest;
}

void ScuMain(void)
{
    Gpt_EnableNotification((Gpt_ChannelType) GptChannelId_0);
    Gpt_StartTimer(GptChannelId_0, 5000u );  /* 80MHz/16/5000tick = 1000hz = 1ms */
    while(1);
}
