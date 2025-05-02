/**
 * @file dvc_dwt.cpp
 * @author lez
 * @brief DWT
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#include "dvc_dwt.h"

static void DWT_CNT_Update(void);

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}



    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

void DWT_SysTimeUpdate(void){
    DWT_CNT_Update();
    volatile uint32_t cnt_now = DWT->CYCCNT;
    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)uint32_max + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;

    CYCCNT_LAST = cnt_now;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}

// 帧率计数器更新函数
float FPS_Counter_Update() 
{
    static uint32_t last_cnt = 0;  // 保存上一次的 DWT 计数器值
    static uint32_t interrupt_count = 0;
    static float time_elapsed = 0;
    static float frame_rate = 0.0f;  // 保存上一次的帧率值
    float dt = DWT_GetDeltaT(&last_cnt);

    // 累计触发次数和时间
    interrupt_count++;
    time_elapsed += dt;

    // 计算帧率
    float current_frame_rate = (float)interrupt_count / time_elapsed;

    // 使用滑动平均或其他滤波算法平滑帧率变化
    frame_rate = 0.8f * frame_rate + 0.2f * current_frame_rate;

    // 如果累计时间超过 1 秒，重置计数器
    if (time_elapsed >= 1.0f) 
    {
        interrupt_count = 0;
        time_elapsed = 0.0f;
    }

    return frame_rate;
}