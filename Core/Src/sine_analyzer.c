/**
  ******************************************************************************
  * @file    sine_analyzer.c
  * @brief   正弦波分析模块实现
  * @details 
  *          1. 频率测量: 通过 TIM1 捕获 PA9 方波的两个连续上升沿，计算时间差得到周期。
  *             由于 PSC=16999，每个 Tick = 0.1ms，直接相减即可，无需处理复杂溢出。
  *          2. 振幅测量: 利用 TIM1 中断时刻的 DMA 位置，回溯“半个周期”长度的数据段。
  *             由于 DMA 是环形缓冲区，需要处理数据跨越缓冲区首尾的情况 (Wrap-around)。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sine_analyzer.h"
#include <string.h> // 用于 memcpy
#include <math.h>   // 用于数学运算

/* Private variables ---------------------------------------------------------*/

// 模块内部持有的硬件句柄指针
static TIM_HandleTypeDef *s_htim_ic = NULL;
static ADC_HandleTypeDef *s_hadc = NULL;
static DMA_HandleTypeDef *s_hdma = NULL;
static uint16_t *s_adc_buf = NULL;

// 状态变量
static volatile uint32_t s_last_dma_pos = 0;   /**< 上次中断发生时，DMA 写入缓冲区的索引位置 */
static volatile uint8_t s_new_data_ready = 0;  /**< 新数据就绪标志: 1=主循环应进行处理 */

// 结果存储
static SineWaveResult_t s_result = {0};

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief 内部辅助函数：对给定的线性数据数组计算振幅和直流偏置
  * @param data_ptr: 数据起始指针
  * @param len: 数据长度
  */
static void Calculate_Params(uint16_t *data_ptr, uint16_t len);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief 初始化模块
  */
void SineAnalyzer_Init(TIM_HandleTypeDef *htim_ic, 
                       ADC_HandleTypeDef *hadc, 
                       DMA_HandleTypeDef *hdma, 
                       uint16_t *adc_buf)
{
    s_htim_ic = htim_ic;
    s_hadc = hadc;
    s_hdma = hdma;
    s_adc_buf = adc_buf;
    
    // 重置结果
    memset(&s_result, 0, sizeof(SineWaveResult_t));
    s_result.is_valid = 0;
}

/**
  * @brief 获取结果
  */
void SineAnalyzer_GetResult(SineWaveResult_t *result)
{
    if (result != NULL)
    {
        // 简单拷贝结果。对于低频应用，这种非原子操作通常是可以接受的。
        // 如果要求极高实时性，可在此处添加临界区保护 (disable/enable irq)。
        *result = s_result;
    }
}

/**
  * @brief 主循环处理逻辑
  */
void SineAnalyzer_Process(void)
{
    // 如果没有新数据，直接返回，节省 CPU
    if (!s_new_data_ready) return;
    
    s_new_data_ready = 0; // 清除标志，防止重复处理
    
    // --- 步骤 1: 计算半个周期对应的采样点数 ---
    // 公式: Points = Period(s) * SamplingRate(Hz) / 2
    uint16_t half_period_points = (uint16_t)(s_result.period_sec * SA_SAMPLING_RATE_HZ / 2.0f);
    
    // 边界保护：防止点数超出缓冲区或过小
    if (half_period_points > SA_ADC_BUF_SIZE) half_period_points = SA_ADC_BUF_SIZE;
    if (half_period_points < 10) half_period_points = 10; // 至少需要少量点才能计算

    // --- 步骤 2: 确定数据在环形缓冲区中的位置 ---
    // 我们想要的是“过去半个周期”的数据。
    // 起始索引 = 当前写入位置 - 半个周期的点数
    int32_t start_pos = (int32_t)s_last_dma_pos - half_period_points;
    
    // --- 步骤 3: 处理环形缓冲区的两种情况 ---
    if (start_pos >= 0)
    {
        // 【情况 A】: 数据连续，未跨越缓冲区起点
        // 例如: BufSize=100, LastPos=80, HalfLen=20 -> Start=60. 数据在 [60...79]
        Calculate_Params(&s_adc_buf[start_pos], half_period_points);
    }
    else
    {
        // 【情况 B】: 数据跨越了缓冲区起点 (Wrap-around)
        // 例如: BufSize=100, LastPos=10, HalfLen=20 -> Start=-10.
        // 数据分布在: 末尾的 10 个点 [90...99] 和 开头的 10 个点 [0...9]
        
        static uint16_t temp_buf[SA_ADC_BUF_SIZE]; // 静态临时缓冲区，避免栈溢出
        
        int32_t tail_len = -start_pos;             // 需要从缓冲区末尾取的长度 (例如 10)
        int32_t head_len = half_period_points - tail_len; // 需要从缓冲区开头取的长度 (例如 10)
        
        // 1. 复制尾部数据到临时缓冲区的开头
        // 源地址: s_adc_buf + (总长度 - 尾部长度)
        memcpy(&temp_buf[0], &s_adc_buf[SA_ADC_BUF_SIZE - tail_len], tail_len * sizeof(uint16_t));
        
        // 2. 复制头部数据到临时缓冲区的后续位置
        memcpy(&temp_buf[tail_len], &s_adc_buf[0], head_len * sizeof(uint16_t));
        
        // 3. 对重组后的连续数据进行计算
        Calculate_Params(temp_buf, half_period_points);
    }
    
    // 标记结果有效
    s_result.is_valid = 1;
}

/**
  * @brief TIM 捕获中断回调入口
  */
void SineAnalyzer_TIM_IC_Callback(TIM_HandleTypeDef *htim)
{
    // 验证是否是我们要处理的定时器和通道 (TIM1 CH2)
//    if (htim->Instance == s_htim_ic->Instance && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	if (htim->Instance == s_htim_ic->Instance)
    {
        // 1. 读取当前捕获值 (计数器值)
        uint16_t current_cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        
        // --- 频率计算部分 ---
        static uint16_t last_cap = 0;
        static uint8_t first_run = 1;
        
        if (!first_run)
        {
            // 计算两次上升沿的 Tick 差值
            uint16_t delta = current_cap - last_cap;
            
            // 处理 16 位计数器自然溢出 (虽然 PSC 很大，但保留此逻辑以防万一)
            if (current_cap < last_cap) 
            {
                delta += 65536; 
            }
            
            // 转换为时间: PSC=16999 -> 1 Tick = 0.1 ms
            float period_ms = (float)delta * 0.1f;
            
            if (period_ms > 0)
            {
                s_result.period_sec = period_ms / 1000.0f;
                s_result.frequency_hz = 1000.0f / period_ms;
            }
        }
        else
        {
            first_run = 0; // 第一次捕获只记录基准，不计算
        }
        last_cap = current_cap;
        
        // --- 数据同步部分 ---
        // 获取 DMA 剩余传输计数
        uint32_t ndtr = __HAL_DMA_GET_COUNTER(s_hdma);
        
        // 计算当前 DMA 正在写入的索引: Total - Remaining
        s_last_dma_pos = SA_ADC_BUF_SIZE - ndtr;
        
        // 通知主循环有新数据
        s_new_data_ready = 1;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief 内部计算函数：遍历数组求最大值、最小值和平均值
  */
static void Calculate_Params(uint16_t *data_ptr, uint16_t len)
{
    if (len == 0) return;

    uint32_t sum = 0;
    uint16_t max_val = 0;
    uint16_t min_val = 65535;

    // 遍历数据段
    for (int i = 0; i < len; i++)
    {
        uint16_t val = data_ptr[i];
        sum += val;
        
        if (val > max_val) max_val = val;
        if (val < min_val) min_val = val;
    }

    // 1. 计算直流偏置 (DC Offset) = 平均值
    float avg_raw = (float)sum / len;
    s_result.dc_offset_v = avg_raw * SA_VREF_VOLTAGE / SA_ADC_RESOLUTION;

    // 2. 计算峰峰值 (Vpp) 和振幅 (Amplitude)
    float vpp_raw = (float)(max_val - min_val);
    s_result.vpp_v = vpp_raw * SA_VREF_VOLTAGE / SA_ADC_RESOLUTION;
    
    // 振幅 = Vpp / 2
    s_result.amplitude_v = s_result.vpp_v / 2.0f;
}
