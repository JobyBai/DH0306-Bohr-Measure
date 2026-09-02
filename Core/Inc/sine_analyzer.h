/**
  ******************************************************************************
  * @file    sine_analyzer.h
  * @brief   基于 STM32G474 的低频正弦波分析模块
  * @note    
  *          - 依赖硬件: TIM1 (PA9 输入捕获), ADC1 (DMA 循环模式)
  *          - 核心逻辑: 利用 TIM1 捕获方波上升沿确定周期，结合 ADC DMA 数据计算振幅
  ******************************************************************************
  */

#ifndef __SINE_ANALYZER_H
#define __SINE_ANALYZER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 正弦波分析结果结构体
  */
typedef struct {
    float frequency_hz;    /**< 频率 (Hz) */
    float period_sec;      /**< 完整周期时间 (秒) */
    float amplitude_v;     /**< 交流振幅 (V)，即 Vpp/2 */
    float dc_offset_v;     /**< 直流偏置电压 (V)，即信号平均值 */
    float vpp_v;           /**< 峰峰值电压 (V) */
    uint8_t is_valid;      /**< 数据有效标志: 1=已计算出有效数据, 0=初始化或无效 */
    float phase_diff_1;  /**< 相位差1 */
    float phase_diff_2;  /**< 相位差2 */
} SineWaveResult_t;

/* Exported constants --------------------------------------------------------*/

/**
  * @brief 用户配置区 (请根据实际硬件参数修改此处)
  */
#define SA_ADC_BUF_SIZE       1000      /**< DMA 缓冲区大小。必须足够大以容纳半个周期的采样点 */
                                          /**< 估算公式: BufferSize > (Max_Period_Seconds / 2) * Sampling_Rate */
#define SA_VREF_VOLTAGE       3.3f       /**< ADC 参考电压 (V) */
#define SA_ADC_RESOLUTION     4095.0f    /**< ADC 最大计数值 (12-bit ADC 为 4095) */
#define SA_SAMPLING_RATE_HZ   1000.0f   /**< ADC 采样率 (Hz)。需与触发 ADC 的定时器(TIM2)频率一致 */

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief 初始化正弦波分析模块
  * @param htim_ic: 用于输入捕获的定时器句柄 (例如 &htim1，对应 PA9)
  * @param hadc: 用于采样的 ADC 句柄 (例如 &hadc1)
  * @param hdma: 用于 ADC 数据传输的 DMA 句柄 (例如 &hdma_adc1)
  * @param adc_buf: 用户分配的 ADC DMA 缓冲区指针 (大小需为 SA_ADC_BUF_SIZE)
  * @retval None
  */
void SineAnalyzer_Init(TIM_HandleTypeDef *htim_ic, 
                       ADC_HandleTypeDef *hadc, 
                       DMA_HandleTypeDef *hdma, 
                       uint16_t *adc_buf);

/**
  * @brief 主循环处理函数
  * @note 必须在 main 函数的 while(1) 中周期性调用。
  *       它负责检测新数据标志，处理环形缓冲区拼接，并执行数学计算。
  * @retval None
  */
void SineAnalyzer_Process(void);

/**
  * @brief 获取最新的分析结果
  * @param result: 指向外部 SineWaveResult_t 结构体的指针，用于接收数据
  * @retval None
  */
void SineAnalyzer_GetResult(SineWaveResult_t *result);

/**
  * @brief TIM 输入捕获中断回调入口
  * @note 必须在 stm32g4xx_it.c 的 HAL_TIM_IC_CaptureCallback 中调用此函数。
  *       它负责记录时间戳和 DMA 位置快照。
  * @param htim: 触发中断的定时器句柄
  * @retval None
  */
void SineAnalyzer_TIM_IC_Callback(TIM_HandleTypeDef *htim);

float Encoder_Delta(int16_t x, int16_t x0);

#ifdef __cplusplus
}
#endif

#endif /* __SINE_ANALYZER_H */


