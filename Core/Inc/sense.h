#ifndef __SENSE_H
#define __SENSE_H

#include "stm32g4xx_hal.h"

// 硬件参数
#define INA240_GAIN						20.0f     // INA240A1 增益
#define R_SHUNT								0.001f    // 1mΩ 采样电阻
#define ADC_REF_VOLTAGE				3.3f      // ADC 参考电压
#define ADC_MAX_VALUE					4095.0f   // 12位 ADC 最大量化值 (0~4095)，电流/母线/NTC 统一使用
#define INA240_VREF						(ADC_REF_VOLTAGE / 2.0f)  // 1.65V
#define I_OFFSET_MAX_A					2.5f			// 零偏合理性上限 (A)

/* ---- 零偏合理性校验阈值（标定时电机必须静止，相电流应为 0）------------------
 * 单位换算：0.02 V/A，1 LSB = 40.3 mA；0~1.65V 对应 ±82.5 A（2047.5 counts）
 * 误差预算（0 A 时）：INA240A1 Vos ±25 µV ≈ 0.001 A；VREF 1% 分压误差 ≈ 0.83 A；
 *                    ADC 自身失调 ≈ 0.1 A  →  正常板 |offset| 应 < 1 A
 * 故障量级：±82.5 A（注入组未启动 / INA240 输出开路或饱和）
 *   I_OFFSET_MAX_A      2.5 A ≈ 62 counts ≈ 50 mV：对正常板 3× 余量，对故障仍有 30× 区分度
 *   I_OFFSET_DIFF_MAX_A 1.0 A：两颗同型号 INA240 + 同一 VREF，A/B 零偏差值正常 < 0.2 A，
 *                       超限多为单通道故障（某相 shunt / 走线 / 焊接），绝对幅度检查抓不到
 * 超限只置 sense_hw_error（不阻塞），由 MotorControl_CommandGate() 锁死控制命令 + LED 慢闪。
 * ------------------------------------------------------------------------- */
#define I_OFFSET_DIFF_MAX_A   1.0f

// 母线电压分压比：R_top=100k，R_bot=7.5k（中点并 100nF，再经 100k 限流进 ADC）
#define VOLTAGE_DIVIDER_RATIO  ((100.0f + 7.5f) / 7.5f)

// MOSFET NTC参数 
#define MOS_B       3450.0f    // B值, 单位: K

// NTC参数 
#define NTC_R0      10000.0f   // 25℃时的标称电阻, 单位: 欧姆
#define NTC_T0      298.15f    // 25℃对应的开尔文温度
#define NTC_B       3950.0f    // B值, 单位: K
#define PULL_DOWN_RES 10000.0f // 下拉电阻（NTC 上拉接法：VCC→NTC→采样点→下拉电阻→GND）, 单位: 欧姆

/* =====================================================================
 * 电流采样触发时序（TIM1_CH4 -> ADC1 注入组，中心对齐 + RCR=1）
 *   实测UEV 落在下溢（谷点 CNT=0），OC4REF 在"下行段经过 CCR4"时产生上升沿，
 *   该时刻距下溢还有 CCR4 个计数 => 提前量 = CCR4 个 tick。
 *   CCR4=288 @72MHz = 4µs；注入组 2 通道转换 2.06µs，UEV 前约 1.94µs 完成。
 *   下限：提前量必须 > 2.06µs（转换）+ ISR 进入余量，建议 3~4µs。
 * ===================================================================== */
#define ADC_TRIG_LEAD_TICKS   288u    // 4µs @72MHz
#define ADC_TRIG_CCR4         ADC_TRIG_LEAD_TICKS  // 中心对齐下行段：提前量 = CCR4

// ADC转换预计算
// ADC_TO_Sense : counts/V（raw / ADC_TO_Sense = 电压 V）
#define ADC_TO_Sense		(ADC_MAX_VALUE / ADC_REF_VOLTAGE)                            // ≈1240.9 counts/V
#define ADC_TO_Sense2		(INA240_GAIN * R_SHUNT)                                      // 0.02 V/A
// I_K  : A/count（3.3/4095/(20*0.001) ≈ 0.04029，量程 ±82A）
#define I_K				    (ADC_REF_VOLTAGE / (ADC_MAX_VALUE * INA240_GAIN * R_SHUNT))
#define I_B0            (INA240_VREF / (INA240_GAIN * R_SHUNT))                      // ≈82.5 A
#define Vbus_K				(ADC_REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO / ADC_MAX_VALUE)    // V/count

// ADC 原始采样值（调试观察用）
extern volatile uint16_t adc1_values_inj[2];		// 注入组：[0]=A相(CH2/PA1) [1]=B相(CH3/PA2)，每拍更新
extern volatile uint16_t adc1_values_reg[2];		// 规则组 DMA：[0]=MOS温度(CH4/PA3) [1]=母线(CH8/PC2)
extern volatile uint16_t adc2_values_ntc[1];    // ADC2 DMA：[0]=线圈 NTC(CH12/PB2)

// 注入组采样未就绪计数（JEOS 未置位）：正常应长期为 0，
// 持续增长说明提前量(CCR4)偏小、TIM1_CH4 触发链未接通或 CC4E 未使能
extern volatile uint32_t adc_inj_miss_cnt;

// 电流零偏标定超限（采样链异常 / 标定时电机未静止）：1 = 锁死控制命令 + LED 慢闪
extern volatile uint8_t sense_hw_error;

// 零偏标定结果快照 (A)：供串口/dbg 观察，正常应接近 0
extern volatile float cal_offset_A;
extern volatile float cal_offset_B;

// 电流结果
extern volatile float current_A;   // A相电流 (A)
extern volatile float current_B;   // B相电流 (A)
extern volatile float current_C;   // C相电流 (A)
extern volatile float bus_voltage;      // 母线电压 (V)
extern volatile float mos_temperature;  // MOS温度 (℃)
extern volatile float ntc_temperature;  // 线圈温度 (℃)
extern volatile uint8_t sense_hw_error;		// 电流零偏错误标志位

extern volatile float bus_voltage;		// 母线电压结果

void CurrentSense_Init(void);
void CurrentSense_Update(void);
void TempSense_Update(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
float CurrentSense_Calculate(uint16_t adc_raw);

#endif
