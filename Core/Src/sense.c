#include "sense.h"
#include "adc.h"
#include "tim.h"
#include "arm_math.h"
#include "dwt_delay.h"   // DWT_DelayMs

/* =====================================================================
 *   规则组 DMA（慢变量）  ：adc1_values_reg[2]  <- CH4(MOS温度) / CH8(母线)
 *   注入组 中断读 JDR（快变量）：adc1_values_inj[2] <- CH2(A相) / CH3(B相)
 *   规则组由 T1_TRGO(UEV) 触发；注入组由 TIM1_CH4 比较事件在 UEV 前 4µs 触发，
 *   转换 2.06µs 后 JEOS 置位，更新中断进入时读到的是当拍新样本。
 * ===================================================================== */

// ADC 采样值存储
volatile uint16_t adc1_values_inj[2];   // [0]=A相(CH2/PA1) [1]=B相(CH3/PA2)
volatile uint16_t adc1_values_reg[2];   // [0]=MOSFET温度(CH4/PA3) [1]=母线电压(CH8/PC2)
volatile uint16_t adc2_values_ntc[1];    // NTC(CH12/PB2)

// 采样就绪标志（规则组 DMA 完成，仅 Init 期探测用，置位后不再清零）
volatile uint8_t adc_dma_ready = 0;
// 注入组未就绪计数（JEOS 未置位）：正常长期为 0
volatile uint32_t adc_inj_miss_cnt = 0;

volatile float current_A = 0.0f;     // 三相电流
volatile float current_B = 0.0f;
volatile float current_C = 0.0f;
volatile float bus_voltage = 0.0f;		// 母线电压
volatile float mos_temperature = 0.0f;		// MOSFET温度
volatile float ntc_temperature = 0.0f;		// 线圈温度（中断采样，主循环/保护读取）
static float current_A_offset = 0.0f;
static float current_B_offset = 0.0f;

/* 零偏标定结果快照（对外暴露，见 sense.h）：热路径仍用上面的 static 变量，
 * 这里只做观测/上报，避免 20kHz 中断里因 volatile 反复重载 */
volatile float cal_offset_A = 0.0f;   // A 相零偏 (A)，正常应接近 0
volatile float cal_offset_B = 0.0f;   // B 相零偏 (A)

volatile uint8_t sense_hw_error = 0;		// 1 = 零偏超限（采样链异常 / 标定时电机未静止），锁死控制命令
	
//电流采样初始化
void CurrentSense_Init(void)
{
    // 电流采样触发链：TIM1_CH4 比较事件在 UEV 前 4µs 触发 ADC1 注入组
    TIM1->CCR4 = ADC_TRIG_CCR4;                    // 提前量 288 = 4µs @72MHz（预装载，下一 UEV 生效）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);      // 使能 CC4E

    // 规则组(母线/温度)/ADC2(NTC) 走 DMA 循环，长度必须等于规则组通道数
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_values_reg, 2)!= HAL_OK) Error_Handler();
		if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_values_ntc, 1)!= HAL_OK) Error_Handler();
		__HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

    // 注入组只武装一次，之后每个 PWM 周期由 T1_CC4 硬件触发（无中断、无 DMA）
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // 等第一个规则组数据到来（同时验证 TIM1 计数、TRGO 与 ADC 触发链正常）
    uint32_t start_tick = HAL_GetTick();
    while (!adc_dma_ready) {
        if (HAL_GetTick() - start_tick > 50) {
            Error_Handler();   // 50ms 无数据：TIM1 未触发、CC4E 未开或 ADC 配置错误
        }
    }
    // 再稳定若干拍，避免用上电残留的注入组数据做零偏
    DWT_DelayMs(1);

    // 采样 64 次求平均偏移（电机必须静止）
    // 零偏条件：TIM1 三路 PWM 未启动（CC1E~CC3E=0），但 DRV8353 已使能（DRV8353_Init 已拉高 ENABLE）。
    // 3x PWM 模式下 INH 三路为低 → 三相下桥全通，只要转子静止则相电流为 0。
    // 反之：上电瞬间转子若被外力带动，下桥全通会刹车并产生电流，此时标定不可信 → 由下方校验兜住。
    float sum_A = 0.0f, sum_B = 0.0f;
    for (int i = 0; i < 64; i++) {
        sum_A += CurrentSense_Calculate((uint16_t)ADC1->JDR1);
        sum_B += CurrentSense_Calculate((uint16_t)ADC1->JDR2);
        DWT_DelayMs(1);
    }
    current_A_offset = sum_A / 64.0f;
    current_B_offset = sum_B / 64.0f;
    cal_offset_A = current_A_offset;
    cal_offset_B = current_B_offset;

    /* 零偏合理性校验（不阻塞：只置标志，由 MotorControl_CommandGate() 锁命令、LED 慢闪）
     * 绝对幅度：抓"注入组未启动 / INA240 输出开路或饱和 / 标定时电机在动"
     * 两相差值：抓"单通道故障（某相 shunt、走线、INA240）"—— 同型号同 VREF 时差值应很小 */
		if (fabsf(current_A_offset) > I_OFFSET_MAX_A ||
            fabsf(current_B_offset) > I_OFFSET_MAX_A ||
            fabsf(current_A_offset - current_B_offset) > I_OFFSET_DIFF_MAX_A) {
        sense_hw_error = 1;
    }
		
		// 清零初始化期的误计数（此刻起时序应已稳定）
    adc_inj_miss_cnt = 0;
}

//根据 ADC 原始值计算电流
float CurrentSense_Calculate(uint16_t adc_raw)
{
    // ADC 值转电压（ADC_TO_Sense = 4095/3.3，单位 counts/V）
    float v_out = (float)adc_raw / ADC_TO_Sense;

    // 计算采样电阻压降（减去 VREF 偏置）
    float v_sense = v_out - INA240_VREF;

    // 计算实际电流
    float current = v_sense / ADC_TO_Sense2 ;

    return current;
}

//更新三相电流值
void CurrentSense_Update(void)
{
    const float kA = I_K, bA = -(I_B0 + current_A_offset);
    const float kB = I_K, bB = -(I_B0 + current_B_offset);

    /* 注入组数据：本拍 UEV 前 4µs 触发、2.06µs 转换完成，
     * 正常情况下进入中断时 JEOS 已置位；未置位说明提前量不足或触发链异常。
     * 先判标志再读 JDR（需要手动清 JEOS）。 */
    if ((ADC1->ISR & ADC_ISR_JEOS) == 0u) {
        adc_inj_miss_cnt++;
    }
    adc1_values_inj[0] = (uint16_t)ADC1->JDR1;
    adc1_values_inj[1] = (uint16_t)ADC1->JDR2;
		ADC1->ISR = ADC_ISR_JEOS; 

    current_A = (float)adc1_values_inj[0] * kA + bA;
    current_B = (float)adc1_values_inj[1] * kB + bB;
    current_C = -(current_A + current_B);

    // 母线电压
    static uint16_t vbus_hist[3] = {0, 0, 0};
    {
        vbus_hist[0] = vbus_hist[1];
        vbus_hist[1] = vbus_hist[2];
        vbus_hist[2] = adc1_values_reg[1];

        uint16_t a = vbus_hist[0], b = vbus_hist[1], c = vbus_hist[2], med;
        if ((a <= b && b <= c) || (c <= b && b <= a))      med = b;
        else if ((b <= a && a <= c) || (c <= a && a <= b)) med = a;
        else                                                med = c;

        bus_voltage = (float)med * Vbus_K ;
    }
}

//更新温度
void TempSense_Update(void){

		// MOS 温度—— 上拉接法：VCC→NTC→采样点→下拉电阻→GND
    uint16_t mos_raw = adc1_values_reg[0];
    if (mos_raw == 0) mos_raw = 1;  // 防除零（低温时 NTC 阻值大，采样电压趋近 0）
    float mos_ntc = PULL_DOWN_RES * (ADC_MAX_VALUE - (float)mos_raw) / (float)mos_raw;
    if (mos_ntc < 1.0f) mos_ntc = 1.0f;   // 防 logf(0)（NTC 短路时）
    float temp_k = 1.0f / ((1.0f / NTC_T0) + (1.0f / MOS_B) * logf(mos_ntc / NTC_R0));
    mos_temperature = temp_k - 273.15f;

    // 线圈 NTC温度
    uint16_t ntc_raw = adc2_values_ntc[0];
    if (ntc_raw == 0) ntc_raw = 1;  // 防除零
    float r_ntc = PULL_DOWN_RES * (ADC_MAX_VALUE - (float)ntc_raw) / (float)ntc_raw;
    if (r_ntc < 1.0f) r_ntc = 1.0f;
    float temp_k_2 = 1.0f / ((1.0f / NTC_T0) + (1.0f / NTC_B) * logf(r_ntc / NTC_R0));
    ntc_temperature = temp_k_2 - 273.15f;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) // 规则组 DMA 每拍完成一次(20kHz)，仅 Init 期用于探测
{
    if (hadc->Instance == ADC1) {
        adc_dma_ready = 1;  // ADC1 一轮(2通道)转换完成
    }
}
