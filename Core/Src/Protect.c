#include "Protect.h"
#include "DRV8353.H"
#include "KTH7823.h"
#include "kth7823_2.h"
#include "sense.h"
#include "tim.h"
#include "foc.h"
#include "arm_math.h"
#include "motor_control.h"


//编码器读数有效性（断线/无磁）监测
volatile uint16_t encoder_stall_max = 0; // 记录观察到的最大连续异常读数次数（诊断用）
static uint8_t encoder_fault_latched = 0;
static volatile uint8_t comm_frame = 0;	// 通讯故障恢复凭据：故障触发后收到过新帧才允许恢复

// 上位机通讯心跳（CAN）：每收到一帧 Protect_CommFeed 递增一次
volatile uint32_t comm_rx_count = 0;    // 通讯心跳计数
static uint32_t comm_last_count = 0;    // 上次检测时的心跳基准
volatile uint32_t comm_silent_ticks = 0;// 通讯静默节拍（连续未收到数据的周期数，供串口观察）
volatile uint8_t  comm_armed = 0;         // 收到首帧后置 1，失联保护才生效（未接上位机时不误触发）

// I²t 热积累量（全局，供 FOC.c 恢复分支判断是否仍过热）
volatile float i2t_accum = 0.0f;
volatile float i_mag2   = 0.0f;   // 三相合成电流幅值平方（供 I²t 累积用）

// 软限位/位置误差阈值（减速器输出端位置，rad，此处定义默认值，MotorControl_Init 可覆盖）
float soft_limit_min  = -3.14159f;  // 默认 -π（约 -180°）
float soft_limit_max  =  3.14159f;  // 默认 +π
float pos_error_limit =  0.05f;     // 默认位置跟随误差 0.05 rad（约 2.9°，按需调整）

// 关闭三路 PWM 输出，但保持 TIM1 计数器继续运行（所有保护分支统一走这里）
void Protect_StopPWM(void)
{
	// 故障掩码汇总：所有保护分支/故障回调都在置好 fault_code 之后调用本函数，
	// 故在此把"当前触发者"记入 fault_mask（位 n = FAULT_CODE_n）。这样即使同一拍有多个
	// 保护同时触发、fault_code 被后写覆盖，掩码里仍保留全部触发过的保护，便于区分谁触发。
	if (fault_code != FAULT_CODE_NONE) {
		fault_mask |= FAULT_MASK_BIT(fault_code);
	}

	// 直接写寄存器封锁三相输出：只清通道使能 + 清主输出使能，不碰 CEN
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

// 检测三相合成电流幅值，超阈值即停驱动并交状态机处理
// 无本地锁存：每拍独立判定，重试后仍过流会自然再次触发，避免竞态
void Protect_StallCheck(void)
{
    // 三相合成电流幅值（比单看一相更稳，能反映真实堵转/过流）
    i_mag2 = (current_A * current_A +
              current_B * current_B +
              current_C * current_C);

    // 堵转判定：大电流 且 转速接近 0（正常高速大电流运行不误触发）
    if (i_mag2 > STALL_CURRENT_LIMIT * STALL_CURRENT_LIMIT &&
        fabsf(KTH7823_Speed_Rads) < STALL_SPEED_THRESHOLD) {
        fault_code = FAULT_CODE_STALL;   // 锁存软件故障码（先于 Disable，供主循环上报）
        // 触发硬件保护：关闭栅极驱动 + 停三路 PWM
        DRV8353_Disable();
        Protect_StopPWM();
        fault_triggered = 1;   // 交由 FOC_Update 顶部的 FOC_STATE_FAULT 状态机走指数退避自动恢复
    }
}

// 编码器读数新鲜度保护：SPI 失败、无效 raw 或被拒绝的解包裹野值均不刷新 stale_ticks。
// 100 个 20kHz 周期（约 5ms）未接受到有效样本即停机；计数从采样启动即开始，
// 因此首次有效样本迟迟未到也会触发，不依赖采样心跳来武装保护。
void Protect_EncoderStallCheck(void)
{
    if (kth7823_stale_ticks < KTH7823_STALE_TICK_LIMIT) {
        encoder_fault_latched = 0;
        return;
    }

    // 记录观察到的最大连续异常读数次数（诊断用）
    if (kth7823_raw_fault_cnt > encoder_stall_max) encoder_stall_max = kth7823_raw_fault_cnt;

    if (encoder_fault_latched) return;

    encoder_fault_latched = 1;
    fault_code = FAULT_CODE_ENCODER_STALL;
    DRV8353_Disable();
    Protect_StopPWM();
    fault_triggered = 1;
}

// 上位机通讯心跳喂狗：CAN 每收到一帧数据调用一次（在接收中断内）
void Protect_CommFeed(void)
{
    comm_rx_count++;
	  comm_frame = 1;   // 收到帧：既有心跳，也是"通讯已恢复"的凭据
    comm_armed = 1;   // 收到首帧后使能失联保护
}

// "通讯已恢复"的凭据
uint8_t Protect_CommFrameSeen(void) 
{ 
		return comm_frame; 
}

// 上位机通讯失联保护：连续 COMM_TIMEOUT_TICKS 个控制周期无新帧则停机
// 原理与编码器失速检测一致：比较心跳计数，无增长则累计静默节拍，超时即保护
void Protect_CommTimeoutCheck(void)
{
    if (!comm_armed) return;   // 尚未收到任何上位机帧：独立运行，不启用失联保护

    if (comm_rx_count != comm_last_count) {
        // 收到新帧：更新基准并清零静默节拍
        comm_last_count = comm_rx_count;
        comm_silent_ticks = 0;
    } else {
        // 无新帧：累计静默节拍（饱和计数，防 uint32 回绕）
        if (comm_silent_ticks < COMM_TIMEOUT_TICKS) comm_silent_ticks++;
        if (comm_silent_ticks >= COMM_TIMEOUT_TICKS) {
            fault_code = FAULT_CODE_COMM_TIMEOUT;   // 锁存通讯失联故障码
						comm_frame = 0;
            DRV8353_Disable();
            Protect_StopPWM();
            fault_triggered = 1;   // 交由 FOC_STATE_FAULT 状态机退避恢复；重新收到帧后自动解除
        }
    }
}
// 关闭失联保护（进入本地调试态）：串口 enable 命令调用
// 清零 armed 使失联检测失效，并复位静默节拍/心跳基准，保持状态干净。
// 之后若 CAN 再次发帧，Protect_CommFeed 会重新置位 armed，失联保护自动恢复。
void Protect_CommDisarm(void)
{
    comm_armed = 0;
    comm_silent_ticks = 0;
    comm_last_count = comm_rx_count;
}

// 失联保护是否已武装（1 = 已收到过 CAN 帧、失联检测生效中）。
// 仅供串口 can 诊断命令显示，不改变任何状态。
uint8_t Protect_CommIsArmed(void)
{
    return comm_armed;
}

// MOSSFET/线圈过温保护
// 温度超过关断阈值即停驱动并交状态机退避；
// 恢复条件在 FOC_Update 的 FOC_STATE_FAULT 分支内：温度未降到恢复阈值则继续冷却（滞回）
void Protect_OvertempCheck(void)
{
    if (ntc_temperature > OVERTEMP_SHUTDOWN || mos_temperature > OVERTEMP_SHUTDOWN) {
        fault_code = FAULT_CODE_OVERTEMP;   // 锁存过温故障码（先于 Disable，供主循环上报）
        DRV8353_Disable();
        Protect_StopPWM();
        fault_triggered = 1;
    }
}

// 母线过压/欠压保护
// 过压：回馈能量泵升母线，立即停机（能量最终需硬件刹车电阻泄放）；
// 欠压：栅极驱动电源不足可能导致 MOSFET 直通，立即停机。
// 恢复条件在 FOC_Update 的 FOC_STATE_FAULT 分支内：电压回到安全范围才恢复（滞回）。
void Protect_BusVoltageCheck(void)
{
    static uint16_t ov_cnt = 0, uv_cnt = 0;
    
    ov_cnt = (bus_voltage > BUS_OVERVOLTAGE_THRESHOLD) ? (ov_cnt + 1) : 0;
    uv_cnt = (bus_voltage < BUS_UNDERVOLTAGE_THRESHOLD) ? (uv_cnt + 1) : 0;

    if (ov_cnt >= BUS_TRIP_CONFIRM_TICKS) {
        ov_cnt = 0;
        fault_code = FAULT_CODE_OVERVOLT;   // 锁存过压故障码
        DRV8353_Disable();
        Protect_StopPWM();
        fault_triggered = 1;
    } else if (uv_cnt >= BUS_TRIP_CONFIRM_TICKS){
        uv_cnt = 0;
        fault_code = FAULT_CODE_UNDERVOLT;  // 锁存欠压故障码
        DRV8353_Disable();
        Protect_StopPWM();
        fault_triggered = 1;
    }
}

// 位置跟随误差保护：仅闭环位置模式调用。误差持续超限 POS_ERR_TIMEOUT_TICKS 控制周期则停驱动
// 注意：此函数由 FOC_Update 的 FOC_STATE_POSITION 分支内调用，不在其他模式触发。
void Protect_PositionErrorCheck(void)
{
    if (foc.mode != FOC_STATE_POSITION) {
        foc.pos_err_integral_acc = 0.0f;   // 未启用位置级联时清零累计
        return;
    }

    float err = fabsf(foc.position_error);
    if (err > pos_error_limit) {
        foc.pos_err_integral_acc++;
        if (foc.pos_err_integral_acc >= POS_ERR_TIMEOUT_TICKS) {
            fault_code = FAULT_CODE_POS_ERROR;   // 锁存位置跟随误差故障码
            DRV8353_Disable();
            Protect_StopPWM();
            fault_triggered = 1;
        }
    } else {
        foc.pos_err_integral_acc = 0.0f;   // 误差回落到安全范围，清理累计
    }
}

// 软限位保护：FOC_SetTargetPosition 已做钳制，但若目标持续被钳制（=外部不断设置超出限位的目标），
// 持续 SOFT_LIMIT_TIMEOUT_TICKS 控制周期则停驱动，防止上层控制逻辑失控反复撞限位。
void Protect_SoftLimitCheck(void)
{
    if (foc.mode != FOC_STATE_POSITION) {
        foc.pos_err_integral_acc = 0.0f;
        return;
    }

    // 检测：当前目标位置等于限位边界（说明被钳制了）
    uint8_t clipped = (foc.target_position >= soft_limit_max - 1e-4f) ||
                      (foc.target_position <= soft_limit_min + 1e-4f);
    if (clipped) {
        foc.pos_err_integral_acc++;
        if (foc.pos_err_integral_acc >= SOFT_LIMIT_TIMEOUT_TICKS) {
            fault_code = FAULT_CODE_SOFT_LIMIT;   // 锁存软限位故障码
            DRV8353_Disable();
            Protect_StopPWM();
            fault_triggered = 1;
        }
    } else {
        foc.pos_err_integral_acc = 0.0f;
    }
}

// 超速保护
// 机械角速度绝对值"连续"超阈值 OVERSPEED_CONFIRM_TICKS 拍才停机（正反转均检测），
// 用于抑制编码器解包裹跳变/差分噪声造成的单拍速度尖峰误触发（判据见 Protect.h）。
// 恢复条件在 FOC_STATE_FAULT 分支内：速度回落到恢复阈值以下才允许恢复（滞回，防震荡）。
void Protect_OverspeedCheck(void)
{
    static uint16_t os_cnt = 0;   // 连续超阈拍数（未超阈即清零）

    os_cnt = (fabsf(KTH7823_Speed_Rads) > OVERSPEED_THRESHOLD) ? (os_cnt + 1) : 0;

    if (os_cnt >= OVERSPEED_CONFIRM_TICKS) {
        os_cnt = 0;
        fault_code = FAULT_CODE_OVERSPEED;   // 锁存超速故障码
        DRV8353_Disable();
        Protect_StopPWM();
        fault_triggered = 1;
    }
}

// 单相瞬时峰值过流：检测单相电流绝对值，捕获不对称尖峰/短路，响应最快
void Protect_PhasePeakCheck(void)
{
    float i_peak = fabsf(current_A);
    float t = fabsf(current_B);
    if (t > i_peak) i_peak = t;
    t = fabsf(current_C);
    if (t > i_peak) i_peak = t;

    if (i_peak > PHASE_PEAK_CURRENT_LIMIT) {
        fault_code = FAULT_CODE_PHASE_OC;   // 锁存单相峰值过流故障码
        DRV8353_Disable();
        Protect_StopPWM();
        fault_triggered = 1;
    }
}

// I²t 热积累保护：累计 (I_rms² - I_nom²)·dt，超阈值停驱动
// 区分"短时峰值"（允许）与"持续过流"（保护）；恢复需热积累回落到复位阈值以下。
void Protect_I2tCheck(void)
{

    float i_rms2 = i_mag2 / 3.0f;   // 单相等效 RMS 电流平方（i_mag = √3·I_rms）
    float nom2   = I2T_NOMINAL_CURRENT * I2T_NOMINAL_CURRENT;

    // 净热积累：发热 - 散热（负值表示冷却，下限 0）
    i2t_accum += (i_rms2 - nom2) * control_dt;
    if (i2t_accum < 0.0f) i2t_accum = 0.0f;

    if (i2t_accum > I2T_TRIP_THRESHOLD) {
        fault_code = FAULT_CODE_I2T;   // 锁存 I²t 过流故障码
        DRV8353_Disable();
        Protect_StopPWM();
        fault_triggered = 1;
    }
}

// 保护巡检
void Protect_FOC(void)
{
	// 单相瞬时峰值过流：捕获不对称尖峰/短路
	Protect_PhasePeakCheck();
	// 检测堵转电流
	Protect_StallCheck();
	// I²t 热积累保护：防持续过流烧电机
	Protect_I2tCheck();
	
	if (control_tick % 10u != 0u) return;		//缓保护降频
	
	// 位置跟随误差
	Protect_PositionErrorCheck();
	// 软限位保护
	Protect_SoftLimitCheck();
	// 编码器新鲜度监测：连续没有接受到有效样本则触发保护
	Protect_EncoderStallCheck();
	// 过温保护：线圈温度超关断阈值则触发保护
	Protect_OvertempCheck();
	// 母线过压/欠压保护：回馈泵升或栅极欠压均需停机
	Protect_BusVoltageCheck();
	// 超速保护：机械角速度超阈值停机（防飞车）
	Protect_OverspeedCheck();
	// 上位机通讯失联监测：CAN 心跳超时则触发保护
	Protect_CommTimeoutCheck();
}
	