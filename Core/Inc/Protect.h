#ifndef __PROTECT_H
#define __PROTECT_H

#include "stm32g4xx_hal.h"

// 堵转保护：大电流 + 转速接近 0 才判定（正常高速大电流不误触发）
// 滞回由 FOC.c 的 FOC_STATE_FAULT 状态机退避冷却实现，无需本地锁存
#define STALL_CURRENT_LIMIT   36.0f   // 堵转判定电流（三相合成幅值 A）
#define STALL_SPEED_THRESHOLD 3.0f   // 堵转判定转速 (rad/s)，|speed| 低于此且大电流才判堵

// 单相峰值过流（瞬时硬保护，响应最快）
// 直接检测单相瞬时电流绝对值，捕获不对称尖峰/短路，超过 40A （额定峰值50A）立即停机
#define PHASE_PEAK_CURRENT_LIMIT  40.0f   // 单相瞬时电流硬上限 (A)

// I²t 热积累保护（模拟绕组热积累，防持续过流烧电机）
// 模型：i2t_accum += (I_rms² - I_nom²) × dt，下限 0；I_rms² = i_mag²/3（三相合成幅值平方的 1/3）
//   I_nom 为允许长期运行的相 RMS 电流（散热平衡点），超过则累积、低于则冷却
//   额定 14A：50A 峰值（RMS 35.4A）净积累速率 = 35.4²-14² ≈ 1054 A²/s，约 1.5s 触发
//   即允许 50A 峰值短时运行约 1.5s，随后必须降回额定内散热
#define I2T_NOMINAL_CURRENT   14.0f     // 允许长期运行的相 RMS 电流 (A) = 电机额定电流
#define I2T_TRIP_THRESHOLD    1600.0f   // 触发阈值 (A²·s)：50A 峰值约 1.5s
#define I2T_RESET_THRESHOLD   100.0f    // 恢复阈值 (A²·s)，热积累回落到此才允许恢复
#define I2T_RECHECK_DELAY     2000      // 恢复重检周期 (ms)

// ---------------------------------------------------------------------------
// 控制节拍基准：TIM1 中心对齐 20kHz PWM（ARR=1799）+ RCR=1
//   -> 每个完整 PWM 周期触发 1 次更新中断（下溢/谷点）= 20kHz，即 1 拍 = 50µs
//   下面快保护的 *_TICKS 宏均按 50µs/拍 换算，缓保护时间要➗10
// ---------------------------------------------------------------------------

// 位置环相关阈值
#define POS_ERR_TIMEOUT_TICKS   100   // 位置跟随误差持续超限的控制周期数（2kHz 下 = 50ms）
#define SOFT_LIMIT_TIMEOUT_TICKS 40   // 目标位置在软限位外持续未收敛的控制周期数（2kHz 下 = 20ms）

// 编码器无有效新样本时的恢复重检间隔
#define ENCODER_RECHECK_DELAY_MS  100u

// 上位机通讯失联保护（CAN 心跳看门狗）
// 上位机以 1kHz（每 1ms 一帧）周期性发送 CAN 帧，每收到一帧调用 Protect_CommFeed() 刷新心跳
#define COMM_TIMEOUT_TICKS   40    // 失联判定阈值（2kHz 下 = 20ms）

// 过温保护阈值（NTC 测线圈温度）
// 超过关断阈值停驱动；温度回落到恢复阈值以下才允许恢复（20℃ 滞回，防震荡）。
// 阈值按电机绕组绝缘等级调整
#define OVERTEMP_SHUTDOWN      90.0f   // 过温关断阈值 (℃)
#define OVERTEMP_RECOVER        70.0f   // 恢复阈值 (℃)，滞回 20℃
#define OVERTEMP_RECHECK_DELAY  2000    // 过温恢复重检周期 (ms)

// 母线过压/欠压保护阈值（26V 系统）
// 过压：关节电机回馈泵升；欠压：栅极驱动不足可能直通，均需立即停机。
#define BUS_TRIP_CONFIRM_TICKS  1  //  1 拍（0.5ms @2kHz）越限才认定
#define BUS_OVERVOLTAGE_THRESHOLD   32.0f   // 过压停机阈值 (V)
#define BUS_OVERVOLTAGE_RECOVER     26.0f   // 过压恢复阈值 (V)，滞回 1.5V
#define BUS_UNDERVOLTAGE_THRESHOLD  16.0f   // 欠压停机阈值 (V)，DRV8353 最低 9V，留足余量
#define BUS_UNDERVOLTAGE_RECOVER    20.0f   // 欠压恢复阈值 (V)，滞回 2V
#define BUS_VOLTAGE_RECHECK_DELAY   2000    // 母线电压恢复重检周期 (ms)

// 超速保护阈值（KTH7823_Speed_Rads，电机轴机械角速度 rad/s）
// 关节电机防飞车，正反转均检测。
#define OVERSPEED_THRESHOLD   230.0f   // 超速停机阈值 (rad/s)，≈1.2×电机轴额定
#define OVERSPEED_RECOVER     200.0f   // 恢复阈值 (rad/s)，≈1.05×额定，滞回 30
#define OVERSPEED_RECOVER_RECHECK_DELAY   2000       

// 超速"连续 N 拍确认"：抑制编码器解包裹跳变导致的单拍速度尖峰误触发
#define OVERSPEED_CONFIRM_TICKS  5u   // 连续超阈拍数（2kHz 下 ≈2.5ms）


// I²t 热积累量（FOC.c 恢复分支据此判断是否仍过热）
extern volatile float i2t_accum;

// 软限位阈值（减速器输出端位置，rad，默认 ±π = ±180° 输出端）
extern float soft_limit_min;
extern float soft_limit_max;

// 通讯心跳计数：每收到一帧本节点 CAN 命令帧 +1（接收中断内自增），串口 can 诊断命令读取
extern volatile uint32_t comm_rx_count;

// 通讯静默节拍：连续多少控制周期（20kHz）未收到新帧，触发阈值 COMM_TIMEOUT_TICKS；
// 收到新帧即清零，用于串口观察失联保护计时（需 Protect_CommTimeoutCheck 已启用才有意义）
extern volatile uint32_t comm_silent_ticks;

// 收到首帧后置 1，失联保护才生效（未接上位机时不误触发）
extern volatile uint8_t  comm_armed;         

// 关闭三路 PWM 输出但保持 TIM1 计数（供所有保护分支/故障回调统一调用）
void Protect_StopPWM(void);

// 检测堵转电流
void Protect_StallCheck(void);

// 监测编码器读数新鲜度，连续无有效样本则触发硬件保护
void Protect_EncoderStallCheck(void);

// CAN 每收到一帧数据时调用（在 HAL_FDCAN_RxFifo0Callback 中断内），刷新通讯心跳
void Protect_CommFeed(void);

// 关闭失联保护（进入本地调试态），串口 enable 命令调用；CAN 再次发帧后自动恢复
void Protect_CommDisarm(void);

// 当前失联保护是否已武装（1 = 收到过 CAN 帧、失联检测已生效），串口 can 诊断命令显示
uint8_t Protect_CommIsArmed(void);

// "通讯已恢复"的凭据 
uint8_t Protect_CommFrameSeen(void);

// 上位机通讯连续中断超时
void Protect_CommTimeoutCheck(void);

// MOSFET/线圈过温
void Protect_OvertempCheck(void);

// 母线过压/欠压
void Protect_BusVoltageCheck(void);

// 机械超速
void Protect_OverspeedCheck(void);

// 单相瞬时峰值过流
void Protect_PhasePeakCheck(void);

// 累计 I²t 热积累并检测持续过流
void Protect_I2tCheck(void);

// 位置跟随误差阈值（rad）
extern float pos_error_limit;

// 位置跟随误差
void Protect_PositionErrorCheck(void);

// 软限位持续被钳制
void Protect_SoftLimitCheck(void);

// 保护巡检
void Protect_FOC(void);
#endif
