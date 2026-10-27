#ifndef __kth7823_2_H
#define __kth7823_2_H

#include "stm32g4xx_hal.h"
#include <math.h>


/* ==================== 输出端编码器（减速器输出轴，SPI3） ====================
 * 与电机轴 kth7823 同款芯片，SPI 协议完全一致；本模块镜像 kth7823.c 的快/慢路径架构：
 *   - 快路径 20kHz（TIM1 更新中断）：寄存器级 SPI 单帧 + 解包裹 + 一拍延迟补偿，
 *     实时刷新 kth7823_2_Angle_Multiturn / kth7823_2_Angle_Multiturn_Comp（位置环角度源）；
 *   - 慢路径 1kHz：sin/cos 一阶低通 + atan2 单圈角（主循环，不碰 SPI），
 *     测速与心跳放在快路径每 20 拍做一次（便宜，与电机轴对称）。
 *
 * 读取协议：kth7823 为流水线 SPI —— 当前帧发送命令，返回"上一命令帧的回读"。
 *           故每拍只发 1 帧，角度固有滞后 1 个控制周期（50µs），
 *           由 kth7823_2_Angle_Multiturn_Comp 用实测角速度外推补偿。
 *
 * CS 规则：每帧独立"拉低 → 1 个 16bit → 拉高"，不支持 CS 连续拉低连读多帧。
 *
 * 单一所有者：SPI3 只允许 kth7823_2_Tick()（中断上下文）访问；主循环只读全局量，
 *             不得再自己发起 SPI 帧（否则与中断抢总线、破坏流水线）。
 *
 * 时钟说明：SPI3 挂在 APB1（72MHz），BaudRatePrescaler=8 → 9MHz
 * ============================================================ */

#define kth7823_2_ANGLE_RESOLUTION   65536.0f  /* 16 位分辨率 */
#define kth7823_2_FILTER_ALPHA       0.10f     /* sin/cos 低通（按 1kHz 调用定义） */
#define kth7823_2_SPEED_FILTER_ALPHA 0.15f     /* 速度一阶低通（按 1kHz 调用定义） */
#define kth7823_2_SPI               (&hspi3)

/* 快/慢路径分频：20kHz 控制中断 ÷ 20 = 1kHz 慢路径 */
#define kth7823_2_SLOW_DIV           20u

// SPI 单帧硬超时（µs）。寄存器级收发 + DWT 周期计数限时，超时只作兜底
#define kth7823_2_SPI_TIMEOUT_US     10u

/* 多圈解包裹"野值剔除"上限 */
#define kth7823_2_UNWRAP_MAX_DELTA   256

/* 一拍延迟补偿：补偿量 = ω × (kth7823_2_ANGLE_DELAY_PERIODS × control_dt) */
#define kth7823_2_ANGLE_DELAY_PERIODS 1.0f


// 编码器输出：volatile 标记跨中断上下文共享的实时值，不保证多字段快照一致性
extern volatile float kth7823_2_Angle_Rad;             // 单圈角度 [0, 2π)（慢路径，滤波后）
extern float kth7823_2_Angle_Deg;                      // 单圈角度（度）
extern volatile float kth7823_2_Angle_Multiturn;       // 多圈总角度（连续无界，弧度，未滤波）
extern volatile float kth7823_2_Angle_Multiturn_Comp;  // 一拍延迟补偿后的多圈角度（位置环角度源）
extern volatile float kth7823_2_Speed_Rads;            // 速度 (rad/s)
extern float kth7823_2_Speed_RPM;                      // 速度 (RPM)

// 刷新心跳计数器：快路径 1kHz 每次 +1，供失速/断线监测判断编码器是否在刷新
extern volatile uint32_t kth7823_2_update_count;

// 连续读到异常原始值（0x0000/0xFFFF）的次数，诊断用；正常 raw 清零
extern volatile uint16_t kth7823_2_raw_fault_cnt;

// 距离最近一次接受有效角度样本的控制周期数（20kHz），饱和计数
extern volatile uint16_t kth7823_2_stale_ticks;

// 被丢弃的解包裹野值累计次数（诊断用，只增不清零）
extern volatile uint16_t kth7823_2_unwrap_glitch_cnt;

// SPI 帧失败次数（超时 / 重入被拒），诊断用，只增不清零
extern volatile uint16_t kth7823_2_spi_err_cnt;

// 输出端机械零点偏置 (rad)：上电从 Flash 加载；未标定为 0（上电位置即零点）
extern float kth7823_2_zero_offset;


// 函数声明
void kth7823_2_Init(void);
void kth7823_2_Tick(uint32_t tick);   /* 仅由 TIM1 20kHz 中断调用（SPI3 唯一所有者） */
void kth7823_2_SlowUpdate(void);      /* 主循环 ≈1kHz：滤波单圈角度（不碰 SPI） */
void kth7823_2_SetZero(void);         /* 把当前输出角标定为机械零点并写入 Flash（调用前需停驱动） */
uint8_t kth7823_2_LoadZero(void);     /* 从 Flash 加载零点偏置：1=有效 0=未标定/损坏 */

// 输出端绝对位置：应用机械零点偏置，归一化到 [-π, π)
static inline float kth7823_2_OutputPos(float multiturn_rad)
{
    float a = multiturn_rad - kth7823_2_zero_offset;
    while (a >   3.14159265f) a -= 6.28318531f;
    while (a <  -3.14159265f) a += 6.28318531f;
    return a;
}

#endif
