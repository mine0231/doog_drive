#ifndef __kth7823_H
#define __kth7823_H

#include "stm32g4xx_hal.h"
#include <math.h>


/* ==================== 刷新方式与读取协议 ====================
 * 刷新位置：TIM1 更新中断（20kHz）内调用 kth7823_Tick(control_tick)，
 *           角度与电流环同拍更新（改造前在主循环 1kHz，最旧角度 1ms，存在混叠）。
 *
 * 两条路径（分开是为了不让昂贵的数学进 20kHz 中断）：
 *   - 快路径 20kHz（中断内）：1 帧 raw + 解包裹 → kth7823_Angle_Multiturn / _Comp（闭环角度源）；
 *     每 20 拍再做一次 DWT 差分测速 + 心跳（合计 <3µs）
 *   - 慢路径 1kHz（主循环）：读中断缓存的 raw，做 sin/cos 一阶低通 + atan2
 *     → kth7823_Angle_Rad / _Deg（开环起始角、对齐平均、上报）
 *     实测该段约 7µs，放进 20kHz 中断会顶穿 50µs 预算，故留在主循环（α 语义不变）
 *
 * 读取协议：kth7823 为流水线 SPI —— 当前帧发送命令，返回"上一命令帧的回读"。
 *           故每拍只发 1 帧（16bit @9MHz ≈ 1.78µs），角度固有滞后 1 个控制周期（50µs），
 *           由 kth7823_Angle_Multiturn_Comp 用实测角速度外推补偿。
 *
 * CS 规则（硬件要求，不可违背）：**不支持 CS 连续拉低做多次读取**，每帧必须是独立的
 *           "拉低 → 1 个 16bit → 拉高"，帧间高电平 ≥50ns。本模块每帧都成对置位/复位，
 *           并用 spi_in_frame 拒绝重入；禁止改成"CS 保持低电平连读多帧"。
 *
 * 单一所有者：SPI1 只允许 kth7823_Tick()（中断上下文）访问；主循环与阻塞式测试命令
 *           只读全局量，不得再自己发起 SPI 帧（否则破坏流水线并与中断抢总线）。
 * ============================================================ */

#define kth7823_ANGLE_RESOLUTION  65536.0f  /* 16 位分辨率 */
#define ANGLE_FILTER_ALPHA        0.10f     /* 慢路径 sin/cos 低通（按 1kHz 调用定义） */
#define SPEED_FILTER_ALPHA        0.15f     /* 速度一阶低通（按 1kHz 调用定义） */
#define kth7823_SPI               (&hspi1)

/* 快/慢路径分频：20kHz 控制中断 ÷ 20 = 1kHz 慢路径 */
#define kth7823_SLOW_DIV          20u

/* SPI 单帧硬超时（µs）。本帧在 20kHz 中断内执行，且 HAL 的 ms 级超时依赖 SysTick
 * （优先级 15，无法抢占控制中断）——一旦卡住会永不返回。故改为寄存器级收发 +
 * DWT 周期计数限时：正常一帧 1.78µs，超时值只作兜底。 */
#define kth7823_SPI_TIMEOUT_US    10u

/* 连续无效读数（0x0000/0xFFFF）阈值：快路径逐拍累加，100 拍 ≈ 5ms。
 * 改造前为 1kHz × 8 次 = 8ms，量级保持一致，避免提高刷新率后抗扰能力下降 10 倍。 */
#define kth7823_RAW_FAULT_LIMIT   100u

/* 多圈解包裹"野值剔除"上限 */
#define kth7823_UNWRAP_MAX_DELTA   1024

/* 一拍延迟补偿：补偿量 = ω × (kth7823_ANGLE_DELAY_PERIODS × control_dt)。
 * 单帧流水线固有滞后正好 1 个控制周期；台架可微调 0.5~1.5 拍。 */
#define kth7823_ANGLE_DELAY_PERIODS  1.0f


// 编码器输出：volatile 标记跨中断上下文共享的实时值，不保证多字段快照一致性
extern volatile float kth7823_Angle_Rad;
extern float kth7823_Angle_Deg;
extern volatile float kth7823_Angle_Multiturn;
extern volatile float kth7823_Angle_Multiturn_Comp;
extern volatile float kth7823_Speed_Rads;
extern volatile float kth7823_Speed_RPM;

// 刷新心跳计数器：慢路径（1kHz）每次 +1，供 Protect 判断编码器是否在刷新
extern volatile uint32_t encoder_update_count;

// 连续读到异常原始值（0x0000/0xFFFF）的次数，诊断用；正常 raw 清零
extern volatile uint16_t kth7823_raw_fault_cnt;

// 距离最近一次接受有效角度样本的控制周期数（20kHz），饱和计数
extern volatile uint16_t kth7823_stale_ticks;

// 被丢弃的解包裹野值累计次数（诊断用，只增不清零）
extern volatile uint16_t kth7823_unwrap_glitch_cnt;

// SPI 帧失败次数（HAL 返回非 OK / 重入被拒），诊断用，只增不清零
extern volatile uint16_t kth7823_spi_err_cnt;

// 函数声明
void kth7823_Init(void);
void kth7823_Tick(uint32_t tick);   /* 仅由 TIM1 20kHz 中断调用（SPI1 唯一所有者） */
void kth7823_SlowUpdate(void);      /* 主循环 ≈1kHz：滤波角度（不碰 SPI） */

#endif
