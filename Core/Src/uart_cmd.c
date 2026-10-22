#include "uart_cmd.h"
#include "usart.h"
#include "DRV8353.h"
#include "foc.h"
#include "sense.h"
#include "kth7823.h"
#include "Protect.h"
#include "fdcan_app.h"  
#include "fdcan.h"       // hfdcan2（串口 can 诊断命令读外设状态/错误计数）
#include "dwt_delay.h"   // [TEST] DWT_DelayMs（开环角度运动阻塞延时）
#include "iwdg.h"        // [TEST] HAL_IWDG_Refresh / hiwdg
#include "motor_control.h"   // MotorControl_CommandGate（硬件错误锁死门控）
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>    // fabsf（对齐结果自检用）

//外部变量
extern float manual_voltage;	
extern float manual_speed;
extern float control_dt;               

#define UART_RX_BUF_SIZE  64
#define UART_RX_DMA_SIZE  256u                    // RX DMA 循环缓冲，必须是 2 的幂
#define UART_RX_DMA_MASK  (UART_RX_DMA_SIZE - 1u)

// 非阻塞发送缓冲（TX 环形缓冲 + DMA 发送）
#define UART_TX_BUF_SIZE   2048u                 // 必须是 2 的幂
#define UART_TX_BUF_MASK   (UART_TX_BUF_SIZE - 1u)

// 开环点动自动归零：输入 v= 后启动计时
#define AUTO_STOP_MS  3000   // 自动归零延时 (ms)

static  uint8_t rx_dma_buf[UART_RX_DMA_SIZE];    // DMA 循环写入的接收缓冲
static  volatile uint16_t rx_dma_pos = 0;        // 已消费到的位置（0..UART_RX_DMA_SIZE）
static  char rx_buf[UART_RX_BUF_SIZE];       // 正在组装的行（回调独占）
static  uint8_t rx_index = 0;                // 组装位置（回调独占）
static  uint8_t rx_overflow = 0;             // 1 = 本行超长，丢弃到行尾（回调独占）
static  char cmd_line[UART_RX_BUF_SIZE];     // 交付给主循环的完整命令行
static  volatile uint8_t cmd_ready = 0;      // 1 = cmd_line 中有完整命令待主循环处理

static uint8_t  tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t tx_head    = 0;   // 生产者写入位置
static volatile uint16_t tx_tail    = 0;   // 消费者读出位置
static volatile uint16_t tx_chunk   = 0;   // 当前 DMA 发送的字节数
static volatile uint8_t  tx_active  = 0;   // 1 = DMA 正在发送
static volatile uint32_t tx_dropped = 0;   // 缓冲满被整条丢弃的消息数

static uint32_t v_start_tick= 0;   // 自动归零截止时间戳 (ms)
static uint8_t  v_active = 0;       // 是否已启动自动归零

// 自动上报开关：log=1 开启
static uint8_t auto_report = 0;
static uint16_t report_fre = 200; //主循环发送实时状态间隔时间

// 帮助信息
static const char* help_msg = 
    "\r\n========== Motor Control Commands ==========\r\n"
    "help           - Show this help\r\n"
    "fault          - Read DRV8353 fault status\r\n"
    "v=<float>      - Set open-loop voltage (V), e.g. v=0.5\r\n"
    "s=<float>      - Set target speed (rad/s), e.g. s=10\r\n"
    "t=<float>      - Set target torque (Nm), e.g. t=1.0\r\n"
    "mode=<0..5>    - Set control mode (0:IDLE 1:OPENLOOP 2:ALIGN 3:TORQUE 4:SPEED 5:POSITION)\r\n"
    "dir=<0|1>      - Set direction (0:forward, 1:reverse)\r\n"
    "status         - Print current status (Mode uses mode= numbering: 6=FAULT)\r\n"
    "can            - Print CAN(FDCAN2) live diagnostics (state/errors/RX/TX/last frame)\r\n"
    "log=<0|1>      - Auto report realtime status every 200ms (Ia/Ib/Ic, Vbus, temp, speed, mode)\r\n"

    "pid            - Print all PID gains (Kp/Ki/Kd)\r\n"
    "pid <ring> <Kp> <Ki> <Kd> - Set PID (ring: speed|id|iq|pos)\r\n"
    "enable         - Enable DRV8353 output\r\n"
    "disable        - Disable DRV8353 output\r\n"
    "ang=<deg>      - [TEST] Open-loop rotate given reducer-output angle (deg),\r\n"
    "                 hold on as electric spring (encoder pull-back) 3s then release\r\n"
    "==============================================\r\n";

static void UART_Rx_Start(void);   // 启动/重启 RX DMA 接收（见下）

//串口命令处理初始化
void UART_Cmd_Init(void)
{
    // 清空缓冲区
    memset(rx_buf, 0, UART_RX_BUF_SIZE);
    memset(cmd_line, 0, UART_RX_BUF_SIZE);
    cmd_ready   = 0;

    // 启动接收：RX DMA 循环缓冲 + IDLE 事件
    UART_Rx_Start();
}

// 启动/重启 RX 接收（DMA 循环 + IDLE 事件）。
// DMA 从缓冲下标 0 重新开始写，故消费位置同时归零，两者保持一致。
static void UART_Rx_Start(void)
{
    rx_index    = 0;
    rx_overflow = 0;
    rx_dma_pos  = 0;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buf, UART_RX_DMA_SIZE);
}

// 把一个已收到的字节喂给行组装器（逐字节逻辑与旧实现完全一致，只换数据来源）。
// '\n' 与 '\r' 都当作行结束符：兼容 CRLF / 仅 LF / 仅 CR 三种上位机行尾。
static void UART_Rx_FeedByte(char c)
{
    if (c == '\n' || c == '\r') {
        if (!rx_overflow && rx_index > 0 && !cmd_ready) {
            rx_buf[rx_index] = '\0';
            memcpy(cmd_line, rx_buf, (size_t)rx_index + 1u);
            cmd_ready = 1;      // 上一行未被主循环取走时，整行丢弃本行，绝不覆盖
        }
        rx_index    = 0;        // 重置，准备接收下一条命令
        rx_overflow = 0;
    } else {
        rx_buf[rx_index] = c;
        rx_index++;
        if (rx_index >= UART_RX_BUF_SIZE - 1) {
            rx_overflow = 1;    // 超长：整行丢弃到行尾，不再把半行当命令解析
            rx_index    = 0;
            UART_SendString("\r\nError: Command too long!\r\n");
        }
    }
}

// RX 事件回调（IDLE / 半满 / 全满三种事件都会进来）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3) {
        uint16_t pos = (uint16_t)(Size & UART_RX_DMA_MASK);
        while (rx_dma_pos != pos) {
            UART_Rx_FeedByte((char)rx_dma_buf[rx_dma_pos]);
            rx_dma_pos = (uint16_t)((rx_dma_pos + 1u) & UART_RX_DMA_MASK);
        }
    }
}

// 对外统一模式编号直接等于 FOC_State_t 枚举值（定义于 FOC.H），无需再做映射：
//   0=IDLE 1=OPENLOOP 2=ALIGN 3=TORQUE 4=SPEED 5=POSITION 6=FAULT
// 串口与本文件的 status/Mode 字段、CAN 反馈帧 byte5 均直接使用 (int)foc.mode。

/* ==================== BEGIN TEST: 开环角度运动（可整段删除）====================
 * 用途：开环模式下按“减速器输出端机械角度”转动指定角度，到位后以“电弹簧”形式保持，
 *       期间用电机轴编码器回拉，保持固定时长后自动释放（v=0, s=0）。
 * 原理：
 *   - 运动：开环无位置反馈，靠对“命令电角度速率(target_speed_ramp)”按控制节拍积分判到位。
 *           输出端机械角 -> 电机轴机械角(×减速比) -> 电角度(×极对数)。
 *   - 保持：到位后冻结电角度（foc.openloop_hold=1），改用 d 轴持续通电（与 ALIGN 磁力定位
 *           同形态），转子被磁力钳在到位点附近，偏离即自动产生回复力矩；不必依赖精确角度。
 *           稳定吸引域约 ±180 电角度 ≈ ±8.6° 电机轴机械角（极对数 21 → 电周期 17.14°）。
 *   - 回拉：以电机轴编码器(KTH7823_Angle_Multiturn，未滤波)为反馈，偏差超出
 *           TEST_OL_HOLD_BAND_DEG 时解除冻结、按低速开环回拉，回到滞环内重新冻结保持。
 * 说明：
 *   - 角度为“相对当前点”的相对量，非绝对零点；打滑/负载会导致偏差（故不要求精度）。
 *   - 阻塞式：动作期间不处理其他串口命令，带超时保护；保持阶段结束自动释放。
 *   - 转速/电压优先取当前 s=/v= 设定，未设定则用下方默认值。
 * 删除方法：删除本段全部内容 + UART_Cmd_Process 中标注 [TEST] 的那一行
 *          + help_msg 中标注 [TEST] 的那一行 + 头部两个 [TEST] include（可选）。
 * ============================================================================ */
#define TEST_GEAR_RATIO_OUT2MOTOR   12.5f   // 减速器输出端 -> 电机轴 的减速比
#define TEST_OL_DEFAULT_SPEED       5.0f   // 未设转速时的默认开环转速 (rad/s，电机轴)
#define TEST_OL_DEFAULT_VOLTAGE     0.6f    // 未设电压时的默认开环电压 (V)
#define TEST_OL_MOVE_TIMEOUT_MS     5000    // 单次动作超时 (ms)

// ---- 到位后“电弹簧”保持参数 ----
#define TEST_OL_HOLD_MS             3000    // 到位后保持时长 (ms)，到点自动释放(v=0, s=0)
#define TEST_OL_HOLD_VOLTAGE_MAX    0.4f    // 保持电压上限 (V)：保持电压取本次运动电压但不超此值。
                                            //   d 轴持续通电 → 越大保持力矩越强、发热越大（与 ALIGN_VOLTAGE 同量级）
#define TEST_OL_HOLD_BAND_DEG       10.0f   // 偏差死区 (±deg，电机轴)：超出即触发编码器回拉
#define TEST_OL_HOLD_HYST_DEG       3.0f    // 回拉滞环 (±deg，电机轴)：回到该范围内即重新冻结保持
#define TEST_OL_HOLD_PULL_SPEED     3.0f    // 回拉速度 (rad/s，电机轴)：取低速，避免越过到位点
#define TEST_OL_HOLD_PULL_TIMEOUT_MS 1000   // 单次回拉最长耗时 (ms)
#define TEST_OL_DEG2RAD             0.0174532925f
#define TEST_OL_TWO_PI              6.283185307f

extern volatile uint32_t control_tick;      // 控制节拍计数（motor_control.c 定义，20kHz）


static char ol_msg[96];                     // 保持阶段回显缓冲（静态区：主栈仅 1KB，不可重入）

// 电角度归一化到 0~2π（与 FOC_Update 开环分支一致）
static float UART_OL_NormElecAngle(float a)
{
    a = fmodf(a, TEST_OL_TWO_PI);
    if (a < 0.0f) a += TEST_OL_TWO_PI;
    return a;
}

// 当前转子电角度（与 FOC_SetMode 进入开环时的表达式/角度源完全一致）
static float UART_OL_RotorElecAngle(void)
{
    return UART_OL_NormElecAngle((KTH7823_Angle_Multiturn - encoder_offset) *
                                 (float)encoder_direction * (float)MOTOR_POLE_PAIRS);
}

// 到位后“电弹簧”保持：冻结电角度 + d 轴通电（保持力矩），
// 用电机轴编码器监测偏差，超出死区自动低速回拉；保持 TEST_OL_HOLD_MS 后释放。
// hold_vol：保持电压 (V)，调用方已限幅。阻塞执行，超时/到点必退出。
static void UART_Test_OpenLoopHold(float hold_vol)
{
    const float band = TEST_OL_HOLD_BAND_DEG * TEST_OL_DEG2RAD;   // 死区 (±rad，电机轴)
    const float hyst = TEST_OL_HOLD_HYST_DEG * TEST_OL_DEG2RAD;   // 回拉滞环 (±rad，电机轴)

    // 到位点（电机轴多圈机械角）；同时清零速度给定，避免解除冻结瞬间角度继续积分
    float hold_mech = KTH7823_Angle_Multiturn;
    manual_speed          = 0.0f;
    foc.target_speed      = 0.0f;
    foc.target_speed_ramp = 0.0f;
    FOC_SetOpenLoopVoltage(hold_vol);

    // 对齐并冻结：平衡点 = 到位点，转子被磁力钳住并具备回复力矩
    float theta_hold   = UART_OL_RotorElecAngle();   // 到位点对应的电角度（回拉后用它复位，防逐次漂移）
    foc.openloop_angle = theta_hold;
    foc.openloop_hold  = 1;

    uint32_t t_hold = HAL_GetTick();
    UART_SendString("Hold on (electric spring)...\r\n");

    while ((HAL_GetTick() - t_hold) < TEST_OL_HOLD_MS) {
        // 阻塞期间保持喂狗 + 发串口即可：编码器刷新已移入 20kHz 中断，
        // 下面的 dev 用到的多圈角持续更新（不再需要自己发起 SPI 帧）。
        UART_Tx_Poll();
        HAL_IWDG_Refresh(&hiwdg);

        // 偏差：正 = 转子朝“正转（+开环方向）”偏出到位点
        float dev = (KTH7823_Angle_Multiturn - hold_mech) * (float)encoder_direction;

        if (fabsf(dev) > band) {
            // ---- 逃出死区：解除冻结，按低速开环回拉 ----
            foc.openloop_hold  = 0;
            foc.openloop_angle = UART_OL_RotorElecAngle();   // 平衡点=当前位置，避免跨极反向弹走
            manual_speed       = (dev > 0.0f) ? -TEST_OL_HOLD_PULL_SPEED : TEST_OL_HOLD_PULL_SPEED;
            foc.target_speed   = manual_speed;
            FOC_SetOpenLoopVoltage(hold_vol);

            uint32_t t_pull = HAL_GetTick();
            while ((HAL_GetTick() - t_pull) < TEST_OL_HOLD_PULL_TIMEOUT_MS) {
                // 阻塞期间喂狗 + 发串口；角度由 20kHz 中断持续刷新
                UART_Tx_Poll();
                HAL_IWDG_Refresh(&hiwdg);

                dev = (KTH7823_Angle_Multiturn - hold_mech) * (float)encoder_direction;
                if (fabsf(dev) <= hyst) break;                            // 回到滞环内
                if ((dev > 0.0f) != (manual_speed < 0.0f)) break;         // 已越过到位点（偏差变号）
                if ((HAL_GetTick() - t_hold) >= TEST_OL_HOLD_MS) break;   // 保持总时长到点，不再回拉
                DWT_DelayMs(1);
            }

            // 重新对齐并冻结保持：
            // 优先冻结回“原始到位点”对应的电角度，让弹簧把转子拉回原位置，避免逐次回拉累积漂移；
            // 若偏差已超出弹簧吸引域(±180 电角度 ≈ ±8.6° 电机轴)，则退化为对齐当前位置，
            // 否则回复力矩方向会反过来把转子推向相邻磁极。
            manual_speed          = 0.0f;
            foc.target_speed      = 0.0f;
            foc.target_speed_ramp = 0.0f;
            FOC_SetOpenLoopVoltage(hold_vol);
            if (fabsf(dev) * (float)MOTOR_POLE_PAIRS < TEST_OL_TWO_PI / 2.0f) {
                foc.openloop_angle = theta_hold;
            } else {
                foc.openloop_angle = UART_OL_RotorElecAngle();
            }
            foc.openloop_hold = 1;

            sprintf(ol_msg, "Hold: pulled back (dev=%.1f deg)\r\n", dev * 57.2957795f);
            UART_SendString(ol_msg);
        }

        DWT_DelayMs(1);
    }

    // 保持时长到点：释放，按要求 v=0、s=0
    foc.openloop_hold     = 0;
    manual_voltage        = 0.0f;
    manual_speed          = 0.0f;
    foc.target_speed      = 0.0f;
    foc.target_speed_ramp = 0.0f;
    FOC_SetOpenLoopVoltage(0.0f);
    UART_SendString("Hold released (v=0, s=0)\r\n");
}

// 开环转动指定“输出端”机械角度（°，正=正转）。返回 0=完成，-1=参数非法，-2=超时
int UART_Test_OpenLoopMoveAngle(float out_deg)
{
    float out_rad = out_deg * 0.0174532925f;
    if (fabsf(out_rad) < 1e-5f) return -1;

    float spd = fabsf(manual_speed);   if (spd < 1.0f) spd = TEST_OL_DEFAULT_SPEED;
    float vol = fabsf(manual_voltage); if (vol < 0.1f) vol = TEST_OL_DEFAULT_VOLTAGE;
    float dir = (out_rad >= 0.0f) ? 1.0f : -1.0f;

    // 输出端机械角 -> 电机轴机械角 -> 电角度 (rad)
    float target_elec = fabsf(out_rad) * TEST_GEAR_RATIO_OUT2MOTOR * (float)MOTOR_POLE_PAIRS;

    // 进入开环并给定转速/电压
    FOC_SetMode(FOC_STATE_OPENLOOP);
    manual_speed     = dir * spd;
    manual_voltage   = vol;
    foc.target_speed = manual_speed;
    FOC_SetOpenLoopVoltage(vol);

    // 用控制节拍对“命令电角度速率”积分：与中断内推进 openloop_angle 同源，判转够
    uint32_t t0        = HAL_GetTick();
    uint32_t last_tick = control_tick;
    float    moved     = 0.0f;

    while (fabsf(moved) < target_elec) {
        uint32_t now      = control_tick;
        uint32_t dt_ticks = now - last_tick;
        last_tick = now;
        moved += foc.target_speed_ramp * (float)MOTOR_POLE_PAIRS * control_dt * (float)dt_ticks;

        // 阻塞期间喂狗 + 发串口；角度/测速由 20kHz 中断维持，运动结束后 hold 阶段拿到的仍是新值
        UART_Tx_Poll();
        HAL_IWDG_Refresh(&hiwdg);  // 阻塞期间喂狗，防 IWDG 复位
        if ((HAL_GetTick() - t0) > TEST_OL_MOVE_TIMEOUT_MS) break;
        DWT_DelayMs(1);
    }

    // 运动阶段结束：先清零给定与保持标志，避免进入保持/释放瞬间角度继续积分
    int timed_out = ((HAL_GetTick() - t0) > TEST_OL_MOVE_TIMEOUT_MS) ? 1 : 0;
    foc.target_speed      = 0.0f;
    foc.target_speed_ramp = 0.0f;
    manual_speed          = 0.0f;
    foc.openloop_hold     = 0;

    if (timed_out) {
        // 运动超时（未转够）：不进入保持，直接断电释放
        manual_voltage = 0.0f;
        FOC_SetOpenLoopVoltage(0.0f);
        return -2;
    }

    // 到位：进入“电弹簧”保持（d 轴通电 + 电机轴编码器回拉），到点自动释放
    UART_Test_OpenLoopHold((vol > TEST_OL_HOLD_VOLTAGE_MAX) ? TEST_OL_HOLD_VOLTAGE_MAX : vol);
    return 0;
}

// 命令解析：ang=<deg>。返回 1=已处理，0=非本命令
static int UART_Test_AngleCmd(const char* cmd_buf, char* response)
{
    if (strncmp(cmd_buf, "ang=", 4) != 0) return 0;

    float deg = atof(cmd_buf + 4);
    if (deg == 0.0f) {
        UART_SendString("Usage: ang=<deg> (reducer output shaft, nonzero)\r\n");
        return 1;
    }

    UART_SendString("Angle move start...\r\n");   // 先回显，避免阻塞后才提示
    int r = UART_Test_OpenLoopMoveAngle(deg);
    if      (r == 0)  sprintf(response, "Angle move done (out): %.1f deg, held %d ms then released\r\n", deg, TEST_OL_HOLD_MS);
    else if (r == -2) sprintf(response, "Angle move TIMEOUT (out): %.1f deg (partial, no hold)\r\n", deg);
    else              sprintf(response, "Angle move invalid\r\n");
    UART_SendString(response);
    return 1;
}
/* ==================== END TEST: 开环角度运动 ==================== */

/* ==================== 故障掩码上报（主循环调用）====================
 * 打印一次故障事件触发过的全部保护位（掩码 + 名称），与 main.c 里按 fault_code
 * 打印的 "FAULT: XXX" 互补——后者只反映最后一个写入者。 */

/* 故障码 -> 名称（仅打印用） */
static const char* UART_FaultName(uint8_t code)
{
    switch (code) {
        case FAULT_CODE_HARDWARE:      return "HARDWARE";
        case FAULT_CODE_STALL:         return "STALL";
        case FAULT_CODE_ENCODER_STALL: return "ENCODER_STALL";
        case FAULT_CODE_OVERTEMP:      return "OVERTEMP";
        case FAULT_CODE_OVERVOLT:      return "OVERVOLT";
        case FAULT_CODE_UNDERVOLT:     return "UNDERVOLT";
        case FAULT_CODE_OVERSPEED:     return "OVERSPEED";
        case FAULT_CODE_PHASE_OC:      return "PHASE_OC";
        case FAULT_CODE_I2T:           return "I2T";
        case FAULT_CODE_POS_ERROR:     return "POS_ERROR";
        case FAULT_CODE_SOFT_LIMIT:    return "SOFT_LIMIT";
        case FAULT_CODE_COMM_TIMEOUT:  return "COMM_TIMEOUT";
        default:                       return "-";
    }
}

/* 故障掩码 -> 名称串（"A|B|C"）；无位时输出 "-" */
static void UART_MaskToStr(uint16_t mask, char *out)
{
    out[0] = '\0';
    uint8_t first = 1u;
    for (uint8_t c = 1u; c <= 12u; c++) {
        if (mask & FAULT_MASK_BIT(c)) {
            if (!first) strcat(out, "|");
            strcat(out, UART_FaultName(c));
            first = 0u;
        }
    }
    if (first) strcpy(out, "-");
}

/* 主循环上报：打印一次故障事件触发过的全部保护（掩码 + 名称）。
 * 与 main.c 里按 fault_code 打印的 "FAULT: XXX" 互补——后者只反映最后一个写入者。 */
void UART_PrintFaultMask(uint16_t mask, uint8_t code)
{
    static char masks[192];   /* 静态区：主栈仅 1KB，避免大数组上栈 */
    static char buf[256];
    UART_MaskToStr(mask, masks);
    sprintf(buf, "FAULT: code=%u(%s) mask=0x%03X {%s}\r\n",
            (unsigned)code, UART_FaultName(code), (unsigned)mask, masks);
    UART_SendString(buf);
}

/* ==================== 控制命令识别（启动自检硬件错误锁死门控用）====================
 * 命中返回 1：这些命令会驱动机器（对齐/开环/闭环/力矩等），需经过门控；
 * 诊断命令（help/fault/status/pid/log=/can 等）返回 0，锁死后仍可用。 */
static int UART_IsControlCmd(const char* c)
{
    return (strncmp(c, "ang=",  4) == 0 ||
            strncmp(c, "mode=", 5) == 0 ||
            strncmp(c, "v=",    2) == 0 ||
            strncmp(c, "s=",    2) == 0 ||
            strncmp(c, "t=",    2) == 0 ||
            strncmp(c, "dir=",  4) == 0 ||
            strcmp (c, "enable")   == 0);
}

//解析并执行命令（在主循环中调用）
void UART_Cmd_Process(void)
{
    if (!cmd_ready) return;

    // 取出完整命令行：cmd_ready=1 期间中断不会改写 cmd_line，故先整体拷出、再清标志，
    // 命令行既不会被撕开，也不与中断争 rx_buf（主循环不再触碰接收缓冲、不再挂接收中断）。
    // 静态区：主栈仅 1KB，且本函数只在主循环调用、不可重入。
    static char cmd_buf[UART_RX_BUF_SIZE];
    memcpy(cmd_buf, cmd_line, UART_RX_BUF_SIZE);
    cmd_ready = 0;

    // 控制命令门控：上电自检硬件错误锁死后拒绝执行（诊断命令不经过此处，仍可用）
    if (UART_IsControlCmd(cmd_buf) && !MotorControl_CommandGate()) return;

    // 解析命令。
    // 说明：response 放静态区而非栈上——主栈只有 1KB，且 UART_Cmd_Process 仅在主循环
    //       调用、不可重入；UART_SendString 是非阻塞的，拷进发送环形缓冲后本缓冲即可复用。
    static char response[384];

    // [TEST] 开环角度运动命令（删除测试功能时连同本行一起删）
    if (UART_Test_AngleCmd(cmd_buf, response)) return;
    
    // help
    if (strcmp(cmd_buf, "help") == 0) {
        UART_SendString(help_msg);
        return;
    }
    
    // fault - 读取故障状态
    if (strcmp(cmd_buf, "fault") == 0) {
        uint16_t status1 = DRV8353_ReadReg(DRV8353_REG_STATUS1);
        uint16_t status2 = DRV8353_ReadReg(DRV8353_REG_STATUS2);
        uint16_t ctrl = DRV8353_ReadReg(DRV8353_REG_DRIVER_CTRL);
        sprintf(response, 
                "STATUS1=0x%04X STATUS2=0x%04X CTRL=0x%04X\r\n",
                status1, status2, ctrl);
        UART_SendString(response);
        
        // 解析具体故障位 (STATUS1 地址 0x00h, DRV8353xS 表 8-11)
        if (status1 & 0x0400) UART_SendString("  FAULT: nFAULT logic OR!\r\n");
        if (status1 & 0x0200) UART_SendString("  VDS_OCP: any phase VDS overcurrent!\r\n");
        if (status1 & 0x0100) UART_SendString("  GDF: Gate driver fault!\r\n");
        if (status1 & 0x0080) UART_SendString("  UVLO: Under voltage lockout!\r\n");
        if (status1 & 0x0040) UART_SendString("  OTSD: Over temperature shutdown!\r\n");
        if (status1 & 0x0020) UART_SendString("  VDS_HA: A high-side VDS!\r\n");
        if (status1 & 0x0010) UART_SendString("  VDS_LA: A low-side VDS!\r\n");
        if (status1 & 0x0008) UART_SendString("  VDS_HB: B high-side VDS!\r\n");
        if (status1 & 0x0004) UART_SendString("  VDS_LB: B low-side VDS!\r\n");
        if (status1 & 0x0002) UART_SendString("  VDS_HC: C high-side VDS!\r\n");
        if (status1 & 0x0001) UART_SendString("  VDS_LC: C low-side VDS!\r\n");
        return;
    }
    
    // status - 打印当前状态
    if (strcmp(cmd_buf, "status") == 0) {
        sprintf(response,
                "Speed=%.2f rad/s (%.2f RPM) | Angle=%.2f deg\r\n"
                "Ia=%.3f A, Ib=%.3f A, Ic=%.3f A\r\n"
                "Id=%.3f A, Iq=%.3f A | Mode=%d\r\n"
                "TargetSpeed=%.2f rad/s | Voltage=%.2f V | V_Bus=%.2f V\r\n | NTC_Temp=%.2f ℃\r\n "
                "Align: offset=%.4f rad (%.2f deg) | dir=%+d\r\n"
                "ADC_A=%u(%.3fV) ADC_B=%u(%.3fV) | InjMiss=%lu | TXdrop=%lu\r\n",
                KTH7823_Speed_Rads, KTH7823_Speed_RPM,
                KTH7823_Angle_Deg,
                current_A, current_B, current_C,
                foc.Id, foc.Iq, (int)foc.mode,
                foc.target_speed, manual_voltage, bus_voltage,ntc_temperature,	
                encoder_offset, encoder_offset * 57.2957795f, encoder_direction,
                adc1_values_inj[0], (float)adc1_values_inj[0] / 4095.0f * 3.3f,
                adc1_values_inj[1], (float)adc1_values_inj[1] / 4095.0f * 3.3f,
                (unsigned long)adc_inj_miss_cnt,
                (unsigned long)UART_TxDropCount());
        UART_SendString(response);
        return;
    }


    // ==================== PID 在线调参 ====================
    //
    // 功能：通过串口在电机运行时实时查看/修改 4 个 PID 环的比例、积分、微分系数。
    //       参数直接写入 PID_Handle_t 的 params 副本，控制中断（20kHz）内的 PID_Calculate
    //       下一个周期即采用新值，无需复位 MCU。
    //
    // 命令格式（两条）：
    //   1) 查询：pid
    //      打印全部 4 个环（速度/d轴电流/q轴电流/位置）当前的 Kp/Ki/Kd。
    //   2) 设置：pid <ring> <Kp> <Ki> <Kd>
    //      <ring> 取值：speed=速度环、id=d轴电流环、iq=q轴电流环、pos=位置环。
    //      后跟 3 个浮点数为新系数，均以空格分隔。
    //
    // 示例：
    //   pid                    -> 查询所有环参数
    //   pid speed 0.2 1.0 0.0  -> 速度环 Kp=0.2, Ki=1.0, Kd=0.0
    //   pid iq 3.0 0.5 0.0     -> q轴电流环 Kp=3.0, Ki=0.5, Kd=0.0
    //
    // 说明：
    //   - 修改后仅影响运行中的 RAM 参数，掉电/复位后恢复为 FOC.c 中的硬编码初始值；
    //   - 本命令直接写 params（不经过 PID_SetParams），因为 PID_SetParams 只改 Kp/Ki/Kd，
    //     此处直接写等价；写完后调用 PID_Reset 清空积分/微分历史，避免旧累积值造成输出突变。
    // ====================================================
    if (strcmp(cmd_buf, "pid") == 0) {
        sprintf(response,
                "PID speed: Kp=%.4f Ki=%.4f Kd=%.4f\r\n"
                "PID id   : Kp=%.4f Ki=%.4f Kd=%.4f\r\n"
                "PID iq   : Kp=%.4f Ki=%.4f Kd=%.4f\r\n"
                "PID pos  : Kp=%.4f Ki=%.4f Kd=%.4f\r\n",
                pid_speed.params.Kp, pid_speed.params.Ki, pid_speed.params.Kd,
                pid_id.params.Kp, pid_id.params.Ki, pid_id.params.Kd,
                pid_iq.params.Kp, pid_iq.params.Ki, pid_iq.params.Kd,
                pid_position.params.Kp, pid_position.params.Ki, pid_position.params.Kd);
        UART_SendString(response);
        return;
    }

    if (strncmp(cmd_buf, "pid ", 4) == 0) {
        // 解析：环名字符串（最长 7 字符，防溢出）+ 3 个浮点系数
        char ring[8];
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        // sscanf 返回成功解析的字段数，必须恰好等于 4，否则参数个数不对，提示用法
        if (sscanf(cmd_buf + 4, "%7s %f %f %f", ring, &kp, &ki, &kd) != 4) {
            UART_SendString("Usage: pid <speed|id|iq|pos> <Kp> <Ki> <Kd>\r\n");
            return;
        }

        PID_Handle_t *pid = NULL;   // 待设置的 PID 句柄
        const char *name = "";      // 环名（用于回显）
        if      (strcmp(ring, "speed") == 0) { pid = &pid_speed;    name = "speed"; }
        else if (strcmp(ring, "id")    == 0) { pid = &pid_id;       name = "id"; }
        else if (strcmp(ring, "iq")    == 0) { pid = &pid_iq;       name = "iq"; }
        else if (strcmp(ring, "pos")   == 0) { pid = &pid_position; name = "pos"; }
        else {
            UART_SendString("Unknown ring! Use speed|id|iq|pos\r\n");
            return;
        }

        pid->params.Kp = kp;
        pid->params.Ki = ki;
        pid->params.Kd = kd;
        PID_Reset(pid);

        // 回显本次设置结果
        sprintf(response, "PID %s updated: Kp=%.4f Ki=%.4f Kd=%.4f\r\n", name, kp, ki, kd);
        UART_SendString(response);
        return;
    }

    // v=<voltage> - 设置开环电压（设为非0 时启动自动停止定时器）
    if (strncmp(cmd_buf, "v=", 2) == 0) {
        float val = atof(cmd_buf + 2);
        manual_voltage = val;
        FOC_SetOpenLoopVoltage(val);
        if (val != 0.0f) {
            v_start_tick = HAL_GetTick();   // 启动 AUTO_STOP_MS 倒计时
            v_active = 1;
            sprintf(response, "Voltage set to %.3f V (auto stop in %d ms)\r\n", val, AUTO_STOP_MS);
        } else {
            v_active = 0;                   // 手动归零则取消定时器
            sprintf(response, "Voltage set to 0 (auto stop cancelled)\r\n");
        }
        UART_SendString(response);
        return;
    }

    // s=<speed> - 设置目标速度
    if (strncmp(cmd_buf, "s=", 2) == 0) {
        float val = atof(cmd_buf + 2);
        foc.target_speed = val;
        manual_speed = val;
        sprintf(response, "Target speed set to %.2f rad/s\r\n", val);
        UART_SendString(response);
        return;
    }

    // log=<0|1> - 自动实时上报状态
    if (strncmp(cmd_buf, "log=", 4) == 0) {
        int on = atoi(cmd_buf + 4);
        auto_report = (on != 0) ? 1 : 0;
        sprintf(response, "Auto report %s (every %d ms)\r\n", auto_report ? "ON" : "OFF",report_fre);
        UART_SendString(response);
        return;
    }

    // mode=<mode> - 切换控制模式
    if (strncmp(cmd_buf, "mode=", 5) == 0) {
        int mode = atoi(cmd_buf + 5);
        switch(mode) {
            case 0: FOC_SetMode(FOC_STATE_IDLE);     break;
            case 1: FOC_SetMode(FOC_STATE_OPENLOOP); break;
            case 2: FOC_SetMode(FOC_STATE_ALIGN);    break;
            case 3: FOC_SetMode(FOC_STATE_TORQUE);   break;   // 闭环力矩
            case 4: FOC_SetMode(FOC_STATE_SPEED);    break;   // 闭环速度
            case 5: FOC_SetMode(FOC_STATE_POSITION); break;   // 闭环位置（需输出端编码器）
            default:
                // 6=FAULT 不允许上位机主动设置（故障只能由保护触发，进入后由退避状态机恢复）
                UART_SendString("Invalid mode! (0:IDLE, 1:OPENLOOP, 2:ALIGN, 3:TORQUE, 4:SPEED, 5:POSITION)\r\n");
                return;
        }
        sprintf(response, "Mode switched to %d\r\n", mode);
        UART_SendString(response);
        return;
    }

    // t=<torque> - 设置目标转矩 (Nm)，力矩子模式使用
    if (strncmp(cmd_buf, "t=", 2) == 0) {
        float val = atof(cmd_buf + 2);
        FOC_SetTargetTorque(val);
        sprintf(response, "Target torque set to %.3f Nm\r\n", val);
        UART_SendString(response);
        return;
    }
    
    // dir=<0|1> - 设置方向
    if (strncmp(cmd_buf, "dir=", 4) == 0) {
        int dir = atoi(cmd_buf + 4);
        if (dir == 0) {
            foc.target_speed = fabs(foc.target_speed);
            manual_speed = foc.target_speed;
            UART_SendString("Direction: Forward\r\n");
        } else if (dir == 1) {
            foc.target_speed = -fabs(foc.target_speed);
            manual_speed = foc.target_speed;
            UART_SendString("Direction: Reverse\r\n");
        } else {
            UART_SendString("Invalid direction! Use 0 (forward) or 1 (reverse)\r\n");
        }
        return;
    }
    
    // enable / disable
    if (strcmp(cmd_buf, "enable") == 0) {
        DRV8353_Recover();           // 清故障 + 恢复PWM + 使能 + 重启三路
        FOC_SetMode(FOC_STATE_OPENLOOP);
        Protect_CommDisarm();        // 进入本地调试态：关闭 CAN 失联保护，避免串口 enable 被失联检测打回 FAULT
        UART_SendString("DRV8353 output enabled (fault cleared)\r\n");
        return;
    }
    if (strcmp(cmd_buf, "disable") == 0) {
        DRV8353_Disable();
        UART_SendString("DRV8353 output disabled\r\n");
        return;
    }

    // ==================== can - 打印 CAN(FDCAN2) 实时诊断（只读，不改变任何状态）====================
    // 排查用法（配合 CAN 联调）：
    //   RX(node)=0                 -> 下位机一帧都没收到：查帧 ID(0x100+NODE_ID)、1Mbps、标准帧/经典帧、
    //                                 DLC=8、接线与两端 120Ω 终端、收发器供电/使能
    //   RX(node) 在涨但电机没反应  -> 帧收到了：看 armed（=1 时 20ms 无帧就会 COMM_TIMEOUT）与 ErrFlags
    //   RX/TX 都在涨且 TEC/REC=0   -> 链路正常，问题在上位机侧解析（ID/字段/门控顺序）
    // 注：TEC/REC 与 LEC 直接读寄存器（缓存值只在 EW/EP/BO 跳变时更新，不能反映当前状态）；
    //     读 PS 会清掉 LEC 锁存，所以每次 can 看到的是"上次读取以来的最后一次总线错误"。
    if (strcmp(cmd_buf, "can") == 0) {
        FDCAN_ErrorCountersTypeDef  ec;
        FDCAN_ProtocolStatusTypeDef ps;
        const char *st;

        switch (hfdcan2.State) {
            case HAL_FDCAN_STATE_RESET: st = "RESET";   break;   // 未初始化/已关闭
            case HAL_FDCAN_STATE_READY: st = "READY";   break;   // 已初始化但未 Start
            case HAL_FDCAN_STATE_BUSY:  st = "BUSY";    break;   // 唯一的正常收发态
            case HAL_FDCAN_STATE_ERROR: st = "ERROR";   break;   // 外设异常（BusOff 恢复失败会落到这里）
            default:                    st = "UNKNOWN"; break;
        }

        (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &ec);
        (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &ps);

        sprintf(response,
                "CAN node=%d CMD_ID=0x%03X FBK_ID=0x%03X | State=%s ErrorCode=0x%08lX\r\n"
                "TEC=%u REC=%u | PS: Act=%lu LEC=%lu DLEC=%lu Warning=%lu ErrPassive=%lu BusOff=%lu\r\n"
                "ErrFlags=0x%02lX (EW=%d EP=%d BO=%d) | BusOffCnt=%lu ProtoErrCnt=%lu\r\n",
                NODE_ID, (unsigned)CMD_FRAME_ID, (unsigned)FBK_FRAME_ID,
                st, (unsigned long)hfdcan2.ErrorCode,
                (unsigned)ec.TxErrorCnt, (unsigned)ec.RxErrorCnt,
                (unsigned long)ps.Activity, (unsigned long)ps.LastErrorCode,
                (unsigned long)ps.DataLastErrorCode, (unsigned long)ps.Warning,
                (unsigned long)ps.ErrorPassive, (unsigned long)ps.BusOff,
                (unsigned long)fdcan_error_flags,
                (fdcan_error_flags & FDCAN_ERR_FLAG_WARNING) ? 1 : 0,
                (fdcan_error_flags & FDCAN_ERR_FLAG_PASSIVE) ? 1 : 0,
                (fdcan_error_flags & FDCAN_ERR_FLAG_BUSOFF)  ? 1 : 0,
                (unsigned long)fdcan_busoff_count, (unsigned long)fdcan_proto_err);
        UART_SendString(response);

        sprintf(response,
                "RX(node)=%lu RX(otherID)=%lu RX(queueFull)=%lu TX=%lu TXfail=%lu | Comm rx_count=%lu armed=%d\r\n"
                "LastRX: ID=0x%03lX DLC=%u Data=%02X %02X %02X %02X %02X %02X %02X %02X t=%lums\r\n",
                (unsigned long)fdcan_rx_count, (unsigned long)fdcan_rx_otherid_count,
                (unsigned long)fdcan_rx_drop_count, (unsigned long)fdcan_tx_count,
                (unsigned long)fdcan_tx_fail_count,
                (unsigned long)comm_rx_count, (int)Protect_CommIsArmed(),
                (unsigned long)fdcan_last_rx_id, (unsigned)fdcan_last_rx_dlc,
                (unsigned)fdcan_last_rx_data[0], (unsigned)fdcan_last_rx_data[1],
                (unsigned)fdcan_last_rx_data[2], (unsigned)fdcan_last_rx_data[3],
                (unsigned)fdcan_last_rx_data[4], (unsigned)fdcan_last_rx_data[5],
                (unsigned)fdcan_last_rx_data[6], (unsigned)fdcan_last_rx_data[7],
                (unsigned long)fdcan_last_rx_ms);
        UART_SendString(response);

        if (!fdcan_last_rx_valid) {
            UART_SendString("Note: no command frame received yet -> check frame ID (0x100+NODE_ID), "
                            "1Mbps, standard/classic frame, DLC=8, wiring & 120ohm terminator\r\n");
        }
        return;
    }

    // 未知命令
    UART_SendString("Unknown command. Type 'help' for available commands.\r\n");
}

// 发送缓冲剩余可写入字节数（保留 1 字节用于区分空/满）
uint16_t UART_TxFree(void)
{
    uint16_t head = tx_head;
    uint16_t tail = tx_tail;
    uint16_t used = (uint16_t)((head - tail) & UART_TX_BUF_MASK);
    return (uint16_t)(UART_TX_BUF_MASK - used);
}

// 因发送缓冲满而被整条丢弃的消息数（正常调试不应增长）
uint32_t UART_TxDropCount(void)
{
    return tx_dropped;
}

// 空闲则把缓冲里的一段连续数据挂到 DMA 发送
static void UART_Tx_Kick(void)
{
    if (tx_active) return;                     // 已有传输在进行，回调会续传
    if (tx_head == tx_tail) return;            // 无待发数据

    uint16_t tail = tx_tail;
    uint16_t head = tx_head;
    uint16_t len  = (head > tail) ? (uint16_t)(head - tail)
                                  : (uint16_t)(UART_TX_BUF_SIZE - tail);

    tx_chunk  = len;
    tx_active = 1;
    /* DMA_NORMAL：一段发完自动停，由 TxCplt 回调再挂下一段 */
    if (HAL_UART_Transmit_DMA(&huart3, &tx_buf[tail], len) != HAL_OK) {
        tx_chunk  = 0;
        tx_active = 0;                         // 启动失败：交由 UART_Tx_Poll / 下次发送重试
    }
}

// 主循环调用：兜底重试。正常情况下数据由 UART_SendString 与 DMA 发送完成回调推进
void UART_Tx_Poll(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    UART_Tx_Kick();
    __set_PRIMASK(primask);
}

//发送字符串（非阻塞：拷入发送环形缓冲后立即返回，可在中断中调用）
void UART_SendString(const char* str)
{
    if (str == NULL) return;

    uint16_t len = (uint16_t)strlen(str);
    if (len == 0u) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t head = tx_head;
    uint16_t tail = tx_tail;
    uint16_t used = (uint16_t)((head - tail) & UART_TX_BUF_MASK);

    if ((uint16_t)(UART_TX_BUF_MASK - used) < len) {
        tx_dropped++;                          // 空间不足：整条丢弃，避免输出半行
    } else {
        for (uint16_t i = 0u; i < len; i++) {
            tx_buf[head] = (uint8_t)str[i];
            head = (uint16_t)((head + 1u) & UART_TX_BUF_MASK);
        }
        tx_head = head;
        UART_Tx_Kick();
    }

    __set_PRIMASK(primask);
}

// DMA 发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        tx_tail   = (uint16_t)((tx_tail + tx_chunk) & UART_TX_BUF_MASK);
        tx_chunk  = 0;
        tx_active = 0;
        UART_Tx_Kick();                        // 已在中断上下文，可直接续传
    }
}

// 串口出错回调：释放发送状态，并在接收被中止时重新挂上 RX DMA 接收
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
			
        tx_chunk  = 0;
        tx_active = 0;

        if (huart->RxState == HAL_UART_STATE_READY) {
            UART_Rx_Start();
        }
    }
}

//发送状态反馈（用串口 log=1 开启）
void UART_SendStatus(void)
{
    if (!auto_report) return;   // 未开启自动上报则直接返回

    // 如果频繁调用，建议加个时间限制，防止刷屏
    static uint32_t last_send = 0;
    if (HAL_GetTick() - last_send < report_fre) return;  // 每report_fre ms发送一次

    // 背压：发送缓冲快满（上位机没在读/终端卡顿）就跳过本次上报，
    // 避免把命令回显等消息挤掉；缓冲一空出来就会立即补发。
    if (UART_TxFree() < 160u) return;

    last_send = HAL_GetTick();

    char buf[192];
    sprintf(buf,
            "Ia=%+.2f Ib=%+.2f Ic=%+.2f A | Vbus=%.1f V | T=%.1f C | S=%.1f rad/s | Mode=%d | ADC_A=%u ADC_B=%u | Miss=%lu\r\n",
            current_A, current_B, current_C,
            bus_voltage,
            ntc_temperature,
            KTH7823_Speed_Rads,
            (int)foc.mode,
            adc1_values_inj[0], adc1_values_inj[1],
            (unsigned long)adc_inj_miss_cnt);
    UART_SendString(buf);

}

// 自动停止
void UART_Cmd_AutoStop(void)
{
    if (!v_active) return;                                    // 定时器未启动
    if (HAL_GetTick() - v_start_tick < AUTO_STOP_MS) return;   // 未到时间

    manual_voltage = 0.0f;
    FOC_SetOpenLoopVoltage(0.0f);   // 电压上限归零 → V/F 算出的 Vq 被限到 0，立即停止输出
    v_active = 0;
    UART_SendString("\r\n[AUTO STOP] voltage -> 0 (timeout)\r\n");
}

// 对齐完成结果上报
void UART_ReportAlignResult(void)
{
    if (!align_report_pending) return;
    align_report_pending = 0;      // 先清标志，避免打印期间又被置位造成重复上报

    float  dlt = align_report_delta;
    float  off = align_report_offset;
    int8_t dir = align_report_direction;

    // 理论机械转角 = 90°(电) / 极对数：0° 与 90° 两步之间转子应转过的角度
    float expect = 1.5707963f / (float)MOTOR_POLE_PAIRS;   // π/2 / 21 ≈ 0.0748 rad

    char buf[192];
    sprintf(buf,
            "\r\n[ALIGN] done: delta=%+.4f rad (%+.2f deg), expect=%.4f rad, dir=%+d, offset=%.4f rad (%.2f deg), nsamp=%u\r\n",
            dlt, dlt * 57.2957795f, expect,
            dir,
            off, off * 57.2957795f,
            (unsigned)align_report_samples);
    UART_SendString(buf);

    // 自检：转角明显偏小 -> 励磁不足，转子基本没动，本次结果不可信
    if (fabsf(dlt) < 0.5f * expect) {
        UART_SendString("[ALIGN] WARN: delta too small, rotor likely did NOT move."
                        " Increase ALIGN_VOLTAGE (0.6V is a good starting point;"
                        " higher is NOT always better, >3V trips stall protection)\r\n");
    }
    // 自检：转角明显偏大 -> 转子滑过一极或被外力带动，编码器读数存疑
    else if (fabsf(dlt) > 1.5f * expect) {
        UART_SendString("[ALIGN] WARN: delta too large, rotor slipped or was disturbed"
                        " (check load / encoder mount)\r\n");
    }		    
}

// 上电参数校验打印
void UART_PrintSystemReport(void)
{
    static char buf[128];   // 静态区，避免占用主循环小栈（不可重入，仅主循环/启动序列调用）

    UART_SendString("\r\n===== SYSTEM PARAM CHECK =====\r\n");

    // ---- PID 四环（增益应非负，负值会改变环的极性）----
    sprintf(buf, "PID speed Kp=%+.4f Ki=%+.4f Kd=%+.4f\r\n",
            pid_speed.params.Kp, pid_speed.params.Ki, pid_speed.params.Kd);
    UART_SendString(buf);
    sprintf(buf, "PID id    Kp=%+.4f Ki=%+.4f Kd=%+.4f\r\n",
            pid_id.params.Kp, pid_id.params.Ki, pid_id.params.Kd);
    UART_SendString(buf);
    sprintf(buf, "PID iq    Kp=%+.4f Ki=%+.4f Kd=%+.4f\r\n",
            pid_iq.params.Kp, pid_iq.params.Ki, pid_iq.params.Kd);
    UART_SendString(buf);
    sprintf(buf, "PID pos   Kp=%+.4f Ki=%+.4f Kd=%+.4f\r\n",
            pid_position.params.Kp, pid_position.params.Ki, pid_position.params.Kd);
    UART_SendString(buf);

    uint8_t pid_ok = (pid_speed.params.Kp    >= 0.0f && pid_speed.params.Ki    >= 0.0f && pid_speed.params.Kd    >= 0.0f &&
                     pid_id.params.Kp       >= 0.0f && pid_id.params.Ki       >= 0.0f && pid_id.params.Kd       >= 0.0f &&
                     pid_iq.params.Kp       >= 0.0f && pid_iq.params.Ki       >= 0.0f && pid_iq.params.Kd       >= 0.0f &&
                     pid_position.params.Kp >= 0.0f && pid_position.params.Ki >= 0.0f && pid_position.params.Kd >= 0.0f) ? 1u : 0u;
    sprintf(buf, "  -> PID gains          : %s\r\n", pid_ok ? "PASS" : "WARN (negative gain)");
    UART_SendString(buf);

    // ---- 位置软限位（输出端，rad；须 min < max）----
    uint8_t lim_ok = (soft_limit_min < soft_limit_max) ? 1u : 0u;
    sprintf(buf, "  -> SoftLimit [%.3f, %.3f] rad : %s\r\n",
            soft_limit_min, soft_limit_max, lim_ok ? "PASS" : "WARN (min >= max)");
    UART_SendString(buf);

    // ---- 位置跟随误差阈值（rad；须 > 0）----
    uint8_t pos_ok = (pos_error_limit > 0.0f) ? 1u : 0u;
    sprintf(buf, "  -> PosErrLimit %.4f rad   : %s\r\n",
            pos_error_limit, pos_ok ? "PASS" : "WARN (> 0 required)");
    UART_SendString(buf);

    // ---- 转矩常数 Kt（Nm/A；须按实测设置，0.1 为代码中的占位值）----
    const char *kt_msg = (foc.torque_constant <= 0.0f) ? "WARN (> 0 required)"
                       : (foc.torque_constant == 0.1f) ? "WARN (placeholder, set actual Kt)"
                       : "PASS";
    sprintf(buf, "  -> TorqueConst %.4f Nm/A : %s\r\n", foc.torque_constant, kt_msg);
    UART_SendString(buf);

    // ---- 母线电压（对比 Protect.h 阈值区间）----
    uint8_t bus_ok = (bus_voltage > BUS_UNDERVOLTAGE_THRESHOLD &&
                      bus_voltage < BUS_OVERVOLTAGE_THRESHOLD) ? 1u : 0u;
    sprintf(buf, "  -> BusVoltage %.2f V (%.1f~%.1f) : %s\r\n",
            bus_voltage, BUS_UNDERVOLTAGE_THRESHOLD, BUS_OVERVOLTAGE_THRESHOLD,
            bus_ok ? "PASS" : "WARN (out of range)");
    UART_SendString(buf);

    // ---- 线圈温度（NTC；低于关断阈值）----
    uint8_t temp_ok = (ntc_temperature < OVERTEMP_SHUTDOWN) ? 1u : 0u;
    sprintf(buf, "  -> CoilTemp %.1f C (limit %.0f) : %s\r\n",
            ntc_temperature, OVERTEMP_SHUTDOWN, temp_ok ? "PASS" : "WARN (over temp)");
    UART_SendString(buf);

    // ---- 控制周期（TIM1 20kHz 更新中断 -> 期望 50µs）----
    float dt_us = control_dt * 1e6f;
    uint8_t dt_ok = (dt_us > 49.0f && dt_us < 51.0f) ? 1u : 0u;
    sprintf(buf, "  -> ControlDt %.2f us (expect 50) : %s\r\n",
            dt_us, dt_ok ? "PASS" : "WARN (check TIM1)");
    UART_SendString(buf);

    // ---- 关键编译期常量 ----
    sprintf(buf, "  -> PolePairs=%d  ClosedLoopEn=%d  NodeID=%d\r\n",
            MOTOR_POLE_PAIRS, FOC_CLOSEDLOOP_ENABLE, NODE_ID);
    UART_SendString(buf);

    UART_SendString("===== END PARAM CHECK =====\r\n");
}
