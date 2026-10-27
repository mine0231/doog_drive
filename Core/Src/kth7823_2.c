#include "kth7823_2.h"
#include "main.h"        // ME2_CS_Pin / ME2_CS_GPIO_Port
#include "arm_math.h"
#include "spi.h"
#include "dwt_delay.h"   // DWT_DelayMs / DWT->CYCCNT
#include "motor_control.h"  // control_dt
#include <stddef.h>         // offsetof（零点偏置 Flash 存储校验）


// 全局变量
float kth7823_2_Angle_Deg        = 0.0f;        // 当前角度（度）
volatile float kth7823_2_Angle_Rad = 0.0f;      // 单圈角度 [0, 2π)（慢路径，滤波后）
volatile float kth7823_2_Angle_Multiturn  = 0.0f;         // 多圈总角度（连续无界，弧度，未滤波）
volatile float kth7823_2_Angle_Multiturn_Comp = 0.0f;     // 一拍延迟补偿后的多圈角度（位置环角度源）
volatile float kth7823_2_Speed_Rads = 0.0f;     // 速度 (rad/s)
float kth7823_2_Speed_RPM        = 0.0f;        // 速度 (RPM)
volatile uint32_t kth7823_2_update_count = 0;   // 刷新心跳计数器：快路径 1kHz 每拍 +1
volatile uint16_t kth7823_2_raw_fault_cnt = 0;  // 连续异常读数（0x0000/0xFFFF）计数：正常 raw 清零
volatile uint16_t kth7823_2_stale_ticks = 0;    // 距离最近一次接受有效角度样本的控制周期数
volatile uint16_t kth7823_2_unwrap_glitch_cnt = 0;  // 被丢弃的解包裹野值累计次数（只增不清零）
volatile uint16_t kth7823_2_spi_err_cnt = 0;    // SPI 帧失败（超时/重入被拒）累计次数
volatile uint32_t kth7823_2_diag_cr1 = 0;       // 失败帧的现场快照（诊断用）
volatile uint32_t kth7823_2_diag_sr  = 0;

// 输出端机械零点偏置 (rad)：默认 0（未标定时 = 上电位置即零点，相对控制）
float kth7823_2_zero_offset = 0.0f;

// 内部变量
static float filtered_cos = 1.0f;              // 滤波变量（慢路径 1kHz）
static float filtered_sin = 0.0f;
static uint8_t filter_initialized = 0;
static volatile uint16_t last_raw_angle = 0;   // 解包裹状态（快路径 20kHz）中断写、主循环读（16 位原子）
static uint8_t  raw_initialized = 0;
static float    angle_unwrapped = 0.0f;        // 连续单圈角度（解包裹后）
static float    angle_prev_valid = 0.0f;       // 上一有效帧的多圈角（供一拍差分算 ω）
static uint32_t last_valid_cyc = 0;            // 上一有效帧的 DWT 时间戳
static float    last_unwrapped_angle = 0.0f;   // 测速状态（慢路径 1kHz）
static uint32_t last_speed_tick = 0;
static volatile uint8_t spi_in_frame = 0;      // 帧重入保护：SPI3 只允许一个调用者

// 角度弧度转换
static float raw_to_rad(uint16_t raw)
{
    return (float)raw / kth7823_2_ANGLE_RESOLUTION * 2.0f * PI;  // 65536 个刻度覆盖 2π
}

/* SPI 收发一帧 16bit（16 位数据帧，单字）
 * CS 规则：每帧独立"拉低 → 传输 → 拉高"，任何返回路径都先恢复高电平。
 * 不用 HAL_SPI_TransmitReceive：它在中断里用 HAL_GetTick() 计超时，而 SysTick 优先级
 * 15 无法抢占 TIM1（优先级 1），一旦卡住就永不返回。这里改为寄存器级 + DWT 限时。
 * 返回 1 = 成功；0 = 超时/重入被拒（调用方按"本拍无新角度"处理）。 */
static uint8_t spi_exchange_frame(uint16_t tx, uint16_t *rx)
{
    SPI_TypeDef *spi = kth7823_2_SPI->Instance;
    uint32_t t0    = DWT->CYCCNT;
    uint32_t limit = SystemCoreClock / (1000000u / kth7823_2_SPI_TIMEOUT_US);  // µs → cycles
    uint16_t rxw   = 0;
    uint8_t  ok    = 1;

    if (spi_in_frame != 0u) {          // 重入（不应发生）：丢弃，避免并发拉低 CS
        return 0;
    }
    spi_in_frame = 1;

    spi->CR1 |= SPI_CR1_SPE;                            // 确保外设使能（幂等）
    HAL_GPIO_WritePin(ME2_CS_GPIO_Port, ME2_CS_Pin, GPIO_PIN_RESET);

    while ((spi->SR & SPI_SR_TXE) == 0u) {              // 等 TX FIFO 有空位
        if ((DWT->CYCCNT - t0) > limit) { ok = 0; break; }
    }
    if (ok) {
        // 必须用 32 位访问：DS=16bit 时 DR 仍是 32 位寄存器（HAL 也是整字写），
        // 半字写不会触发传输，会表现为 RXNE 永不置位 → 每帧白等一个超时。
        spi->DR = (uint32_t)tx;                         // 16 位帧，硬件 MSB 先发
        while ((spi->SR & SPI_SR_RXNE) == 0u) {         // 等整帧收完
            if ((DWT->CYCCNT - t0) > limit) { ok = 0; break; }
        }
    }
    if (ok) {
        rxw = (uint16_t)spi->DR;
        uint32_t t1 = DWT->CYCCNT;
        uint32_t bsy_limit = SystemCoreClock / 1000000u;   // 1µs：等最后一位移出，非致命
        while ((spi->SR & SPI_SR_BSY) != 0u) {
            if ((DWT->CYCCNT - t1) > bsy_limit) break;
        }
    }

    HAL_GPIO_WritePin(ME2_CS_GPIO_Port, ME2_CS_Pin, GPIO_PIN_SET);   // 任何分支都先拉高

    if (!ok) {   // 恢复：先读 DR/SR 清 OVR，再关-开 SPE 复位 FIFO，避免坏帧卡死后续所有帧
        kth7823_2_diag_cr1 = spi->CR1;
        kth7823_2_diag_sr  = spi->SR;
        (void)spi->DR;
        (void)spi->SR;
        spi->CR1 &= ~SPI_CR1_SPE;
        spi->CR1 |= SPI_CR1_SPE;
    }

    spi_in_frame = 0;

    if (!ok) {
        return 0;
    }
    *rx = rxw;
    return 1;
}

/* 20kHz 快路径：读 1 帧 → 无效判定 → 解包裹 → 更新多圈角与延迟补偿角。
 * 不做 sin/cos/atan2（那是慢路径的事）。 */
static void kth7823_2_sample_fast(void)
{
    uint16_t raw;

    if (kth7823_2_stale_ticks < 0xFFFFu) kth7823_2_stale_ticks++;

    if (!spi_exchange_frame(0x0000u, &raw)) {
        if (kth7823_2_spi_err_cnt < 0xFFFFu) kth7823_2_spi_err_cnt++;
        return;                                        // 本拍无新角度，保留上次值
    }

    // 异常读数统计：0x0000/0xFFFF 视为断线/无磁等无效读数，连续累加；正常读数清零
    if (raw == 0x0000u || raw == 0xFFFFu) {
        if (kth7823_2_raw_fault_cnt < 0xFFFFu) kth7823_2_raw_fault_cnt++;
        return;                                        // 无效值不参与解包裹
    }
    kth7823_2_raw_fault_cnt = 0;

    if (!raw_initialized) {                            // 首帧建基准
        last_raw_angle  = raw;
        angle_unwrapped = raw_to_rad(raw);
        angle_prev_valid = angle_unwrapped;
        last_valid_cyc   = DWT->CYCCNT;
        raw_initialized  = 1;
        kth7823_2_stale_ticks = 0;
        kth7823_2_Angle_Multiturn      = angle_unwrapped;
        kth7823_2_Angle_Multiturn_Comp = angle_unwrapped;
        return;
    }

    // 多圈解包裹：用 16 位带符号差值累积
    int16_t delta_raw = (int16_t)(raw - last_raw_angle);   // 自动处理跨 0xFFFF 边界
    // 野值剔除：单帧跳变超物理可能上限，判为坏读数/解包裹野值，丢弃本次——
    // 不更新 last_raw_angle、不累积角度，等下一次好读数再接续，避免多圈角阶跃与测速尖峰
    if (delta_raw > kth7823_2_UNWRAP_MAX_DELTA || delta_raw < -kth7823_2_UNWRAP_MAX_DELTA) {
        if (kth7823_2_unwrap_glitch_cnt < 0xFFFFu) kth7823_2_unwrap_glitch_cnt++;
        return;
    }

    last_raw_angle = raw;
    angle_unwrapped += (float)delta_raw / kth7823_2_ANGLE_RESOLUTION * 2.0f * PI;

    // 一拍延迟补偿：用相邻两个"有效帧"的 DWT 间隔算角速度（不做低通）。
    uint32_t now = DWT->CYCCNT;
    float dt = (float)(now - last_valid_cyc) / (float)SystemCoreClock;
    if (dt > 0.0f) {
        float omega = (angle_unwrapped - angle_prev_valid) / dt;
        // 限幅：坏帧/抖动时避免补偿项飞掉（±400 rad/s 远高于 230 rad/s 超速阈值）
        if (omega >  400.0f) omega =  400.0f;
        if (omega < -400.0f) omega = -400.0f;
        kth7823_2_Angle_Multiturn_Comp = angle_unwrapped
                                     + omega * (kth7823_2_ANGLE_DELAY_PERIODS * control_dt);
    } else {
        kth7823_2_Angle_Multiturn_Comp = angle_unwrapped;
    }
    angle_prev_valid = angle_unwrapped;
    last_valid_cyc   = now;

    kth7823_2_Angle_Multiturn = angle_unwrapped;   // 未补偿值（位置环/上报用）
    kth7823_2_stale_ticks = 0;
}

/* 1kHz 慢路径：用最近一次有效 raw 做 sin/cos 低通 + atan2（不读 SPI）。
 * 只读中断缓存的 raw（16 位原子），不碰 SPI，因此不会与中断抢总线。 */
static void kth7823_2_update_filtered_angle(void)
{
    if (!raw_initialized) {
        return;                                      // 还没有有效读数
    }

    float raw_rad = raw_to_rad(last_raw_angle);

    if (!filter_initialized) {
        filtered_cos = cosf(raw_rad);
        filtered_sin = sinf(raw_rad);
        filter_initialized = 1;
    } else {
        filtered_cos += kth7823_2_FILTER_ALPHA * (cosf(raw_rad) - filtered_cos);
        filtered_sin += kth7823_2_FILTER_ALPHA * (sinf(raw_rad) - filtered_sin);

        float norm = sqrtf(filtered_cos * filtered_cos + filtered_sin * filtered_sin);
        if (norm > 1e-6f) {
            filtered_cos /= norm;
            filtered_sin /= norm;
        }
    }

    float filtered_angle = atan2f(filtered_sin, filtered_cos);
    if (filtered_angle < 0.0f) filtered_angle += 2.0f * PI;

    kth7823_2_Angle_Rad = filtered_angle;
    kth7823_2_Angle_Deg = filtered_angle * 180.0f / PI;
}

/* 1kHz 慢路径：DWT 差分测速（窗口 ≈1ms，避免 20kHz 逐拍差分带来的量化噪声）。
 * 放在快路径每 20 拍调用一次，不进主循环，保证阻塞命令期间速度仍持续更新。 */
static void kth7823_2_update_speed(void)
{
    uint32_t now = DWT->CYCCNT;

    if (last_speed_tick == 0) {   // 首帧建基准
        last_speed_tick = now;
        last_unwrapped_angle = angle_unwrapped;
        kth7823_2_Speed_Rads = 0.0f;
        kth7823_2_Speed_RPM  = 0.0f;
        return;
    }

    uint32_t elapsed = now - last_speed_tick;
    if (elapsed == 0) return;                                  // 同周期重复调用，跳过

    float dt = (float)elapsed / (float)SystemCoreClock;        // 秒

    float delta_angle = angle_unwrapped - last_unwrapped_angle;
    if (delta_angle >  PI) delta_angle -= 2.0f * PI;
    if (delta_angle < -PI) delta_angle += 2.0f * PI;

    float speed_raw = delta_angle / dt;
    kth7823_2_Speed_Rads += kth7823_2_SPEED_FILTER_ALPHA * (speed_raw - kth7823_2_Speed_Rads);
    kth7823_2_Speed_RPM = kth7823_2_Speed_Rads * 60.0f / (2.0f * PI);

    last_unwrapped_angle = angle_unwrapped;
    last_speed_tick = now;
}

// 输出端编码器初始化
void kth7823_2_Init(void)
{
    kth7823_2_Angle_Rad = 0.0f;
    kth7823_2_Angle_Deg = 0.0f;
    kth7823_2_Angle_Multiturn = 0.0f;
    kth7823_2_Angle_Multiturn_Comp = 0.0f;
    kth7823_2_Speed_Rads = 0.0f;
    kth7823_2_Speed_RPM = 0.0f;

    filtered_cos = 1.0f;
    filtered_sin = 0.0f;
    filter_initialized = 0;
    raw_initialized = 0;
    angle_unwrapped = 0.0f;
    angle_prev_valid = 0.0f;
    last_valid_cyc = 0;
    last_unwrapped_angle = 0.0f;
    last_speed_tick = 0;
    spi_in_frame = 0;

    kth7823_2_raw_fault_cnt = 0;
    kth7823_2_stale_ticks = 0;
    kth7823_2_unwrap_glitch_cnt = 0;
    kth7823_2_spi_err_cnt = 0;
    kth7823_2_update_count = 0;

    // 建立流水线首拍：单帧协议下这一帧既完成上电通信初始化，也让下一帧能取到有效回读。
    // 其返回值是上电旧值/无效值，直接丢弃（首个 Tick 取到的才是该命令对应的角度）。
    uint16_t dummy;
    (void)spi_exchange_frame(0x0000u, &dummy);
    DWT_DelayMs(10);
}

/* 编码器刷新入口：唯一由 TIM1 20kHz 更新中断调用（SPI3 单一所有者）。
 *   - 每拍：快路径 raw + 解包裹（位置环角度源）
 *   - 每 20 拍（1kHz）：测速 + 心跳计数（便宜，且阻塞命令期间也必须保持更新）
 * 滤波角度（cos/sin/atan2）不在这里做，放到主循环的 kth7823_2_SlowUpdate() 里按 1kHz 跑。 */
void kth7823_2_Tick(uint32_t tick)
{
    kth7823_2_sample_fast();

    if ((tick % kth7823_2_SLOW_DIV) == 0u) {
        kth7823_2_update_speed();
        kth7823_2_update_count++;      // 心跳按 1kHz 递增，语义与电机轴对称
    }
}

/* 慢路径：滤波角度（sin/cos 低通 + atan2），由主循环按 ≈1kHz 调用。
 * 只读中断缓存的 raw（16 位原子），不碰 SPI。 */
void kth7823_2_SlowUpdate(void)
{
    kth7823_2_update_filtered_angle();
}

/* ==================== 输出端机械零点偏置的 Flash 存储 ====================
 * 不新建文件：直接放在本模块（kth7823_2 拥有输出编码器角度，机械零点偏置是其固有属性）。
 * 使用最后 2KB Flash 页 0x0801F800（128KB 器件，页大小 0x800），存 {magic, offset, checksum, reserved}。
 * 注意：
 *   1) 擦除会让 CPU 停摆约 20~40ms，必须在停驱动状态下调用（见 uart_cmd.c 的 zero 命令）；
 *   2) 写入前需确认链接器未把代码放到该页（stm32g474xx_flash.sct 已把 ER_IROM1 缩到 0x1F800）。 */
#define KTH7823_2_ZERO_ADDR   0x0801F800u
#define KTH7823_2_ZERO_MAGIC  0x5A5A0FF5u

typedef struct {
    uint32_t magic;
    float    zero_offset;
    uint32_t checksum;
    uint32_t reserved;                 // 补齐到 16 字节（G4 Flash 按 64bit 双字编程）
} ZeroStore_t;

static uint32_t zero_store_checksum(const uint8_t *d, uint32_t n)
{
    uint32_t c = 0x811C9DC5u;          // FNV-1a
    for (uint32_t i = 0; i < n; i++) { c ^= d[i]; c *= 16777619u; }
    return c;
}

uint8_t kth7823_2_LoadZero(void)
{
    const ZeroStore_t *p = (const ZeroStore_t *)KTH7823_2_ZERO_ADDR;
    if (p->magic != KTH7823_2_ZERO_MAGIC) return 0;
    if (zero_store_checksum((const uint8_t *)p, offsetof(ZeroStore_t, checksum)) != p->checksum) return 0;
    kth7823_2_zero_offset = p->zero_offset;
    return 1;
}

void kth7823_2_SetZero(void)
{
    kth7823_2_zero_offset = kth7823_2_Angle_Multiturn;   // 当前输出角即机械零点

    ZeroStore_t d;
    d.magic       = KTH7823_2_ZERO_MAGIC;
    d.zero_offset = kth7823_2_zero_offset;
    d.reserved    = 0u;
    d.checksum    = zero_store_checksum((const uint8_t *)&d, offsetof(ZeroStore_t, checksum));

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef e = {0};
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Page      = (KTH7823_2_ZERO_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
    e.NbPages   = 1;
    uint32_t err = 0;
    HAL_FLASHEx_Erase(&e, &err);

    // 按 64bit 双字写：从 4 字节对齐的 struct 按 32bit 读、再合成 64bit 值，
    // 避免把 &d 强转成 uint64_t* 触发的 8 字节对齐问题。
    const uint32_t *w = (const uint32_t *)&d;
    for (uint32_t i = 0; i < sizeof(ZeroStore_t) / 8u; i++) {
        uint64_t val = (uint64_t)w[i * 2u] | ((uint64_t)w[i * 2u + 1u] << 32u);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, KTH7823_2_ZERO_ADDR + i * 8u, val);
    }
    HAL_FLASH_Lock();
}
