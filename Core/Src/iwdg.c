/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 125;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */

/**
  * @brief  运行时修改 IWDG 超时（分频器不变，只改重装值）
  * @param  reload 重装值：超时 ≈ reload × 256 / LSI(≈32kHz)，即 125 → ≈1.0s，500 → ≈4.0s
  * @retval 0 = 成功；1 = 失败（两次写入均未被接受，RLR 保持原值，调用方可继续）
  * @note   原因：上电启动序列含"自动对齐"约 1.95s，超过默认 1s 超时 —— 零余量，
  *         任何一段耗时抖动都会触发看门狗复位（复位后又重跑启动序列，表现为反复自动对齐）。
  *         故启动期放宽到 ≈4s，进入主循环前收紧回 ≈1s，兼顾"启动不误复位"与"运行时严格保护"。
  *         直接写 IWDG->KR，绕开 HAL 互斥锁（启动序列可在任意上下文调用）。
  */
uint8_t IWDG_SetReload(uint16_t reload)
{
    uint32_t primask;
    uint32_t guard;
    uint8_t  attempt;

    /* 最多尝试 2 次：第 1 次可能撞上 RVU=1（上一次更新尚未加载完成）导致写入被忽略，
     * 稍后重试通常即成功。 */
    for (attempt = 0u; attempt < 2u; attempt++) {
        /* 1) 写 RLR 要求 RVU=0，否则本次写入会被静默忽略。
         *    guard ≈ 2ms @72MHz，远超 LSI 域加载所需的数十 µs；仅作兜底，
         *    避免 RVU 异常时不至于死等（两次尝试合计仍远小于最短看门狗周期 1s）。 */
        guard = 20000u;
        while (((IWDG->SR & IWDG_SR_RVU) != 0u) && (guard != 0u)) {
            guard--;
        }

        /* 2) 临界区：解锁 + 写 RLR 必须原子。
         *    IWDG 的写保护会在"下一次写 KR"时重新启用，而中断/启动序列里的喂狗正是写
         *    KR=0xAAAA；若被插入，随后的 RLR 写入会被静默忽略。临界区仅两条语句（≈0.2µs），
         *    对 20kHz 控制中断无影响。 */
        primask = __get_PRIMASK();
        __disable_irq();
        IWDG->KR  = 0x00005555u;                 /* 解除 PR/RLR/WINR 写保护 */
        IWDG->RLR = (uint32_t)reload;            /* 写入新的重装值 */
        __set_PRIMASK(primask);

        /* 3) 重装计数器：新值即刻加载，同时完成一次喂狗 */
        IWDG->KR = 0x0000AAAAu;

        /* 4) 判据用"读回 RLR"——直接确认写入是否被接受。
         *    不能用 RVU 是否落 0 作判据：该标志的清除与 LSI 域的加载事件相关，
         *    可能拖到一个计数周期之后，实测会造成约 10% 的误报（并非真的写入失败）。 */
        if (IWDG->RLR == (uint32_t)reload) {
            return 0u;                           /* 写入成功 */
        }
    }

    /* 两次都没写进去：补喂一次狗，保证调用方仍有一段完整超时可用 */
    IWDG->KR = 0x0000AAAAu;
    return 1u;                                   /* 调用方据此上报并保留原超时 */
}

/* USER CODE END 1 */

