/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hrtim_pwm.c
  * @brief   HRTIM1 PWM 驱动（寄存器级，不依赖 HAL_HRTIM）
  *
  * 配置说明（160MHz HRTIM 时钟）：
  *   PERIOD = 7999  -> 20 kHz
  *   CMP1   = duty * 8000          （高电平时间 = CMP1）
  *   OUT1(高边)：PER 置位、CMP1 复位   -> 周期开始导通
  *   OUT2(低边)：死区使能时由 OUT1 互补生成，源与 OUT1 相同
  *   死区：DTR = DTF = 32 -> 200 ns（1 count = 6.25 ns）
  ******************************************************************************
  */
/* USER CODE END Header */

#include "hrtim_pwm.h"

#define HRTIM_PWM_PERIOD   7999u    /* 160MHz / 20kHz - 1 */
#define HRTIM_PWM_UPDATE   (HRTIM_PWM_PERIOD + 1u)
/* 死区：DTR/DTF 只有 9 位（最大 511），超了会写进符号位变成负死区！
 * 时基 = 6.25ns × 2^PSC：PSC=0 -> 6.25ns，PSC=1 -> 12.5ns，PSC=2 -> 25ns
 * 这里：PSC=0、计数 32 -> 32 × 6.25ns = 200ns
 */
#define HRTIM_PWM_DT_PSC    0u
#define HRTIM_PWM_DT_VAL    32u

/* 每个定时器对应的输出使能位（OUT1 + OUT2） */
static const uint32_t hrtim_oen_bits[6] =
{
  HRTIM_OENR_TA1OEN | HRTIM_OENR_TA2OEN,
  HRTIM_OENR_TB1OEN | HRTIM_OENR_TB2OEN,
  HRTIM_OENR_TC1OEN | HRTIM_OENR_TC2OEN,
  HRTIM_OENR_TD1OEN | HRTIM_OENR_TD2OEN,
  HRTIM_OENR_TE1OEN | HRTIM_OENR_TE2OEN,
  HRTIM_OENR_TF1OEN | HRTIM_OENR_TF2OEN
};

/* 每个定时器对应的计数器使能位 */
static const uint32_t hrtim_cen_bits[6] =
{
  HRTIM_MCR_TACEN, HRTIM_MCR_TBCEN, HRTIM_MCR_TCCEN,
  HRTIM_MCR_TDCEN, HRTIM_MCR_TECEN, HRTIM_MCR_TFCEN
};

/**
  * @brief  初始化 HRTIM1 六个定时器的时基/比较/死区/输出极性
  * @note   只配置，不启动计数、不打开输出
  */
uint32_t hrtim_pwm_init(void)
{
  uint32_t i;

  /* 1. 时钟 */
  __HAL_RCC_HRTIM1_CLK_ENABLE();

  /* 1.1 时钟预分频：复位值 CK_PSC=000 是 fHRTIM × 32！
   *     必须显式设为 DIV1(0x5)，否则计数器跑在 5.12GHz，PWM 变 640kHz、死区变 6ns
   */
  HRTIM1->sMasterRegs.MCR = (HRTIM1->sMasterRegs.MCR & ~HRTIM_MCR_CK_PSC) |
                            (0x5u << HRTIM_MCR_CK_PSC_Pos);

  /* 2. DLL 校准（ST 例程要求：不校准，输出/死区不会正常翻转） */
  HRTIM1->sCommonRegs.DLLCR = HRTIM_DLLCR_CALEN |
                              HRTIM_DLLCR_CALRTE_0 | HRTIM_DLLCR_CALRTE_1;  /* 连续校准, 速率 3 */
  HRTIM1->sCommonRegs.DLLCR |= HRTIM_DLLCR_CAL;

  {
    uint32_t t0 = HAL_GetTick();

    while (((HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY) == 0u) &&
           ((HAL_GetTick() - t0) < 10u))
    {
    }
    if ((HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY) == 0u)
    {
      return 1u;   /* DLL 没锁上：调用者负责报错 */
    }
  }

  /* 3. 先关闭全部 12 路输出 */
  HRTIM1->sCommonRegs.ODISR = 0x0FFFu;

  /* 4. 关闭连续模式与所有计数器 */
  HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_CONT |
                               HRTIM_MCR_TACEN | HRTIM_MCR_TBCEN |
                               HRTIM_MCR_TCCEN | HRTIM_MCR_TDCEN |
                               HRTIM_MCR_TECEN | HRTIM_MCR_TFCEN);

  /* 4. 清 ADC 触发（阶段2再配） */
  HRTIM1->sCommonRegs.ADC1R = 0;
  HRTIM1->sCommonRegs.ADC2R = 0;

  /* 5. 逐一定时器配置（TA ~ TF 完全一致） */
  for (i = 0; i < 6u; i++)
  {
    HRTIM_Timerx_TypeDef *t = &HRTIM1->sTimerxRegs[i];

    /* 从模式控制寄存器：
     *   bit[3] CONT = 1   连续计数
     *   bit[2:0] CK_PSC = 5  = fHRTIM/1（复位值 0 是 ×32！）
     */
    t->TIMxCR = HRTIM_TIMCR_CONT | (0x5u << HRTIM_TIMCR_CK_PSC_Pos);
    /* Re-align phase counters to the ADC master on every period. */
    t->RSTxR = HRTIM_RSTR_MSTPER;
    t->PERxR  = HRTIM_PWM_PERIOD;
    t->REPxR  = 0;
    t->CMP1xR = HRTIM_PWM_PERIOD / 2u;  /* 初始 50% */
    t->CMP2xR = 0;
    t->CMP3xR = 0;
    t->CMP4xR = 0;

    /* 死区：上升/下降各 4µs（PSC=1 时基 12.5ns，计数 320） */
    t->DTxR = (HRTIM_PWM_DT_VAL << 0) |
              (HRTIM_PWM_DT_PSC << 10) |
              (HRTIM_PWM_DT_VAL << 16);

    /* 输出置位/复位源
     * 注意：DTEN=1 时 OUT1/OUT2 不再独立，死区单元用 OUT1 的这组事件
     *       自动生成互补的 OUT2（与 ST 的 HRTIM_Buck_Sync_Rect 例程一致）。
     *       所以 OUT2 也写同一组源，不要写镜像源。
     *
     * 采用“下管先导通”：周期起点 OUT1=低（高边关、低边开），
     * 到 CMP1 才翻成高边导通 —— 上电时下管先通，给自举电容充电。
     */
    t->SETx1R = HRTIM_SET1R_CMP1;       /* 高边：CMP1 置位 */
    t->RSTx1R = HRTIM_RST1R_PER;        /*       周期复位（周期起点为低） */
    t->SETx2R = HRTIM_SET2R_CMP1;       /* 低边：同一组源，由死区单元取反 */
    t->RSTx2R = HRTIM_RST2R_PER;

    /* 使能死区，输出极性高、空闲低电平 */
    t->OUTxR = HRTIM_OUTR_DTEN;
  }

  /* 6. 软件更新：把 PER/CMP 写入活动寄存器（ST 例程里也有这一步） */
  HRTIM1->sCommonRegs.CR2 = HRTIM_CR2_TASWU | HRTIM_CR2_TBSWU | HRTIM_CR2_TCSWU |
                            HRTIM_CR2_TDSWU | HRTIM_CR2_TESWU | HRTIM_CR2_TFSWU;

  return 0u;
}

/**
  * @brief  打开指定定时器的两路输出并启动其计数器
  * @param  timers_mask 位 i = 1 表示启动 HRTIM_IDX_xx（0=A ... 5=F）
  */
void hrtim_pwm_outputs_enable(uint32_t timers_mask)
{
  uint32_t i;
  uint32_t oen = 0u;
  uint32_t cen = 0u;

  for (i = 0; i < 6u; i++)
  {
    if (timers_mask & (1u << i))
    {
      oen |= hrtim_oen_bits[i];
      cen |= hrtim_cen_bits[i];
    }
  }

  /* Called at the END of an injected sequence, with all outputs disabled.
   * Reset/start master and phases together so the first ADC trigger also
   * falls in the low-side conduction window, not a retained counter phase. */
  HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
  HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_MCEN | HRTIM_MCR_TACEN |
      HRTIM_MCR_TBCEN | HRTIM_MCR_TCCEN | HRTIM_MCR_TDCEN |
      HRTIM_MCR_TECEN | HRTIM_MCR_TFCEN);
  HRTIM1->sCommonRegs.CR2 = HRTIM_CR2_MRST | HRTIM_CR2_TARST |
      HRTIM_CR2_TBRST | HRTIM_CR2_TCRST | HRTIM_CR2_TDRST |
      HRTIM_CR2_TERST | HRTIM_CR2_TFRST;
  HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_CONT | HRTIM_MCR_MCEN | cen;

  /* 打开输出 */
  HRTIM1->sCommonRegs.OENR = oen;
}

/**
  * @brief  关闭全部输出并停止相定时器，保留 Master 连续采样时钟
  */
void hrtim_pwm_outputs_disable(void)
{
  HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
  HRTIM1->sMasterRegs.MCR &= ~(
                               HRTIM_MCR_TACEN | HRTIM_MCR_TBCEN |
                               HRTIM_MCR_TCCEN | HRTIM_MCR_TDCEN |
                               HRTIM_MCR_TECEN | HRTIM_MCR_TFCEN);
}

/**
  * @brief  只打开某个定时器的指定输出（调试防直通用）
  * @param  timer_idx HRTIM_IDX_xx
  * @param  which     1 = 只开 OUT1(高边)，2 = 只开 OUT2(低边)，3 = 两个都开
  */
void hrtim_pwm_output_enable_one(uint32_t timer_idx, uint32_t which)
{
  uint32_t oen = 0u;

  if (timer_idx > 5u)
  {
    return;
  }

  if ((which & 1u) != 0u)
  {
    oen |= (0x1u << (2u * timer_idx));      /* OUT1 使能位 */
  }
  if ((which & 2u) != 0u)
  {
    oen |= (0x2u << (2u * timer_idx));      /* OUT2 使能位 */
  }

  HRTIM1->sMasterRegs.MCR |= (HRTIM_MCR_CONT | hrtim_cen_bits[timer_idx]);
  HRTIM1->sCommonRegs.OENR = oen;
}

/**
  * @brief  配置 ADC 触发：Master CMP4 -> ADC Trigger 2（ADC1/2 注入）
  * @param  cmp4 周期起点之后多少个 HRTIM 计数触发（160MHz 下 1 计数 = 6.25ns）
  * @note   Master 计数器独立启动，不需要使能功率输出
  */
void hrtim_pwm_adc_trigger_init(uint32_t cmp4)
{
  HRTIM1->sMasterRegs.MPER   = HRTIM_PWM_PERIOD;
  HRTIM1->sMasterRegs.MREP   = 0;
  HRTIM1->sMasterRegs.MCMP4R = cmp4;
  HRTIM1->sCommonRegs.ADC2R  = HRTIM_ADC2R_AD2MC4;   /* ADC 触发2 = Master CMP4 */
  HRTIM1->sMasterRegs.MCR   |= (HRTIM_MCR_CONT | HRTIM_MCR_MCEN);
}

/* DWT 周期计数器（微秒延时用） */
#define BEEP_DEMCR   (*(volatile uint32_t *)0xE000EDFCu)
#define BEEP_DWTCTRL (*(volatile uint32_t *)0xE0001000u)
#define BEEP_CYCCNT  (*(volatile uint32_t *)0xE0001004u)

static void beep_delay_us(uint32_t us)
{
  uint32_t start = BEEP_CYCCNT;
  uint32_t ticks = us * 160u;          /* 160MHz：1µs = 160 周期 */
  while ((BEEP_CYCCNT - start) < ticks)
  {
  }
}

/* 绕过 5% 限幅，直接写比较值（自检要更低的占空比） */
static void beep_set_duty_raw(uint32_t idx, float duty)
{
  uint32_t cmp = (uint32_t)((1.0f - duty) * (float)HRTIM_PWM_UPDATE);

  if (cmp > HRTIM_PWM_PERIOD)
  {
    cmp = HRTIM_PWM_PERIOD;
  }
  HRTIM1->sTimerxRegs[idx].CMP1xR = cmp;
}

/**
  * @brief  上电三声自检：依次给 A-B、B-C、C-A 通电，电机会“嘀”三声
  * @note   低占空比 + 短脉冲：绕组电流来不及冲起来，避免顶限流
  *         必须在 hrtim_pwm_init() 之后、正常输出使能之前调用
  */
void hrtim_selfcheck_beep(void)
{
  /* 定时器对：{高边, 低边} */
  static const uint32_t pairs[3][2] =
  {
    { HRTIM_IDX_A, HRTIM_IDX_B },
    { HRTIM_IDX_B, HRTIM_IDX_C },
    { HRTIM_IDX_C, HRTIM_IDX_A },
  };
  uint32_t k;

  BEEP_DEMCR   |= (1u << 24);   /* TRCENA */
  BEEP_DWTCTRL |= 1u;           /* CYCCNTENA */

  /* 先确保所有输出关闭 */
  HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
  HAL_Delay(50);

  for (k = 0u; k < 3u; k++)
  {
    uint32_t hi = pairs[k][0];
    uint32_t lo = pairs[k][1];
    uint32_t oen = (0x1u << (2u * hi)) | (0x2u << (2u * lo));  /* 高边 OUT1 + 低边 OUT2 */
    uint32_t i;

    /* 两相都只开 10%：6V 母线时平均约 0.6V，声音明显、电流可控 */
    beep_set_duty_raw(hi, 0.10f);   /* OUT1 高 10% */
    beep_set_duty_raw(lo, 0.90f);   /* OUT2 高 10%（互补） */

    /* 启动三相计数 */
    HRTIM1->sMasterRegs.MCR |= (HRTIM_MCR_CONT |
                                HRTIM_MCR_TACEN | HRTIM_MCR_TBCEN | HRTIM_MCR_TCCEN);
    HAL_Delay(2);

    /* 约 1kHz、每声 300ms：500µs 通 + 500µs 断（能量更大，更容易听到） */
    for (i = 0u; i < 300u; i++)
    {
      HRTIM1->sCommonRegs.OENR  = oen;
      beep_delay_us(500u);
      HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
      beep_delay_us(500u);
    }

    /* 这一对结束：全关，留间隔 */
    HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
    HAL_Delay(150);
  }
}

/**
  * @brief  设置某一路的占空比
  * @param  timer_idx HRTIM_IDX_xx
  * @param  duty      0.0 ~ 1.0（内部限幅到 5% ~ 95%）
  */
void hrtim_set_duty(uint32_t timer_idx, float duty)
{
  uint32_t cmp;

  if (timer_idx > 5u)
  {
    return;
  }

  if (duty < 0.05f)
  {
    duty = 0.05f;
  }
  if (duty > 0.95f)
  {
    duty = 0.95f;
  }

  /* 高边占空比 = (PERIOD - CMP1) / PERIOD，所以 CMP1 = (1 - duty) × 8000 */
  cmp = (uint32_t)((1.0f - duty) * (float)HRTIM_PWM_UPDATE);
  if (cmp > HRTIM_PWM_PERIOD)
  {
    cmp = HRTIM_PWM_PERIOD;
  }

  HRTIM1->sTimerxRegs[timer_idx].CMP1xR = cmp;
}
