# STM32G474VET6（LQFP100）引脚分配表 v2.0

**核对依据**：DS12288 Rev 4《STM32G474xB/xC/xE》Table 12 pin definition（p.60–73）LQFP100 列，逐脚核对。
**「5V容忍」列**：是 = FT 系列（可承受 5V）；否 = TT 系列（只能 3.3V，超过 VDDA+0.3V 会经模拟开关倒灌）。

---

## 1. 电源 / 时钟 / 复位 / 调试

| LQFP100 | 引脚 | 分配 | 复用功能 | 5V容忍 | 备注 |
|---:|---|---|---|:--:|---|
| 6 | VBAT | 备份电源 | — | — | 不用 RTC 备份电池时直接接 3.3V，就近 100nF |
| 12 | PF0-OSC_IN | HSE 晶振输入 | OSC_IN / ADC1_IN10 | 是 | 8 MHz（手册 4–48 MHz 区间），负载电容按晶体规格 |
| 13 | PF1-OSC_OUT | HSE 晶振输出 | OSC_OUT / ADC2_IN10 | 是 | |
| 14 | **PG10-NRST** | 硬件复位 | NRST | 是 | **100nF + 10kΩ 上拉到 3.3V**（内部另有约 40kΩ 弱上拉） |
| 23 | VSS | 数字地 | — | — | 与 24 脚配 100nF |
| 24 | VDD | 3.3V | — | — | |
| 35 | VSSA | 模拟地 | — | — | 与 VSS 单点连接 |
| 36 | VREF+ | ADC 参考 | VREFBUF_OUT | — | 100nF + 1µF 到 VSSA；不用 VREFBUF 时接 VDDA |
| 37 | VDDA | 模拟 3.3V | — | — | 磁珠 + 1µF + 100nF |
| 48 | VSS | 数字地 | — | — | 与 49 脚配 100nF |
| 49 | VDD | 3.3V | — | — | |
| 63 | VSS | 数字地 | — | — | 与 64 脚配 100nF |
| 64 | VDD | 3.3V | — | — | |
| 74 | VSS | 数字地 | — | — | 与 75 脚配 100nF |
| 75 | VDD | 3.3V | — | — | |
| 76 | PA13 | SWDIO | JTMS-SWDIO | 是 | 调试口，100k 上拉可选 |
| 77 | PA14 | SWCLK | JTCK-SWCLK | 是 | 调试口，内部下拉 |
| 95 | **PB8-BOOT0** | BOOT0 | — | 是 | **10kΩ 常驻下拉 + 3 脚跳线帽**（见 1.1）；复位后为普通 GPIO，不要挂负载 |
| 99 | VSS | 数字地 | — | — | 与 100 脚配 100nF |
| 100 | VDD | 3.3V | — | — | |

> **调试口必须配成 Serial Wire（SWD）**，不要用 JTAG——否则 PB3/PB4/PA15 会被 JTAG 占用，磁编1 的 SPI1 起不来。

### 1.1 BOOT0 跳线电路

```
            3V3
             │
   ┌─────────┴─────────┐
   │  1     2     3    │   J1 1x3 排针 2.54mm
   └──┬─────┬─────┬────┘
      │     │     │
     3V3   BOOT0  GND
             │
          [10kΩ]  ← 常驻下拉
             │
            GND
```

BOOT0 = PB8（95 脚）。三种状态：

| 跳线 | BOOT0 电平 | 启动位置 | 用途 |
|---|---|---|---|
| **2–3 短接** | 0 V | 用户 Flash | **正常运行**（平时就放这里） |
| 1–2 短接 | 3.3 V | 系统存储器（bootloader） | 串口/USB 救砖、量产烧录 |
| 不插 | 0 V（10k 下拉） | 用户 Flash | 安全默认，不会误进 bootloader |

- 跳线帽要**居中短接**，1×3 排针不存在跨接 1–3 的可能，无需额外保护；若想更保险，可在排针 2 脚与 BOOT0 之间串 1 kΩ。
- BOOT0 **只在复位/上电瞬间采样**：改完跳线必须**断电重启或按一次复位**才生效，运行中插拔不影响当前程序。
- 丝印建议：`BOOT` 旁边标 `1=3V3 2=BOOT0 3=GND`，或直接画“短接 2-3 运行 / 1-2 下载”。
- 排针靠近 MCU 放置，走线短；不要与功率器件或栅极驱动区域紧邻。

---

## 2. 六相 PWM（HRTIM1，共 12 路）

| LQFP100 | 引脚 | 网络名 | 功能 | 5V容忍 | 备注 |
|---:|---|---|---|:--:|---|
| 69 | PA8 | H1A | HRTIM1_CHA1 | 是 | 也是 ADC5_IN1 |
| 70 | PA9 | L1A | HRTIM1_CHA2 | 是 | 也是 ADC5_IN2 |
| 71 | PA10 | H1B | HRTIM1_CHB1 | 是 | |
| 72 | PA11 | L1B | HRTIM1_CHB2 | 是 | 也是 USB_DM |
| 51 | PB12 | H1C | HRTIM1_CHC1 | **否** | 也是 ADC1_IN11 / ADC4_IN3 |
| 52 | PB13 | L1C | HRTIM1_CHC2 | **否** | 也是 ADC3_IN5 |
| 53 | PB14 | H2A | HRTIM1_CHD1 | **否** | 也是 ADC1_IN5 / ADC4_IN4 |
| 54 | PB15 | L2A | HRTIM1_CHD2 | **否** | 也是 ADC2_IN15 / ADC4_IN5 |
| 65 | PC6 | H2B | HRTIM1_CHF1 | 是 | |
| 66 | PC7 | L2B | HRTIM1_CHF2 | 是 | 也是 HRTIM1_FLT5 |
| 67 | PC8 | H2C | HRTIM1_CHE1 | 是 | |
| 68 | PC9 | L2C | HRTIM1_CHE2 | 是 | |

> CHx1 = 高边、CHx2 = 互补低边；两路电机各用 3 个独立 HRTIM 定时器，死区可分别设置。

---

## 3. 电流采样与母线电压

| LQFP100 | 引脚 | 网络名 | ADC 通道 | 5V容忍 | 备注 |
|---:|---|---|---|:--:|---|
| 20 | PA0 | I_M1A | ADC1_IN1 / ADC2_IN1 | **否** | 电机1 A 相 |
| 21 | PA1 | I_M1B | ADC1_IN2 / ADC2_IN2 | **否** | 电机1 B 相 |
| 26 | PA4 | I_M2A | **仅 ADC2_IN17** | **否** | 电机2 A 相；也是 DAC1_OUT1 |
| 27 | PA5 | I_M2B | **仅 ADC2_IN13** | **否** | 电机2 B 相；也是 DAC1_OUT2 |
| 50 | PB11 | V_BUS | ADC1_IN14 / ADC2_IN14 | **否** | 母线电压分压；也是 HRTIM1_FLT4 |

> **四个电流采样脚全是 TT 型（3.3V 耐压，非 5V 容忍）**，INA240 输出绝不能越过 VDDA。
> 采样架构：ADC1 注入组吃 PA0/PA1，ADC2 注入组吃 PA4/PA5，同一 HRTIM 事件触发 → 两台电机同刻采样。
> 母线电压可放 ADC1 规则组，不参与电流环时序。

---

## 4. 磁编码器（两路 SPI）

| LQFP100 | 引脚 | 网络名 | 功能 | 5V容忍 | 备注 |
|---:|---|---|---|:--:|---|
| 90 | PB3 | ENC1_SCK | SPI1_SCK | 是 | 复位后为 JTDO，SWD 模式下释放 |
| 91 | PB4 | ENC1_MISO | SPI1_MISO | 是 | 复位后为 JTRST，内部上拉 |
| 92 | PB5 | ENC1_MOSI | SPI1_MOSI | 是 | |
| 93 | PB6 | ENC1_CS | GPIO（软件片选） | 是 | **10kΩ 上拉到 3.3V** |
| 43 | PE12 | ENC2_SCK | SPI4_SCK | 是 | 也是 ADC345_IN16 |
| 44 | PE13 | ENC2_MISO | SPI4_MISO | 是 | 也是 ADC3_IN3 |
| 45 | PE14 | ENC2_MOSI | SPI4_MOSI | 是 | 也是 ADC4_IN1 |
| 42 | PE11 | ENC2_CS | GPIO（或 SPI4_NSS） | 是 | **10kΩ 上拉到 3.3V** |

> 两组各是封装上连续的 4 个脚。CS 上拉是必须的：上电复位期间 CS 是浮空输入，磁编可能被误选中并在 MISO 上乱驱动。

---

## 5. 通信与人机接口

| LQFP100 | 引脚 | 网络名 | 功能 | 5V容忍 | 备注 |
|---:|---|---|---|:--:|---|
| 79 | PC10 | UART_TX | UART4_TX / USART3_TX | 是 | 与主控板通信 |
| 80 | PC11 | UART_RX | UART4_RX / USART3_RX | 是 | |
| 15 | PC0 | KEY | GPIO 输入 | **否** | 10kΩ 上拉到 3V3，按键另一端接 GND |
| 73 | PA12 | LED_FAULT | GPIO 输出 | 是 | 故障指示 |
| 84 | PD2 | LED_RUN | GPIO 输出 | 是 | 运行指示 |
| 7 | PC13 | 备用 | GPIO | 是 | **只能 3mA，不能驱动 LED** |

---

## 6. 空闲 / 备用引脚

| LQFP100 | 引脚 | 可用能力 | 5V容忍 |
|---:|---|---|:--:|
| 10 / 11 | PF9 / PF10 | 空闲，可作 SPI2_SCK / TIM15 | 是 |
| 19 | PF2 | 空闲，可作 TIM20_CH3 | 是 |
| 22 / 25 | PA2 / PA3 | 空闲，USART2_TX/RX、TIM2、TIM15 | **否** |
| 28 / 29 | PA6 / PA7 | 空闲，**ADC2_IN3 / ADC2_IN4** | **否** |
| 30 / 31 | PC4 / PC5 | 空闲，**ADC2_IN5 / ADC2_IN11** | 是 / **否** |
| 32 | PB0 | 空闲，**HRTIM1_FLT5**、ADC1_IN15、ADC3_IN12 | **否** |
| 33 | PB1 | 空闲，**ADC1_IN12 / ADC3_IN1**、HRTIM1_SCOUT | **否** |
| 16 / 17 / 18 | PC1 / PC2 / PC3 | 空闲，**ADC12_IN7 / IN8 / IN9**、TIM1_CH2~CH4 | **否** |
| 34 | PB2 | 空闲，ADC2_IN12 | **否** |
| 47 | PB10 | 空闲，**HRTIM1_FLT3**、USART3_TX | **否** |
| 78 | PA15 | 空闲（JTDI），SPI1_NSS / SPI3_NSS | 是 |
| 81 | PC12 | 空闲，SPI3_MOSI、UART5_TX | 是 |
| 94 / 96 | PB7 / PB9 | 空闲，USART1_RX（PB7）/ USART3_TX（PB9）、I2C1 | 是 |
| 97 / 98 | PE0 / PE1 | 空闲 | 是 |
| 1–5 | PE2–PE6 | 空闲，SPI4 备用组、TIM3 | 是 |
| 38–41, 46 | PE7–PE10, PE15 | 空闲，ADC3/ADC4 通道 | 是 |
| 55–62 | PD8–PD15 | 空闲，ADC3/4/5 通道、SPI2_NSS | **否** |
| 82–89 | PD0–PD7 | 空闲，FDCAN1、SPI2_NSS、UART5 | 是 |
| 8 / 9 | PC14 / PC15 | 空闲（LSE 不用时作 GPIO，≤3mA） | 是 |

> **过流保护预留**：PB0（HRTIM1_FLT5）与 PB10（HRTIM1_FLT3）刻意留空 —— DRV8300 无故障输出，
> 建议用 G474 的 COMP + DAC 做硬件比较，比较结果送到这两个脚，由 HRTIM 硬件直接关断 PWM。

---

## 7. 相对旧表的变更记录

| 项目 | 旧 | 新 | 原因 |
|---|---|---|---|
| 复位 | PG0 | **PG10（14 脚）** | 手册固定为 PG10-NRST；PG0 在 LQFP100 未引出 |
| 母线电压 | PB6 | **PB11（50 脚，ADC1_IN14）** | PB6 无 ADC 通道 |
| 磁编1 SCK | PB3 | PB3（不变） | 唯一正确的一根 |
| 磁编1 MISO | PC1 | **PB4（91）** | PC1 无 SPI 功能 |
| 磁编1 MOSI | PC2 | **PB5（92）** | PC2 无 SPI 功能 |
| 磁编1 CS | PC3 | **PB6（93）** | 改用紧邻的空闲脚，PC3 归还作 ADC |
| 磁编2 SCK | PB10 | **PE12（43）** | PB10 无 SPI 功能 |
| 磁编2 MISO | PC4 | **PE13（44）** | PC4 无 SPI 功能 |
| 磁编2 MOSI | PC5 | **PE14（45）** | PC5 无 SPI 功能 |
| 磁编2 CS | PC12 | **PE11（42）** | 四脚连续，便于布线 |
| 12 路 HRTIM | — | 不变 | 全部核对正确 |
| BOOT0 / PB8 | — | 不变 | PB8 与 BOOT0 共用，原表正确 |
| PC13 空闲 | — | 不变 | 原表正确（3mA 限制） |

---

## 8. DRV8300 / INA240 关键约束（供后续外围设计引用）

### DRV8300（ZHCSMX4D Rev D）
- **无 nFAULT、无 nSLEEP、无使能脚**，MCU 侧只需 12 根 PWM（INHx/INLx）。
- 输入电平：VIH(min) = 2.0 V，VIL(max) = 0.8 V → **3.3V 直连可用**；绝对最大为 GVDD+0.3V。
- INHx/INLx 内部有 120–280 kΩ 下拉（非反相型号）→ MCU 悬空时输出保持关断。
- GVDD 范围 4.8–20 V，典型 12 V；逻辑脚电压上限跟着 GVDD 走。
- 死区：TSSOP 20 脚**固定 200 ns typ**；VQFN 24 脚有 DT 脚，接电阻到 GND 可在 200 ns–2 µs 调整，悬空 = 200 ns。
- MODE 脚（仅 24 脚）只能接 GVDD 或悬空/接地，**不能接 3.3V 逻辑**（VIL_MODE 0.6V / VIH_MODE 3.7V，回差 2V）。
- INHx 与 INLx 同时为高时内部互锁、两路输出全关——但**死区仍应由 HRTIM 插入**，不要依赖它。
- 自举供电：占空比不能长期 100%，需低边导通刷新自举电容（FOC 最大占空比建议限到 97–98%）。

### INA240（ZHCSFM1C Rev C）
- 四种增益：A1 = 20、A2 = 50、A3 = 100、A4 = 200 V/V。
- 输出 = Gain × (IN+ − IN−) + (REF1 + REF2)/2；REF 脚范围 0 V ~ VS。
- VS = 2.7–5.5 V；共模范围 −4 V ~ 80 V。
- **因为 PA0/PA1/PA4/PA5 是 TT 型（3.3V 耐压），建议 VS = 3.3 V、REF = 1.65 V**，
  或 5 V 供电但必须保证满量程电流时输出 < 3.3 V。
