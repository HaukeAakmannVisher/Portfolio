# PV System Efficiency Monitor — Code Map (STM32F4)

> Scope: This folder contains the **key source files** to understand the firmware structure, pin mapping, calibration, and calculations.  
> Note: This is **not** a full Cube project; only the important sources are included for review.

---

## Files at a glance

- **`main.c` / `main.h`**
  - Application entry + global config.
  - Measurement loop (PV V/I sampling, power calculation, LCD updates).
  - Button/EXTI state handling, simple UI pages.
  - Where to find:
    - **Pin names & app-level symbols:** `main.h` (`#define` for LCD pins, buttons, any sensor aliases).
    - **Top-level constants:** scaling factors, sample counts, debounce/settle timings (near the top of `main.c`).
    - **Init** blocks: `MX_GPIO_Init`, `MX_ADC1_Init`, `MX_TIM*_Init`, (and `MX_USART*_Init` if present).
    - **Main loop:** the sequence that sets load duty → samples ADC → converts to engineering units → computes power → updates LCD.
    - **Calibration hooks:** look for functions/sections marked with comments like `// Calibration`, or variables for zero-offsets / reference gains.
    - **EXTI callbacks:** if UI buttons generate interrupts, see `HAL_GPIO_EXTI_Callback()` in `main.c`.

- **`lcd16x2.c` / `lcd16x2.h`**
  - HD44780 16×2 driver (minimal).
  - Typical API surface:
    - `lcd16x2_init()`, `lcd16x2_clear()`, `lcd16x2_set_cursor(row, col)`, `lcd16x2_print(const char*)`, and a `printf`-style helper if present.
  - Where to find:
    - Pin wiring mode (4-bit vs 8-bit), enable pulse timing, command/data writes inside `lcd16x2.c`.

- **`stm32f4xx_hal_msp.c`**
  - Low-level **pinmux & peripheral MSP** init.
  - Where to find:
    - **ADC1 channel pin config** (analog mode, PAx/PCx).
    - **TIM PWM** channel pin (e.g., `TIM5_CH1` on PA0) used for the active load.
    - **USART** pinmux (if logging is enabled).
    - **NVIC priorities/enables** for ADC/TIM/EXTI.

- **`stm32f4xx_it.c`**
  - **Interrupt vector** stubs that dispatch to HAL.
  - Where to find:
    - Enabled ISRs (e.g., `ADC_IRQHandler`, `TIM5_IRQHandler`, `EXTIx_IRQHandler`, `USART2_IRQHandler`).
    - If any user logic is added, it will be just before/after the HAL handler calls.

- **`system_stm32f4xx.c`**
  - CMSIS **system clock** and vector table setup (standard template).
  - Where to find:
    - HSI/HSE/system clock config baseline used by the app.

- **`syscalls.c` / `sysmem.c`**
  - Newlib **printf/heap** stubs; included so `printf`/malloc-like calls compile in embedded context.

---

## Where the important things live (by topic)

### 1) Pin mapping & symbols
- **App-level pin names / roles:** `main.h`  
  Look for `#define` of LCD `RS/E/D4–D7`, buttons, LEDs, and any sensor GPIO labels used in the app state machine/UI.
- **Peripheral pinmux (alternate/analog):** `stm32f4xx_hal_msp.c`  
  ADC input pins (analog), TIM PWM output pin (AF mode), USART pins (AF mode).

### 2) Peripheral initialisation (HAL)
- In **`main.c`**, search `MX_*_Init`:
  - `MX_ADC1_Init()` → channel config & sampling time.
  - `MX_TIM5_Init()` (or similar) → PWM base frequency for active load.
  - `MX_GPIO_Init()` → button inputs with pull-ups, LCD control/data pins as output.
  - `MX_USART*_Init()` (if present) → UART for debug prints.

### 3) Measurement loop (V/I sampling → power)
- In **`main.c`**, find the main `while(1)` or a function it calls (e.g., `MeasureSP()` / `RunSweep()`).
- Typical pattern:
  1. **Set PWM duty** for the active load (TIM compare register).
  2. **Settle delay** (ms) to allow analog to stabilise.
  3. **Start ADC conversions**, accumulate **N samples** per channel.
  4. **Convert ADC → volts/amps** using scaling constants (divider ratio, shunt value, gain).
  5. Compute **power** `P = V * I`.
  6. Track **peak power** / summary stats.
  7. **LCD update** (current V, I, P or page-based summaries).

- Quick grep keys:
  - `HAL_ADC_Start` / `HAL_ADC_PollForConversion` / `HAL_ADC_GetValue`
  - `__HAL_TIM_SET_COMPARE` or `htimX.Instance->CCR1`
  - `lcd16x2_` (to see what’s displayed and when)

### 4) Calculations & constants
- In **`main.c`** near the top:
  - **ADC reference voltage** (e.g., 3.3 V) and **resolution** (12-bit → 4095).
  - **Voltage divider ratio** for \(V_{PV}\) conversion.
  - **Shunt resistance** & **amplifier gain** for \(I_{PV}\).
  - Optional **averaging window** sizes (number of samples per point).
- In the loop:
  - **Unit conversion** helpers or inline math → volts, amps, watts.
  - Any **moving average / median** to reduce noise.

### 5) Calibration
- In **`main.c`**, look for a clearly marked block or function:
  - Zero-offset capture for channels (`offset_V`, `offset_I`).
  - Reference scaling adjustment constants if you saved them.
  - Button-driven calibration flow: check in `HAL_GPIO_EXTI_Callback()` and the state machine.

### 6) UI & state machine
- **UI pages / modes:** typically small `switch`/`if` logic in `main.c` controlling:
  - ENV/SP/Calibration selections.
  - What the LCD shows on each page.
- **Buttons:** edge handling in `HAL_GPIO_EXTI_Callback()` (in `main.c`) with simple **debounce** logic/guards.

### 7) LCD driver entry points
- `lcd16x2_init()`, `lcd16x2_clear()`, `lcd16x2_set_cursor()`, `lcd16x2_print()`  
  These are called from `main.c` after init and during measurement/page updates.

---

## Search cheatsheet (quick navigation)

- **Pins & symbols:** `#define LCD_`, `BTN_`, `ADC_CH`, `SHUNT`, `DIVIDER`
- **Init calls:** `MX_GPIO_Init`, `MX_ADC1_Init`, `MX_TIM`, `MX_USART`
- **ADC path:** `HAL_ADC_Start`, `HAL_ADC_GetValue`, `adc`, `raw`
- **PWM duty:** `CCR1`, `__HAL_TIM_SET_COMPARE`, `duty`
- **Math:** `V_PV`, `I_PV`, `power`, `P =`
- **Calibration:** `calib`, `offset`, `zero`
- **LCD:** `lcd16x2_`
- **Buttons/EXTI:** `HAL_GPIO_EXTI_Callback`, `EXTI`

---

## Minimal notes

- Figures/diagrams are in the **report PDF** (one level up).
- If constants don’t match your board values, check the **scaling defines** at the top of `main.c`.
- The HAL MSP file shows **which MCU pins** the ADC/TIM/UART are actually routed to.

