# IOC Config

## Hardware-to-Software Mapping
| Peripheral / Pin | Configuration | Software Usage | File Reference |
| :--- | :--- | :--- | :--- |
| **TIM (Control)** | 2000 Hz Interrupt | Inner Loop (2000 Hz) & Outer Loop (200 Hz) | `cascade.h` |
| **TIM (PWM)** | PWM Generation | `PWM(float dutycycle)` | `main.h` |
| **TIM (Encoder)** | Encoder Mode | `encoder_data` tracking | `system_state.h` |
| **LPUART1** | Asynchronous Tx/Rx | Telemetry / Debug Data Stream | `usart.h` |
| **GPIOC_PIN_13** | EXTI15_10_IRQn | User Button (`B1_Pin`) | `main.h` |
| **PA13 / PA14** | SWDIO / SWCLK | Serial Wire Debug (SWD) | `main.h` |

## TIM Config
* **Control Loop Base Timer (2000 Hz):** A hardware timer interrupt is required to run at 2000 Hz (`TS = 0.0005f`) to drive the control loop deterministically. 
  * *Where it is used:* This triggers the inner loop of `CASCADE_Compute()` in `cascade.h`. The outer loop runs at 200 Hz (executed every 10 inner loop ticks).
* **PWM Generation Timer:** Configured to generate the PWM signals for the motor driver.
  * *Where it is used:* Interacts with the `PWM(float dutycycle)` function declared in `main.h`.
* **Encoder Timer:** Configured in encoder mode to read quadrature signals.
  * *Where it is used:* Feeds into `encoder_data` in the structures inside `system_state.h`.
*(Note: `tim.h` setups TIM1-6 and TIM20, but only the timers actively mapped to the software loops above are documented here per your request).*

## UART Config
* **LPUART1:** Initialized via `MX_LPUART1_UART_Init()` (handle `hlpuart1` defined in `usart.h`). 
  * *Where it is used:* Currently initialized in the hardware configuration, but specific transmission calls (e.g., HAL_UART_Transmit for telemetry) are not explicitly present in the provided headers yet.

## GPIO Config
* **User Button (B1_Pin):** `GPIOC_PIN_13`. 
  * *Where it is used:* Configured as an external interrupt (`EXTI15_10_IRQn`) in `main.h`, typically used for resetting states or triggering trajectories.
* **System Clocks:** * HSE (High-Speed External): `GPIOF_PIN_0` (IN) / `GPIOF_PIN_1` (OUT)
  * LSE (Low-Speed External): `GPIOC_PIN_14` (IN) / `GPIOC_PIN_15` (OUT)
* **Debug Pins:** Serial Wire Debug (SWD) is configured on `PA13` (SWDIO), `PA14` (SWCLK), and `PB3` (SWO) for real-time variable monitoring.

## How to use
1. **Initialize the System:** Call `CASCADE_Controller_Init()` and `CASCADE_Feedforward_Init()` to define your PID gains (`kp`, `ki`, `kd`), limits (12V for inner, 4.188 rad/s for outer), and motor parameters.
2. **Link the Controllers:** Call `CASCADE_Cascade_Start()` to link your inner PID, outer PID, and feedforward structures into the main `Cascade` struct.
3. **Start the Control Loop:** Start your 2000 Hz timer interrupt. Inside the interrupt callback:
   * Read the latest `encoder_data` and update your system states.
   * Call `CASCADE_Compute(&csc, pos_setpoint, velo_setpoint)`.
4. **Output to Motor:** Take the calculated `voltage_output` from the cascade computation and pass it into `PWM(dutycycle)`.
5. **Plan Movements:** Use `TRAJ_Trapezoidal_Plan()` or `TRAJ_SCurveLimits_Plan()` (from `trajectory.h`) to calculate smooth setpoints, and feed those setpoints into the Cascade compute function.