/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include <stdio.h>
#include <string.h>
#include <ble_gap.h>
#include <nrf_delay.h>
#include "nrf_sdh.h"
#include "nrfx_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrfx_clock.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "button.h"
#include "ble_services.h"
#include "timers.h"
#include "sensors.h"
#include "pwm.h"
#include <boards.h>
#include <ble_dfu.h>
#include "ec_sensor.h"
#include "bat_sensor.h"
#include "storage.h"

#define FPU_EXCEPTION_MASK               0x0000009F                      //!< FPU exception mask used to clear exceptions in FPSCR register.
#define FPU_FPSCR_REG_STACK_OFF          0x40                            //!< Offset of FPSCR register stacked during interrupt handling in FPU part stack.

bool adv_update = false;

static void clock_evt_handler(nrfx_clock_evt_type_t event) { }

static void init_clock()
{
  APP_ERROR_CHECK(nrfx_clock_init(clock_evt_handler));
  nrfx_clock_lfclk_start();
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
//  bsp_board_leds_on();
  app_error_save_and_stop(id, pc, info);
}

/* Function for initializing the nrf log module. */
static void log_init(void)
{
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
}

/* Function for handling the idle state (main loop).
 * If there is no pending log operation, then go_to_sleep until next the next event occurs. */
static void idle_state_handle(void)
{
  if (NRF_LOG_PROCESS() == false)
  {
    nrf_pwr_mgmt_run();
  }
}

/**
 * @brief FPU Interrupt handler. Clearing exception flag at the stack.
 *
 * Function clears exception flag in FPSCR register and at the stack. During interrupt handler
 * execution FPU registers might be copied to the stack (see lazy stacking option) and
 * it is necessary to clear data at the stack which will be recovered in the return from
 * interrupt handling.
 */
void FPU_IRQHandler(void)
{
  // Prepare pointer to stack address with pushed FPSCR register.
  uint32_t * fpscr = (uint32_t * )(FPU->FPCAR + FPU_FPSCR_REG_STACK_OFF);
  // Execute FPU instruction to activate lazy stacking.
  (void)__get_FPSCR();
  // Clear flags in stacked FPSCR register.
  *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}

/* Function for initializing power management. */
void power_management_init(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/* power management */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
  switch (event)
  {
    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
      NRF_LOG_INFO("Power management wants to reset to DFU mode\r\n");
      // Change this code to tailor to your reset strategy.
      // Returning false here means that the device is not ready to jump to DFU mode yet.
      //
      // Here is an example using a variable to delay resetting the device:
//      if (!m_ready_for_reset)
//      {
//        return false;
//      }
      break;

    default:
      // Implement any of the other events available from the power management module:
      //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
      //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
      //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
      return true;
  }
  NRF_LOG_INFO("Power management allowed to reset to DFU mode\r\n");
  return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void button_action_handler(button_state_t state)
{
  static uint8_t calibration_mode = 0;
  ec_calibration_mode_t calibration[] = {EC_CAL_LOW, EC_CAL_MID, EC_CAL_HI};

  switch (state)
  {
    case RELEASED:
      if (get_ec_calibration_status())
      {
        NRF_LOG_INFO("BTN SHORT PRESS, switch calibration_mode %u", calibration_mode);
        do_ec_calibration(calibration[calibration_mode++]);

        if (calibration_mode == 3)
          calibration_mode = 0;
      } else {
        NRF_LOG_INFO("BTN SHORT PRESS, start read sensors");

        /* Start read sensor timer */
        read_sensor_timer_stop();
        read_sensor_timer_start(true);
      }
      break;

    case LONG_PRESSED:
      NRF_LOG_INFO("BTN LONG PRESS, is_calibrating? %u", get_ec_calibration_status());
      if (!get_ec_calibration_status())
      {
        calibration_mode = 0;
        NRF_LOG_INFO("start calibration_mode %u", calibration_mode);
        do_ec_calibration(calibration[calibration_mode++]);
      }
      break;

    case LONG_LONG_PRESSED:
      NRF_LOG_INFO("BTN LONG LONG PRESS, factory defaults");
      factory_settings();
      break;

    default:
      break;
  }
}

int main(void)
{
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  log_init();

  // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
  ret_code_t err_code;
  err_code = ble_dfu_buttonless_async_svci_init();
  APP_ERROR_CHECK(err_code);

  timers_init();
  init_clock();
  init_led_pwm();
  power_management_init();

  /* Enable internal DC/DC converter */
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

  /* Welcome Message */
  NRF_LOG_INFO("Start EcSensor App");

  /* Init Storage */
  if (storage_init() > 0)
  {
    settings_t * settings = get_settings();
    set_ec_calibration(settings->low_us, settings->mid_us, settings->hi_us);
    set_scan_interval(settings->scan_interval);
  }

  /* Init Button */
  button_init();
  set_button_action_handler(button_action_handler);

  /* Start read sensor timer */
  read_sensor_timer_start(true);

  /* Init BLE services */
  init_ble();

  // Enable FPU interrupt
  NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOWEST);
  NVIC_ClearPendingIRQ(FPU_IRQn);
  NVIC_EnableIRQ(FPU_IRQn);

  for (;;)
  {
    NRF_LOG_FLUSH();
    /* Clear FPSCR register and clear pending FPU interrupts. This code is base on
         * nRF5x_release_notes.txt in documentation folder. It is necessary part of code when
         * application using power saving mode and after handling FPU errors in polling mode.
         */
    __set_FPSCR(__get_FPSCR() & ~(FPU_EXCEPTION_MASK));
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);

    idle_state_handle();
  }
}


/** @} */
