/**
  * @file    app.c
  * @brief   Main-loop glue: CAN command -> setpoints, 100 ms control step,
  *          feedback, command timeout, bus-off reaction, IWDG refresh.
  */
#include "app.h"
#include "app_config.h"
#include "can_proto.h"
#include "cmd_timeout.h"
#include "drive_ctrl.h"
#include "drv_can.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "encoder_math.h"
#include "iwdg.h"
#include "kinematics.h"
#include "main.h"
#include "pi_ctrl.h"
#include "tim.h"

#define WHEEL_RADIUS_M   (WHEEL_DIAMETER_M * 0.5f)
#define ALL_MOTORS_MASK  ((1U << MOTOR_COUNT) - 1U)
#define MS_PER_S         1000.0f

typedef struct
{
  uint8_t   wheel_no;
  float     target_m_s;   /* from the command, direction flip applied */
  float     meas_rad_s;   /* from the encoder */
  int64_t   enc_total;    /* counts since boot */
  pi_ctrl_t pi;
} wheel_t;

typedef struct  /* diagnostics, read with a debugger */
{
  uint32_t rx_invalid;
  uint32_t lin_y_ignored;
  uint32_t feedback_dropped;
} app_stats_t;

static const uint8_t motor_wheel_no[MOTOR_COUNT] = MOTOR_WHEEL_NO;
static const pi_ctrl_params_t pi_params =
{
  PI_KP, PI_KI, PI_KD, PI_INTEGRAL_LIMIT, PI_OUTPUT_LIMIT, PI_ZERO_SETPOINT_EPS
};

static volatile uint32_t tick_count;  /* TIM6 ticks; a new value is the tick flag */
static wheel_t       wheels[MOTOR_COUNT];
static drive_mode_t  drive_mode;
static cmd_timeout_t cmd_timeout;
static bool          outputs_enabled;
static uint32_t      ctrl_tick;
static uint32_t      wdg_tick;
static uint32_t      fb_pending;      /* bit per motor still to send */
static app_stats_t   stats;

void app_init(void)
{
  drv_motor_init();
  drv_encoder_init();
  for (uint8_t m = 0U; m < MOTOR_COUNT; m++)
  {
    wheels[m].wheel_no = motor_wheel_no[m];
    pi_ctrl_init(&wheels[m].pi, &pi_params);
  }
  drive_mode = DRIVE_MODE_X_STOP;
  cmd_timeout_init(&cmd_timeout, CMD_TIMEOUT_MS);
  drv_can_init();
  if (HAL_TIM_Base_Start_IT(&CONTROL_TICK_TIM) != HAL_OK)
  {
    Error_Handler();
  }
}

static void apply_command(const drive_cmd_t *cmd)
{
  if (cmd->lin_y != 0.0f)
  {
    stats.lin_y_ignored++;
  }
  drive_mode = cmd->mode;
  for (uint8_t m = 0U; m < MOTOR_COUNT; m++)
  {
    wheels[m].target_m_s = kinematics_wheel_velocity(wheels[m].wheel_no, cmd->lin_x, cmd->ang_z);
  }
}

static void process_rx(uint32_t now_ms)
{
  can_frame_t frame;
  drive_cmd_t cmd;

  while (drv_can_receive(&frame))
  {
    if (!can_proto_decode_drive_cmd(&frame, &cmd))
    {
      stats.rx_invalid++;
      continue;
    }
    apply_command(&cmd);
    cmd_timeout_feed(&cmd_timeout, now_ms);
  }
}

/* Bridges follow the command watchdog: disabled with zero duty unless a
 * valid command arrived within the timeout. */
static void update_output_enable(void)
{
  bool active = cmd_timeout_is_active(&cmd_timeout);

  if (active == outputs_enabled)
  {
    return;
  }
  if (!active)
  {
    for (uint8_t m = 0U; m < MOTOR_COUNT; m++)
    {
      drv_motor_set_duty(m, 0.0f);
      pi_ctrl_reset(&wheels[m].pi);
    }
  }
  drv_motor_enable_all(active);
  outputs_enabled = active;
}

static void control_step(float dt_s)
{
  for (uint8_t m = 0U; m < MOTOR_COUNT; m++)
  {
    wheel_t *w = &wheels[m];
    int32_t delta = drv_encoder_read_delta(m);

    w->enc_total += delta;
    w->meas_rad_s = encoder_math_rad_per_s(delta, dt_s);
    if (outputs_enabled)
    {
      float signal = drive_ctrl_signal(drive_mode, &w->pi, w->target_m_s,
                                       w->meas_rad_s * WHEEL_RADIUS_M, dt_s);
      drv_motor_set_duty(m, signal / MOTOR_SIGNAL_FULL_SCALE);
    }
  }
  if (fb_pending != 0U)
  {
    stats.feedback_dropped++;
  }
  fb_pending = ALL_MOTORS_MASK;
}

/* The TX FIFO holds 3 frames, so the 4 feedback frames may need more than
 * one pass; unsent frames are retried on the next loop. */
static void send_feedback(void)
{
  can_frame_t frame;

  for (uint8_t m = 0U; (m < MOTOR_COUNT) && (fb_pending != 0U); m++)
  {
    if ((fb_pending & (1U << m)) == 0U)
    {
      continue;
    }
    can_proto_encode_feedback(wheels[m].wheel_no, encoder_math_counts_to_rad(wheels[m].enc_total),
                              wheels[m].meas_rad_s, &frame);
    if (!drv_can_send(&frame))
    {
      return;
    }
    fb_pending &= ~(1U << m);
  }
}

/* Refresh only when this loop runs, TIM6 has ticked since the last refresh
 * and CAN recovery has not given up. A stuck main loop, a stopped TIM6 or
 * exhausted bus-off retries therefore end in an IWDG reset. */
static void refresh_watchdog(uint32_t tick)
{
  if ((tick != wdg_tick) && drv_can_is_healthy())
  {
    (void)HAL_IWDG_Refresh(&WATCHDOG_HANDLE);
    wdg_tick = tick;
  }
}

void app_loop(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t tick = tick_count;

  process_rx(now_ms);
  if (drv_can_service(now_ms))
  {
    cmd_timeout_expire(&cmd_timeout);
  }
  cmd_timeout_update(&cmd_timeout, now_ms);
  update_output_enable();
  if ((tick - ctrl_tick) >= CONTROL_TICKS_PER_STEP)
  {
    control_step((float)(tick - ctrl_tick) * (float)CONTROL_TICK_MS / MS_PER_S);
    ctrl_tick = tick;
  }
  send_feedback();
  refresh_watchdog(tick);
}

/* TIM6 ISR only counts. The control step runs in app_loop because FDCAN RX
 * and TIM6 share priority 0: float math and HAL calls here would delay
 * draining the 3-deep RX FIFO. Main-loop latency is negligible against the
 * 100 ms control period. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == CONTROL_TICK_TIM.Instance)
  {
    tick_count++;
  }
}
