#include "gravity.h"
#include "gravity_config.h"
#include "gravity_log.h"
#include "gravity_model.h"
#include "remote_control.h"
#include "dm4310.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include <math.h>

volatile gravity_debug_t gravity_debug;
volatile float gravity_output_scale[2] = {1.0f, 1.0f};//置信系数

static const uint8_t motor_id[2] = {GRAVITY_MOTOR1_ID, GRAVITY_MOTOR2_ID};
static const float q_min[2] = {GRAVITY_Q1_MIN_RAD, GRAVITY_Q2_MIN_RAD};
static const float q_max[2] = {GRAVITY_Q1_MAX_RAD, GRAVITY_Q2_MAX_RAD};
static const float q_home[2] = {GRAVITY_Q1_HOME_RAD, GRAVITY_Q2_HOME_RAD};
static const float max_speed[2] = {GRAVITY_Q1_MAX_SPEED_RAD_S, GRAVITY_Q2_MAX_SPEED_RAD_S};
static const float torque_limit[2] = {GRAVITY_Q1_TORQUE_LIMIT_NM, GRAVITY_Q2_TORQUE_LIMIT_NM};
static const float output_sign[2] = {GRAVITY_OUTPUT_SIGN_1, GRAVITY_OUTPUT_SIGN_2};

static gravity_mode_t mode = GRAVITY_MODE_SAFE;
static gravity_sample_state_t sample_state = GRAVITY_SAMPLE_IDLE;
static uint32_t seen_rx[2];//
static uint32_t last_rx_ms[2];
static uint32_t last_disable_ms;
static uint8_t enable_pending[2] = {1U, 1U};
static uint8_t enable_sent[2];
static uint32_t enable_rx_count[2];
static uint32_t last_enable_ms[2];
static uint32_t accum_seen_rx[2];
static uint32_t move_start_ms;
static uint32_t move_duration_ms;
static uint32_t stable_start_ms;
static uint32_t wait_start_ms;
static uint32_t seq;
static uint16_t pose_index;
static uint8_t reverse_pass;
static uint8_t home_phase;
static uint8_t sample_fault_latched;
static uint16_t accum_count;
static float accum_q[2];
static float accum_t[2];
static float move_start[2];
static float move_target[2];
static float ramp;

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float halton(uint16_t index, uint16_t base)
{
    float result = 0.0f;
    float fraction = 1.0f / (float)base;
    while (index != 0U) {
        result += fraction * (float)(index % base);
        index = (uint16_t)(index / base);
        fraction /= (float)base;
    }
    return result;
}

static void pose_for_index(uint16_t index, float target[2])
{
    target[0] = q_min[0] + (q_max[0] - q_min[0]) * halton((uint16_t)(index + 1U), 2U);
    target[1] = q_min[1] + (q_max[1] - q_min[1]) * halton((uint16_t)(index + 1U), 3U);
}

static uint8_t config_valid(void)
{
    uint8_t i;
    if (GRAVITY_CONFIG_READY == 0U || GRAVITY_SAMPLE_POSE_COUNT < 4U ||
        DM4310_P_MIN >= DM4310_P_MAX || DM4310_T_MAX <= 0.0f) return 0U;
    for (i = 0U; i < 2U; ++i) {
        if (!(q_min[i] < q_max[i]) || q_min[i] < DM4310_P_MIN ||
            q_max[i] > DM4310_P_MAX || q_home[i] < q_min[i] ||
            q_home[i] > q_max[i] || max_speed[i] <= 0.0f ||
            torque_limit[i] < 0.0f || torque_limit[i] > DM4310_T_MAX) return 0U;
    }
    return 1U;
}

static void send_command(uint8_t axis, float p, float kp, float kd, float t)
{
    if (dm4310_mit_control(&hcan2, motor_id[axis], p, 0.0f, kp, kd, t) != HAL_OK)
        gravity_debug.can_send_fail_count++;
}

static void disable_both(void)
{
    uint8_t i;
    for (i = 0U; i < 2U; ++i) {
        if (dm4310_disable(&hcan2, motor_id[i], DM4310_MODE_MIT) != HAL_OK)
            gravity_debug.can_send_fail_count++;
        enable_pending[i] = 1U;
        enable_sent[i] = 0U;
    }
}

static void enable_both(uint32_t now)
{
    uint8_t i;
    for (i = 0U; i < 2U; ++i) {
        const volatile dm4310_feedback_t *feedback;
        uint32_t mask, rx_count;
        uint8_t state;
        if (enable_pending[i] == 0U) continue;
        feedback = get_dm4310_feedback_point(motor_id[i]);
        mask = __get_PRIMASK();
        __disable_irq();
        rx_count = feedback->rx_count;
        state = feedback->state;
        __set_PRIMASK(mask);
        /* A CAN mailbox acceptance is not an acknowledgement from the motor. */
        if (enable_sent[i] != 0U && rx_count != enable_rx_count[i] && state == 1U) {
            enable_pending[i] = 0U;
            continue;
        }
        if (enable_sent[i] != 0U && now - last_enable_ms[i] < 20U) continue;
        if (dm4310_enable(&hcan2, motor_id[i], DM4310_MODE_MIT) == HAL_OK) {
            enable_sent[i] = 1U;
            enable_rx_count[i] = rx_count;
            last_enable_ms[i] = now;
        } else {
            gravity_debug.can_send_fail_count++;
        }
    }
}

static void no_force(const float q[2])
{
    uint8_t i;
    (void)q;
    for (i = 0U; i < 2U; ++i) {
        send_command(i, 0.0f, 0.0f, 0.0f, 0.0f);
        gravity_debug.command_torque_nm[i] = 0.0f;
    }
}

static void begin_move(const float q[2], const float target[2], uint32_t now)//两点间平滑移动初始化
{
    uint8_t i;
    float seconds = 0.2f;
    for (i = 0U; i < 2U; ++i) {
        /*  */
        float t = 1.5f * fabsf(target[i] - q[i]) / max_speed[i];
        if (t > seconds) seconds = t;
        move_start[i] = q[i];
        move_target[i] = target[i];
        gravity_debug.target_rad[i] = target[i];
    }
    move_duration_ms = (uint32_t)(1000.0f * seconds + 1.0f);
    move_start_ms = now;
    wait_start_ms = now;
    stable_start_ms = 0U;
    sample_state = GRAVITY_SAMPLE_MOVE;
}

static void begin_sampling(const float q[2], uint32_t now)
{
    pose_index = 0U;
    reverse_pass = 0U;
    home_phase = 1U;
    seq = 0U;
    begin_move(q, q_home, now);
}

static void next_pose(const float q[2], uint32_t now)
{
    float target[2];
    if (reverse_pass == 0U) {
        if (pose_index + 1U < GRAVITY_SAMPLE_POSE_COUNT) {
            pose_index++;
        } else {
            reverse_pass = 1U;
            pose_index = (uint16_t)(GRAVITY_SAMPLE_POSE_COUNT - 2U);
        }
    } else if (pose_index > 0U) {
        pose_index--;
    } else {
        home_phase = 2U;
        begin_move(q, q_home, now);
        return;
    }
    pose_for_index(pose_index, target);
    begin_move(q, target, now);
}

static void finish_sample(const float q[2], uint32_t now)
{
    gravity_log_record_t record;
    (void)q;
    record.seq = seq++;
    record.pair_id = pose_index;
    record.pass = reverse_pass ? 'R' : 'F';
    record.tick_ms = now;
    record.q1_mrad = (int32_t)(1000.0f * accum_q[0] / (float)accum_count);
    record.q2_mrad = (int32_t)(1000.0f * accum_q[1] / (float)accum_count);
    record.t1_mnm = (int32_t)(1000.0f * accum_t[0] / (float)accum_count);
    record.t2_mnm = (int32_t)(1000.0f * accum_t[1] / (float)accum_count);
    gravity_log_push(&record);
    gravity_debug.sample_count++;
}

static void sampling_service(const float q[2], const float speed[2],
                             const float torque[2], uint32_t now)
{
    uint8_t i;
    float target[2];
    if (sample_state == GRAVITY_SAMPLE_MOVE) {
        float x = clampf_local((float)(now - move_start_ms) /
                               (float)move_duration_ms, 0.0f, 1.0f);
        float s = x * x * (3.0f - 2.0f * x);
        for (i = 0U; i < 2U; ++i) {
            target[i] = move_start[i] + (move_target[i] - move_start[i]) * s;
            send_command(i, target[i], 15.0f, 0.6f, 0.0f);
        }
        if (x >= 1.0f) {
            sample_state = GRAVITY_SAMPLE_STABLE;
            wait_start_ms = now;
        }
        return;
    }
    if (sample_state == GRAVITY_SAMPLE_STABLE) {
        uint8_t stable = 1U;
        for (i = 0U; i < 2U; ++i) {
            send_command(i, move_target[i], 12.0f, 0.6f, 0.0f);
            if (fabsf(q[i] - move_target[i]) > GRAVITY_STABLE_ERROR_RAD ||
                fabsf(speed[i]) > GRAVITY_STABLE_SPEED_RAD_S) stable = 0U;
        }
        if (stable) {
            if (stable_start_ms == 0U) stable_start_ms = now;
            if (now - stable_start_ms >= GRAVITY_STABLE_TIME_MS) {
                if (home_phase == 1U) {
                    home_phase = 0U;
                    pose_for_index(0U, target);
                    begin_move(q, target, now);
                } else if (home_phase == 2U) {
                    sample_state = GRAVITY_SAMPLE_DONE;
                } else {
                    accum_count = 0U;
                    accum_q[0] = accum_q[1] = 0.0f;
                    accum_t[0] = accum_t[1] = 0.0f;
                    accum_seen_rx[0] = seen_rx[0];
                    accum_seen_rx[1] = seen_rx[1];
                    sample_state = GRAVITY_SAMPLE_ACCUMULATE;
                }
            }
        } else stable_start_ms = 0U;
        if (now - wait_start_ms > GRAVITY_SETTLE_TIMEOUT_MS) {
            sample_state = GRAVITY_SAMPLE_FAULT;
            gravity_debug.fault_count++;
        }
        return;
    }
    if (sample_state == GRAVITY_SAMPLE_ACCUMULATE) {
        for (i = 0U; i < 2U; ++i)
            send_command(i, move_target[i], 12.0f, 0.6f, 0.0f);
        for (i = 0U; i < 2U; ++i) {
            if (fabsf(q[i] - move_target[i]) > GRAVITY_STABLE_ERROR_RAD ||
                fabsf(speed[i]) > GRAVITY_STABLE_SPEED_RAD_S) {
                sample_state = GRAVITY_SAMPLE_STABLE;
                stable_start_ms = 0U;
                wait_start_ms = now;
                return;
            }
        }
        if (seen_rx[0] != accum_seen_rx[0] && seen_rx[1] != accum_seen_rx[1]) {
            for (i = 0U; i < 2U; ++i) {
                accum_q[i] += q[i];
                accum_t[i] += torque[i];
                accum_seen_rx[i] = seen_rx[i];
            }
            accum_count++;
            if (accum_count >= GRAVITY_AVERAGE_FRESH_PAIRS) {
                finish_sample(q, now);
                next_pose(q, now);
            }
        }
        return;
    }
    if (sample_state == GRAVITY_SAMPLE_DONE)
        for (i = 0U; i < 2U; ++i) send_command(i, q_home[i], 12.0f, 0.6f, 0.0f);
    if (sample_state == GRAVITY_SAMPLE_FAULT) no_force(q);
}

static void compensation_service(const float q[2])//最终的输出，包含输出比例、方向和约 1 秒的渐入系数
{
    float model[2];
    uint8_t i;
    gravity_model_eval(q, model);
    ramp = clampf_local(ramp + 0.002f, 0.0f, 1.0f);
    for (i = 0U; i < 2U; ++i) {
        float limit = torque_limit[i];
        float command = clampf_local(model[i] * gravity_output_scale[i] *
                                     output_sign[i] * ramp, -limit, limit);
        send_command(i, q[i], 0.0f, 0.2f, command);
        gravity_debug.command_torque_nm[i] = command;
    }
    gravity_debug.ramp = ramp;
}

void gravity_init(void)
{
    gravity_log_init();
    remote_control_init();
    disable_both();
    last_disable_ms = HAL_GetTick();
}

void gravity_service_2ms(void)
{
    const RC_ctrl_t *remote = get_remote_control_point();//接口函数，判断拨杆，遥控存活
    uint8_t rc_left = 0U;
    uint8_t rc_valid = 0U;
    uint32_t rc_last_tick_ms, irq_mask;
    const volatile dm4310_feedback_t *feedback[2];
    float q[2], speed[2], torque[2];
    uint8_t motor_state[2];
    uint32_t now = HAL_GetTick();
    uint8_t i;
    uint8_t valid = 1U, enabled = 1U, motor_fault = 0U, driver_fault = 0U;
    uint8_t config_ready = config_valid();
    gravity_mode_t requested = GRAVITY_MODE_SAFE;
    irq_mask = __get_PRIMASK();
    __disable_irq();
    rc_left = (uint8_t)remote->rc.s[1];
    rc_valid = (remote_control_frame_count != 0U) ? 1U : 0U;
    rc_last_tick_ms = remote_control_last_tick_ms;
    __set_PRIMASK(irq_mask);
    gravity_debug.loop_count++;
    gravity_debug.left_switch = rc_left;
    if (rc_left != RC_SW_UP) sample_fault_latched = 0U;

    for (i = 0U; i < 2U; ++i) {
        uint32_t mask;
        feedback[i] = get_dm4310_feedback_point(motor_id[i]);
        mask = __get_PRIMASK();
        __disable_irq();
        q[i] = feedback[i]->position_rad;
        speed[i] = feedback[i]->velocity_rad_s;
        torque[i] = feedback[i]->torque_nm;
        motor_state[i] = feedback[i]->state;
        if (seen_rx[i] != feedback[i]->rx_count) {
            seen_rx[i] = feedback[i]->rx_count;
            last_rx_ms[i] = now;
        }
        if (feedback[i]->rx_count == 0U) valid = 0U;

        if (motor_state[i] != 1U) enabled = 0U;
        if (feedback[i]->rx_count != 0U && motor_state[i] > 1U) {
            valid = 0U;
            motor_fault = 1U;
            driver_fault = 1U;
        }
        __set_PRIMASK(mask);
        if (motor_state[i] == 0U && enable_pending[i] == 0U) {
            enable_pending[i] = 1U;
            enable_sent[i] = 0U;
        }
        if (feedback[i]->rx_count != 0U &&
            (now - last_rx_ms[i] > GRAVITY_FEEDBACK_TIMEOUT_MS ||
             (config_ready && (q[i] < q_min[i] - 0.05f ||
                               q[i] > q_max[i] + 0.05f)))) {
            valid = 0U;
            motor_fault = 1U;
        }
        gravity_debug.q_rad[i] = q[i];
        gravity_debug.velocity_rad_s[i] = speed[i];
        gravity_debug.feedback_torque_nm[i] = torque[i];
    }
    gravity_debug.feedback_valid = valid;
    if (rc_valid &&
        now - rc_last_tick_ms <= GRAVITY_REMOTE_TIMEOUT_MS &&
        (rc_left == RC_SW_UP || rc_left == RC_SW_MID || rc_left == RC_SW_DOWN) &&
        !driver_fault) {
        requested = GRAVITY_MODE_NO_FORCE;
        if (config_ready && valid && enabled && !motor_fault && rc_left == RC_SW_MID)
            requested = GRAVITY_MODE_COMPENSATION;
        if (config_ready && valid && enabled && !motor_fault &&
            rc_left == RC_SW_UP && !sample_fault_latched)
            requested = GRAVITY_MODE_SAMPLING;
    }
    if (requested != mode) {
        if (requested == GRAVITY_MODE_SAFE) {
            disable_both();
            last_disable_ms = now;
        }
        if (requested == GRAVITY_MODE_SAMPLING) begin_sampling(q, now);
        else sample_state = GRAVITY_SAMPLE_IDLE;
        ramp = 0.0f;
        mode = requested;
    }
    if (mode != GRAVITY_MODE_SAFE) {
        enable_both(now);
        if (enable_pending[0] != 0U || enable_pending[1] != 0U) {
            gravity_debug.mode = (uint8_t)mode;
            gravity_debug.sample_state = (uint8_t)sample_state;
            return;
        }
    }
    if (mode == GRAVITY_MODE_NO_FORCE) no_force(q);
    else if (mode == GRAVITY_MODE_SAMPLING) sampling_service(q, speed, torque, now);
    else if (mode == GRAVITY_MODE_COMPENSATION) compensation_service(q);
    else if (now - last_disable_ms >= 100U) {
        disable_both();
        last_disable_ms = now;
    }
    if (sample_state == GRAVITY_SAMPLE_FAULT && mode == GRAVITY_MODE_SAMPLING) {
        sample_fault_latched = 1U;
        disable_both();
        last_disable_ms = now;
        mode = GRAVITY_MODE_SAFE;
    }
    gravity_debug.mode = (uint8_t)mode;
    gravity_debug.sample_state = (uint8_t)sample_state;
    gravity_debug.pair_id = pose_index;
    gravity_debug.pass_reverse = reverse_pass;
}
