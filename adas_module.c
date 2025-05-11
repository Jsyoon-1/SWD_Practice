/*============================================================================
 * File: adas_module.c
 * Description: Implementation of ADAS Core Functions
 *============================================================================*/
#include "adas_module.h"
#include <string.h>

/* --------------------------------------------------------
 * 1. Ego Vehicle Estimation
 *-------------------------------------------------------*/
void update_ego_speed(float imu_ax, float imu_ay, float dt,
                      const WheelSpeedSet *ws,
                      float ax_bias, float ay_bias,
                      float alpha, float stationary_th,
                      EVE_State *s)
{
    float ax = imu_ax - ax_bias;
    float v_pred = s->ego_speed + ax * dt;

    if (ws) {
        float wsum = 0.0f;
        for (int i = 0; i < MAX_WHEELS; ++i) wsum += ws->vx[i];
        float wavg = wsum / MAX_WHEELS;
        s->ego_speed = alpha * wavg + (1.0f - alpha) * v_pred;
    } else {
        s->ego_speed = alpha * v_pred + (1.0f - alpha) * s->ego_speed;
    }
    s->ego_speed = clampf(s->ego_speed, 0.0f, 70.0f);
    if (s->ego_speed < stationary_th) s->ego_speed = 0.0f;

    /* optional lateral */
    float ay = imu_ay - ay_bias;
    s->ego_lat_speed += ay * dt;
    s->ego_lat_speed = clampf(s->ego_lat_speed, -15.0f, 15.0f);
}

/* --------------------------------------------------------
 * 2. Target Selection - Preceding
 *-------------------------------------------------------*/
static float lane_y(const LanePoly *lp, float x) {
    return lp->c0 + lp->c1 * x + lp->c2 * x*x + lp->c3 * x*x*x;
}

void target_selection_preceding(const Object objs[], uint32_t obj_count,
                                const LanePoly *lane, float lane_width,
                                float lateral_margin, float horizon,
                                float ego_speed,
                                TS_PrecedingOut *out)
{
    out->valid = false;
    float min_dist = 1e6f;
    for (uint32_t i = 0; i < obj_count; ++i) {
        float x_p = objs[i].x + objs[i].vx * horizon;
        float y_p = objs[i].y + objs[i].vy * horizon;
        float y_lane = lane_y(lane, x_p);
        if (fabsf_safe(y_p - y_lane) > lane_width*0.5f + lateral_margin) continue;
        if (x_p > 0.0f && x_p < min_dist) {
            min_dist = x_p;
            out->target_id    = objs[i].id;
            out->rel_distance = x_p;
            out->rel_speed    = ego_speed - objs[i].vx;
            out->valid        = true;
        }
    }
}

/* --------------------------------------------------------
 * 2. Target Selection - Intersection
 *-------------------------------------------------------*/
void target_selection_intersection(const Object objs[], uint32_t obj_count,
                                   const float conflict_pts[][2], uint32_t cp_count,
                                   float ttc_th, float lat_speed_min,
                                   float ego_speed,
                                   TS_IntersectionOut *out)
{
    out->valid = false;
    float best_ttc = 1e6f;
    for (uint32_t i = 0; i < obj_count; ++i) {
        float vlat = fabsf_safe(objs[i].vy);
        if (vlat < lat_speed_min) continue;
        for (uint32_t k = 0; k < cp_count; ++k) {
            float dx_e = conflict_pts[k][0];
            float dy_e = conflict_pts[k][1];
            float te = sqrtf(dx_e*dx_e + dy_e*dy_e) / fmaxf(ego_speed, EPSILON);
            float dx_o = conflict_pts[k][0] - objs[i].x;
            float dy_o = conflict_pts[k][1] - objs[i].y;
            float to = sqrtf(dx_o*dx_o + dy_o*dy_o) / fmaxf(sqrtf(objs[i].vx*objs[i].vx + objs[i].vy*objs[i].vy), EPSILON);
            float ttc = fabsf_safe(te - to);
            if (ttc < ttc_th && ttc < best_ttc) {
                best_ttc = ttc;
                out->target_id = objs[i].id;
                out->ttc_cross = ttc;
                out->valid = true;
            }
        }
    }
}

/* --------------------------------------------------------
 * 3. Adaptive Cruise Control (ACC)
 *-------------------------------------------------------*/
float pid_update(PID *pid, float error, float dt, float output_min, float output_max)
{
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    float out = pid->kp*error + pid->ki*pid->integral + pid->kd*derivative;
    pid->prev_error = error;
    return clampf(out, output_min, output_max);
}

float calculate_accel_for_speed_pid(PID *pid, float desired_speed,
                                    float ego_speed, float dt, float accel_limit)
{
    float err = desired_speed - ego_speed;
    return pid_update(pid, err, dt, -accel_limit, accel_limit);
}

float calculate_accel_for_distance_pid(PID *pid, float rel_distance,
                                       float rel_speed, float ego_speed,
                                       float dt, float headway_time,
                                       float D_min, float accel_limit)
{
    float D_des = headway_time * ego_speed + D_min;
    float err = rel_distance - D_des;
    return pid_update(pid, err, dt, -accel_limit, accel_limit);
}

float acc_mode_selection(bool preceding_valid, float rel_distance,
                         float follow_distance_max,
                         float hysteresis_time, float dt,
                         float accel_speed, float accel_dist)
{
    static bool dist_mode = false;
    static float timer = 0.0f;
    bool req = preceding_valid && (rel_distance < follow_distance_max);
    if (req != dist_mode) {
        timer += dt;
        if (timer >= hysteresis_time) { dist_mode = req; timer = 0.0f; }
    } else {
        timer = 0.0f;
    }
    return dist_mode ? accel_dist : accel_speed;
}

/* --------------------------------------------------------
 * 4. Autonomous Emergency Brake (AEB)
 *-------------------------------------------------------*/
void info_calculation_for_preceding(float rel_distance,
                                    float rel_speed,
                                    float ego_speed,
                                    float max_decel,
                                    float warn_margin,
                                    AEB_Info *out)
{
    float closing_speed = fmaxf(-rel_speed, EPSILON);
    out->ttc        = rel_distance / closing_speed;
    out->brake_time = ego_speed / fabsf(max_decel);
    out->warn_time  = out->brake_time + warn_margin;
}

void info_calculation_for_intersection_crossing(float ttc_cross,
                                                float safety_ratio,
                                                float warn_margin,
                                                AEB_Info *out)
{
    out->ttc        = ttc_cross;
    out->brake_time = ttc_cross * safety_ratio;
    out->warn_time  = out->brake_time + warn_margin;
}

void aeb_mode_selection(const AEB_Info *front_info,
                        const AEB_Info *cross_info,
                        float ego_speed,
                        float stop_threshold_kmh,
                        float max_decel,
                        float state_hold_time,
                        float dt,
                        AEB_Output *out)
{
    static enum AEB_State curr = AEB_STATE_NORMAL;
    static float hold_timer = 0.0f;
    /* 1) choose active scenario */
    const AEB_Info *active = NULL;
    if (front_info->ttc > 0.0f &&
        (cross_info->ttc <= 0.0f || front_info->ttc <= cross_info->ttc)) {
        active = front_info;
    } else if (cross_info->ttc > 0.0f) {
        active = cross_info;
    }
    /* 2) determine requested state */
    enum AEB_State req = AEB_STATE_NORMAL;
    if (active) {
        if (active->ttc < active->brake_time)      req = AEB_STATE_BRAKE;
        else if (active->ttc < active->warn_time)  req = AEB_STATE_ALARM;
    }
    /* 3) debounced transition */
    if (req != curr) {
        hold_timer += dt;
        if (hold_timer >= state_hold_time) { curr = req; hold_timer = 0.0f; }
    } else {
        hold_timer = 0.0f;
    }
    /* 4) release if stopped */
    if (curr == AEB_STATE_BRAKE && ego_speed * 3.6f < stop_threshold_kmh) {
        curr = AEB_STATE_NORMAL;
    }
    /* 5) outputs */
    out->state     = curr;
    out->accel_cmd = (curr == AEB_STATE_BRAKE) ? max_decel : 0.0f;
}

/* --------------------------------------------------------
 * 5. Lane Following Assist (LFA)
 *-------------------------------------------------------*/
float calculate_steer_in_low_speed_pid(PID *pid,
                                       float lane_center_offset,
                                       float dt,
                                       float steer_limit)
{
    return clampf(pid_update(pid, lane_center_offset, dt,
                             -steer_limit, steer_limit),
                  -steer_limit, steer_limit);
}

float calculate_steer_in_high_speed_pid(PID *pid,
                                        float lane_center_offset,
                                        float lane_heading_error,
                                        float dt,
                                        float w_offset,
                                        float w_heading,
                                        float steer_limit)
{
    float err = w_offset * lane_center_offset +
                w_heading * lane_heading_error;
    return clampf(pid_update(pid, err, dt,
                             -steer_limit, steer_limit),
                  -steer_limit, steer_limit);
}

float lfa_mode_selection(float ego_speed_kmh,
                         float speed_switch_kmh,
                         float steer_low,
                         float steer_high)
{
    return (ego_speed_kmh <= speed_switch_kmh) ? steer_low : steer_high;
}

/* --------------------------------------------------------
 * 6. Arbitration
 *-------------------------------------------------------*/
void arbitration(float driver_accel_cmd,
                 float aeb_accel_cmd,
                 float acc_accel_cmd,
                 float driver_steer_torque,
                 float aes_steer_torque,
                 float lfa_steer_cmd,
                 float torque_threshold,
                 float *final_accel_cmd,
                 float *final_steer_cmd)
{
    /* longitudinal */
    if (fabsf_safe(driver_accel_cmd) > 1e-2f) {
        *final_accel_cmd = driver_accel_cmd;
    } else if (fabsf_safe(aeb_accel_cmd) > 1e-3f) {
        *final_accel_cmd = aeb_accel_cmd;
    } else {
        *final_accel_cmd = acc_accel_cmd;
    }
    /* lateral */
    if (fabsf_safe(driver_steer_torque) > torque_threshold) {
        *final_steer_cmd = 0.0f;
    } else if (fabsf_safe(aes_steer_torque) > 1e-3f) {
        *final_steer_cmd = aes_steer_torque * 0.1f;
    } else {
        *final_steer_cmd = lfa_steer_cmd;
    }
}