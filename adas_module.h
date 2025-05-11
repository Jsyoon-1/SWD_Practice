/*============================================================================
 * File: adas_module.h
 * Description: ADAS Core Functions - Header Definitions
 * Language: C11
 *============================================================================*/
#ifndef ADAS_MODULE_H
#define ADAS_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* --- Constants --- */
#define MAX_OBJECTS        64
#define MAX_WHEELS          4
#define EPSILON            1e-6f

/* --- Utility Macros --- */
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float fabsf_safe(float v) { return v < 0.0f ? -v : v; }

/* --- Data Structures --- */
/** Object: Perceived target vehicle or obstacle */
typedef struct {
    int32_t id;
    float   x, y;        /* position in ego frame [m] */
    float   vx, vy;      /* velocity [m/s] */
    int32_t lane_id;
} Object;

/** Lane polynomial: 3rd-order fit */
typedef struct {
    float c0, c1, c2, c3;
} LanePoly;

/** Wheel speed array for ego estimation */
typedef struct {
    float vx[MAX_WHEELS];
} WheelSpeedSet;

/** Generic PID container for reentrancy */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
} PID;

/* --- 1. Ego Vehicle Estimation --- */
typedef struct {
    float ego_speed;     /* longitudinal [m/s] */
    float ego_lat_speed; /* lateral [m/s] (optional) */
} EVE_State;

/**
 * update_ego_speed: Fuse IMU accel and optional wheel speeds
 * Parameters:
 *   imu_ax, imu_ay: raw accelerations [m/s^2]
 *   dt: time step [s]
 *   ws: pointer to wheel speeds (NULL if unused)
 *   ax_bias, ay_bias: calibration biases
 *   alpha: complementary filter weight (0..1)
 *   stationary_th: threshold below which speed -> 0
 *   state: in/out state struct
 */
void update_ego_speed(float imu_ax,
                      float imu_ay,
                      float dt,
                      const WheelSpeedSet *ws,
                      float ax_bias,
                      float ay_bias,
                      float alpha,
                      float stationary_th,
                      EVE_State *state);

/* --- 2. Target Selection --- */
typedef struct {
    int32_t target_id;
    float   rel_distance; /* [m] */
    float   rel_speed;    /* ego - target [m/s] */
    bool    valid;
} TS_PrecedingOut;

typedef struct {
    int32_t target_id;
    float   ttc_cross;    /* [s] */
    bool    valid;
} TS_IntersectionOut;

/**
 * target_selection_preceding: select closest object in same lane ahead
 */

void target_selection_preceding(const Object objs[],
                                uint32_t obj_count,
                                const LanePoly *lane,
                                float lane_width,
                                float lateral_margin,
                                float prediction_horizon_s,
                                float ego_speed,
                                TS_PrecedingOut *out);

/**
 * target_selection_intersection: select object likely to collide at intersection
 */
void target_selection_intersection(const Object objs[],
                                   uint32_t obj_count,
                                   const float conflict_pts[][2],
                                   uint32_t cp_count,
                                   float ttc_threshold,
                                   float lat_speed_min,
                                   float ego_speed,
                                   TS_IntersectionOut *out);

/* --- 3. Adaptive Cruise Control --- */

float pid_update(PID *pid,
                 float error,
                 float dt,
                 float output_min,
                 float output_max);

/**
 * calculate_accel_for_speed_pid: PID on speed error
 */
float calculate_accel_for_speed_pid(PID *pid,
                                    float desired_speed,
                                    float ego_speed,
                                    float dt,
                                    float accel_limit);

/**
 * calculate_accel_for_distance_pid: PID on distance error
 */
float calculate_accel_for_distance_pid(PID *pid,
                                       float rel_distance,
                                       float rel_speed,
                                       float ego_speed,
                                       float dt,
                                       float headway_time,
                                       float D_min,
                                       float accel_limit);

/**
 * acc_mode_selection: choose speed or distance mode accel
 */
float acc_mode_selection(bool preceding_valid,
                         float rel_distance,
                         float follow_distance_max,
                         float hysteresis_time,
                         float dt,
                         float accel_speed,
                         float accel_dist);

/* --- 4. Autonomous Emergency Brake --- */

typedef struct { float ttc; float warn_time; float brake_time; } AEB_Info;

enum AEB_State { AEB_STATE_NORMAL = 0, AEB_STATE_ALARM, AEB_STATE_BRAKE };

typedef struct { enum AEB_State state; float accel_cmd; } AEB_Output;

/**
 * info_calculation_for_preceding: compute TTC, warn, brake times
 */
void info_calculation_for_preceding(float rel_distance,
                                    float rel_speed,
                                    float ego_speed,
                                    float max_decel,
                                    float warn_margin,
                                    AEB_Info *out);

/**
 * info_calculation_for_intersection_crossing: compute intersection timings
 */
void info_calculation_for_intersection_crossing(float ttc_cross,
                                                float safety_ratio,
                                                float warn_margin,
                                                AEB_Info *out);

/**
 * aeb_mode_selection: state machine for NORMAL/ALARM/BRAKE
 */
void aeb_mode_selection(const AEB_Info *front_info,
                        const AEB_Info *cross_info,
                        float ego_speed,
                        float stop_threshold_kmh,
                        float max_decel,
                        float state_hold_time,
                        float dt,
                        AEB_Output *out);

/* --- 5. Lane Following Assist --- */

/** low-speed PID steering */
float calculate_steer_in_low_speed_pid(PID *pid,
                                       float lane_center_offset,
                                       float dt,
                                       float steer_limit);

/** high-speed PID steering */
float calculate_steer_in_high_speed_pid(PID *pid,
                                        float lane_center_offset,
                                        float lane_heading_error,
                                        float dt,
                                        float w_offset,
                                        float w_heading,
                                        float steer_limit);

/** select steer based on speed threshold */
float lfa_mode_selection(float ego_speed_kmh,
                         float speed_switch_kmh,
                         float steer_low,
                         float steer_high);

/* --- 6. Arbitration --- */

/**
 * arbitration: fuse driver/AEB/ACC/LFA/AES inputs
 */
void arbitration(float driver_accel_cmd,
                 float aeb_accel_cmd,
                 float acc_accel_cmd,
                 float driver_steer_torque,
                 float aes_steer_torque,
                 float lfa_steer_cmd,
                 float torque_threshold,
                 float *final_accel_cmd,
                 float *final_steer_cmd);

#endif /* ADAS_MODULE_H */
