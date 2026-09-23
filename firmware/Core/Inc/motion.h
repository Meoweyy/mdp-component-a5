#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "motion_math.h"
#include "robot_types.h"

/* ============================ CALIBRATION ============================
 * Every empirical constant lives here. Values marked MEASURED come from
 * the existing tuned firmware. Values marked GUESS are starting points
 * and must be tuned on the bench -- see the design doc, section 6.
 * ==================================================================== */

/* --- Steering ------------------------------------------------------- */
/* MEASURED. Every legacy call site used SERVO_RIGHT_MAX + 55, which means
 * SERVO_RIGHT_MAX itself is mis-calibrated. Folded into one constant. */
#define MTN_SERVO_RIGHT_LOCK   200.0f
#define MTN_SERVO_LEFT_LOCK    112.5f
/* Reverse-arc locks. Seeded at the forward values so behaviour is unchanged,
 * but split out so REVL/REVR can be tuned without moving TURNL/TURNR. */
#define MTN_SERVO_REVL_LOCK    99.0f   /* 112.5 -> R~33.5, 106 -> R~29, 99 -> R~24: lands dy=30 with no pre-pad */
#define MTN_SERVO_REVR_LOCK    225.0f   /* 200 -> dy=44, 225 -> dy=37. 250/270 identical: servo is on its
                                         * mechanical stop at ~225, so R~29.5 is the tightest right arc. */
/* MEASURED. Asymmetric recentre offsets compensate servo backlash -- the
 * servo does not return to true centre identically from each side. */
#define MTN_SERVO_CENTRE_L     174.0f   /* SERVO_CENTER + 20.0 */
#define MTN_SERVO_CENTRE_R     152.5f   /* SERVO_CENTER -  1.5 */
#define MTN_SERVO_CENTRE_REVL  157.5f   /* SERVO_CENTER +  3.5 */

/* --- Turning -------------------------------------------------------- */
/* Ships at 0.0f so behaviour is IDENTICAL to the pre-branch firmware:
 * motorTurnF has five live legacy callers (TURN90L, TURN90R, task2Loop)
 * that were bench-calibrated against its one-tick overshoot. Raise this
 * deliberately on the bench, then re-check MTN_TURN_L90 / MTN_TURN_R90.
 * Try 2.0f first. */
#define MTN_TURN_STOP_DEG       0.0f
/* MEASURED. Calibrated 90-degree values from the legacy composites. */
#define MTN_TURN_L90           89.0f
#define MTN_TURN_R90           89.5f
/* MEASURED. Pre/post nudges in the legacy 90-degree composites. */
#define MTN_TURN_PRE_CM         3.2f
#define MTN_TURN_POST_CM        0.75f

/* --- Straight-line -------------------------------------------------- */
/* MUST default to 0.0f. At zero, mtn_straight is numerically identical to
 * the legacy motorPidForwardF. Raise slowly on the bench to enable
 * heading hold; try 20.0f first. */
#define MTN_YAW_KP              2.0f    /* servo counts per degree of heading error.
                                         * Applied to the SERVO in motorPidForward /
                                         * motorPidReverse (14 Sep). 0.0f = off. */
/* Reverse needs more gain: trailing steered wheels have less authority than
 * leading ones, so at the forward gain it corrects too slowly and drifts
 * sideways while doing so. 5-deg test at 2.0 shifted 4.5cm; tune from 3.0. */
#define MTN_YAW_KP_REV          3.0f
/* GUESS. Hard cap so a dead sensor cannot drive the car into a wall.
 * The arena is 200 cm across. */
#define MTN_STRAIGHT_MAX_CM   200.0f

/* --- Ultrasonic ----------------------------------------------------- */
/* GUESS. Abort a sensor-terminated move after this long with no valid
 * reading, rather than continuing blind. */
#define MTN_US_DEAD_MS        500u
/* GUESS. Ride through a dropout shorter than this by reusing the median;
 * beyond it, report the sensor as unavailable. Must stay BELOW
 * MTN_US_DEAD_MS so a real failure is reported before the abort window. */
#define MTN_US_STALE_MS       200u
/* GUESS. Standoff from an obstacle. Checklist A.2 places images 20-50 cm
 * from the robot's midpoint. */
#define MTN_STANDOFF_CM        25.0f

/* --- Obstacle avoidance (checklist A.5) ----------------------------- */
/* GUESS, all four. Tune as a set -- changing one changes where the robot
 * ends up. */
#define MTN_AVOID_OUT_DEG      45.0f
#define MTN_AVOID_SIDE_CM      30.0f
#define MTN_AVOID_PAST_CM      40.0f
#define MTN_AVOID_PWM          3000u
/* GUESS. Dead-reckoned final approach for mtn_avoidObstacle, used instead
 * of an ultrasonic-terminated approach so A.5 needs only encoders + gyro
 * (the ultrasonic is unverified hardware). The RPi is expected to drive the
 * robot close to the obstacle with MOTOR/FWD first (camera-guided); this is
 * just the committing nudge before the steering arc begins.
 * Set to 0.0f to skip the approach entirely -- the RPi then does 100% of
 * the positioning and AVOID is purely the go-around arc. */
#define MTN_AVOID_APPROACH_CM  10.0f
/* GUESS. Cap on how many times the along-side leg may repeat while IR still
 * reports the obstacle. Bounds a stuck-high IR sensor, whose failure mode
 * would otherwise drive the robot forward indefinitely. */
#define MTN_AVOID_MAX_REPEAT   3u

/* --- Task 2 --------------------------------------------------------- */
/* GUESS, all. Obstacles are 60-150 cm apart with 50 cm minimum clearance. */
#define MTN_T2_STANDOFF_CM     30.0f
#define MTN_T2_AROUND_DEG      90.0f
#define MTN_T2_AROUND_CM       40.0f
#define MTN_T2_RETURN_CM      120.0f
#define MTN_T2_PWM           4000u

/* --- Safety --------------------------------------------------------- */
/* GUESS. No encoder movement for this long while PWM is applied = stall. */
#define MTN_STALL_MS         1500u
/* GUESS. Absolute ceiling on any single primitive. */
#define MTN_CMD_TIMEOUT_MS  20000u

/* --- Motor limits (MEASURED, do not change) ------------------------- */
#define MTN_PWM_MAX          7199

/* ======================== END CALIBRATION =========================== */

#define MTN_LEFT   ((int8_t)-1)
#define MTN_RIGHT  ((int8_t)+1)

typedef enum {
    MTN_OK = 0,
    MTN_ERR_NO_SENSOR,
    MTN_ERR_STALL,
    MTN_ERR_TIMEOUT
} mtn_result_t;

/* --- Init ---------------------------------------------------------- */
void    mtn_init(void);           /* one-time; run before the RTOS starts */

/* --- Sensors -------------------------------------------------------- */
float   mtn_yaw(void);            /* signed heading, degrees */
void    mtn_yawReset(void);       /* zero the heading reference */
float   mtn_usDistanceCm(void);   /* median-filtered; -1.0f if unavailable */
uint8_t mtn_irLeft(void);
uint8_t mtn_irRight(void);

/* --- Primitives ----------------------------------------------------- */
/* All return 1 when complete, 0 while in progress. Pass reset=1 on the
 * first call of a new movement, 0 on every subsequent call. */

/* Travel `cm` centimetres. Negative reverses. */
uint8_t mtn_straight(float cm, uint16_t pwm, uint8_t reset);

/* Turn `deg` degrees (always positive) in direction `dir`. */
uint8_t mtn_turn(float deg, int8_t dir, uint16_t pwm, uint8_t reset);

/* Drive forward until the ultrasonic reads `standoffCm`. */
uint8_t mtn_straightUntil(float standoffCm, uint16_t pwm, uint8_t reset);

/* --- Composites ----------------------------------------------------- */
uint8_t mtn_avoidObstacle(int8_t side, uint8_t reset);
uint8_t mtn_task2(int8_t arrow1, int8_t arrow2, uint8_t reset);

/* --- Status --------------------------------------------------------- */
/* Result of the most recently completed or aborted primitive. */
mtn_result_t mtn_lastResult(void);
const char  *mtn_resultName(mtn_result_t r);

#endif /* MOTION_H */
