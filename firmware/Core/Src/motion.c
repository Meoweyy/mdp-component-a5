/* motion.c — movement policy for the SC2079 MDP robot.
 *
 * Sits on top of the hardware primitives in main.c and reaches them via
 * extern declarations. The tuned engines (motorPidForwardF, motorTurnF)
 * are reused rather than reimplemented.
 *
 * Design: docs/superpowers/specs/2026-08-31-stm32-motion-layer-design.md
 */

#include "main.h"
#include "cmsis_os.h"
#include "motion.h"
#include "motion_math.h"
#include "ir_sensor.h"
#include <math.h>

/* --- from main.c ---------------------------------------------------- */
extern volatile float    yawAngle;
extern volatile uint16_t echo;      /* ultrasonic pulse width, timer ticks */

static float        g_yawOffset  = 0.0f;
static mtn_result_t g_lastResult = MTN_OK;

/* Set when the AV_ALONG IR interlock exhausts its repeat budget. Kept
 * separate from g_lastResult because every primitive's reset clears that,
 * so a composite cannot store state there mid-manoeuvre. */
static uint8_t g_avoidInterlockFailed = 0;

/* ==================== init ==================== */

/* One-time init for the motion layer. Must run before the RTOS starts.
 * IR_Sensors_Init() configures PC6/PC9 as inputs; nothing else calls it —
 * the legacy irSensor task that used to is entirely commented out. */
void mtn_init(void)
{
    IR_Sensors_Init();
}

/* ==================== sensors ==================== */

float mtn_yaw(void)
{
    return mtn_normalizeDeg(yawAngle - g_yawOffset);
}

void mtn_yawReset(void)
{
    g_yawOffset = yawAngle;
}

float mtn_usDistanceCm(void)
{
    /* Ring buffer of the last 5 valid readings. The ultrasonic is
     * unverified hardware, so a single bad sample must not steer a
     * decision — median-filter it. A brief raw dropout is ridden through
     * by reusing the last median, but only for MTN_US_STALE_MS; past that
     * the reading is reported as unavailable (-1.0f) so callers' dead-
     * sensor aborts stay reachable. */
    static float    ring[5]        = {0};
    static uint8_t  filled         = 0;
    static uint8_t  idx            = 0;
    static uint32_t lastValidTick  = 0;

    float cm = mtn_echoToCm(echo);
    if (cm < 0.0f) {
        /* Ride through a brief dropout with the last good median, but only
         * briefly — returning a stale value forever would make the callers'
         * MTN_ERR_NO_SENSOR abort unreachable. filled==0 on a cold start
         * also forces -1.0f here, so lastValidTick==0 is never trusted. */
        if (filled >= 5u && (HAL_GetTick() - lastValidTick) < MTN_US_STALE_MS)
            return mtn_median5(ring);
        return -1.0f;
    }

    lastValidTick = HAL_GetTick();

    ring[idx] = cm;
    idx = (uint8_t)((idx + 1u) % 5u);
    if (filled < 5u) filled++;

    if (filled < 5u) return cm;   /* not enough history to filter yet */
    return mtn_median5(ring);
}

uint8_t mtn_irLeft(void)  { return IR_LeftDetected(); }
uint8_t mtn_irRight(void) { return IR_RightDetected(); }

/* ==================== status ==================== */

mtn_result_t mtn_lastResult(void) { return g_lastResult; }

const char *mtn_resultName(mtn_result_t r)
{
    switch (r) {
        case MTN_OK:            return "OK";
        case MTN_ERR_NO_SENSOR: return "NO_SENSOR";
        case MTN_ERR_STALL:     return "STALL";
        case MTN_ERR_TIMEOUT:   return "TIMEOUT";
        default:                return "UNKNOWN";
    }
}

/* ==================== turn ==================== */

/* from main.c — the tuned turn engine. Closes the loop on the gyro,
 * drives one wheel while braking the other to tighten the arc. */
extern uint8_t motorTurnF(MotorCommandF_t cmd, uint8_t isStateChanged);

uint8_t mtn_turn(float deg, int8_t dir, uint16_t pwm, uint8_t reset)
{
    static MotorCommandF_t sub;
    static uint32_t        startTick = 0;

    if (reset) {
        sub.cmdId           = 0;
        sub.command         = (dir == MTN_LEFT) ? TURNL : TURNR;
        sub.param1Speed     = (pwm > MTN_PWM_MAX) ? MTN_PWM_MAX : pwm;
        sub.param2DistAngle = fabsf(deg);
        startTick           = HAL_GetTick();
        g_lastResult        = MTN_OK;
    }

    if (HAL_GetTick() - startTick > MTN_CMD_TIMEOUT_MS) {
        motorStop();
        setServoAngle(SERVO_CENTER);
        g_lastResult = MTN_ERR_TIMEOUT;
        return 1;
    }

    return motorTurnF(sub, reset);
}

/* ==================== straight ==================== */

/* from main.c — the bench-tuned float-precision straight-line engines.
 * motorPidForwardF holds a straight line off the encoder differential;
 * the optional absolute-heading term it now carries reads mtnYawTarget. */
extern uint8_t        motorPidForwardF(MotorCommandF_t cmd, uint8_t isStateChanged);
extern uint8_t        motorPidReverseF(MotorCommandF_t cmd, uint8_t isStateChanged);
extern volatile float mtnYawTarget;

uint8_t mtn_straight(float cm, uint16_t pwm, uint8_t reset)
{
    static MotorCommandF_t sub;
    static uint32_t        startTick = 0;
    static uint8_t         reverse   = 0;

    if (reset) {
        reverse             = (cm < 0.0f) ? 1u : 0u;
        sub.cmdId           = 0;
        sub.command         = reverse ? REVS : FWD;
        sub.param1Speed     = (pwm > MTN_PWM_MAX) ? MTN_PWM_MAX : pwm;
        sub.param2DistAngle = (fabsf(cm) > MTN_STRAIGHT_MAX_CM)
                              ? MTN_STRAIGHT_MAX_CM : fabsf(cm);
        mtnYawTarget        = yawAngle;   /* hold the heading we start on */
        startTick           = HAL_GetTick();
        g_lastResult        = MTN_OK;
    }

    if (HAL_GetTick() - startTick > MTN_CMD_TIMEOUT_MS) {
        motorStop();
        g_lastResult = MTN_ERR_TIMEOUT;
        return 1;
    }

    if (reverse) return motorPidReverseF(sub, reset);
    return motorPidForwardF(sub, reset);
}

/* ==================== sensor-terminated straight ==================== */

uint8_t mtn_straightUntil(float standoffCm, uint16_t pwm, uint8_t reset)
{
    static uint32_t startTick    = 0;
    static uint32_t lastGoodTick = 0;
    float           cm;
    uint16_t        speed;

    if (reset) {
        setServoAngle(SERVO_CENTER);
        startTick    = HAL_GetTick();
        lastGoodTick = startTick;
        g_lastResult = MTN_OK;
        /* Not a control input: this function drives both wheels at equal
         * PWM and applies no heading correction (that lives in
         * motorPidForwardF, which this path deliberately avoids on the
         * bench-unverified ultrasonic sensor). The reset simply
         * establishes a fresh heading reference the caller can read via
         * mtn_yaw() after the move completes. */
        mtn_yawReset();
    }

    if (HAL_GetTick() - startTick > MTN_CMD_TIMEOUT_MS) {
        motorStop();
        g_lastResult = MTN_ERR_TIMEOUT;
        return 1;
    }

    cm = mtn_usDistanceCm();

    if (cm < 0.0f) {
        /* No valid reading. Keep going briefly — a single dropout is
         * normal — but never drive blind for long. */
        if (HAL_GetTick() - lastGoodTick > MTN_US_DEAD_MS) {
            motorStop();
            g_lastResult = MTN_ERR_NO_SENSOR;
            return 1;
        }
        return 0;
    }

    lastGoodTick = HAL_GetTick();

    if (cm <= standoffCm) {
        motorStop();
        g_lastResult = MTN_OK;
        return 1;
    }

    /* Decelerate on approach: a crude open-loop step-down over four
     * distance bands, not a closed-loop ramp. */
    speed = (pwm > MTN_PWM_MAX) ? MTN_PWM_MAX : pwm;
    {
        float remaining = cm - standoffCm;
        if      (remaining <  5.0f && speed >  800u) speed =  800u;
        else if (remaining < 10.0f && speed > 1200u) speed = 1200u;
        else if (remaining < 20.0f && speed > 4000u) speed = 4000u;
        else if (remaining < 50.0f && speed > 5000u) speed = 5000u;
    }

    motorForwardA(speed);
    motorForwardB(speed);
    return 0;
}

/* ==================== obstacle avoidance (A.5) ==================== */

uint8_t mtn_avoidObstacle(int8_t side, uint8_t reset)
{
    /* side = the side the OBSTACLE is on (MTN_LEFT or MTN_RIGHT). The robot
     * first steers away toward -side, drives past, then arcs back. The
     * AV_ALONG interlock polls the IR sensor on `side` (the obstacle side). */
    static enum { AV_APPROACH, AV_OUT, AV_ALONG, AV_BACK,
                  AV_PAST, AV_SQUARE_A, AV_SQUARE_B, AV_DONE } phase = AV_APPROACH;
    static uint8_t sub = 1;
    static uint8_t repeat = 0;   /* along-side leg repeats while IR still fires */

    if (reset) {
        phase = AV_APPROACH; sub = 1; repeat = 0;
        g_lastResult = MTN_OK; g_avoidInterlockFailed = 0;
    }

    switch (phase) {

    case AV_APPROACH:
        /* Ultrasonic-free A.5: dead-reckoned final approach instead of an
         * ultrasonic-terminated one, so the manoeuvre needs only encoders
         * + gyro. The RPi drives the bulk of the approach with MOTOR/FWD;
         * this just commits a short fixed nudge before the steering arc.
         * To restore the ultrasonic approach, comment the mtn_straight line
         * back to the mtn_straightUntil line below and drop the skip guard. */
        if (MTN_AVOID_APPROACH_CM <= 0.0f) {        /* RPi did all positioning */
            phase = AV_OUT; sub = 1;
            break;
        }
        /* if (mtn_straightUntil(MTN_STANDOFF_CM, MTN_AVOID_PWM, sub)) { */
        if (mtn_straight(MTN_AVOID_APPROACH_CM, MTN_AVOID_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;   /* propagate failure (e.g. stall/timeout) */
            phase = AV_OUT; sub = 1;
        } else sub = 0;
        break;

    case AV_OUT:
        if (mtn_turn(MTN_AVOID_OUT_DEG, (int8_t)-side, MTN_AVOID_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;   /* propagate failure */
            phase = AV_ALONG; sub = 1; repeat = 0;   /* fresh along-side leg */
        } else sub = 0;
        break;

    case AV_ALONG:
        if (mtn_straight(MTN_AVOID_SIDE_CM, MTN_AVOID_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;   /* propagate failure */
            /* Safety interlock: if the obstacle is still alongside, travel
             * further rather than turning back into it. Bounded so a
             * stuck-high IR sensor cannot drive the robot forward forever;
             * on exhaustion we proceed anyway (completing the arc leaves a
             * predictable heading, aborting does not). */
            uint8_t stillThere = (side == MTN_RIGHT) ? mtn_irRight() : mtn_irLeft();
            if (stillThere && repeat < MTN_AVOID_MAX_REPEAT) {
                repeat++;
                sub = 1;
                break;                              /* repeat this phase */
            }
            if (stillThere) {
                g_avoidInterlockFailed = 1;         /* interlock exhausted */
            }
            phase = AV_BACK; sub = 1;
        } else sub = 0;
        break;

    case AV_BACK:
        if (mtn_turn(MTN_AVOID_OUT_DEG, side, MTN_AVOID_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;   /* propagate failure */
            phase = AV_PAST; sub = 1;
        } else sub = 0;
        break;

    case AV_PAST:
        if (mtn_straight(MTN_AVOID_PAST_CM, MTN_AVOID_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;   /* propagate failure */
            phase = AV_SQUARE_A; sub = 1;
        } else sub = 0;
        break;

    case AV_SQUARE_A:
        if (mtn_turn(MTN_AVOID_OUT_DEG, side, MTN_AVOID_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;   /* propagate failure */
            phase = AV_SQUARE_B; sub = 1;
        } else sub = 0;
        break;

    case AV_SQUARE_B:
        if (mtn_turn(MTN_AVOID_OUT_DEG, (int8_t)-side, MTN_AVOID_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;   /* propagate failure */
            phase = AV_DONE; sub = 1;
        } else sub = 0;
        break;

    case AV_DONE:
    default:
        motorStop();
        setServoAngle(SERVO_CENTER);
        if (g_avoidInterlockFailed && g_lastResult == MTN_OK)
            g_lastResult = MTN_ERR_NO_SENSOR;   /* stuck IR: re-assert after primitives done */
        phase = AV_APPROACH;
        return 1;
    }

    return 0;
}

/* ==================== Task 2: fastest car ==================== */

uint8_t mtn_task2(int8_t arrow1, int8_t arrow2, uint8_t reset)
{
    static enum { T2_APPROACH_1, T2_AROUND_1, T2_APPROACH_2, T2_AROUND_2,
                  T2_RETURN, T2_PARK, T2_DONE } phase = T2_APPROACH_1;
    static uint8_t sub  = 1;
    static uint8_t leg  = 0;   /* sub-step within an around-the-obstacle set */

    if (reset) { phase = T2_APPROACH_1; sub = 1; leg = 0; g_lastResult = MTN_OK; }

    switch (phase) {

    case T2_APPROACH_1:
        if (mtn_straightUntil(MTN_T2_STANDOFF_CM, MTN_T2_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;
            phase = T2_AROUND_1; sub = 1; leg = 0;
        } else sub = 0;
        break;

    case T2_AROUND_1:
        /* Arc around obstacle 1 in the direction its arrow indicates:
         * out, across, back. */
        if (leg == 0) {
            if (mtn_turn(MTN_T2_AROUND_DEG, arrow1, MTN_T2_PWM, sub)) {
                if (g_lastResult != MTN_OK) return 1;
                leg = 1; sub = 1;
            } else sub = 0;
        } else if (leg == 1) {
            if (mtn_straight(MTN_T2_AROUND_CM, MTN_T2_PWM, sub)) {
                if (g_lastResult != MTN_OK) return 1;
                leg = 2; sub = 1;
            } else sub = 0;
        } else {
            if (mtn_turn(MTN_T2_AROUND_DEG, (int8_t)-arrow1, MTN_T2_PWM, sub)) {
                if (g_lastResult != MTN_OK) return 1;
                phase = T2_APPROACH_2; sub = 1; leg = 0;
            } else sub = 0;
        }
        break;

    case T2_APPROACH_2:
        if (mtn_straightUntil(MTN_T2_STANDOFF_CM, MTN_T2_PWM, sub)) {
            if (g_lastResult != MTN_OK) return 1;
            phase = T2_AROUND_2; sub = 1; leg = 0;
        } else sub = 0;
        break;

    case T2_AROUND_2:
        if (leg == 0) {
            if (mtn_turn(MTN_T2_AROUND_DEG, arrow2, MTN_T2_PWM, sub)) {
                if (g_lastResult != MTN_OK) return 1;
                leg = 1; sub = 1;
            } else sub = 0;
        } else if (leg == 1) {
            if (mtn_straight(MTN_T2_AROUND_CM, MTN_T2_PWM, sub)) {
                if (g_lastResult != MTN_OK) return 1;
                leg = 2; sub = 1;
            } else sub = 0;
        } else {
            if (mtn_turn(MTN_T2_AROUND_DEG, (int8_t)-arrow2, MTN_T2_PWM, sub)) {
                if (g_lastResult != MTN_OK) return 1;
                phase = T2_RETURN; sub = 1; leg = 0;
            } else sub = 0;
        }
        break;

    case T2_RETURN:
        /* U-turn back toward the carpark. */
        if (leg == 0) {
            if (mtn_turn(180.0f, arrow2, MTN_T2_PWM, sub)) {
                if (g_lastResult != MTN_OK) return 1;
                leg = 1; sub = 1;
            } else sub = 0;
        } else {
            if (mtn_straight(MTN_T2_RETURN_CM, MTN_T2_PWM, sub)) {
                if (g_lastResult != MTN_OK) return 1;
                phase = T2_PARK; sub = 1; leg = 0;
            } else sub = 0;
        }
        break;

    case T2_PARK:
        /* Creep in until the carpark wall is in range. If the ultrasonic
         * is unavailable the return leg above has already covered the
         * measured distance, so treat NO_SENSOR as parked rather than
         * failing the run. */
        if (mtn_straightUntil(MTN_T2_STANDOFF_CM, 1200u, sub)) {
            if (g_lastResult == MTN_ERR_NO_SENSOR) g_lastResult = MTN_OK;
            if (g_lastResult != MTN_OK) return 1;   /* propagate e.g. TIMEOUT */
            phase = T2_DONE; sub = 1;
        } else sub = 0;
        break;

    case T2_DONE:
    default:
        motorStop();
        setServoAngle(SERVO_CENTER);
        phase = T2_APPROACH_1;
        return 1;
    }

    return 0;
}
