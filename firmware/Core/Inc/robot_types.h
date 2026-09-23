#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

/* Shared between main.c and motion.c. These were previously defined
 * inside main.c, which made them unreachable from any other translation
 * unit -- types cannot be forward-declared with extern. */

enum cmdList
{
    FWD,
    REVS,
    REVL,
    REVR,
    STOP,
    TURNL,
    TURNR,
    TURN90L,
    TURN90R,
    TASK2,
    PWMTURNL,
    PWMTURNR,
    TURNRA,
    MTNTURN,
    MTNFWDUS,
    MTNAVOID,
    /* Append new commands here only -- enum ordinals are the wire contract. */
    MTNTASK2,
    /* A.5 straight approach: drive to an ultrasonic standoff while holding
     * heading with the gyro. See motorApproachUntil() in main.c. */
    APPROACHUS
};

typedef struct
{
    enum cmdList command;
    uint32_t param1Speed;
    int32_t  param2DistAngle;
    uint32_t cmdId;
} MotorCommand_t;

typedef struct
{
    enum cmdList command;
    uint32_t param1Speed;
    float    param2DistAngle;
    uint32_t cmdId;
} MotorCommandF_t;

/* --- servo positions (from the tuned firmware, do not change) -------- */
#define SERVO_CENTER     154.0f
#define SERVO_LEFT_MAX   100.6f
#define SERVO_RIGHT_MAX  208.6f

/* --- hardware layer, implemented in main.c -------------------------- */
void motorStop(void);
void motorStopA(void);
void motorStopB(void);
void motorForwardA(int pwmVal);
void motorForwardB(int pwmVal);
void motorReverseA(int pwmVal);
void motorReverseB(int pwmVal);
void setServoAngle(int pwm);

#endif /* ROBOT_TYPES_H */
