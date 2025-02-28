#include <AP_Math/AP_Math.h>
#include <AP_Math/matrixN.h>
#include <AP_Math/ftype.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include "Copter.h"
void Trajectory_Generate_START(float timeInThisRun,
                               Vector3f *targetPos,
                               Vector3f *targetVel,
                               Vector3f *targetAcc,
                               Vector3f *targetJerk,
                               Vector3f *targetSnap,
                               Vector2f *targetYaw,
                               Vector2f *targetYaw_dot,
                               Vector2f *targetYaw_ddot);
void Trajectory_Generate_EXIT(float timeInThisRun,
                              float v_circle,
                              Vector3f *targetPos,
                              Vector3f *targetVel,
                              Vector3f *targetAcc,
                              Vector3f *targetJerk,
                              Vector3f *targetSnap,
                              Vector2f *targetYaw,
                              Vector2f *targetYaw_dot,
                              Vector2f *targetYaw_ddot);

void Trajectory_Generate_LINE(float timeInThisRun,
                              Vector3f *targetPos,
                              Vector3f *targetVel,
                              Vector3f *targetAcc,
                              Vector3f *targetJerk,
                              Vector3f *targetSnap,
                              Vector2f *targetYaw,
                              Vector2f *targetYaw_dot,
                              Vector2f *targetYaw_ddot);

void Trajectory_Generate_SINWAVE(float timeInThisRun,
                                 Vector3f *targetPos,
                                 Vector3f *targetVel,
                                 Vector3f *targetAcc,
                                 Vector3f *targetJerk,
                                 Vector3f *targetSnap,
                                 Vector2f *targetYaw,
                                 Vector2f *targetYaw_dot,
                                 Vector2f *targetYaw_ddot);

void Trajectory_Generate_BIGSINWAVE(float timeInThisRun,
                                    Vector3f *targetPos,
                                    Vector3f *targetVel,
                                    Vector3f *targetAcc,
                                    Vector3f *targetJerk,
                                    Vector3f *targetSnap,
                                    Vector2f *targetYaw,
                                    Vector2f *targetYaw_dot,
                                    Vector2f *targetYaw_ddot);

void Trajectory_Generate_CIRCLE(float timeInThisRun,
                                float r_circle,
                                float T_circle,
                                Vector3f *targetPos,
                                Vector3f *targetVel,
                                Vector3f *targetAcc,
                                Vector3f *targetJerk,
                                Vector3f *targetSnap,
                                Vector2f *targetYaw,
                                Vector2f *targetYaw_dot,
                                Vector2f *targetYaw_ddot);

void Trajectory_Generate_EIGHT(float timeInThisRun,
                               float r_circle,
                               float T_circle,
                               Vector3f *targetPos,
                               Vector3f *targetVel,
                               Vector3f *targetAcc,
                               Vector3f *targetJerk,
                               Vector3f *targetSnap,
                               Vector2f *targetYaw,
                               Vector2f *targetYaw_dot,
                               Vector2f *targetYaw_ddot);

void Trajectory_Generate_POS(
    Vector3f *targetPos,
    Vector3f *targetVel,
    Vector3f *targetAcc,
    Vector3f *targetJerk,
    Vector3f *targetSnap,
    Vector2f *targetYaw,
    Vector2f *targetYaw_dot,
    Vector2f *targetYaw_ddot);

float polyEval(float polyCoef[], float x, int N);
float polyDiffEval(float polyDiffCoef[], float x, int N);
float polyDiff2Eval(float polyDiffDiffCoef[], float x, int N);
float polyDiff3Eval(float polyDiffDiffCoef[], float x, int N);
float polyDiff4Eval(float polyDiffDiffCoef[], float x, int N);
