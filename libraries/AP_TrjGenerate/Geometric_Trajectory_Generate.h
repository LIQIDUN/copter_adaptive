#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>


void Trajectory_Generate_TAKEOFF_TO_ALT_AUTO(float timeInThisRun,
                                             float targetAlt,
                                             float takeofftime,
                                             Vector3f *targetPos,
                                             Vector3f *targetVel,
                                             Vector3f *targetAcc,
                                             Vector3f *targetJerk,
                                             Vector3f *targetSnap,
                                             Vector2f *targetYaw,
                                             Vector2f *targetYaw_dot,
                                             Vector2f *targetYaw_ddot);

void Trajectory_Generate_ALT_TO_LAND_AUTO(float timeInThisRun,
                                          float targetAlt,
                                          float takeofftime,
                                          Vector3f *targetPos,
                                          Vector3f *targetVel,
                                          Vector3f *targetAcc,
                                          Vector3f *targetJerk,
                                          Vector3f *targetSnap,
                                          Vector2f *targetYaw,
                                          Vector2f *targetYaw_dot,
                                          Vector2f *targetYaw_ddot);
void Trajectory_Generate_POS_AUTO(float timeInThisRun, float targetAlt,
                                  float takeofftime,
                                  float T_circle,
                                  bool *in_flight,
                                  Vector3f *targetPos,
                                  Vector3f *targetVel,
                                  Vector3f *targetAcc,
                                  Vector3f *targetJerk,
                                  Vector3f *targetSnap,
                                  Vector2f *targetYaw,
                                  Vector2f *targetYaw_dot,
                                  Vector2f *targetYaw_ddot);

void Trajectory_Generate_START_AUTO(float timeInThisRun,
                                    float finalVel,
                                    Vector3f *targetPos,
                                    Vector3f *targetVel,
                                    Vector3f *targetAcc,
                                    Vector3f *targetJerk,
                                    Vector3f *targetSnap,
                                    Vector2f *targetYaw,
                                    Vector2f *targetYaw_dot,
                                    Vector2f *targetYaw_ddot);
void Trajectory_Generate_EXIT_AUTO(float timeInThisRun,
                                   float finalVel,
                                   Vector3f *targetPos,
                                   Vector3f *targetVel,
                                   Vector3f *targetAcc,
                                   Vector3f *targetJerk,
                                   Vector3f *targetSnap,
                                   Vector2f *targetYaw,
                                   Vector2f *targetYaw_dot,
                                   Vector2f *targetYaw_ddot);

void Trajectory_Generate_CIRCLE_AUTO(float timeInThisRun,
                                     float targetAlt,
                                     float takeofftime,
                                     float r_circle,
                                     float T_circle,
                                     bool *in_flight,
                                     bool *in_trj,
                                     Vector3f *targetPos,
                                     Vector3f *targetVel,
                                     Vector3f *targetAcc,
                                     Vector3f *targetJerk,
                                     Vector3f *targetSnap,
                                     Vector2f *targetYaw,
                                     Vector2f *targetYaw_dot,
                                     Vector2f *targetYaw_ddot);

void Trajectory_Generate_EIGHT_AUTO(float timeInThisRun,
                                    float targetAlt,
                                    float takeofftime,
                                    float r_circle,
                                    float T_circle,
                                    bool *in_flight,
                                    bool *in_trj,
                                    Vector3f *targetPos,
                                    Vector3f *targetVel,
                                    Vector3f *targetAcc,
                                    Vector3f *targetJerk,
                                    Vector3f *targetSnap,
                                    Vector2f *targetYaw,
                                    Vector2f *targetYaw_dot,
                                    Vector2f *targetYaw_ddot);
void Trajectory_Generate_LSR_AUTO(float timeInThisRun,
                                    float targetAlt,
                                    float takeofftime,
                                    float r_circle,
                                    float T_circle,
                                    bool *in_flight,
                                    bool *in_trj,
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
