#include "Geometric_Trajectory_Generate.h"
#include "mode.h"

// 实现起飞到悬停在targetalt高度，时间从0开始计算，位置零点(0,0,0)测试可用
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
                                             Vector2f *targetYaw_ddot)
{
    // NED a=0.08
    // const float acc_climb = 0.08; // 加速度0.08

    // 加速时间
    float time_climb = takeofftime / 2;
    // time_climb = sqrtf(targetAlt / acc_climb); // 加速时间
    float acc_climb = targetAlt / (time_climb * time_climb);

    if (timeInThisRun >= 0 && timeInThisRun <= time_climb)
    {
        float PolyCoef[8] = {0, 0, 0, 0, 0, acc_climb / 2, 0, 0}; // 加速度0.08 +方向

        {
            *targetPos = (Vector3f){0, 0, -polyEval(PolyCoef, timeInThisRun, 8)}; //-方向

            *targetVel = (Vector3f){0, 0, -polyDiffEval(PolyCoef, timeInThisRun, 8)};

            *targetAcc = (Vector3f){0, 0, -polyDiff2Eval(PolyCoef, timeInThisRun, 8)};

            *targetJerk = (Vector3f){0, 0, -polyDiff3Eval(PolyCoef, timeInThisRun, 8)};

            *targetSnap = (Vector3f){0, 0, -polyDiff4Eval(PolyCoef, timeInThisRun, 8)};

            *targetYaw = (Vector2f){1, 0};
            *targetYaw_dot = (Vector2f){0, 0};
            *targetYaw_ddot = (Vector2f){0, 0};
        }
    }
    else if (timeInThisRun > time_climb && timeInThisRun <= time_climb * 2)
    {
        float PolyCoef[8] = {0, 0, 0, 0, 0, -acc_climb / 2, acc_climb * time_climb, targetAlt / 2}; // 加速度0.08 +  速度0.4 -方向

        {
            float time_decrease = timeInThisRun - time_climb;                     // 减速时间
            *targetPos = (Vector3f){0, 0, -polyEval(PolyCoef, time_decrease, 8)}; //-方向

            *targetVel = (Vector3f){0, 0, -polyDiffEval(PolyCoef, time_decrease, 8)};

            *targetAcc = (Vector3f){0, 0, -polyDiff2Eval(PolyCoef, time_decrease, 8)};

            *targetJerk = (Vector3f){0, 0, -polyDiff3Eval(PolyCoef, time_decrease, 8)};

            *targetSnap = (Vector3f){0, 0, -polyDiff4Eval(PolyCoef, time_decrease, 8)};

            *targetYaw = (Vector2f){1, 0};
            *targetYaw_dot = (Vector2f){0, 0};
            *targetYaw_ddot = (Vector2f){0, 0};
        }
    }
    else
    {

        *targetPos = (Vector3f){0, 0, -targetAlt};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

// 实现targetalt高度到降落0高度，时间从0开始计算，位置零点(0,0,-targAlt),测试可用
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
                                          Vector2f *targetYaw_ddot)
{
    // NED a=0.08
    // const float acc_climb = 0.08; // 加速度0.08

    // // 加速时间
    // float time_climb;
    // time_climb = sqrtf(targetAlt / acc_climb); // 加速时间

    // 加速时间
    float time_climb = takeofftime / 2;
    // time_climb = sqrtf(targetAlt / acc_climb); // 加速时间
    float acc_climb = targetAlt / (time_climb * time_climb);

    if (timeInThisRun >= 0 && timeInThisRun <= time_climb)
    {
        float PolyCoef[8] = {0, 0, 0, 0, 0, -acc_climb / 2, 0, targetAlt}; // 加速度0.08 -方向

        {
            float time_decrease = timeInThisRun;
            *targetPos = (Vector3f){0, 0, -polyEval(PolyCoef, time_decrease, 8)}; //-方向

            *targetVel = (Vector3f){0, 0, -polyDiffEval(PolyCoef, time_decrease, 8)};

            *targetAcc = (Vector3f){0, 0, -polyDiff2Eval(PolyCoef, time_decrease, 8)};

            *targetJerk = (Vector3f){0, 0, -polyDiff3Eval(PolyCoef, time_decrease, 8)};

            *targetSnap = (Vector3f){0, 0, -polyDiff4Eval(PolyCoef, time_decrease, 8)};

            *targetYaw = (Vector2f){1, 0};
            *targetYaw_dot = (Vector2f){0, 0};
            *targetYaw_ddot = (Vector2f){0, 0};
        }
    }
    else if (timeInThisRun > time_climb && timeInThisRun <= time_climb * 2)
    {
        float PolyCoef[8] = {0, 0, 0, 0, 0, acc_climb / 2, -acc_climb * time_climb, targetAlt / 2}; // 加速度0.08 +  速度0.4 -方向

        {
            float time_decrease = timeInThisRun - time_climb;
            *targetPos = (Vector3f){0, 0, -polyEval(PolyCoef, time_decrease, 8)}; //-方向

            *targetVel = (Vector3f){0, 0, -polyDiffEval(PolyCoef, time_decrease, 8)};

            *targetAcc = (Vector3f){0, 0, -polyDiff2Eval(PolyCoef, time_decrease, 8)};

            *targetJerk = (Vector3f){0, 0, -polyDiff3Eval(PolyCoef, time_decrease, 8)};

            *targetSnap = (Vector3f){0, 0, -polyDiff4Eval(PolyCoef, time_decrease, 8)};

            *targetYaw = (Vector2f){1, 0};
            *targetYaw_dot = (Vector2f){0, 0};
            *targetYaw_ddot = (Vector2f){0, 0};
        }
    }
    else if (timeInThisRun > time_climb * 2)
    {
        *targetPos = (Vector3f){0, 0, 0};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

// 全自动起飞2m高度，悬停10s，降落，时间从0开始计算，仿真可用
void Trajectory_Generate_POS_AUTO(float timeInThisRun,
                                  float targetAlt,
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
                                  Vector2f *targetYaw_ddot)
{
    // NED a=0.08
    // const float acc_climb = 0.08; // 加速度0.08

    // 加速时间
    float time_takeoff = takeofftime;
    // time_takeoff = 2 * sqrtf(targetAlt / acc_climb); // 加速时间，起飞全过程
    // float acc_climb = 4 * targetAlt / (time_takeoff * time_takeoff);

    float time_land = time_takeoff;
    // 悬停时间
    const float time_in_pos = T_circle;

    if (timeInThisRun >= 0 && timeInThisRun <= time_takeoff)
    {
        *in_flight = 0;
        Trajectory_Generate_TAKEOFF_TO_ALT_AUTO(timeInThisRun,
                                                targetAlt,
                                                time_takeoff,
                                                targetPos,
                                                targetVel,
                                                targetAcc,
                                                targetJerk,
                                                targetSnap,
                                                targetYaw,
                                                targetYaw_dot,
                                                targetYaw_ddot);
    }

    else if (timeInThisRun > time_takeoff && timeInThisRun <= time_takeoff + time_in_pos)
    {
        // 实现悬停

        *in_flight = 1;
        *targetPos = (Vector3f){0, 0, -targetAlt};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
    else if (timeInThisRun > time_takeoff + time_in_pos && timeInThisRun <= time_takeoff + time_in_pos + time_land)
    {
        *in_flight = 0;
        float time_land_operate = timeInThisRun - (time_takeoff + time_in_pos);
        Trajectory_Generate_ALT_TO_LAND_AUTO(time_land_operate,
                                             targetAlt,
                                             time_takeoff,
                                             targetPos,
                                             targetVel,
                                             targetAcc,
                                             targetJerk,
                                             targetSnap,
                                             targetYaw,
                                             targetYaw_dot,
                                             targetYaw_ddot);
    }
    else
    {
        *in_flight = 0;
        *targetPos = (Vector3f){0, 0, 0};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

// 水平加速度0.5 x+方向,时间从0开始计算，位置零点(0,0,0),仿真可用
void Trajectory_Generate_START_AUTO(float timeInThisRun,
                                    float finalVel,
                                    Vector3f *targetPos,
                                    Vector3f *targetVel,
                                    Vector3f *targetAcc,
                                    Vector3f *targetJerk,
                                    Vector3f *targetSnap,
                                    Vector2f *targetYaw,
                                    Vector2f *targetYaw_dot,
                                    Vector2f *targetYaw_ddot)
{
    const float acc_start = 0.5;                              // 加速度
    float PolyCoef[8] = {0, 0, 0, 0, 0, acc_start / 2, 0, 0}; // 加速度0.5 x+方向
    float acc_time = finalVel / acc_start;
    if (timeInThisRun >= 0 && timeInThisRun <= acc_time)
    {
        *targetPos = (Vector3f){polyEval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetVel = (Vector3f){polyDiffEval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetAcc = (Vector3f){polyDiff2Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetJerk = (Vector3f){polyDiff3Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetSnap = (Vector3f){polyDiff4Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
    else
    {
        float final_pos;
        final_pos = 0.5 * acc_start * acc_time * acc_time;
        *targetPos = (Vector3f){final_pos, 0, 0};

        *targetVel = (Vector3f){finalVel, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}
// 水平加速度0.5 x-方向,时间从0开始计算，位置零点(0,0,0),初始速度finalvel,仿真可用
void Trajectory_Generate_EXIT_AUTO(float timeInThisRun,
                                   float finalVel,
                                   Vector3f *targetPos,
                                   Vector3f *targetVel,
                                   Vector3f *targetAcc,
                                   Vector3f *targetJerk,
                                   Vector3f *targetSnap,
                                   Vector2f *targetYaw,
                                   Vector2f *targetYaw_dot,
                                   Vector2f *targetYaw_ddot)
{
    const float acc_start = 0.5;                                      // 加速度
    float PolyCoef[8] = {0, 0, 0, 0, 0, -acc_start / 2, finalVel, 0}; // 加速度0.5 x-方向
    float acc_time = finalVel / acc_start;
    if (timeInThisRun >= 0 && timeInThisRun <= acc_time)
    {
        *targetPos = (Vector3f){polyEval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetVel = (Vector3f){polyDiffEval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetAcc = (Vector3f){polyDiff2Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetJerk = (Vector3f){polyDiff3Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetSnap = (Vector3f){polyDiff4Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
    else
    {
        float final_pos;
        final_pos = 0.5 * acc_start * acc_time * acc_time;
        *targetPos = (Vector3f){final_pos, 0, 0};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}
// 时间从0开始计算，从0爬到targetalt，进行圆周飞行，降落，仿真可用
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
                                     Vector2f *targetYaw_ddot)
{
    // 起飞段时间计算
    // const float acc_climb = 0.08; // 加速度0.08// NED a=0.08

    float time_takeoff = takeofftime; // 起飞时间
    // time_takeoff = 2 * sqrtf(targetAlt / acc_climb); // 加速时间，起飞全过程
    // float acc_climb = 4 * targetAlt / (time_takeoff * time_takeoff);

    float time_land = time_takeoff;

    // 加速段时间和最终速度计算
    float v_circle = 2 * M_PI * r_circle / T_circle;
    const float acc_start = 0.5; // 加速度
    float acc_time = v_circle / acc_start;
    float acc_range = 0.5 * acc_start * acc_time * acc_time;

    if (timeInThisRun >= 0 && timeInThisRun < time_takeoff)
    { // 起飞
        *in_flight = 0;
        *in_trj = 0;
        Trajectory_Generate_TAKEOFF_TO_ALT_AUTO(timeInThisRun,
                                                targetAlt,
                                                time_takeoff,
                                                targetPos,
                                                targetVel,
                                                targetAcc,
                                                targetJerk,
                                                targetSnap,
                                                targetYaw,
                                                targetYaw_dot,
                                                targetYaw_ddot);
    }
    else if (timeInThisRun >= time_takeoff && timeInThisRun < time_takeoff + acc_time)
    { // 水平加速
        *in_flight = 1;
        *in_trj = 0;
        float time_in_acc = timeInThisRun - time_takeoff;
        Trajectory_Generate_START_AUTO(time_in_acc,
                                       v_circle,
                                       targetPos,
                                       targetVel,
                                       targetAcc,
                                       targetJerk,
                                       targetSnap,
                                       targetYaw,
                                       targetYaw_dot,
                                       targetYaw_ddot);
        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
    }
    else if (timeInThisRun >= time_takeoff + acc_time && timeInThisRun < time_takeoff + acc_time + T_circle)
    { // 圆周
        *in_flight = 1;
        *in_trj = 1;
        float w_circle = 2 * M_PI / T_circle;
        float time_in_circle;
        time_in_circle = timeInThisRun - (time_takeoff + acc_time);

        *targetPos = (Vector3f){sinf(w_circle * time_in_circle) * r_circle, -r_circle + r_circle * cosf(w_circle * time_in_circle), 0};

        *targetVel = (Vector3f){cosf(w_circle * time_in_circle) * r_circle * w_circle, -sinf(w_circle * time_in_circle) * r_circle * w_circle, 0};

        *targetAcc = (Vector3f){-sinf(w_circle * time_in_circle) * r_circle * w_circle * w_circle, -cosf(w_circle * time_in_circle) * r_circle * w_circle * w_circle, 0};

        *targetJerk = (Vector3f){-cosf(w_circle * time_in_circle) * r_circle * w_circle * w_circle * w_circle, sinf(w_circle * time_in_circle) * r_circle * w_circle * w_circle * w_circle, 0};

        *targetSnap = (Vector3f){sinf(w_circle * time_in_circle) * r_circle * w_circle * w_circle * w_circle * w_circle, cosf(w_circle * time_in_circle) * r_circle * w_circle * w_circle * w_circle * w_circle, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};

        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle && timeInThisRun < time_takeoff + acc_time + T_circle + acc_time)
    { // 水平减速
        *in_flight = 1;
        *in_trj = 0;
        float time_in_acc = timeInThisRun - (time_takeoff + acc_time + T_circle);
        Trajectory_Generate_EXIT_AUTO(time_in_acc,
                                      v_circle,
                                      targetPos,
                                      targetVel,
                                      targetAcc,
                                      targetJerk,
                                      targetSnap,
                                      targetYaw,
                                      targetYaw_dot,
                                      targetYaw_ddot);
        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle + acc_time && timeInThisRun < time_takeoff + acc_time + T_circle + acc_time + time_land)
    { // 降落
        *in_flight = 0;
        *in_trj = 0;
        float time_in_land = timeInThisRun - (time_takeoff + acc_time + T_circle + acc_time);
        Trajectory_Generate_ALT_TO_LAND_AUTO(time_in_land,
                                             targetAlt,
                                             time_takeoff,
                                             targetPos,
                                             targetVel,
                                             targetAcc,
                                             targetJerk,
                                             targetSnap,
                                             targetYaw,
                                             targetYaw_dot,
                                             targetYaw_ddot);
        // Trajectory_Generate_ALT_TO_LAND_AUTO初始高度-targetALT，不需要加这一句了
        //  *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};

        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle + acc_time + time_land)
    {
        // 停在地面2*acc_range
        *in_flight = 0;
        *in_trj = 0;
        *targetPos = (Vector3f){acc_range + acc_range, 0, 0};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}
// 时间从0开始计算，从0爬到targetalt，进行8飞行，降落，仿真可用
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
                                    Vector2f *targetYaw_ddot)
{

    // 起飞段时间计算
    // const float acc_climb = 0.08; // 加速度0.08// NED a=0.08

    float time_takeoff = takeofftime; // 起飞时间
    // time_takeoff = 2 * sqrtf(targetAlt / acc_climb); // 加速时间，起飞全过程
    // float acc_climb = 4 * targetAlt / (time_takeoff * time_takeoff);

    float time_land = time_takeoff;

    // 加速段时间和最终速度计算
    float v_circle = 2 * M_PI * r_circle / T_circle;
    const float acc_start = 0.5; // 加速度
    float acc_time = v_circle / acc_start;
    float acc_range = 0.5 * acc_start * acc_time * acc_time;

    if (timeInThisRun >= 0 && timeInThisRun < time_takeoff)
    { // 起飞
        *in_flight = 0;
        *in_trj = 0;
        Trajectory_Generate_TAKEOFF_TO_ALT_AUTO(timeInThisRun,
                                                targetAlt,
                                                time_takeoff,
                                                targetPos,
                                                targetVel,
                                                targetAcc,
                                                targetJerk,
                                                targetSnap,
                                                targetYaw,
                                                targetYaw_dot,
                                                targetYaw_ddot);
    }
    else if (timeInThisRun >= time_takeoff && timeInThisRun < time_takeoff + acc_time)
    { // 水平加速
        *in_flight = 1;
        *in_trj = 0;
        float time_in_acc = timeInThisRun - time_takeoff;
        Trajectory_Generate_START_AUTO(time_in_acc,
                                       v_circle,
                                       targetPos,
                                       targetVel,
                                       targetAcc,
                                       targetJerk,
                                       targetSnap,
                                       targetYaw,
                                       targetYaw_dot,
                                       targetYaw_ddot);
        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
    }
    else if (timeInThisRun >= time_takeoff + acc_time && timeInThisRun < time_takeoff + acc_time + T_circle + T_circle)
    {
        // 圆周
        *in_flight = 1;
        *in_trj = 1;
        float w_circle = 2 * M_PI / T_circle;
        float time_in_circle;
        time_in_circle = timeInThisRun - (time_takeoff + acc_time);

        int8_t left_or_right = 0; // left  1  right  -1
        if (sinf(w_circle * time_in_circle * 0.5) >= 0)
        {
            left_or_right = 1;
        }
        else if (sinf(w_circle * time_in_circle * 0.5) < 0)
        {
            left_or_right = -1;
        }

        *targetPos = (Vector3f){sinf(w_circle * time_in_circle) * r_circle, left_or_right * (-r_circle + r_circle * cosf(w_circle * time_in_circle)), 0};

        *targetVel = (Vector3f){cosf(w_circle * time_in_circle) * r_circle * w_circle, left_or_right * (-sinf(w_circle * time_in_circle) * r_circle * w_circle), 0};

        *targetAcc = (Vector3f){-sinf(w_circle * time_in_circle) * r_circle * w_circle * w_circle, left_or_right * (-cosf(w_circle * time_in_circle) * r_circle * w_circle * w_circle), 0};

        *targetJerk = (Vector3f){-cosf(w_circle * time_in_circle) * r_circle * w_circle * w_circle * w_circle, left_or_right * (sinf(w_circle * time_in_circle) * r_circle * w_circle * w_circle * w_circle), 0};

        *targetSnap = (Vector3f){sinf(w_circle * time_in_circle) * r_circle * w_circle * w_circle * w_circle * w_circle, left_or_right * (cosf(w_circle * time_in_circle) * r_circle * w_circle * w_circle * w_circle * w_circle), 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};

        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle + T_circle && timeInThisRun < time_takeoff + acc_time + T_circle + T_circle + acc_time)
    { // 水平减速
        *in_flight = 1;
        *in_trj = 0;
        float time_in_acc = timeInThisRun - (time_takeoff + acc_time + T_circle + T_circle);
        Trajectory_Generate_EXIT_AUTO(time_in_acc,
                                      v_circle,
                                      targetPos,
                                      targetVel,
                                      targetAcc,
                                      targetJerk,
                                      targetSnap,
                                      targetYaw,
                                      targetYaw_dot,
                                      targetYaw_ddot);
        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle + T_circle + acc_time && timeInThisRun < time_takeoff + acc_time + T_circle + T_circle + acc_time + time_land)
    { // 降落
        *in_flight = 0;
        *in_trj = 0;
        float time_in_land = timeInThisRun - (time_takeoff + acc_time + T_circle + T_circle + acc_time);
        Trajectory_Generate_ALT_TO_LAND_AUTO(time_in_land,
                                             targetAlt,
                                             time_takeoff,
                                             targetPos,
                                             targetVel,
                                             targetAcc,
                                             targetJerk,
                                             targetSnap,
                                             targetYaw,
                                             targetYaw_dot,
                                             targetYaw_ddot);
        // Trajectory_Generate_ALT_TO_LAND_AUTO初始高度-targetALT，不需要加这一句了
        //  *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};

        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle + T_circle + acc_time + time_land)
    {
        // 停在地面2*acc_range
        *in_flight = 0;
        *in_trj = 0;
        *targetPos = (Vector3f){acc_range + acc_range, 0, 0};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

// 时间从0开始计算，从0爬到targetalt，进行LSR飞行，降落，仿真可用
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
                                  Vector2f *targetYaw_ddot)
{

    // 起飞段时间计算
    // const float acc_climb = 0.08; // 加速度0.08// NED a=0.08

    float time_takeoff = takeofftime; // 起飞时间
    // time_takeoff = 2 * sqrtf(targetAlt / acc_climb); // 加速时间，起飞全过程
    // float acc_climb = 4 * targetAlt / (time_takeoff * time_takeoff);

    float time_land = time_takeoff;

    // 加速段时间和最终速度计算
    float v_circle = 2 * 2 * M_PI * r_circle / T_circle;
    const float acc_start = 0.5; // 加速度
    float acc_time = v_circle / acc_start;
    float acc_range = 0.5 * acc_start * acc_time * acc_time;

    if (timeInThisRun >= 0 && timeInThisRun < time_takeoff)
    { // 起飞
        *in_flight = 0;
        *in_trj = 0;
        Trajectory_Generate_TAKEOFF_TO_ALT_AUTO(timeInThisRun,
                                                targetAlt,
                                                time_takeoff,
                                                targetPos,
                                                targetVel,
                                                targetAcc,
                                                targetJerk,
                                                targetSnap,
                                                targetYaw,
                                                targetYaw_dot,
                                                targetYaw_ddot);
    }
    else if (timeInThisRun >= time_takeoff && timeInThisRun < time_takeoff + acc_time)
    { // 水平加速
        *in_flight = 1;
        *in_trj = 0;
        float time_in_acc = timeInThisRun - time_takeoff;
        Trajectory_Generate_START_AUTO(time_in_acc,
                                       v_circle,
                                       targetPos,
                                       targetVel,
                                       targetAcc,
                                       targetJerk,
                                       targetSnap,
                                       targetYaw,
                                       targetYaw_dot,
                                       targetYaw_ddot);
        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
    }
    else if (timeInThisRun >= time_takeoff + acc_time && timeInThisRun < time_takeoff + acc_time + T_circle)
    { // LSR
        *in_flight = 1;
        *in_trj = 1;
        float w_circle = 2 * M_PI / T_circle;
        float time_in_circle;
        time_in_circle = timeInThisRun - (time_takeoff + acc_time);

        // int8_t left_or_right = 0; // left  1  right  -1
        // if (sinf(w_circle * time_in_circle * 0.5) >= 0)
        // {
        //     left_or_right = 1;
        // }
        // else if (sinf(w_circle * time_in_circle * 0.5) < 0)
        // {
        //     left_or_right = -1;
        // }

        *targetPos = (Vector3f){sinf(2 * w_circle * time_in_circle) * r_circle, (-r_circle + r_circle * sinf(w_circle * time_in_circle + M_PI / 2)), 0};

        *targetVel = (Vector3f){cosf(2 * w_circle * time_in_circle) * r_circle * 2 * w_circle, (cosf(w_circle * time_in_circle + M_PI / 2) * r_circle * w_circle), 0};

        *targetAcc = (Vector3f){-sinf(2 * w_circle * time_in_circle) * r_circle * 4 * w_circle * w_circle, (-sinf(w_circle * time_in_circle + M_PI / 2) * r_circle * w_circle * w_circle), 0};

        *targetJerk = (Vector3f){-cosf(2 * w_circle * time_in_circle) * r_circle * 8 * w_circle * w_circle * w_circle, (-cosf(w_circle * time_in_circle + M_PI / 2) * r_circle * w_circle * w_circle * w_circle), 0};

        *targetSnap = (Vector3f){sinf(2 * w_circle * time_in_circle) * r_circle * 16 * w_circle * w_circle * w_circle * w_circle, (sinf(w_circle * time_in_circle + M_PI / 2) * r_circle * w_circle * w_circle * w_circle * w_circle), 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};

        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle && timeInThisRun < time_takeoff + acc_time + T_circle + acc_time)
    { // 水平减速
        *in_flight = 1;
        *in_trj = 0;
        float time_in_acc = timeInThisRun - (time_takeoff + acc_time + T_circle);
        Trajectory_Generate_EXIT_AUTO(time_in_acc,
                                      v_circle,
                                      targetPos,
                                      targetVel,
                                      targetAcc,
                                      targetJerk,
                                      targetSnap,
                                      targetYaw,
                                      targetYaw_dot,
                                      targetYaw_ddot);
        *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle + acc_time && timeInThisRun < time_takeoff + acc_time + T_circle + acc_time + time_land)
    { // 降落
        *in_flight = 0;
        *in_trj = 0;
        float time_in_land = timeInThisRun - (time_takeoff + acc_time + T_circle + acc_time);
        Trajectory_Generate_ALT_TO_LAND_AUTO(time_in_land,
                                             targetAlt,
                                             time_takeoff,
                                             targetPos,
                                             targetVel,
                                             targetAcc,
                                             targetJerk,
                                             targetSnap,
                                             targetYaw,
                                             targetYaw_dot,
                                             targetYaw_ddot);
        // Trajectory_Generate_ALT_TO_LAND_AUTO初始高度-targetALT，不需要加这一句了
        //  *targetPos = *targetPos + (Vector3f){0, 0, -targetAlt};

        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
        *targetPos = *targetPos + (Vector3f){acc_range, 0, 0};
    }
    else if (timeInThisRun >= time_takeoff + acc_time + T_circle + acc_time + time_land)
    {
        // 停在地面2*acc_range
        *in_flight = 0;
        *in_trj = 0;
        *targetPos = (Vector3f){acc_range + acc_range, 0, 0};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

void Trajectory_Generate_LINE(float timeInThisRun,
                              Vector3f *targetPos,
                              Vector3f *targetVel,
                              Vector3f *targetAcc,
                              Vector3f *targetJerk,
                              Vector3f *targetSnap,
                              Vector2f *targetYaw,
                              Vector2f *targetYaw_dot,
                              Vector2f *targetYaw_ddot)
{
    float PolyCoef[8] = {0, 0, 0, 0, 0, 0, -1, 0};
    if (timeInThisRun >= 0)
    {
        *targetPos = (Vector3f){-polyEval(PolyCoef, timeInThisRun, 8), 0, -1};

        *targetVel = (Vector3f){-polyDiffEval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetAcc = (Vector3f){-polyDiff2Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetJerk = (Vector3f){-polyDiff3Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetSnap = (Vector3f){-polyDiff4Eval(PolyCoef, timeInThisRun, 8), 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

void Trajectory_Generate_SINWAVE(float timeInThisRun,
                                 Vector3f *targetPos,
                                 Vector3f *targetVel,
                                 Vector3f *targetAcc,
                                 Vector3f *targetJerk,
                                 Vector3f *targetSnap,
                                 Vector2f *targetYaw,
                                 Vector2f *targetYaw_dot,
                                 Vector2f *targetYaw_ddot)
{
    float PolyCoef[8] = {0, 0, 0, 0, 0, 0, -1, 0};
    if (timeInThisRun >= 0)
    {
        *targetPos = (Vector3f){-polyEval(PolyCoef, timeInThisRun, 8), cosf(timeInThisRun) - 1, -1};

        *targetVel = (Vector3f){-polyDiffEval(PolyCoef, timeInThisRun, 8), -sinf(timeInThisRun), 0};

        *targetAcc = (Vector3f){-polyDiff2Eval(PolyCoef, timeInThisRun, 8), -cosf(timeInThisRun), 0};

        *targetJerk = (Vector3f){-polyDiff3Eval(PolyCoef, timeInThisRun, 8), sinf(timeInThisRun), 0};

        *targetSnap = (Vector3f){-polyDiff4Eval(PolyCoef, timeInThisRun, 8), cosf(timeInThisRun), 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

void Trajectory_Generate_BIGSINWAVE(float timeInThisRun,
                                    Vector3f *targetPos,
                                    Vector3f *targetVel,
                                    Vector3f *targetAcc,
                                    Vector3f *targetJerk,
                                    Vector3f *targetSnap,
                                    Vector2f *targetYaw,
                                    Vector2f *targetYaw_dot,
                                    Vector2f *targetYaw_ddot)
{
    float PolyCoef[8] = {0, 0, 0, 0, 0, 0, -1, 0};
    float scale_index = 2;
    float time_factor = 0.5;
    if (timeInThisRun >= 0)
    {
        *targetPos = (Vector3f){-polyEval(PolyCoef, timeInThisRun, 8), scale_index * (cosf(timeInThisRun * time_factor) - 1), -1};

        *targetVel = (Vector3f){-polyDiffEval(PolyCoef, timeInThisRun, 8), -sinf(timeInThisRun * time_factor) * scale_index, 0};

        *targetAcc = (Vector3f){-polyDiff2Eval(PolyCoef, timeInThisRun, 8), -cosf(timeInThisRun * time_factor) * scale_index, 0};

        *targetJerk = (Vector3f){-polyDiff3Eval(PolyCoef, timeInThisRun, 8), sinf(timeInThisRun * time_factor) * scale_index, 0};

        *targetSnap = (Vector3f){-polyDiff4Eval(PolyCoef, timeInThisRun, 8), cosf(timeInThisRun * time_factor) * scale_index, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

// 全自动起飞2m高度，悬停10s，降落，时间从0开始计算，仿真可用
void Trajectory_Generate_POS_TILT_AUTO(float timeInThisRun,
                                       float targetAlt,
                                       float takeofftime,
                                       float T_circle,
                                       bool *in_flight,
                                       Vector3f *targetPos,
                                       Vector3f *targetVel,
                                       Vector3f *targetAcc,
                                       Vector3f *targetJerk,
                                       Vector3f *targetSnap,
                                       Vector3f *targetHead,
                                       Vector3f *targetHead_dot,
                                       Vector3f *targetHead_ddot)
{
    // NED a=0.08
    // const float acc_climb = 0.08; // 加速度0.08
    Vector2f *targetYaw;
    Vector2f *targetYaw_dot;
    Vector2f *targetYaw_ddot;
    Vector2f targetNull;
    targetNull = {0, 0};

    // 加速时间
    float time_takeoff = takeofftime;
    // time_takeoff = 2 * sqrtf(targetAlt / acc_climb); // 加速时间，起飞全过程
    // float acc_climb = 4 * targetAlt / (time_takeoff * time_takeoff);
    targetYaw = &targetNull;
    targetYaw_dot = &targetNull;
    targetYaw_ddot = &targetNull;

    float time_land = time_takeoff;
    // 悬停时间
    const float time_in_pos = T_circle;

    if (timeInThisRun >= 0 && timeInThisRun <= time_takeoff)
    {
        *in_flight = 0;
        Trajectory_Generate_TAKEOFF_TO_ALT_AUTO(timeInThisRun,
                                                targetAlt,
                                                time_takeoff,
                                                targetPos,
                                                targetVel,
                                                targetAcc,
                                                targetJerk,
                                                targetSnap,
                                                targetYaw,
                                                targetYaw_dot,
                                                targetYaw_ddot);
        *targetHead = Vector3f{1, 0, 0};
        *targetHead_dot = Vector3f{0, 0, 0};
        *targetHead_ddot = Vector3f{0, 0, 0};
    }

    else if (timeInThisRun > time_takeoff && timeInThisRun <= time_takeoff + time_in_pos)
    {
        // 实现悬停

        *in_flight = 1;
        *targetPos = (Vector3f){0, 0, -targetAlt};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};
        float max_tilt = 20;
        max_tilt = max_tilt * M_PI / 180;
        // if (timeInThisRun - time_takeoff < time_in_pos / 2)
        // {

        //     float angle_pitch = cosf((timeInThisRun - time_takeoff) * 2 * (max_tilt / time_in_pos));
        //     *targetHead = Vector3f{angle_pitch, 0, -sqrtf(1 - angle_pitch * angle_pitch)};
        //     *targetHead_dot = Vector3f{0, 0, 0};
        //     *targetHead_ddot = Vector3f{0, 0, 0};
        // }
        // else
        // {
        //     float angle_pitch = cosf(max_tilt - (timeInThisRun - time_takeoff - time_in_pos / 2) * 2 * (max_tilt / time_in_pos));
        //     *targetHead = Vector3f{angle_pitch, 0, -sqrtf(1 - angle_pitch * angle_pitch)};
        //     *targetHead_dot = Vector3f{0, 0, 0};
        //     *targetHead_ddot = Vector3f{0, 0, 0};
        // }
        float slope = 2.0f * (max_tilt / time_in_pos); // 角速度 (rad/s)
        
        float current_theta = 0.0f;
        float current_rate = 0.0f; // d(theta)/dt

        float dt_run = timeInThisRun - time_takeoff;

        if (dt_run < time_in_pos / 2)
        {
            // 上升段
            current_theta = dt_run * slope;
            current_rate = slope;
        }
        else
        {
            // 下降段
            float dt_down = dt_run - time_in_pos / 2;
            current_theta = max_tilt - dt_down * slope;
            current_rate = -slope;
        }

        // 计算三角函数
        float c = cosf(current_theta);
        float s = sinf(current_theta);

        // 1. 机头指向向量 (假设 Pitch 为正代表抬头)
        // targetHead = [cos, 0, -sin] (NED系)
        *targetHead = Vector3f{c, 0, -s};

        // 2. 一阶导数 (角速度 x 向量)
        // d(cos)/dt = -sin * rate
        // d(-sin)/dt = -cos * rate
        *targetHead_dot = Vector3f{-s * current_rate, 0, -c * current_rate};

        // 3. 二阶导数 (角加速度 + 向心项)
        // 假设角加速度为0 (线性变化)，只保留向心项: -rate^2 * vector
        float rate_sq = current_rate * current_rate;
        *targetHead_ddot = Vector3f{-c * rate_sq, 0, s * rate_sq};

    }
    else if (timeInThisRun > time_takeoff + time_in_pos && timeInThisRun <= time_takeoff + time_in_pos + time_land)
    {
        *in_flight = 0;
        float time_land_operate = timeInThisRun - (time_takeoff + time_in_pos);
        Trajectory_Generate_ALT_TO_LAND_AUTO(time_land_operate,
                                             targetAlt,
                                             time_takeoff,
                                             targetPos,
                                             targetVel,
                                             targetAcc,
                                             targetJerk,
                                             targetSnap,
                                             targetYaw,
                                             targetYaw_dot,
                                             targetYaw_ddot);
        *targetHead = Vector3f{1, 0, 0};
        *targetHead_dot = Vector3f{0, 0, 0};
        *targetHead_ddot = Vector3f{0, 0, 0};
    }
    else
    {
        *in_flight = 0;
        *targetPos = (Vector3f){0, 0, 0};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetHead = Vector3f{1, 0, 0};
        *targetHead_dot = Vector3f{0, 0, 0};
        *targetHead_ddot = Vector3f{0, 0, 0};
    }
}

// 纯定点000轨迹
void Trajectory_Generate_POS(
    Vector3f *targetPos,
    Vector3f *targetVel,
    Vector3f *targetAcc,
    Vector3f *targetJerk,
    Vector3f *targetSnap,
    Vector2f *targetYaw,
    Vector2f *targetYaw_dot,
    Vector2f *targetYaw_ddot)
{

    if (1)
    {

        *targetPos = (Vector3f){0, 0, 0};

        *targetVel = (Vector3f){0, 0, 0};

        *targetAcc = (Vector3f){0, 0, 0};

        *targetJerk = (Vector3f){0, 0, 0};

        *targetSnap = (Vector3f){0, 0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
}

float polyEval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N; i++)
    {
        result += polyCoef[i] * powf(x, N - 1 - i);
    }
    return result;
}

float polyDiffEval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 1; i++)
    {
        result += polyCoef[i] * powf(x, N - 2 - i) * (float)(N - 1 - i);
    }
    return result;
}

float polyDiff2Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 2nd derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 2; i++)
    {
        result += polyCoef[i] * powf(x, N - 3 - i) * (float)(N - 1 - i) * (float)(N - 2 - i);
    }
    return result;
}

float polyDiff3Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 3rd derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 3; i++)
    {
        result += polyCoef[i] * powf(x, N - 4 - i) * (float)(N - 1 - i) * (float)(N - 2 - i) * (float)(N - 3 - i);
    }
    return result;
}

float polyDiff4Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 4th derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 4; i++)
    {
        result += polyCoef[i] * powf(x, N - 5 - i) * (float)(N - 1 - i) * (float)(N - 2 - i) * (float)(N - 3 - i) * (float)(N - 4 - i);
    }
    return result;
}

// 实现从起飞、倾转加速到纯固定翼前飞的完整轨迹
void Trajectory_Generate_FW_TRANSITION(float timeInThisRun, float targetAlt, float takeofftime, float transition_time, float forward_vel, bool *in_flight, Vector3f *targetPos, Vector3f *targetVel, Vector3f *targetAcc, Vector3f *targetJerk, Vector3f *targetSnap, Vector3f *targetHead, Vector3f *targetHead_dot, Vector3f *targetHead_ddot)
{
    // 阶段 1: 垂直起飞到目标高度
    if (timeInThisRun <= takeofftime)
    {
        *in_flight = 0; 
        Vector2f targetYaw = {1, 0}; Vector2f targetYaw_dot = {0, 0}; Vector2f targetYaw_ddot = {0, 0};
        Trajectory_Generate_TAKEOFF_TO_ALT_AUTO(timeInThisRun, targetAlt, takeofftime,
                                                targetPos, targetVel, targetAcc, targetJerk, targetSnap,
                                                &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        *targetHead = Vector3f{1, 0, 0};
        *targetHead_dot = Vector3f{0, 0, 0};
        *targetHead_ddot = Vector3f{0, 0, 0};
    }
    // 阶段 2: 倾转过渡段
    else if (timeInThisRun <= takeofftime + transition_time)
    {
        *in_flight = 1;
        float dt = timeInThisRun - takeofftime;
        float progress = dt / transition_time; // 进度 0 到 1
        
        // 【核心修改】：调用独立函数获取当前倾转角
        float tilt = calculate_tilt_angle(progress);
        
        float cur_vel = progress * forward_vel;
        float cur_pos = 0.5f * forward_vel * (dt * dt / transition_time); 
        
        *targetPos = Vector3f{cur_pos, 0, -targetAlt};
        *targetVel = Vector3f{cur_vel, 0, 0};
        *targetAcc = Vector3f{forward_vel / transition_time, 0, 0};
        *targetJerk = Vector3f{0, 0, 0};
        *targetSnap = Vector3f{0, 0, 0};

        *targetHead = Vector3f{cosf(tilt), 0, -sinf(tilt)};
        // 粗略导数（如果改为非线性，这里最好用数值差分或写出解析导数）
        float tilt_rate = (calculate_tilt_angle(progress + 0.01f) - tilt) / 0.01f / transition_time;
        *targetHead_dot = Vector3f{-sinf(tilt)*tilt_rate, 0, -cosf(tilt)*tilt_rate};
        *targetHead_ddot = Vector3f{0, 0, 0}; 
    }
    // 阶段 3: 固定翼前飞
    else
    {
        *in_flight = 1;
        float dt_fw = timeInThisRun - (takeofftime + transition_time);
        float trans_dist = 0.5f * forward_vel * transition_time;

        *targetPos = Vector3f{trans_dist + forward_vel * dt_fw, 0, -targetAlt};
        *targetVel = Vector3f{forward_vel, 0, 0};
        *targetAcc = Vector3f{0, 0, 0};
        *targetJerk = Vector3f{0, 0, 0};
        *targetSnap = Vector3f{0, 0, 0};

        *targetHead = Vector3f{0, 0, -1};
        *targetHead_dot = Vector3f{0, 0, 0};
        *targetHead_ddot = Vector3f{0, 0, 0};
    }
}

// ====================================================================
// 1. 倾转角调度函数 (Tilt Angle Profile)
// 输入: progress (过渡进度 0.0 到 1.0)
// 输出: 当前目标倾转角 (弧度，0 到 PI/2)
// ====================================================================
float calculate_tilt_angle(float progress)
{
    float max_tilt = M_PI / 2.0f; // 最大倾转角 90 度
    
    // 占位：纯粹的线性关系
    // 你后续可以在这里改为 S型曲线 (Sigmoid)、多项式平滑等关系
    return progress * max_tilt; 
}

// ====================================================================
// 1. 倾转角调度函数 (Tilt Angle Profile)
// ====================================================================
// float calculate_tilt_angle(float progress)
// {
//     float max_tilt = M_PI / 2.0f; // 最大倾转角 90 度
    
//     // 使用 Smoothstep 平滑曲线替代线性关系
//     // 公式: 3x^2 - 2x^3，保证在 progress=0 和 progress=1 时的导数(速度)均为 0
//     float smooth_progress = progress * progress * (3.0f - 2.0f * progress);
    
//     return smooth_progress * max_tilt; 
// }


// ====================================================================
// 1. 倾转角调度函数 (基于 MATLAB 优化的 pOpt 节点插值)
// ====================================================================
// float calculate_tilt_angle(float progress)
// {
//     float max_tilt = M_PI / 2.0f; // 90度

//     // 确保 progress 在 0~1 之间
//     if (progress <= 0.0f) return 0.0f;
//     if (progress >= 1.0f) return max_tilt;

//     // 定义 7 个控制点 (对应 MATLAB 代码中的 5个自由节点 + 首尾2个固定节点)
//     const int NUM_NODES = 7;
    
//     // 等价于 MATLAB 的 nodeFrac = linspace(0, 1, 7);
//     const float progress_nodes[NUM_NODES] = {
//         0.0f, 0.166667f, 0.333333f, 0.5f, 0.666667f, 0.833333f, 1.0f
//     };

//     // 等价于 MATLAB 的 lambdak = [0, pOpt, 1]; (这里采用了优化基线 pOpt)
//     const float lambda_nodes[NUM_NODES] = {
//         0.0f, 0.110f, 0.166f, 0.351f, 0.709f, 0.940f, 1.0f
//     };

//     // 线性插值算法
//     float lambda_out = 0.0f;
//     for (int i = 0; i < NUM_NODES - 1; i++) {
//         if (progress >= progress_nodes[i] && progress <= progress_nodes[i + 1]) {
//             // 计算当前段的插值比例
//             float t = (progress - progress_nodes[i]) / (progress_nodes[i + 1] - progress_nodes[i]);
//             // 计算输出
//             lambda_out = lambda_nodes[i] + t * (lambda_nodes[i + 1] - lambda_nodes[i]);
//             break;
//         }
//     }

//     // 将 0~1 的 lambda 比例转换为实际弧度
//     return lambda_out * max_tilt;
// }

// float calculate_tilt_angle(float progress)
// {
//     float max_tilt = M_PI / 2.0f; // 90度

//     // 确保 progress 在 0~1 之间
//     if (progress <= 0.0f) return 0.0f;
//     if (progress >= 1.0f) return max_tilt;

//     // 定义 7 个控制点 (对应 MATLAB 代码中的 5个自由节点 + 首尾2个固定节点)
//     const int NUM_NODES = 7;
    
//     // 等价于 MATLAB 的 nodeFrac = linspace(0, 1, 7);
//     const float progress_nodes[NUM_NODES] = {
//         0.0f, 0.166667f, 0.333333f, 0.5f, 0.666667f, 0.833333f, 1.0f
//     };

//     // 等价于 MATLAB 的 lambdak = [0, pOpt, 1]; (这里采用了优化基线 pOpt)
//     const float lambda_nodes[NUM_NODES] = {
//         0.0f, 0.04f, 0.09f, 0.16f, 0.30f, 0.55f, 1.0f
//     };

//     // 线性插值算法
//     float lambda_out = 0.0f;
//     for (int i = 0; i < NUM_NODES - 1; i++) {
//         if (progress >= progress_nodes[i] && progress <= progress_nodes[i + 1]) {
//             // 计算当前段的插值比例
//             float t = (progress - progress_nodes[i]) / (progress_nodes[i + 1] - progress_nodes[i]);
//             // 计算输出
//             lambda_out = lambda_nodes[i] + t * (lambda_nodes[i + 1] - lambda_nodes[i]);
//             break;
//         }
//     }

//     // 将 0~1 的 lambda 比例转换为实际弧度
//     return lambda_out * max_tilt;
// }

// float calculate_tilt_angle(float progress)
// {
//     float max_tilt = M_PI / 2.0f; // 90度

//     // 确保 progress 在 0~1 之间
//     if (progress <= 0.0f) return 0.0f;
//     if (progress >= 1.0f) return max_tilt;

//     // 定义 7 个控制点 (对应 MATLAB 代码中的 5个自由节点 + 首尾2个固定节点)
//     const int NUM_NODES = 7;
    
//     // 等价于 MATLAB 的 nodeFrac = linspace(0, 1, 7);
//     const float progress_nodes[NUM_NODES] = {
//         0.0f, 0.166667f, 0.333333f, 0.5f, 0.666667f, 0.833333f, 1.0f
//     };

//     // 等价于 MATLAB 的 lambdak = [0, pOpt, 1]; (这里采用了优化基线 pOpt)
//     const float lambda_nodes[NUM_NODES] = {
//         0.0f, 0.40f, 0.65f, 0.80f, 0.90f, 0.95f, 1.0f
//     };

//     // 线性插值算法
//     float lambda_out = 0.0f;
//     for (int i = 0; i < NUM_NODES - 1; i++) {
//         if (progress >= progress_nodes[i] && progress <= progress_nodes[i + 1]) {
//             // 计算当前段的插值比例
//             float t = (progress - progress_nodes[i]) / (progress_nodes[i + 1] - progress_nodes[i]);
//             // 计算输出
//             lambda_out = lambda_nodes[i] + t * (lambda_nodes[i + 1] - lambda_nodes[i]);
//             break;
//         }
//     }

//     // 将 0~1 的 lambda 比例转换为实际弧度
//     return lambda_out * max_tilt;
// }
// // [0.1029, 0.21985, 0.33117, 0.53781, 0.78641]

// float calculate_tilt_angle(float progress)
// {
//     float max_tilt = M_PI / 2.0f; // 90度

//     // 确保 progress 在 0~1 之间
//     if (progress <= 0.0f) return 0.0f;
//     if (progress >= 1.0f) return max_tilt;

//     // 定义 7 个控制点 (对应 MATLAB 代码中的 5个自由节点 + 首尾2个固定节点)
//     const int NUM_NODES = 7;
    
//     // 等价于 MATLAB 的 nodeFrac = linspace(0, 1, 7);
//     const float progress_nodes[NUM_NODES] = {
//         0.0f, 0.166667f, 0.333333f, 0.5f, 0.666667f, 0.833333f, 1.0f
//     };

//     // 等价于 MATLAB 的 lambdak = [0, pOpt, 1]; (这里采用了优化基线 pOpt)
//     const float lambda_nodes[NUM_NODES] = {
//         0.0f, 0.1029, 0.21985, 0.33117, 0.53781, 0.78641, 1.0f
//     };

//     // 线性插值算法
//     float lambda_out = 0.0f;
//     for (int i = 0; i < NUM_NODES - 1; i++) {
//         if (progress >= progress_nodes[i] && progress <= progress_nodes[i + 1]) {
//             // 计算当前段的插值比例
//             float t = (progress - progress_nodes[i]) / (progress_nodes[i + 1] - progress_nodes[i]);
//             // 计算输出
//             lambda_out = lambda_nodes[i] + t * (lambda_nodes[i + 1] - lambda_nodes[i]);
//             break;
//         }
//     }

//     // 将 0~1 的 lambda 比例转换为实际弧度
//     return lambda_out * max_tilt;
// }

// ====================================================================
// 2. 混合控制权重分配函数 (Fuzzy Blending Logic)
// 输入: airspeed (当前空速，单位 m/s)
// 输出: mc_coeff (多旋翼控制权重 0~1), fw_coeff (固定翼控制权重 0~1)
// ====================================================================
void calculate_blend_coefficients(float airspeed, float &mc_coeff, float &fw_coeff)
{
    // 假设：8m/s 以下纯多旋翼，12m/s 以上纯固定翼
    const float min_airspeed = 1.0f;  
    const float max_airspeed = 17.0f; 
    
    if (airspeed <= min_airspeed) {
        mc_coeff = 1.0f;
        fw_coeff = 0.0f;
    } 
    else if (airspeed >= max_airspeed) {
        mc_coeff = 0.0f;
        fw_coeff = 1.0f;
    } 
    else {
        // 占位：简单的线性模糊过渡
        // 你后续可以改为高斯型、梯形等模糊隶属度函数
        fw_coeff = (airspeed - min_airspeed) / (max_airspeed - min_airspeed);
        mc_coeff = 1.0f - fw_coeff;
        // fw_coeff =  1.0f;
    }
}