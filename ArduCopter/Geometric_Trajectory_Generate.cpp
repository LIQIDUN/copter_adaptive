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
