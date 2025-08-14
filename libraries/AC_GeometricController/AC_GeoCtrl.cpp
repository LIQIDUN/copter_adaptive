#include "AC_GeoCtrl.h"

VectorN<float, 4> GeometricTrajectoryController(
    GeoInput ControllIn)
{
    Vector3f r_error;
    Vector3f v_error;
    Vector3f target_force;
    Vector3f z_axis;
    Vector3f x_axis_desired;
    Vector3f y_axis_desired;
    Vector3f x_c_des;
    Vector3f eR, ew, M;
    Vector3f e3 = {0, 0, 1};

    Vector3f targetPos = ControllIn.targetPos;
    Vector3f targetVel = ControllIn.targetVel;
    Vector3f targetAcc = ControllIn.targetAcc;
    Vector3f targetJerk = ControllIn.targetJerk;
    Vector3f targetSnap = ControllIn.targetSnap;
    Vector2f targetYaw = ControllIn.targetYaw;
    Vector2f targetYaw_dot = ControllIn.targetYaw_dot;
    Vector2f targetYaw_ddot = ControllIn.targetYaw_ddot;

    Vector3f statePos = ControllIn.statePos;
    Vector3f stateVel = ControllIn.stateVel;

    Vector3f euler = ControllIn.euler;

    float kg_vehicleMass = ControllIn.kg_vehicleMass;

    float GRAVITY_MAGNITUDE = ControllIn.GRAVITY_MAGNITUDE;

    Matrix3f J = ControllIn.J;

    float GeoCtrl_Kpx = ControllIn.kpx;
    float GeoCtrl_Kpy = ControllIn.kpy;
    float GeoCtrl_Kpz = ControllIn.kpz;
    float GeoCtrl_Kvx = ControllIn.kvx;
    float GeoCtrl_Kvy = ControllIn.kvy;
    float GeoCtrl_Kvz = ControllIn.kvz;
    float GeoCtrl_Krx = ControllIn.krx;
    float GeoCtrl_Kry = ControllIn.kry;
    float GeoCtrl_Krz = ControllIn.krz;
    float GeoCtrl_Kox = ControllIn.kox;
    float GeoCtrl_Koy = ControllIn.koy;
    float GeoCtrl_Koz = ControllIn.koz;


    // Position Error (ep)
    r_error = statePos - targetPos;

    // Velocity Error (ev)
    v_error = stateVel - targetVel;

    // kg_vehicleMass = g.GeoCtrl_MAS; // weight for the real drone

    // Target force
    target_force.x = kg_vehicleMass * targetAcc.x - GeoCtrl_Kpx * r_error.x - GeoCtrl_Kvx * v_error.x;
    target_force.y = kg_vehicleMass * targetAcc.y - GeoCtrl_Kpy * r_error.y - GeoCtrl_Kvy * v_error.y;
    target_force.z = kg_vehicleMass * (targetAcc.z - GRAVITY_MAGNITUDE) - GeoCtrl_Kpz * r_error.z - GeoCtrl_Kvz * v_error.z;

    // // Z-Axis [zB]
    // Quaternion q;
    // ahrs.get_quat_body_to_ned(q);

    // Matrix3f R;
    // q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    Matrix3f R;
    // Vector3f euler;
    // if (!ahrs.get_secondary_attitude(euler))
    // {
    //     VectorN<float, 4> no_out_put;
    //     no_out_put[0] = 0;
    //     no_out_put[1] = 0;
    //     no_out_put[2] = 0;
    //     no_out_put[3] = 0;
    //     return no_out_put;
    // }
    float roll_rad = euler.x;
    float pitch_rad = euler.y;
    float yaw_rad = euler.z;

    float cosPhi = cosf(roll_rad);
    float sinPhi = sinf(roll_rad);
    float cosTheta = cosf(pitch_rad);
    float sinTheta = sinf(pitch_rad);
    float cosPsi = cosf(yaw_rad);
    float sinPsi = sinf(yaw_rad);

    // R(Vector3f(), Vector3f(), Vector3f());

    // 第一行
    R[0][0] = cosPsi * cosTheta;
    R[0][1] = cosPsi * sinTheta * sinPhi - sinPsi * cosPhi;
    R[0][2] = cosPsi * sinTheta * cosPhi + sinPsi * sinPhi;

    // 第二行
    R[1][0] = sinPsi * cosTheta;
    R[1][1] = sinPsi * sinTheta * sinPhi + cosPsi * cosPhi;
    R[1][2] = sinPsi * sinTheta * cosPhi - cosPsi * sinPhi;

    // 第三行
    R[2][0] = -sinTheta;
    R[2][1] = cosTheta * sinPhi;
    R[2][2] = cosTheta * cosPhi;

    z_axis = R.colz();

    // target thrust [F]
    float target_thrust = -target_force * z_axis;

    // Calculate axis [zB_des]
    Vector3f z_axis_desired = -target_force;
    z_axis_desired.normalize();

    // [xC_des]
    // x_axis_desired = z_axis_desired x [cos(yaw), sin(yaw), 0]^T
    x_c_des[0] = targetYaw[0]; // x
    x_c_des[1] = targetYaw[1]; // y
    x_c_des[2] = 0;            // z

    Vector3f x_c_des_dot = {targetYaw_dot, 0};   // time derivative of x_c_des
    Vector3f x_c_des_ddot = {targetYaw_ddot, 0}; // time derivative of x_c_des_dot

    // [yB_des]
    y_axis_desired = (z_axis_desired % x_c_des);
    y_axis_desired.normalize();
    // [xB_des]
    x_axis_desired = y_axis_desired % z_axis_desired;

    // [eR]
    Matrix3f Rdes(Vector3f(x_axis_desired.x, y_axis_desired.x, z_axis_desired.x),
                  Vector3f(x_axis_desired.y, y_axis_desired.y, z_axis_desired.y),
                  Vector3f(x_axis_desired.z, y_axis_desired.z, z_axis_desired.z));

    Matrix3f eRM = (Rdes.transposed() * R - R.transposed() * Rdes) / 2;
    eR = veeOperator(eRM);

    Vector3f Omega = AP::ahrs().get_gyro();
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", Omega.z);

    // compute Omegad: this comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf
    Vector3f a_error; // error on acceleration
    a_error = e3 * GRAVITY_MAGNITUDE - R.colz() * target_thrust / kg_vehicleMass - targetAcc;

    Vector3f target_force_dot; // derivative of target_force
    target_force_dot.x = -GeoCtrl_Kpx * v_error.x - GeoCtrl_Kvx * a_error.x + kg_vehicleMass * targetJerk.x;
    target_force_dot.y = -GeoCtrl_Kpy * v_error.y -GeoCtrl_Kvy * a_error.y + kg_vehicleMass * targetJerk.y;
    target_force_dot.z = -GeoCtrl_Kpz * v_error.z - GeoCtrl_Kvz * a_error.z + kg_vehicleMass * targetJerk.z;

    Vector3f b3_dot = R * hatOperator(Omega) * e3;

    float target_thrust_dot = -target_force_dot * R.colz() - target_force * b3_dot;

    Vector3f j_error; // error on jerk
    j_error = -R.colz() * target_thrust_dot / kg_vehicleMass - b3_dot * target_thrust / kg_vehicleMass - targetJerk;

    Vector3f target_force_ddot; // derivative of target_force_dot
    target_force_ddot.x = -GeoCtrl_Kpx * a_error.x - GeoCtrl_Kvx * j_error.x + kg_vehicleMass * targetSnap.x;
    target_force_ddot.y = -GeoCtrl_Kpy * a_error.y - GeoCtrl_Kvy * j_error.y + kg_vehicleMass * targetSnap.y;
    target_force_ddot.z = -GeoCtrl_Kpz * a_error.z - GeoCtrl_Kvz * j_error.z + kg_vehicleMass * targetSnap.z;

    VectorN<float, 9> b3cCollection;                                                // collection of three three-dimensional vectors b3c, b3c_dot, b3c_ddot
    b3cCollection = unit_vec(-target_force, -target_force_dot, -target_force_ddot); // unit_vec function is from geometric controller's git repo: https://github.com/fdcl-gwu/uav_geometric_control/blob/master/matlab/aux_functions/deriv_unit_vector.m

    Vector3f b3c;
    Vector3f b3c_dot;
    Vector3f b3c_ddot;

    b3c[0] = b3cCollection[0];
    b3c[1] = b3cCollection[1];
    b3c[2] = b3cCollection[2];

    b3c_dot[0] = b3cCollection[3];
    b3c_dot[1] = b3cCollection[4];
    b3c_dot[2] = b3cCollection[5];

    b3c_ddot[0] = b3cCollection[6];
    b3c_ddot[1] = b3cCollection[7];
    b3c_ddot[2] = b3cCollection[8];

    Vector3f A2 = -hatOperator(x_c_des) * b3c;
    Vector3f A2_dot = -hatOperator(x_c_des_dot) * b3c - hatOperator(x_c_des) * b3c_dot;
    Vector3f A2_ddot = -hatOperator(x_c_des_ddot) * b3c - hatOperator(x_c_des_dot) * b3c_dot * 2 - hatOperator(x_c_des) * b3c_ddot;

    VectorN<float, 9> b2cCollection;               // collection of three three-dimensional vectors b2c, b2c_dot, b2c_ddot
    b2cCollection = unit_vec(A2, A2_dot, A2_ddot); // unit_vec function is from geometric controller's git repo: https://github.com/fdcl-gwu/uav_geometric_control/blob/master/matlab/aux_functions/deriv_unit_vector.m

    Vector3f b2c;
    Vector3f b2c_dot;
    Vector3f b2c_ddot;

    b2c[0] = b2cCollection[0];
    b2c[1] = b2cCollection[1];
    b2c[2] = b2cCollection[2];

    b2c_dot[0] = b2cCollection[3];
    b2c_dot[1] = b2cCollection[4];
    b2c_dot[2] = b2cCollection[5];

    b2c_ddot[0] = b2cCollection[6];
    b2c_ddot[1] = b2cCollection[7];
    b2c_ddot[2] = b2cCollection[8];

    Vector3f b1c_dot = hatOperator(b2c_dot) * b3c + hatOperator(b2c) * b3c_dot;
    Vector3f b1c_ddot = hatOperator(b2c_ddot) * b3c + hatOperator(b2c_dot) * b3c_dot * 2 + hatOperator(b2c) * b3c_ddot;

    Matrix3f Rd_dot;  // derivative of Rdes
    Matrix3f Rd_ddot; // derivative of Rd_dot

    Rd_dot.a = b1c_dot;
    Rd_dot.b = b2c_dot;
    Rd_dot.c = b3c_dot;
    Rd_dot.transpose();

    Rd_ddot.a = b1c_ddot;
    Rd_ddot.b = b2c_ddot;
    Rd_ddot.c = b3c_ddot;
    Rd_ddot.transpose();

    Vector3f Omegad = veeOperator(Rdes.transposed() * Rd_dot);
    Vector3f Omegad_dot = veeOperator(Rdes.transposed() * Rd_ddot - hatOperator(Omegad) * hatOperator(Omegad));

    // eomega (angular velocity error)
    ew = Omega - R.transposed() * Rdes * Omegad;

    // Compute the moment
    M.x = -GeoCtrl_Krx * eR.x - GeoCtrl_Kox * ew.x;
    M.y = -GeoCtrl_Kry * eR.y - GeoCtrl_Koy * ew.y;
    M.z = -GeoCtrl_Krz * eR.z - GeoCtrl_Koz * ew.z;
    M = M - J * (hatOperator(Omega) * R.transposed() * Rdes * Omegad - R.transposed() * Rdes * Omegad_dot);
    Vector3f momentAdd = Omega % (J * Omega); // J is the inertia matrix
    M = M + momentAdd;

    VectorN<float, 4> thrustMomentCmd;
    thrustMomentCmd[0] = target_thrust;
    thrustMomentCmd[1] = M.x;
    thrustMomentCmd[2] = M.y;
    thrustMomentCmd[3] = M.z;

    // logging
    AP::logger().Write("GEOM", "TimeUS,exx,exy,exz,evx,evy,evz,erx,ery,erz,ewx,ewy,ewz", "Qffffffffffff",
                       AP_HAL::micros64(),
                       (r_error.x),
                       (r_error.y),
                       (r_error.z),
                       (v_error.x),
                       (v_error.y),
                       (v_error.z),
                       (eR.x),
                       (eR.y),
                       (eR.z),
                       (ew.x),
                       (ew.y),
                       (ew.z));
    AP::logger().Write("GECT", "TimeUS,tfx,tfy,tfz,tt,mx,my,mz", "Qfffffff",
                       AP_HAL::micros64(),
                       (target_force.x),
                       (target_force.y),
                       (target_force.z),
                       (target_thrust),
                       (M.x),
                       (M.y),
                       (M.z));
    AP::logger().Write("GETA", "TimeUS,tpx,tpy,tpz,spx,spy,spz", "Qffffff",
                       AP_HAL::micros64(),
                       (targetPos.x),
                       (targetPos.y),
                       (targetPos.z),
                       (statePos.x),
                       (statePos.y),
                       (statePos.z));
    // log the desired rotation matrix and the actual rotation matrix
    AP::logger().Write("L1AF", "TimeUS,Rd11,Rd12,Rd13,Rd21,Rd22,Rd23,Rd31,Rd32,Rd33", "Qfffffffff",
                       AP_HAL::micros64(),
                       Rdes.a.x,
                       Rdes.a.y,
                       Rdes.a.z,
                       Rdes.b.x,
                       Rdes.b.y,
                       Rdes.b.z,
                       Rdes.c.x,
                       Rdes.c.y,
                       Rdes.c.z);
    AP::logger().Write("L1AG", "TimeUS,R11,R12,R13,R21,R22,R23,R31,R32,R33", "Qfffffffff",
                       AP_HAL::micros64(),
                       R.a.x,
                       R.a.y,
                       R.a.z,
                       R.b.x,
                       R.b.y,
                       R.b.z,
                       R.c.x,
                       R.c.y,
                       R.c.z);
    return thrustMomentCmd;
}
