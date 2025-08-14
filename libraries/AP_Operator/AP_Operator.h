#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>
#include <AP_Math/matrixN.h>
#include <AP_Math/chirp.h>
#include <cmath>

Matrix3f hatOperator(Vector3f input);
Vector3f veeOperator(Matrix3f input);

VectorN<float, 9> unit_vec(Vector3f q, Vector3f q_dot, Vector3f q_ddot);
float vector_2norm(Vector3f A);