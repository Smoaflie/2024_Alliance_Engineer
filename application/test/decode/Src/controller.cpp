#include "controller.hpp"
#include <math.h>
#include "usermode.hpp"
using namespace std;

Attitue NowAttitue;
Attitue LastAttitue;

const float rs  = 68.2426464f;
const float rl2 = 153.851648f * 153.851648f;
const float rb  = 100.0f - 26.5f;

Vector3 c_12;
Vector3 c_23;
Vector3 c_31;
Vector3 nfo;
Vector3 nef;
const Vector3 a_t = Vector3(0, rb, 0);
const Vector3 a_l = Vector3(rb * -0.866025404f, rb * -0.5f, 0);
const Vector3 a_r = Vector3(rb * 0.866025404f, rb * -0.5f, 0);
Vector3 phi_t;
Vector3 phi_l;
Vector3 phi_r;
const Vector3 theta_t = Vector3(0, rs, rs);
const Vector3 theta_l = Vector3(rs * -0.866025404f, rs * -0.5f, rs);
const Vector3 theta_r = Vector3(rs * 0.866025404f, rs * -0.5f, rs);
Vector3 theta_t_t     = Vector3(0, rs, rs);
Vector3 theta_l_t     = Vector3(rs * -0.866025404f, rs * -0.5f, rs);
Vector3 theta_r_t     = Vector3(rs * 0.866025404f, rs * -0.5f, rs);
float c[4];
float R2;
float temp_sin;
float R[2];

float roll, yaw, pitch;

extern float Me02_Data[3];
Vector3 Me02_Pos_Zero_Data;
Vector3 Me02_Ang_Zero_Data;

Controller::Controller(float *me02, float *mpu)
{
    me02_Data = me02;
    mpu_data  = mpu;
}

Controller::~Controller()
{
}

void Controller::ZeroController()
{
    Me02_Pos_Zero_Data = NowAttitue.Position;
    Me02_Ang_Zero_Data = Vector3(pitch, roll, yaw);
}
Attitue Controller::GetAttitue()
{
    return attitue;
}

void CalculateNowPos()
{
    theta_t_t = theta_t;
    theta_l_t = theta_l;
    theta_r_t = theta_r;

    temp_sin = cosf(Me02_Data[0]);
    phi_t    = Vector3(temp_sin, temp_sin, sinf(Me02_Data[0]));
    temp_sin = cosf(Me02_Data[1]);
    phi_r    = Vector3(temp_sin, temp_sin, sinf(Me02_Data[1]));
    temp_sin = cosf(Me02_Data[2]);
    phi_l    = Vector3(temp_sin, temp_sin, sinf(Me02_Data[2]));

    theta_t_t.times(phi_t);
    theta_r_t.times(phi_r);
    theta_l_t.times(phi_l);

    theta_t_t += a_t;
    theta_r_t += a_r;
    theta_l_t += a_l;

    c_12 = theta_r_t - theta_t_t;
    c_23 = theta_l_t - theta_r_t;
    c_31 = theta_t_t - theta_l_t;

    nfo = c_12.Cross(c_23);
    nef = nfo.Cross(c_12);

    nfo /= nfo.normalize();
    nef /= nef.normalize();

    c[0] = c_12.normalize();
    c[1] = c_23.normalize();
    c[2] = c_31.normalize();
    c[3] = (c[0] + c[1] + c[2]) / 2;

    R2 = (c[0] * c[1] * c[2] * c[0] * c[1] * c[2]) / 16 / (c[3] * (c[3] - c[2]) * (c[3] - c[1]) * (c[3] - c[0]));
    nef *= sqrtf(R2 - c[0] * c[0] / 4);
    nfo *= sqrtf(rl2 - R2);

    NowAttitue.Position = (theta_t_t + theta_r_t);
    NowAttitue.Position /= 2.0f;
    NowAttitue.Position += nef;
    NowAttitue.Position -= nfo;
}
void Controller::UpdateAttitue()
{
    LastAttitue = NowAttitue;
    CalculateNowPos();
    attitue.Position = NowAttitue.Position + LastAttitue.Position;
    attitue.Position /= 2;
    attitue.Position -= Me02_Pos_Zero_Data;
    Me02_Ang_Zero_Data -= Vector3(0, 0, 0.0057f / FPS);
    // update_mpu_data(&pitch, &roll, &yaw);
    pitch = mpu_data[0];roll = mpu_data[1];yaw = mpu_data[2];
    attitue.EularAngle = Vector3(pitch, roll, yaw) - Me02_Ang_Zero_Data;
}
