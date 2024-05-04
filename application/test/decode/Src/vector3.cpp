#include "vector3.hpp"
#include "math.h"

Vector3::Vector3()
{
    vector3_Data.x = vector3_Data.y = vector3_Data.z = 0;
}

Vector3::Vector3(float x, float y, float z)
{
    vector3_Data.x = x;
    vector3_Data.y = y;
    vector3_Data.z = z;
}

Vector3::~Vector3()
{
}

Vector3 Vector3::Cross(const Vector3 &v) const
{
    _Vector3_Data temp = v.Get();
    float xt, yt, zt;
    xt = vector3_Data.y * temp.z - temp.y * vector3_Data.z;
    yt = vector3_Data.z * temp.x - temp.z * vector3_Data.x;
    zt = vector3_Data.x * temp.y - temp.x * vector3_Data.y;
    return Vector3(xt, yt, zt);
}

float Vector3::Distance(const Vector3 &v) const
{
    _Vector3_Data temp = v.Get();
    return sqrtf(pow(vector3_Data.x - temp.x, 2) + pow(vector3_Data.y - temp.y, 2) + pow(vector3_Data.z - temp.z, 2));
}

float Vector3::Dot(const Vector3 &v) const
{
    _Vector3_Data temp = v.Get();
    return vector3_Data.x * temp.x + vector3_Data.y * temp.y + vector3_Data.z * temp.z;
}

Vector3 &Vector3::times(const Vector3 &v)
{
    _Vector3_Data temp = v.Get();
    vector3_Data.x *= temp.x;
    vector3_Data.y *= temp.y;
    vector3_Data.z *= temp.z;
    return *this;
}

float Vector3::normalize() const
{
    return sqrtf(vector3_Data.x * vector3_Data.x + vector3_Data.y * vector3_Data.y + vector3_Data.z * vector3_Data.z);
}

Vector3::_Vector3_Data Vector3::Get() const
{
    return vector3_Data;
}

Vector3 &Vector3::operator+=(const Vector3 &v)
{
    _Vector3_Data temp = v.Get();
    vector3_Data.x += temp.x;
    vector3_Data.y += temp.y;
    vector3_Data.z += temp.z;
    return *this;
}

Vector3 &Vector3::operator-=(const Vector3 &v)
{
    _Vector3_Data temp = v.Get();
    vector3_Data.x -= temp.x;
    vector3_Data.y -= temp.y;
    vector3_Data.z -= temp.z;
    return *this;
}

Vector3 Vector3::operator+(const Vector3 &v)
{
    _Vector3_Data temp = v.Get();
    Vector3 vec        = Vector3(vector3_Data.x + temp.x,
                                 vector3_Data.y + temp.y,
                                 vector3_Data.z + temp.z);
    return vec;
}

Vector3 Vector3::operator-(const Vector3 &v)
{
    _Vector3_Data temp = v.Get();
    Vector3 vec        = Vector3(vector3_Data.x - temp.x,
                                 vector3_Data.y - temp.y,
                                 vector3_Data.z - temp.z);
    return vec;
}

Vector3 &Vector3::operator*=(const float &f)
{

    vector3_Data.x *= f;
    vector3_Data.y *= f;
    vector3_Data.z *= f;
    return *this;
}

Vector3 &Vector3::operator/=(const float &v)
{
    if (v == 0)
        return *this;
    vector3_Data.x /= v;
    vector3_Data.y /= v;
    vector3_Data.z /= v;
    return *this;
}
float Vector3::operator&=(const Vector3 &v)
{
    _Vector3_Data temp = v.Get();
    return vector3_Data.x * temp.x + vector3_Data.y * temp.y + vector3_Data.z * temp.z;
}

Vector3 Vector3::operator^=(const Vector3 &v)
{
    _Vector3_Data temp = v.Get();
    float xt, yt, zt;
    xt = vector3_Data.y * temp.z - temp.y * vector3_Data.z;
    yt = vector3_Data.z * temp.x - temp.z * vector3_Data.x;
    zt = vector3_Data.x * temp.y - temp.x * vector3_Data.y;
    return Vector3(xt, yt, zt);
}

Vector3 &Vector3::operator=(const Vector3 &v)
{
    _Vector3_Data temp = v.Get();
    vector3_Data.x     = temp.x;
    vector3_Data.y     = temp.y;
    vector3_Data.z     = temp.z;
    return *this;
}
