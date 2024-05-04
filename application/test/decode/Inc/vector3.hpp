#ifndef VECTOR3_HPP
#define VECTOR3_HPP

class Vector3
{
private:
    struct _Vector3_Data {
        float x;
        float y;
        float z;
    } vector3_Data;

public:
    Vector3(/* args */);
    Vector3(float x, float y, float z);
    ~Vector3();

    float Dot(const Vector3 &v) const;
    float Distance(const Vector3 &v) const;
    Vector3 Cross(const Vector3 &v) const;

    float normalize() const;
    Vector3 &times(const Vector3 &v);

    _Vector3_Data Get() const;

    Vector3 &operator+=(const Vector3 &v);
    Vector3 &operator-=(const Vector3 &v);
    Vector3 operator+(const Vector3 &v);
    Vector3 operator-(const Vector3 &v);
    Vector3 &operator*=(const float &v);
    Vector3 &operator/=(const float &v);
    // dot
    float operator&=(const Vector3 &v);
    // cross
    Vector3 operator^=(const Vector3 &v);
    Vector3 &operator=(const Vector3 &v);
};

#endif
