#ifndef CONTROLLER_UM_HPP
#define CONTROLLER_UM_HPP

#include "vector3.hpp"

typedef struct _Attitue {
    Vector3 Position;
    Vector3 EularAngle;
} Attitue, *PAttitue;


class Controller
{
private:
    float *me02_Data;
    float *mpu_data;
    /* data */
    Attitue attitue;

public:
    Controller(float *me02, float *mpu);
    ~Controller();

    Attitue GetAttitue();
    void UpdateAttitue();
    void ZeroController();
};


#endif // !CONTROLLER_UM_HPP