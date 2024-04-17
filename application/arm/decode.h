#pragma once
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "stdio.h"

typedef struct {
    float big_yaw_angle;
    float height;
    float mid_yaw_angle;
    float assorted_yaw_angle;
    float assorted_roll_angle;
    float tail_motor_angle;
} arm_controller_data_s;

typedef struct {
    float x;
    float y;
    float z;
} Vector3;

typedef struct {
    float x;
    float y;
    float z;
    float w;
} Quaternion;

typedef struct {
    Vector3 localPosition;
    Quaternion localRotation;
} Transform;

typedef struct _Matrix {
    int row;
    int column;
    float *data;
} Matrix;

#define row_max    4
#define column_max 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef DEG2RAD
#define DEG2RAD 3.1416 / 360.0
#endif
#ifndef limit
#define limit(value, max, min) ((value) > (max) ? (max) : ((value) < (min) ? (min) : (value)))
#endif
#ifndef limit_bool
#define limit_bool(value, max, min) ((value) > (max) ? 0 : ((value) < (min) ? 0 : 1))
#endif

// 获得向量的模
#define vectorMagnitude(v) (sqrtf(v.x * v.x + v.y * v.y + v.z * v.z))
// 向量点乘
#define vectorDotProduct(v1, v2) (((v1).x * (v2).x + (v1).y * (v2).y + (v1).z * (v2).z))

const float height = 0.3, arm1 = 0.26f, arm2 = 0.21f, arm3 = 0.16f, arm4 = 0.05f;

static float Matrix_data_keep[row_max * column_max]; // 矩阵数据保留空间
static Matrix Matrix_keep = {4,4,Matrix_data_keep};

arm_controller_data_s pub = {0};

Vector3 Vector3_sub(Vector3 a, Vector3 b)
{
    Vector3 c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
}
Vector3 Vector3_mul(float a, Vector3 b)
{
    Vector3 c;
    c.x = a * b.x;
    c.y = a * b.y;
    c.z = a * b.z;
    return c;
}
// 向量叉乘
Vector3 crossProduct(Vector3 v1, Vector3 v2)
{
    Vector3 result;
    result.x = v1.y * v2.z - v2.y * v1.z;
    result.y = v1.z * v2.x - v2.z * v1.x;
    result.z = v1.x * v2.y - v2.x * v1.y;
    return result;
}

// 四元数转向量
Vector3 quaternionToVector3(Quaternion q)
{
    float _mat[3][3];
    _mat[0][0] = 1 - 2 * q.y * q.y - 2 * q.z * q.z;
    _mat[0][1] = 2 * q.x * q.y - 2 * q.w * q.z;
    _mat[0][2] = 2 * q.x * q.z + 2 * q.w * q.y;

    _mat[1][0] = 2 * q.x * q.y + 2 * q.w * q.z;
    _mat[1][1] = 1 - 2 * q.x * q.x - 2 * q.z * q.z;
    _mat[1][2] = 2 * q.y * q.z - 2 * q.w * q.x;

    _mat[2][0] = 2 * q.x * q.z - 2 * q.w * q.y;
    _mat[2][1] = 2 * q.y * q.z + 2 * q.w * q.x;
    _mat[2][2] = 1 - 2 * q.x * q.x - 2 * q.y * q.y;

    Vector3 vector;
    vector.x = _mat[0][0]; // 主轴X分量
    vector.y = -_mat[1][0]; // 主轴Y分量
    vector.z = _mat[2][0]; // 主轴Z分量
    return vector;
}
// 求两个向量间夹角
float angleBetweenVector3(Vector3 v1, Vector3 v2)
{
    float dot       = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    float magV1     = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    float magV2     = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
    float angle_rad = acosf(limit(dot / (magV1 * magV2), 1, -1));
    // float angle_deg = angle_rad * 180.0 / M_PI;

    return angle_rad;
}
// 将变换矩阵转换为四元数
Quaternion transformToQuaternion(float *m)
{
    Quaternion q;
    float tr = ((float (*)[4])m)[0][0] + ((float (*)[4])m)[1][1] + ((float (*)[4])m)[2][2];

    if (tr > 0) {
        float s = sqrt(tr + 1.0);
        q.w      = s / 2.0;
        q.x      = (((float (*)[4])m)[2][1] - ((float (*)[4])m)[1][2]) / (2.0 * s);
        q.y      = (((float (*)[4])m)[0][2] - ((float (*)[4])m)[2][0]) / (2.0 * s);
        q.z      = (((float (*)[4])m)[1][0] - ((float (*)[4])m)[0][1]) / (2.0 * s);
    } else {
        int i, j, k;
        float q_[4];

        i = 0;
        if (((float (*)[4])m)[1][1] > ((float (*)[4])m)[0][0]) i = 1;
        if (((float (*)[4])m)[2][2] > ((float (*)[4])m)[i][i]) i = 2;

        j = (i + 1) % 3;
        k = (j + 1) % 3;

        float s = sqrt((((float (*)[4])m)[i][i] - ((float (*)[4])m)[j][j] - ((float (*)[4])m)[k][k]) + 1.0);

        q_[i] = s / 2.0;
        if (s != 0) s = 0.5 / s;

        q_[3] = (((float (*)[4])m)[k][j] - ((float (*)[4])m)[j][k]) * s;
        q_[j] = (((float (*)[4])m)[i][j] + ((float (*)[4])m)[j][i]) * s;
        q_[k] = (((float (*)[4])m)[i][k] + ((float (*)[4])m)[k][i]) * s;

        q.w = q_[3];
        q.x = q_[0];
        q.y = q_[1];
        q.z = q_[2];
    }

    float s = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w = q.w / s;
    q.x = q.x / s;
    q.y = q.y / s;
    q.z = q.z / s;
    return q;
}
// 矩阵加减法
Matrix *M_add_sub(Matrix *_mat_result, float scale_mat_subed, Matrix *_mat_subed, float scale_mat_minus, Matrix *_mat_minus)
{
    if ((_mat_subed->column == _mat_minus->column) && (_mat_subed->row == _mat_minus->row)) {
        int size = (_mat_subed->row) * (_mat_subed->column), i;

        _mat_result->column = _mat_minus->column;
        _mat_result->row    = _mat_minus->row;

        for (i = 0; i < size; i++) {
            _mat_result->data[i] = (_mat_result->data[i]) * scale_mat_subed + (_mat_minus->data[i]) * scale_mat_minus;
        }
    } else {
        return NULL; // Error
    }
    return _mat_result;
}
// 矩阵乘法
Matrix *M_mul(Matrix *_mat_result, Matrix *_mat_left, Matrix *_mat_right)
{
    if (_mat_left->column != _mat_right->row) {
        return NULL; // Error
    } else {
        int row    = _mat_left->row;
        int mid    = _mat_left->column;
        int column = _mat_right->column;
        int i, j, k;
        float temp          = 0;
        _mat_result->row    = row;
        _mat_result->column = column;

        /*Ergodic*/
        for (i = 0; i < row; i++) {
            for (j = 0; j < column; j++) {
                /*Caculate Element*/
                temp = 0;
                for (k = 0; k < mid; k++) {
                    temp += (_mat_left->data[i * mid + k]) * (_mat_right->data[k * column + j]);
                }
                _mat_result->data[i * column + j] = temp;
            }
        }
    }
    return _mat_result;
}
// 计算三角形两边(side1&side2)之间的夹角角度
float get_triangle_angle(float side_1, float side_2, float side_3)
{
    float num_cos   = (side_1 * side_1 + side_2 * side_2 - side_3 * side_3) / (2 * side_1 * side_2);
    float num_angle = acosf(limit(num_cos, 1, -1));
    // float num_angle_deg = num_angle * 180.0 / M_PI;
    return num_angle;
}

// 更新关节位姿及角度
uint8_t Update_angle(Transform target, arm_controller_data_s *pub_s)
{
    // 大yaw 计划不加入逆解
    pub.big_yaw_angle = 0;

    // 确定 yaw关节角度
    // 由于big_yaw不移动，将origin设为mid_yaw
    /* 变换矩阵命名约定：名称__参考坐标__(类型p/q)_m */
    // Matrix target__tail__p_m = {
    //     .column = 3,
    //     .row    = 1,
    // };
    float target__tail__p_m_data[3] = {arm4, 0, 0};
    Matrix target__tail__p_m        = {1, 3, target__tail__p_m_data};

    float target__origin__q_m_data[3][3] = {
        {1 - 2 * powf(target.localRotation.y, 2) - 2 * powf(target.localRotation.z, 2), 2 * (target.localRotation.x * target.localRotation.y - target.localRotation.z * target.localRotation.w), 2 * (target.localRotation.x * target.localRotation.z + target.localRotation.y * target.localRotation.w)},
        {2 * (target.localRotation.x * target.localRotation.y + target.localRotation.z * target.localRotation.w), 1 - 2 * powf(target.localRotation.x, 2) - 2 * powf(target.localRotation.z, 2), 2 * (target.localRotation.y * target.localRotation.z - target.localRotation.x * target.localRotation.w)},
        {2 * (target.localRotation.x * target.localRotation.z - target.localRotation.y * target.localRotation.w), 2 * (target.localRotation.y * target.localRotation.z + target.localRotation.x * target.localRotation.w), 1 - 2 * powf(target.localRotation.x, 2) - 2 * powf(target.localRotation.y, 2)},
    };

    Matrix target__origin__q_m = {3, 3, (float *)target__origin__q_m_data};

    Vector3 target_tail__origin__p;
    M_mul(&Matrix_keep, &target__tail__p_m, &target__origin__q_m);
    memcpy(&target_tail__origin__p, Matrix_keep.data, 3 * sizeof(float));

    Vector3 tail__origin__p = Vector3_sub(target.localPosition, target_tail__origin__p);

    // 确定 Z轴高度
    pub.height                 = tail__origin__p.z;
    Vector3 mid_yaw__origin__p = {arm1, 0, pub.height};
    // tail__mid_yaw__p arm2 arm3 组成一个三角形，根据几何关系计算mid_yaw和YawAndRoll的yaw旋转角度
    Vector3 tail__mid_yaw__p = Vector3_sub(tail__origin__p, mid_yaw__origin__p);
    float n                  = vectorMagnitude(tail__mid_yaw__p);
    // 如果构不成三角形
    if(n > (arm2+arm3)) return 0;
    Vector3 left_vector      = {0, 1, 0};
    float a1                 = angleBetweenVector3(left_vector, tail__mid_yaw__p);
    a1 += get_triangle_angle(n, arm2, arm3); // a1为左轴与mid_yaw之间的夹角
    pub.mid_yaw_angle = M_PI / 2 - a1;

    // 可同理得assorted_yaw的偏转角度
    float a2               = get_triangle_angle(arm2, arm3, n);
    a2                     = M_PI - a2;
    pub.assorted_yaw_angle = a2;
    /* @note 该解法能避免在(1,0,0)方向上有歧义解，但会导致assorted_yaw不会向反方向旋转 */

    // 确定 assorted_roll关节角度
    // 计算assorted__origin__q_m，通过两个欧拉角旋转矩阵相乘得到，过程略
    float assorted__origin__q_data[3][3] = {
        {cosf(pub.mid_yaw_angle) * cosf(pub.assorted_yaw_angle) - sinf(pub.mid_yaw_angle) * sinf(pub.assorted_yaw_angle), -cosf(pub.mid_yaw_angle) * sinf(pub.assorted_yaw_angle) - sinf(pub.mid_yaw_angle) * cosf(pub.assorted_yaw_angle), 0},
        {sinf(pub.mid_yaw_angle) * cosf(pub.assorted_yaw_angle) + cosf(pub.mid_yaw_angle) * sinf(pub.assorted_yaw_angle), -sinf(pub.mid_yaw_angle) * sinf(pub.assorted_yaw_angle) + cosf(pub.mid_yaw_angle) * cosf(pub.assorted_yaw_angle), 0},
        {0, 0, 1},
    };

    Matrix assorted__origin__q = {3, 3, (float *)assorted__origin__q_data};

    float tmp_data[3]             = {1, 0, 0};
    Matrix main_vector__origin__p = {3, 3, tmp_data};

    // 获得三条向量，通过法向量计算夹角
    M_mul(&Matrix_keep, &main_vector__origin__p, &assorted__origin__q);
    Vector3 assorted_main_vector__origin__p;
    memcpy(&assorted_main_vector__origin__p, Matrix_keep.data, 3 * 4);
    Vector3 target_p_vec = quaternionToVector3(target.localRotation);
    // 先计算tail_motor的旋转角度`大小`
    a1 = angleBetweenVector3(target_p_vec, assorted_main_vector__origin__p);
    // 如tail_motor未偏转，则assorted_roll保持不动（法向量计算法会出问题）
    if (a1 <= 0.0001 && a1 >= -0.0001)
        pub.assorted_roll_angle = pub.assorted_roll_angle;
    else {
        Vector3 left__assorted__vec = {assorted__origin__q_data[1][0], assorted__origin__q_data[1][1], 0};
        Vector3 n_1             = crossProduct(assorted_main_vector__origin__p, left__assorted__vec);
        Vector3 n_2       = crossProduct(assorted_main_vector__origin__p, target_p_vec);
        a2                = angleBetweenVector3(n_1, n_2);

        pub.assorted_roll_angle = a2 - M_PI/2;
    }

    // 再借助assorted_roll的偏转角度通过向量法计算tail_motor的偏转方向
    Vector3 up__assorted_roll__vec;
    float up__assorted_roll__q_data[] = {0, sinf(pub.assorted_roll_angle), cosf(pub.assorted_roll_angle)};
    Matrix  up__assorted_roll__q = {1,3,up__assorted_roll__q_data};
    M_mul(&Matrix_keep,&up__assorted_roll__q,&assorted__origin__q);
    memcpy(&up__assorted_roll__vec,Matrix_keep.data,3*sizeof(float));
    a1                             = angleBetweenVector3(up__assorted_roll__vec, target_p_vec);
    pub.tail_motor_angle           = M_PI/2 - a1;

    // 将弧度转换为角度
    pub.mid_yaw_angle       = pub.mid_yaw_angle * 180.0 / M_PI;
    pub.assorted_yaw_angle  = pub.assorted_yaw_angle * 180.0 / M_PI;
    pub.assorted_roll_angle = -pub.assorted_roll_angle * 180.0 / M_PI;
    pub.tail_motor_angle    = pub.tail_motor_angle * 180.0 / M_PI;
    if (limit_bool(pub.mid_yaw_angle, 90, -90) && limit_bool(pub.assorted_yaw_angle, 90, -90) &&
        limit_bool(pub.assorted_roll_angle, 180, -180) && limit_bool(pub.tail_motor_angle, 90, -90)) {
        memcpy(pub_s, &pub, sizeof(arm_controller_data_s));
        return 1;
    } else {
        return 0;
    }
}
