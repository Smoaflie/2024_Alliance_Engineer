using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Simulator : MonoBehaviour
{
    private Transform Up;
    private Transform LD;
    private Transform RD;
    public Transform Target;

    Vector3 up, left, right;

    public float arm0 = 0.067f, arm1 = 0.108f, arm2 = 0.211f, mg = 500;
    public float horizontal_rate = 2.0f; 
    public float vertical_rate = 0.6f; 
    public float depth_rate = 0.9f; 
    public Vector3 origin = new Vector3(0, 0.3957691f, 0.63f + 0.2071497f);
    public static bool SimulatorEnable = false,SimulatorPositionEnable = false,SimulatorTargetEnable = false; 
    // Start is called before the first frame update
    void Start()
    {
        up = new Vector3(0,arm0,0);
        float degree = 60 * Mathf.Deg2Rad;
        left = new Vector3(-arm0*Mathf.Sin(degree),-arm0*Mathf.Cos(degree),0);
        right = new Vector3(arm0*Mathf.Sin(degree),-arm0*Mathf.Cos(degree),0);

        up += origin;
        left += origin;
        right += origin;
    }

    // public float _Angle_up, _Angle_left, _Angle_right;
    private float _Angle_up => -Contro.encoder_Data[2];
    private float _Angle_left => -Contro.encoder_Data[1];
    private float _Angle_right => -Contro.encoder_Data[0];

    private float Angle_up => _Angle_up / 360 * 2 * Mathf.PI;
    private float Angle_left => _Angle_left / 360 * 2 * Mathf.PI;
    private float Angle_right => _Angle_right / 360 * 2 * Mathf.PI;

    private Vector3 TranslationOrigin,ControllerPositionOrigin;
    // Update is called once per frame
    void Update()
    {
        // up = new Vector3(0,arm0,0);
        // float degree = 60 * Mathf.Deg2Rad;
        // left = new Vector3(-arm0*Mathf.Sin(degree),-arm0*Mathf.Cos(degree),0);
        // right = new Vector3(arm0*Mathf.Sin(degree),-arm0*Mathf.Cos(degree),0);

        // up += origin;
        // left += origin;
        // right += origin;

        Calcu(out Vector3 tar);

        if(SimulatorPositionEnable)
        {
            Target.localPosition = TranslationOrigin + (tar - ControllerPositionOrigin);
        }
        else 
        {
            TranslationOrigin = Target.localPosition;
            ControllerPositionOrigin = tar;
            if(SimulatorTargetEnable)
            {
                Target.localPosition = tar;

                Target.localRotation = Contro.qua_rec;// * Quaternion.Inverse(Quaternion.Euler(0,0,Contro.qua_rec.eulerAngles.z));

                Target.position = Arm.ArmBase.rotation * Target.position;
                Target.rotation = Arm.ArmBase.rotation * Target.rotation;
            }
        }
    }

    void Calcu(out Vector3 tar)
    {
        Vector3 fup = up + new Vector3(0, arm1 * Mathf.Cos(Angle_up), -arm1 * Mathf.Sin(Angle_up));
        Vector3 fle = left + Quaternion.AngleAxis(120, Vector3.forward) * new Vector3(0, arm1 * Mathf.Cos(Angle_left), -arm1 * Mathf.Sin(Angle_left));
        Vector3 fri = right + Quaternion.AngleAxis(-120, Vector3.forward) * new Vector3(0, arm1 * Mathf.Cos(Angle_right), -arm1 * Mathf.Sin(Angle_right));

        Debug.DrawLine(fup, up, Color.blue, -1);
        Debug.DrawLine(fle, left, Color.blue, -1);
        Debug.DrawLine(fri, right, Color.blue, -1);

        centerCircle3d(fup, fle, fri, out Vector3 centry, out float radius);
        Vector3 centryforward = Vector3.Cross(fup - fle, fri - fup).normalized;

        tar = centryforward * Mathf.Sqrt(arm2 * arm2 - radius * radius) + centry;

        // Debug.DrawLine(fup, tar, Color.red, -1);
        // Debug.DrawLine(fle, tar, Color.red, -1);
        // Debug.DrawLine(fri, tar, Color.red, -1);

        Vector3 offset = tar-origin;
        tar.x = origin.x + offset.x * horizontal_rate;
        tar.y = origin.y + offset.y *  vertical_rate;
        tar.z = origin.z + offset.z *  depth_rate;

        Debug.DrawLine(fup, tar, Color.green, -1);
        Debug.DrawLine(fle, tar, Color.green, -1);
        Debug.DrawLine(fri, tar, Color.green, -1);
    }
    //x1,y1,z1对应一个点的坐标
    //x,y,z,radius是用来返回求解出来的圆心坐标和圆半径
    void centerCircle3d(Vector3 v1, Vector3 v2, Vector3 v3, out Vector3 center, out float radius)
    {
        float x1 = v1.x;
        float x2 = v2.x;
        float x3 = v3.x;
        float y1 = v1.y;
        float y2 = v2.y;
        float y3 = v3.y;
        float z1 = v1.z;
        float z2 = v2.z;
        float z3 = v3.z;

        float a1 = y1 * z2 - y2 * z1 - y1 * z3 + y3 * z1 + y2 * z3 - y3 * z2,
            b1 = -(x1 * z2 - x2 * z1 - x1 * z3 + x3 * z1 + x2 * z3 - x3 * z2),
            c1 = x1 * y2 - x2 * y1 - x1 * y3 + x3 * y1 + x2 * y3 - x3 * y2,
            d1 = -(x1 * y2 * z3 - x1 * y3 * z2 - x2 * y1 * z3 + x2 * y3 * z1 + x3 * y1 * z2 - x3 * y2 * z1);

        float a2 = 2 * (x2 - x1),
            b2 = 2 * (y2 - y1),
            c2 = 2 * (z2 - z1),
            d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

        float a3 = 2 * (x3 - x1),
            b3 = 2 * (y3 - y1),
            c3 = 2 * (z3 - z1),
            d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

        float x = center.x = -(b1 * c2 * d3 - b1 * c3 * d2 - b2 * c1 * d3 + b2 * c3 * d1 + b3 * c1 * d2 - b3 * c2 * d1)
            / (a1 * b2 * c3 - a1 * b3 * c2 - a2 * b1 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a3 * b2 * c1);
        float y = center.y = (a1 * c2 * d3 - a1 * c3 * d2 - a2 * c1 * d3 + a2 * c3 * d1 + a3 * c1 * d2 - a3 * c2 * d1)
              / (a1 * b2 * c3 - a1 * b3 * c2 - a2 * b1 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a3 * b2 * c1);
        float z = center.z = -(a1 * b2 * d3 - a1 * b3 * d2 - a2 * b1 * d3 + a2 * b3 * d1 + a3 * b1 * d2 - a3 * b2 * d1)
             / (a1 * b2 * c3 - a1 * b3 * c2 - a2 * b1 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a3 * b2 * c1);
        radius = Mathf.Sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y) + (z1 - z) * (z1 - z));
    }


    /// <summary>
    /// Input Target position
    /// return Torque of three motor
    /// RD 是你看着控制器正面时右边的电机
    /// </summary>
    /// <param name="Target"></param>
    /// <param name="TorqueUp"></param>
    /// <param name="TorqueLeft"></param>
    /// <param name="TorqueRight"></param>
    void CalcuTorque(in Vector3 Target, out float TorqueUp, out float TorqueLeft, out float TorqueRight)
    {
        Vector3 fup = up + new Vector3(0, arm1 * Mathf.Cos(Angle_up), -arm1 * Mathf.Sin(Angle_up));
        Vector3 fle = left + Quaternion.AngleAxis(120, Vector3.forward) * new Vector3(0, arm1 * Mathf.Cos(Angle_left), arm1 * Mathf.Sin(Angle_left));
        Vector3 fri = right + Quaternion.AngleAxis(-120, Vector3.forward) * new Vector3(0, arm1 * Mathf.Cos(Angle_right), arm1 * Mathf.Sin(Angle_right));

        Vector3 delta_1 = (Target - fup).normalized;
        Vector3 delta_2 = (Target - fle).normalized;
        Vector3 delta_3 = (Target - fri).normalized;

        Vector3 alpha = new(delta_1.y / delta_1.x, delta_2.y / delta_2.x, delta_3.y / delta_3.x);
        Vector3 beta = new(delta_1.y / delta_1.z, delta_2.y / delta_2.z, delta_3.y / delta_3.z);

        /// <summary>
        /// 矩阵求方程组
        /// </summary>
        /// <returns></returns>
        Vector3 v1 = new(1, 1, 1); float b1 = mg;
        Vector3 v2 = new(0, alpha.y - alpha.x, alpha.z - alpha.x); float b2 = -mg * alpha.x;
        Vector3 v3 = new(0, 0, beta.z - beta.x - v2.z / v2.y * (beta.y - beta.x)); float b3 = -mg * beta.x + mg * alpha.x / v2.y * (beta.y - beta.x);


        b1 -= b3 / v3.z;
        b2 -= b3 / v3.z * v2.z;

        b1 -= b2 / v2.y;


        //长臂方向的力
        Vector3 F1 = new(0, b1, 0), F2 = new(0, b2 / v2.y, 0), F3 = new(0, b3 / v3.y, 0);
        F1.x = F1.y / alpha.x;
        F1.z = F1.y / beta.x;
        F2.x = F2.y / alpha.y;
        F2.z = F2.y / beta.y;
        F3.x = F3.y / alpha.z;
        F3.z = F3.y / beta.z;

        Vector3 torqueUp = Vector3.Cross(Up.right, fup - up).normalized;
        TorqueUp = Vector3.Dot(torqueUp, F1);


        Vector3 torqueLeft = Vector3.Cross(LD.right, fle - left).normalized;
        TorqueLeft = Vector3.Dot(torqueLeft, F2);


        Vector3 torqueRight = Vector3.Cross(RD.right, fri - right).normalized;
        TorqueRight = Vector3.Dot(torqueRight, F3);
    }
}
