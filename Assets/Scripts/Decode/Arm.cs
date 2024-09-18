using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class Arm : MonoBehaviour
{
    static float arm1 = 0.227f, arm2 = 0.208f, arm3 = 0.2035f, arm4 = 0;//arm4 = 0.057f;//arm4 = 0.06868f;arm3 = 0.203;
    public static Transform ArmBase, BigYaw, Height, MidYaw, YawAndRoll, Pitch, Roll, Target;  
    public  Transform ArmBase_;
    public  Transform BigYaw_;
    public  Transform Height_;
    public  Transform MidYaw_;
    public  Transform YawAndRoll_;
    public  Transform Pitch_;
    public  Transform Roll_;

    public  Transform Target_;
    public Transform GeneratedTarget;
    public Renderer objectRenderer; // 需要改变颜色的物体的 Renderer
    public Text uiText; // 拖放你的 Text 对象到此变量中

    public enum ErrorCode_
    {
        None = 0,
        CannotGetTarget = 1 << 0,   // 1
        BigYawOutRange = 1 << 1,    // 2
        MidYawOutRange = 1 << 2,    // 4
        YawAndRollOutRange = 1 << 3, // 8
        PitchOutRange = 1 << 4      // 16
    }
    ErrorCode_ errorCode = ErrorCode_.None;

    public static bool ChangeDecodeFuncBool = false;
    public struct LastTarget { public static Vector3 Pos; public static Quaternion Rotate;};
    public struct LastJoint {   public static Quaternion LastBigYaw;
                                public static Vector3 LastHeight;
                                public static Quaternion LastMidYaw;
                                public static Quaternion LastAssorted;
                                public static Quaternion LastPitch;};
    public static Quaternion LastBigYawRotate;
    public static Quaternion LastAssortedYawRotate;
    static float Height_origin = 0.3957691f;
    // Update is called once per frame

    void Start()
    {
        ArmBase = ArmBase_;
        BigYaw = BigYaw_;
        MidYaw = MidYaw_;
        YawAndRoll = YawAndRoll_;
        Height = Height_;
        Pitch = Pitch_;
        Roll = Roll_;
        Target = Target_;

        Target.position = new Vector3(0,Height_origin,arm1+arm2+arm3+arm4);
        Target.rotation = Quaternion.identity;
        LastTarget.Pos = Target.localPosition;
        LastTarget.Rotate = Target.localRotation;
        LastBigYawRotate = transform.rotation;
        LastAssortedYawRotate = YawAndRoll.rotation;

        LastJoint.LastBigYaw = BigYaw.rotation;
        LastJoint.LastMidYaw = MidYaw.rotation;
        LastJoint.LastAssorted = YawAndRoll.rotation;
        LastJoint.LastPitch = Pitch.rotation;
        LastJoint.LastHeight = Height.position;
    }
    bool debug = false;
    void Update()
    {
        if(Input.GetKeyDown(KeyCode.M))
            debug = !debug;
        uiText.text = "";

        // if(Input.GetKeyDown(KeyCode.P))
        if(Contro.ArmControMode == Contro.ArmControMode_.ArmCustomContro)
        {
            decode();
            generateTarget();
            examineTarget();
            customControlErrorExamine();
        }
        else if(Contro.ArmControMode == Contro.ArmControMode_.TargetDecode)
        {
            decode();
            generateTarget();
            examineTarget();
            decodeErrorExamine();
            moveTarget();   
        }
        else if(Contro.ArmControMode == Contro.ArmControMode_.ArmGenerate)
        {
            moveTarget();   
            examineTarget();
            moveTargetErrorExamine();
        }
        else if(Contro.ArmControMode == Contro.ArmControMode_.ArmReset)
        {
            Target.localPosition = new Vector3(0,Height.position.y,0.48f+arm4);
            Target.rotation = Quaternion.identity;
            ArmBase.rotation = Quaternion.identity;
        
            decode();
            moveTarget();
            examineTarget();
            decodeErrorExamine();
        }
        else if(Contro.ArmControMode == Contro.ArmControMode_.MCU)
        {
            decode();
            generateTarget();
            examineTarget();
            customControlErrorExamine();
        }
        uiText.text += "cubePosition" + Roll.position.ToString() + "\n";
    }
    private bool IsTransformEqual(Transform a, Transform b)
    {
        float positionTolerance = 0.03f; // 位置容忍度
        float rotationTolerance = 1.0f; // 旋转容忍度 (以度为单位)

        bool positionEqual = Mathf.Abs(a.position.x - b.position.x) < positionTolerance && Mathf.Abs(a.position.z - b.position.z) < positionTolerance;
        bool rotationEqual = Mathf.DeltaAngle(a.eulerAngles.x, b.eulerAngles.x) < rotationTolerance &&
                             Mathf.DeltaAngle(a.eulerAngles.y, b.eulerAngles.y) < rotationTolerance;
        return positionEqual && rotationEqual;
    }
    private bool IsJointAngleInRange(float angle, float min, float max)
    {
        // 将角度转换到 [0, 360] 范围内
        angle = angle % 360;
        if (angle < 0)
            angle += 360;

        // 将角度转换到 [-180, 180] 范围内
        if (angle > 180)
            angle -= 360;

        // 限制角度在指定范围内
        return angle >= min && angle <= max;
    }
    float Reform(float t)
    {
        return -(t > 180 ? t - 360 : t);
    }

    void decode()
    {
        float AssortedYaw_offset = 0;
        if(Vector3.Dot(Target.forward, ArmBase.forward) < 0)
        {
            Vector3 t3;
            if(Vector3.Dot(Target.forward, ArmBase.right) < 0)
            {
                t3 = Quaternion.Euler(0, 90, 0) * Target.forward;
                t3.y = 0;
                AssortedYaw_offset  = -Vector3.Angle(t3, ArmBase.forward);
            }
            else
            {
                t3 = Quaternion.Euler(0, -90, 0) * Target.forward;
                t3.y = 0;
                AssortedYaw_offset  = Vector3.Angle(t3, ArmBase.forward);
            }
        }
        // a[0] - Height
        float[] a = new float[6];
        a[0] = Target.localPosition.y;
        Height.localPosition = new Vector3(0, a[0], 0);

        // a[1] - BigYaw
        Vector3 vec = Target.position - Target.forward * arm4 - Quaternion.Euler(0, AssortedYaw_offset, 0) * ArmBase.forward * arm3;
        // Vector3 vec = Target.localPosition - ArmBase.InverseTransformDirection(Target.forward) * arm4 - Quaternion.Euler(0, AssortedYaw_offset, 0) * Vector3.forward * arm3;
        // Vector3 vec = Target.localPosition - ArmBase.InverseTransformDirection(Target.forward) * arm4 - arm3 * Vector3.forward;
        vec.y = 0;
        Debug.DrawLine(vec, Vector3.zero);
        float n = vec.magnitude;
        float num = (float)Math.Sqrt(ArmBase.right.sqrMagnitude * vec.sqrMagnitude);
        float num2 = Mathf.Clamp(Vector3.Dot(ArmBase.forward, vec) / num, -1f, 1f);
        float a1 = (float)Math.Acos(num2) * Mathf.Rad2Deg;
        a1 = Mathf.Acos(Mathf.Clamp((arm1 * arm1 - arm2 * arm2 + n * n) / 2 / arm1 / n, -1, 1)) * Mathf.Rad2Deg + (vec.x >= 0 ? a1 : -a1);
        a[1] = a1;

        // a[2] - MidYaw
        a1 = Mathf.Clamp((arm2 * arm2 + arm1 * arm1 - n * n) / 2 / arm1 / arm2, -1, 1);
        a1 = - (180 - Mathf.Acos(a1) * Mathf.Rad2Deg);
        a[2] = a1;

        // 按需转换左右解
        if(ChangeDecodeFuncBool)
        {
            a1 = Mathf.Acos(Mathf.Clamp((arm2 * arm2 + n * n - arm1 * arm1) / 2 / arm2 / n, -1, 1)) * Mathf.Rad2Deg;
            a[1] = a[1] - 2 * (180 - (180 + a[2]) - a1);
            a[2] = -a[2];
        }
        BigYaw.localEulerAngles = new Vector3(0, a[1], 0);
        MidYaw.localEulerAngles = new Vector3(0, a[2], 0);

        // a[3] - AssortedYaw
        vec = MidYaw.right;
        num = (float)Math.Sqrt(ArmBase.forward.sqrMagnitude * vec.sqrMagnitude);
        // num2 = Mathf.Clamp(Vector3.Dot(Vector3.forward, vec) / num, -1f, 1f);
        num2 = Mathf.Clamp(Vector3.Dot(Quaternion.Euler(0, AssortedYaw_offset, 0) * ArmBase.forward, vec) / num, -1f, 1f);
        a1 = 90 - (float)Math.Acos(num2) * Mathf.Rad2Deg;
        a[3] = a1;

        // YawAndRoll.localEulerAngles = new Vector3(0, a[3], YawAndRoll.localEulerAngles.z);
        // a[4] - AssortedRoll
        a[4] = 0;
        if(Target.forward != ArmBase.forward)
        {
            Vector3 n1 = Vector3.Cross(YawAndRoll.forward, Target.forward);
            Vector3 n2 = YawAndRoll.right;
            num = (float)Math.Sqrt(n1.sqrMagnitude * n2.sqrMagnitude);
            num2 = Mathf.Clamp(Vector3.Dot(n1, n2) / num, -1f, 1f);
            a1 = (float)Math.Acos(num2) * Mathf.Rad2Deg * Mathf.Sign(Vector3.Dot(n1,YawAndRoll.up));

            // if (n1.x > 0)
            //     a1 = -a1;

            float mul_angle = Vector3.Dot(n1, n2);
            if(mul_angle >= 0)
                a[4] = a1;
            else
                a[4] = a1+180;
            a[4] = YawAndRoll.localEulerAngles.z + a[4];

        }
        YawAndRoll.localEulerAngles = new Vector3(0, a[3], a[4]);
            
            
        // a[5] - TailAngle
        num = (float)Math.Sqrt(YawAndRoll.up.sqrMagnitude * Target.forward.sqrMagnitude);
        num2 = Mathf.Clamp(Vector3.Dot(YawAndRoll.up, Target.forward) / num, -1f, 1f);
        a1 = (float)Math.Acos(num2) * Mathf.Rad2Deg - 90;
        a[5] = a1;
        Pitch.localEulerAngles = new Vector3(a[5], 0, 0);
    }
    void customControlErrorExamine()
    {
        // 对错误进行处理
        if (errorCode == ErrorCode_.None)
        {
            // 设置颜色为绿色
            objectRenderer.material.color = Color.green;

            LastTarget.Pos = Target.localPosition;
            LastTarget.Rotate = Target.localRotation;
            LastBigYawRotate = transform.rotation;
            LastAssortedYawRotate = YawAndRoll.rotation;

            LastJoint.LastBigYaw = BigYaw.rotation;
            LastJoint.LastMidYaw = MidYaw.rotation;
            LastJoint.LastAssorted = YawAndRoll.rotation;
            LastJoint.LastPitch = Pitch.rotation;
            LastJoint.LastHeight = Height.position;
        }
        else
        {
            // 设置颜色为红色
            objectRenderer.material.color = Color.red;
            
            // transform.rotation = LastBigYawRotate;
            // Target.localPosition = LastTarget.Pos;
            // Target.localRotation = LastTarget.Rotate;

            // BigYaw.rotation = LastJoint.LastBigYaw;
            // MidYaw.rotation = LastJoint.LastMidYaw;
            // YawAndRoll.rotation = LastJoint.LastAssorted ;
            // Pitch.rotation = LastJoint.LastPitch;
            // Height.position = LastJoint.LastHeight;
        }
    }
    void decodeErrorExamine()
    {
        // 对错误进行处理
        if (errorCode == ErrorCode_.None)
        {
            // 设置颜色为绿色
            objectRenderer.material.color = Color.green;

            LastTarget.Pos = Target.localPosition;
            LastTarget.Rotate = Target.localRotation;
            LastBigYawRotate = transform.rotation;
            LastAssortedYawRotate = YawAndRoll.rotation;

            LastJoint.LastBigYaw = BigYaw.rotation;
            LastJoint.LastMidYaw = MidYaw.rotation;
            LastJoint.LastAssorted = YawAndRoll.rotation;
            LastJoint.LastPitch = Pitch.rotation;
            LastJoint.LastHeight = Height.position;
        }
        else
        {
            // 设置颜色为红色
            objectRenderer.material.color = Color.red;

            transform.rotation = LastBigYawRotate;
            Target.localPosition = LastTarget.Pos;
            Target.localRotation = LastTarget.Rotate;

            BigYaw.rotation = LastJoint.LastBigYaw;
            MidYaw.rotation = LastJoint.LastMidYaw;
            YawAndRoll.rotation = LastJoint.LastAssorted ;
            Pitch.rotation = LastJoint.LastPitch;
            Height.position = LastJoint.LastHeight;
        }
    }
    void moveTargetErrorExamine()
    {
        if (errorCode == ErrorCode_.None)
        {
            // 设置颜色为绿色
            objectRenderer.material.color = Color.green;

            LastTarget.Pos = Target.localPosition;
            LastTarget.Rotate = Target.localRotation;
            LastBigYawRotate = transform.rotation;
            LastAssortedYawRotate = YawAndRoll.rotation;

            LastJoint.LastBigYaw = BigYaw.rotation;
            LastJoint.LastMidYaw = MidYaw.rotation;
            LastJoint.LastAssorted = YawAndRoll.rotation;
            LastJoint.LastPitch = Pitch.rotation;
            LastJoint.LastHeight = Height.position;
        }
        else
        {
            // 设置颜色为红色
            objectRenderer.material.color = Color.red;

            // transform.rotation = LastBigYawRotate;
            // Target.localPosition = LastTarget.Pos;
            // Target.localRotation = LastTarget.Rotate;
            // YawAndRoll.rotation = LastAssortedYawRotate;
            
            // BigYaw.rotation = LastJoint.LastBigYaw;
            // MidYaw.rotation = LastJoint.LastMidYaw;
            // YawAndRoll.rotation = LastJoint.LastAssorted ;
            // Pitch.rotation = LastJoint.LastPitch;
            // Height.position = LastJoint.LastHeight;
        }
    }
    void examineTarget()
    {
        errorCode = (ErrorCode_)(
                ((IsTransformEqual(Target, GeneratedTarget) ? 0 : 1) << 0) |
                ((IsJointAngleInRange(BigYaw.eulerAngles.y, -120, 120) ? 0 : 1) << 1) |
                ((IsJointAngleInRange(MidYaw.localEulerAngles.y, -105, 105) ? 0 : 1) << 2) |
                ((IsJointAngleInRange(YawAndRoll.localEulerAngles.y, -90, 90) ? 0 : 1) << 3) |
                ((IsJointAngleInRange(Pitch.localEulerAngles.x, -90, 90) ? 0 : 1) << 4)
                 );

        uiText.text += "ErrorCode = " + errorCode.ToString() + "\n";
    }
    void generateTarget()
    {
        /* 根据关节角度求末端位姿 */
        Vector3 endPosition;
        Quaternion endRotation;
        endPosition = Vector3.zero;
        endRotation = Quaternion.identity;
        
        Quaternion currentRotation = Quaternion.Euler(0, BigYaw.eulerAngles.y, 0);
        endRotation *= currentRotation;
        Vector3 translation = new Vector3(0, 0, arm1);
        endPosition += endRotation * translation;

        currentRotation = Quaternion.Euler(0, MidYaw.localEulerAngles.y, 0);
        endRotation *= currentRotation;
        translation = new Vector3(0, 0, arm2);
        endPosition += endRotation * translation;

        currentRotation = Quaternion.Euler(0, YawAndRoll.localEulerAngles.y, YawAndRoll.localEulerAngles.z);
        endRotation *= currentRotation;
        translation = new Vector3(0, 0, arm3);
        endPosition += endRotation * translation;

        currentRotation = Quaternion.Euler(Pitch.localEulerAngles.x, 0, 0);
        endRotation *= currentRotation;
        translation = new Vector3(0, 0, arm4);
        endPosition += endRotation * translation;

        Vector3 euler = endRotation.eulerAngles;
        euler.z = 0;
        endRotation = Quaternion.Euler(euler);

        GeneratedTarget.position = endPosition;
        GeneratedTarget.rotation = endRotation;
    }
    void moveTarget()
    {
        /* 根据关节角度求末端位姿 */
        Vector3 endPosition;
        Quaternion endRotation;
        endPosition = Vector3.zero;
        endRotation = Quaternion.identity;


        Quaternion currentRotation = Quaternion.Euler(0, BigYaw.eulerAngles.y, 0);
        endRotation *= currentRotation;
        Vector3 translation = new Vector3(0, 0, arm1);
        endPosition += endRotation * translation;

        currentRotation = Quaternion.Euler(0, MidYaw.localEulerAngles.y, 0);
        endRotation *= currentRotation;
        translation = new Vector3(0, 0, arm2);
        endPosition += endRotation * translation;

        currentRotation = Quaternion.Euler(0, YawAndRoll.localEulerAngles.y, YawAndRoll.localEulerAngles.z);
        endRotation *= currentRotation;
        translation = new Vector3(0, 0, arm3);
        endPosition += endRotation * translation;

        Quaternion BigYawAngle = Quaternion.Euler(0, endRotation.eulerAngles.y, 0);
        Quaternion BigYawOffsetAngle = Quaternion.Inverse(transform.rotation * Quaternion.Inverse(BigYawAngle));

        currentRotation = Quaternion.Euler(Pitch.localEulerAngles.x, 0, 0);
        endRotation *= currentRotation;
        translation = new Vector3(0, 0, arm4);
        endPosition += endRotation * translation;

        endPosition.y = Height.position.y; 
        
        Vector3 euler = endRotation.eulerAngles;
        euler.z = 0;
        endRotation = Quaternion.Euler(euler);

        Target.position = endPosition;
        Target.rotation = endRotation;

        ArmBase.rotation *= BigYawOffsetAngle;
        BigYawOffsetAngle = Quaternion.Inverse(BigYawOffsetAngle);
        BigYaw.rotation *= BigYawOffsetAngle;
        // MidYaw.rotation *= BigYawOffsetAngle;
        // YawAndRoll.rotation *= BigYawOffsetAngle;
        // Pitch.rotation *= BigYawOffsetAngle;

        GeneratedTarget.position = Target.position;
        GeneratedTarget.rotation = Target.rotation;
    }

    public static void SetJointAngle(float[] joint_angle)
    {
        ArmBase.rotation = Quaternion.identity;
        BigYaw.localRotation = Quaternion.Euler(0, -joint_angle[0], 0);
        Height.position = new Vector3(0, joint_angle[1], 0);
        MidYaw.localRotation = Quaternion.Euler(0, -joint_angle[2], 0);
        YawAndRoll.localRotation = Quaternion.Euler(0, -joint_angle[3], -joint_angle[4]);
        Pitch.localRotation = Quaternion.Euler(joint_angle[5], 0, 0);
    }
    public static void BigYawRotate(Vector3 rotation)
    {
        BigYaw.Rotate(rotation, Space.World);
    }
    public static void MidYawRotate(Vector3 rotation)
    {
        MidYaw.Rotate(rotation, Space.World);
    }
    public static void AssortedYawRotate(Vector3 rotation)
    {
        YawAndRoll.Rotate(rotation, Space.World);
    }
    public static void TargetRotate(Vector3 rotation)
    {
        Target.Rotate(rotation, Space.Self);
    }
    public static void TargetTranslate_baseWorld(Vector3 translation)
    {
        Target.position += ArmBase.TransformDirection(translation);
    }
    public static void TargetTranslate_baseSucker(Vector3 translation)
    {
        Target.position += Target.TransformDirection(translation);
    }
    public static void TargetRotate_Joint(float Up_Down, float Left_Right)
    {
        YawAndRoll.Rotate(new Vector3(0, 0, Left_Right), Space.Self);
        Pitch.Rotate(new Vector3(Up_Down, 0, 0), Space.Self);
    }
}
