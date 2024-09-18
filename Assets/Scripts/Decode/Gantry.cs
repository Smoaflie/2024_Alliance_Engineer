using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class Gantry : MonoBehaviour
{
    public Transform Height;
    public Transform ConnectArm;
    public Transform RightArm;
    public Transform LeftArm;
    public Transform TailX;
    public Transform Tail;
    public Transform PitchAndRoll;
    public Transform Target;


    const float connectArmLen = 0.45f, destLenZ = 0.16f, tailXSize = 0.17f, tailSize = 0.15f, detectSize = 0.06f;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        Vector3 vec = Target.forward;
        vec.y = 0;
        vec /= vec.magnitude;
        float num = Mathf.Clamp(Vector3.Dot(Vector3.forward, vec), -1f, 1f);
        TailX.localEulerAngles = new Vector3(0, Mathf.Acos(num) * Mathf.Rad2Deg * Mathf.Sign(Vector3.Dot(Vector3.right, vec)), 0);

        num = Mathf.Clamp(Vector3.Dot(Vector3.forward, vec), -1f, 1f);
        float a1 = Mathf.Acos(num) * Mathf.Rad2Deg * Mathf.Sign(Vector3.Dot(Vector3.right, vec));
        float a2 = Mathf.Sin(Mathf.Abs(a1) * Mathf.Deg2Rad) * tailXSize;

        Debug.Log(a2);

        if (a1 > 0)
        {
            RightArm.localPosition = new Vector3(RightArm.localPosition.x, 0, connectArmLen - a2);
            LeftArm.localPosition = new Vector3(LeftArm.localPosition.x, 0, connectArmLen);
        }
        else
        {
            RightArm.localPosition = new Vector3(RightArm.localPosition.x, 0, connectArmLen);
            LeftArm.localPosition = new Vector3(LeftArm.localPosition.x, 0, connectArmLen - a2);
        }

        vec = Target.position - Target.forward * detectSize - vec.normalized * tailSize;
        Height.localPosition = new Vector3(Height.localPosition.x, vec.y, Height.localPosition.z);

        vec.y = 0;

        a2 = vec.x / Mathf.Cos(Mathf.Deg2Rad);
        Vector3 vec2 = RightArm.localPosition - LeftArm.localPosition;
        vec2 = (RightArm.localPosition + LeftArm.localPosition) / 2 + a2 * vec2 / vec2.magnitude + destLenZ * Vector3.forward;
        a1 = vec.z - vec2.z;
        if (a1 > 0)
            ConnectArm.localPosition = new Vector3(ConnectArm.localPosition.x, 0, a1);
        else
        {
            RightArm.localPosition += a1 * Vector3.forward;
            LeftArm.localPosition += a1 * Vector3.forward;
        }

        TailX.position = (RightArm.position + LeftArm.position) / 2 + destLenZ * Vector3.forward;
        Tail.localPosition = a2 * Vector3.right + tailSize * Vector3.forward;


        num = Mathf.Clamp(Vector3.Dot(Target.forward, Tail.forward), -1f, 1f);
        a1 = -Mathf.Acos(num) * Mathf.Rad2Deg * Mathf.Sign(Vector3.Dot(Vector3.up, Target.forward));
        PitchAndRoll.localEulerAngles = new Vector3(a1, 0, 0);
    }
}
