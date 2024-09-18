using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO.Ports;
using System.Linq;
using UnityEngine.Playables;
using System.Runtime.InteropServices;

public class CommunicateWithMCU : SeriesPort
{
    public static SerialPort serialPort;
    private static bool isSerialPortInitialized = false;
    private static byte[] frameHeader = { 0xFF, 0x52 };  // 帧头，根据实际情况修改
    public byte[] user_flag = new byte[1];

    public static bool MUC_ReceiveFlag = false;
    void Start()
    {
        if (!isSerialPortInitialized)
        {
            var portName = "/dev/ttyUSBSTM";
            var baudRate = 115200;

            // 初始化串口
            serialPort = new SerialPort(portName, baudRate);

            // 设置串口参数
            serialPort.Parity = Parity.None;
            serialPort.DataBits = 8;
            serialPort.StopBits = StopBits.One;
            serialPort.Handshake = Handshake.None;
            serialPort.ReadTimeout = 1000;
            serialPort.WriteTimeout = 1000;

            isSerialPortInitialized = true;

            ConnectSerialPort(serialPort);
        }
    }

    // Update is called once per frame
    void Update()
    {
        SerialReadData(serialPort, frameHeader, 36);
        SerialSendJointData();
    }

    protected override void ProcessData(byte[] data)
    {

        bool TranslateMode,RotateMode;
        float TranslateFront_Back, TranslateLeft_Right;
        float RotateUp_Down, RotateLeft_Right, YawRotation;
        string rec_log = "Data receive success: ";

        MUC_ReceiveFlag = true;
        user_flag[0] = data[0];

        Contro._KeyCode = 0;

        if (user_flag[0] == 0)
        {
            rec_log += "Mode=none";
        }
        else if ((user_flag[0] & 0x01) == 0x01)
        {
            for (int i = 0; i < Contro.encoder_Data.Length; i++)
            {
                Contro.encoder_Data[i] = BitConverter.ToSingle(data, 1 + i * sizeof(float));
            }

            if (data.Length >= 13 + sizeof(float) * 4)
            {
                // 左右手坐标系转换
                float w = BitConverter.ToSingle(data, 13);
                float x = -BitConverter.ToSingle(data, 13 + sizeof(float));
                float y = -BitConverter.ToSingle(data, 13 + sizeof(float) * 2);
                float z = BitConverter.ToSingle(data, 13 + sizeof(float) * 3);

                // 变换基坐标系
                Contro.qua_rec = new Quaternion(x, y, z, w);

                Contro.qua_rec *= Quaternion.Inverse(Quaternion.Euler(Contro.qua_rec.eulerAngles.x, 0, 0));

                Contro.qua_rec = new Quaternion(Contro.qua_rec.z, Contro.qua_rec.y, Contro.qua_rec.x, Contro.qua_rec.w);

                Contro.qua_rec *= Quaternion.Inverse(Quaternion.Euler(0, 0, Contro.qua_rec.eulerAngles.z));
            }
            Contro.SetControKeyCode(Contro.ControKeyCode.EnableCustomController);
            Contro.SetControKeyCode(Contro.ControKeyCode.CustomTargetContro);

            // Contro.ArmControMode = Contro.ArmControMode_.TargetDecode;
            rec_log += "Mode=current_target";
        }   
        else if ((user_flag[0] & 0x02) == 0x02)
        {
            for (int i = 0; i < Contro.encoder_Data.Length; i++)
            {
                Contro.encoder_Data[i] = BitConverter.ToSingle(data, 1 + i * sizeof(float));
            }
            Contro.SetControKeyCode(Contro.ControKeyCode.EnableCustomController);
            Contro.SetControKeyCode(Contro.ControKeyCode.CustomPositionContro);

            
            // RotateMode = BitConverter.ToBoolean(data,13);
            // RotateUp_Down = BitConverter.ToSingle(data, 14);
            // RotateLeft_Right = BitConverter.ToSingle(data, 18);
            // YawRotation = BitConverter.ToSingle(data, 22);
            // ControTargetRotation(RotateUp_Down, RotateLeft_Right, YawRotation, RotateMode);
            rec_log += "Mode=current_position";
        }
        else if ((user_flag[0] & 0x04) == 0x04)
        {
            TranslateMode = BitConverter.ToBoolean(data,9);
            TranslateFront_Back = BitConverter.ToSingle(data, 1);
            TranslateLeft_Right = BitConverter.ToSingle(data, 5);
            ControTargetTranslation(TranslateFront_Back, TranslateLeft_Right, TranslateMode);

            RotateMode = BitConverter.ToBoolean(data,14);
            RotateUp_Down = BitConverter.ToSingle(data, 17);
            RotateLeft_Right = BitConverter.ToSingle(data, 21);
            YawRotation = BitConverter.ToSingle(data, 25);

            RotateUp_Down = RotateUp_Down<0.1?0:RotateUp_Down;
            RotateLeft_Right = RotateLeft_Right<0.1?0:RotateLeft_Right;
            ControTargetRotation(RotateUp_Down, RotateLeft_Right, YawRotation, RotateMode);

            rec_log += "Mode=Controller_Control ";
            rec_log = data[13].ToString() + " " + data[14].ToString() + " "+ data[15].ToString() + " "+ data[16].ToString() + " "+ data[17].ToString() + " " + data[18].ToString() + " "+ data[19].ToString() + " "+ data[20].ToString() + " "+ data[21].ToString() + " "+ data[22].ToString() + " ";
        }
        else if ((user_flag[0] & 0x08) == 0x08)
        {
            float[] jointAngle = new float[6];
            for (int i = 0; i < jointAngle.Length; i++)
            {
                jointAngle[i] = BitConverter.ToSingle(data, 1 + i * sizeof(float));
            }
            jointAngle[1]/=1000.0f;
            Arm.SetJointAngle(jointAngle);
        }
        
        if ((user_flag[0] & 0x80) == 0x80)
            Contro.SetControKeyCode(Contro.ControKeyCode.EnableChangeDecodeFunc);
        else
            Contro.ResetControKeyCode(Contro.ControKeyCode.EnableChangeDecodeFunc);

        if ((user_flag[0] & 0x40) == 0x40)
            Contro.ArmControMode = Contro.ArmControMode_.ArmReset;
        
        Debug.Log(rec_log);
    }
    public void ControTargetTranslation(float Front_Back, float Left_Right, bool SwitchToWordCoordinate)
    {
        Vector3 Translation = new Vector3(Left_Right, 0, Front_Back);
        if(SwitchToWordCoordinate)
            Arm.TargetTranslate_baseWorld(Translation);
        else
            Arm.TargetTranslate_baseSucker(Translation);
        Contro.ArmControMode = Contro.ArmControMode_.TargetDecode;
    }
    public void ControTargetRotation(float Up_Down, float Left_Right,float YawRotation, bool SwitchRotateMode)
    {
        Vector3 Rotate = new Vector3(Up_Down, Left_Right, 0);
        Vector3 YawRotate = new Vector3(0, YawRotation, 0);
        if(SwitchRotateMode)
        {
            Arm.TargetRotate_Joint(Up_Down, Left_Right);
            Arm.AssortedYawRotate(YawRotate);
            
            Contro.ArmControMode = Contro.ArmControMode_.ArmGenerate;
        }
        else
        {
            Arm.TargetRotate(Rotate);
            Arm.BigYawRotate(YawRotate);
            
            Contro.ArmControMode = Contro.ArmControMode_.TargetDecode;
        }
    }
    public void SerialSendJointData()
    {
        if(!MUC_ReceiveFlag)    return;

        var joint_data = new byte[sizeof(float) * 6];
        var data = new byte[frameHeader.Length + user_flag.Length + joint_data.Length];

        /* BigYaw | Height | MidYaw | AssortedYaw | AssortedRoll | Pitch */
        Transform BigYaw, Height, MidYaw, YawAndRoll, Pitch;  
        BigYaw = Arm.BigYaw;Height = Arm.Height;MidYaw = Arm.MidYaw;YawAndRoll = Arm.YawAndRoll;Pitch = Arm.Pitch;
        Array.Copy(BitConverter.GetBytes(Mathf.Repeat(-BigYaw.eulerAngles.y + 180f, 360f) - 180f), 0, joint_data, 0, sizeof(float));
        Array.Copy(BitConverter.GetBytes(Height.localPosition.y), 0, joint_data, sizeof(float), sizeof(float));
        Array.Copy(BitConverter.GetBytes(Mathf.Repeat(-MidYaw.localEulerAngles.y + 180f, 360f) - 180f), 0, joint_data, sizeof(float) * 2, sizeof(float));
        Array.Copy(BitConverter.GetBytes(Mathf.Repeat(-YawAndRoll.localEulerAngles.y + 180f, 360f) - 180f), 0, joint_data, sizeof(float) * 3, sizeof(float));
        Array.Copy(BitConverter.GetBytes(Mathf.Repeat(-YawAndRoll.localEulerAngles.z + 180f, 360f) - 180f), 0, joint_data, sizeof(float) * 4, sizeof(float));
        Array.Copy(BitConverter.GetBytes(Mathf.Repeat(Pitch.localEulerAngles.x + 180f, 360f) - 180f), 0, joint_data, sizeof(float) * 5, sizeof(float));

        Array.Copy(frameHeader, 0, data, 0, frameHeader.Length);
        Array.Copy(user_flag, 0, data, frameHeader.Length, user_flag.Length);
        Debug.Log(data[frameHeader.Length].ToString() + frameHeader.Length.ToString());
        
        Array.Copy(joint_data, 0, data, frameHeader.Length + user_flag.Length, joint_data.Length);

        SendSerialData(serialPort, data);
        Debug.Log(data[2].ToString());

        MUC_ReceiveFlag = false;
    }
}
