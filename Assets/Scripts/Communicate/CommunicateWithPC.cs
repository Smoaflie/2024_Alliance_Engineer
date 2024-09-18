using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System;
using System.Linq;

public class CommunicateWithPC : SeriesPort
{
    // private byte[] frameHeader = { 0x5A, 0xA5 };  // 帧头，根据实际情况修改
    private byte[] frameHeader = { 0xA5, 0x1E, 0x00 };  // 帧头，根据实际情况修改
    protected static SerialPort serialPort;
    private static bool isSerialPortInitialized = false;
    
    void Start()
    {
        if(!isSerialPortInitialized)
        {
            var portName = "/dev/ttyUSB0";
            var baudRate = 115200;

            // 初始化串口
            serialPort = new SerialPort(portName, baudRate);
            
            // 设置串口参数
            serialPort.Parity = Parity.None;
            serialPort.DataBits = 8;
            serialPort.StopBits = StopBits.One;
            serialPort.Handshake = Handshake.None;
            serialPort.ReadTimeout = 34;
            serialPort.WriteTimeout = 34;

            isSerialPortInitialized = true;
        
            ConnectSerialPort(serialPort);
        }
    }

    // Update is called once per frame
    void Update()
    {
        SerialReadData(serialPort, frameHeader, 36);
        // SerialReadData(serialPort, frameHeader, 80);
    }
    
    protected override void ProcessData(byte[] data)
    {
        // 复制并转换数据
        for (int i = 0; i < Contro.encoder_Data.Length; i++)
        {
            Contro.encoder_Data[i] = BitConverter.ToSingle(data, 6 + i * sizeof(float));
        }

        if (data.Length >= 18 + sizeof(float) * 4)
        {
            // 左右手坐标系转换
            float w = BitConverter.ToSingle(data, 18);
            float x = -BitConverter.ToSingle(data, 18 + sizeof(float));
            float y = -BitConverter.ToSingle(data, 18 + sizeof(float) * 2);
            float z = BitConverter.ToSingle(data, 18 + sizeof(float) * 3);

            // 变换基坐标系
                Contro.qua_rec = new Quaternion(x, y, z, w);

                // Contro.qua_rec *= Quaternion.Inverse(Quaternion.Euler(Contro.qua_rec.eulerAngles.x, 0, 0));

                Contro.qua_rec = new Quaternion(Contro.qua_rec.z, Contro.qua_rec.y, Contro.qua_rec.x, Contro.qua_rec.w);

                // Contro.qua_rec *= Quaternion.Inverse(Quaternion.Euler(0, 0, Contro.qua_rec.eulerAngles.z));
            // if((data[5] & 1) == 1)
            // {
                // Contro._KeyCode |= Contro.ControKeyCode.EnableCustomController;
            // }
            // else{
            //     Contro._KeyCode &= ~Contro.ControKeyCode.EnableCustomController;
            // }
        }
        Contro.SetControKeyCode(Contro.ControKeyCode.EnableCustomController);
        Contro.SetControKeyCode(Contro.ControKeyCode.CustomTargetContro);
        // if (data.Length >= 64 + sizeof(float) * 4)
        // {
        //     // 左右手坐标系转换
        //     float w = BitConverter.ToSingle(data, 64);
        //     float x = -BitConverter.ToSingle(data, 64 + sizeof(float));
        //     float y = -BitConverter.ToSingle(data, 64 + sizeof(float) * 2);
        //     float z = -BitConverter.ToSingle(data, 64 + sizeof(float) * 3);

        //     // 变换基坐标系
        //     Contro.qua_rec = new Quaternion(x, y, z, w);

        //     // Contro.qua_rec *= Quaternion.Inverse(Quaternion.Euler(Contro.qua_rec.eulerAngles.x,0,0));
        //     // Contro.qua_rec *= Quaternion.Inverse(Quaternion.Euler(Contro.qua_rec.eulerAngles.x,0,0));

        //     Contro.qua_rec = new Quaternion(Contro.qua_rec.x, Contro.qua_rec.z, Contro.qua_rec.y, Contro.qua_rec.w);
        //     // var r = Contro.qua_rec * Vector3.right;
        //     // var f = Contro.qua_rec * Vector3.forward;
        //     // f = Vector3.Cross(Vector3.up,f).normalized;
        //     // float num2 = Mathf.Clamp(Vector3.Dot(f, r) , -1f, 1f);
        //     // var angle = (float)Math.Acos(num2) * 57.29578f;
        //     // f = Vector3.Cross(r,f);
        //     // Contro.qua_rec = Quaternion.AngleAxis(angle,f) * Contro.qua_rec;
        //     // if(Contro.qua_rec.eulerAngles.y > 90 && Contro.qua_rec.eulerAngles.y < 270)
        //     //     Contro.qua_rec = Quaternion.Euler(-Contro.qua_rec.eulerAngles.x, Contro.qua_rec.eulerAngles.y, Contro.qua_rec.eulerAngles.z);
        //     Contro.qua_rec *= Quaternion.Inverse(Quaternion.Euler(0,0,Contro.qua_rec.eulerAngles.z));
            
        //     Debug.Log(Contro.qua_rec);
        //     // Debug.Log((w,x,y,z));
        //     Cube.rotation = Contro.qua_rec;
        // }
        // if((data[5] & 1) == 1)
        // {
        //     Contro._KeyCode |= Contro.ControKeyCode.EnableCustomController;
        // }
        // else{
        //     Contro._KeyCode &= ~Contro.ControKeyCode.EnableCustomController;
        // }
    }
    protected void OnApplicationQuit()
    {
        if (serialPort != null)
        {
            if (serialPort.IsOpen)
            {
                serialPort.Close();
                Debug.Log("Serial port closed");
            }
            else
            {
                Debug.Log("Serial port was already closed");
            }
        }
        else
        {
            Debug.LogWarning("Serial port object was null");
        }
    }

}
