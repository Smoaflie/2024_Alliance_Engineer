using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System;
using System.Linq;

public abstract class SeriesPort : MonoBehaviour
{
    protected static void ConnectSerialPort(SerialPort serialPort)
    {
        // 打开串口
        try
        {
            if(serialPort.IsOpen)
                serialPort.Close();
            serialPort.Open();
            Debug.Log("Serial port opened successfully");
        }
        catch (System.Exception ex)
        {
            Debug.LogError("Error opening serial port: " + ex.Message);
        }
    }
    
    protected void SerialReadData(SerialPort serialPort, byte[] frameHeader, int read_len)
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            try
            {
                // 读取原始字节数据
                byte[] buffer = new byte[1];
                int frameHeaderIndex = 0;

                while (serialPort.BytesToRead > 0)
                {
                    serialPort.Read(buffer, 0, 1);

                    if (buffer[0] == frameHeader[frameHeaderIndex])
                    {
                        frameHeaderIndex++;
                        if (frameHeaderIndex == frameHeader.Length)
                        {
                            // 已经完全匹配到帧头，开始读取数据
                            byte[] data = new byte[read_len];
                            int bytesRead = 0;
                            while (bytesRead < read_len)
                            {
                                serialPort.Read(data, bytesRead, 1);
                                bytesRead++;
                            }
                            ProcessData(data);
                            frameHeaderIndex = 0; // 重置帧头索引以寻找新的帧头
                        }
                    }
                    else
                    {
                        frameHeaderIndex = 0; // 重置帧头索引，因为匹配失败
                    }
                }
            }
            catch (TimeoutException)
            {
                // 超时异常不处理，继续读取
            }
            catch (Exception ex)
            {
                if (ex.Message == "Resource temporarily unavailable")
                {
                    // Temporary resource unavailability; continue reading.
                }
                else
                {
                    try
                    {
                        ConnectSerialPort(serialPort);
                    }
                    catch (Exception connectEx)
                    {
                        Debug.LogError("Error reconnecting serial port: " + connectEx.Message);
                    }
                    
                    Debug.LogError("Error reading from serial port: aa" + ex.Message + "aa");
                }
            }
        }
        else
        {
            ConnectSerialPort(serialPort);
            Debug.Log("[Version:1]Try to reconnect Serial Port: " + serialPort.PortName.ToString());
        }
            
    }
    protected static void SendSerialData(SerialPort serialPort, byte[] data)
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            try
            {
                serialPort.Write(data, 0, data.Length);
                Debug.Log("Data sent successfully");
            }
            catch (Exception ex)
            {
                ConnectSerialPort(serialPort);
                Debug.LogError("Error sending data to serial port: " + ex.Message);
            }
        }
        else
        {
            ConnectSerialPort(serialPort);
            Debug.LogError("Serial port is not open. Cannot send data.");
        }
    }
    // 数据解析
    protected abstract void ProcessData(byte[] data);
    //Unity隐式调用
    // protected void OnApplicationQuit()
    // {
    //     if (serialPort != null)
    //     {
    //         if (serialPort.IsOpen)
    //         {
    //             serialPort.Close();
    //             Debug.Log("Serial port closed");
    //         }
    //         else
    //         {
    //             Debug.Log("Serial port was already closed");
    //         }
    //     }
    //     else
    //     {
    //         Debug.LogWarning("Serial port object was null");
    //     }
    // }
}
