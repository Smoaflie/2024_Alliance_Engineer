// using System;
// using System.Collections;
// using System.Collections.Generic;
// using System.Net;
// using System.Net.Sockets;
// using System.Text;
// using System.Threading;
// using UnityEngine;

// public class TCPTransform : MonoBehaviour
// {

//     static public Dictionary<int, float[]> data = new Dictionary<int, float[]>();
//     static Dictionary<int, Socket> portSock = new Dictionary<int, Socket>();
//     // Start is called before the first frame update


//     //建立tcp通信链接
//     static public void Connectint(int inputPort)
//     {
//         try
//         {
//             int _port = Convert.ToInt32(inputPort);         //获取端口号
//             string _ip = "10.31.1.53";                           //获取ip地址

//             Debug.Log(" ip 地址是 ：" + _ip);
//             Debug.Log(" 端口号是 ：" + _port);


//             //点击开始监听时 在服务端创建一个负责监听IP和端口号的Socket
//             var socketWatch = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
//             IPAddress ip = IPAddress.Parse(_ip);
//             IPEndPoint point = new IPEndPoint(ip, _port);   //创建对象端口

//             socketWatch.Bind(point);                        //绑定端口号

//             Debug.Log("监听成功!");

//             socketWatch.Listen(1);                         //设置监听，最大同时连接10台

//             //创建监听线程
//             Thread thread = new Thread(Listen)
//             {
//                 IsBackground = true
//             };
//             thread.Start(socketWatch);
//         }
//         catch { }
//     }
//     static void Listen(object o)
//     {
//         try
//         {
//             Socket socketWatch = o as Socket;
//             while (true)
//             {
//                 var socketSend = socketWatch.Accept();           //等待接收客户端连接

//                 Debug.Log(socketSend.RemoteEndPoint.ToString() + ":" + "连接成功!");
//                 portSock.Add(40425, socketSend);

//                 Thread r_thread = new Thread(()
//                 =>
//                 {
//                     while (true)
//                     {
//                         byte[] buffer = new byte[1024 * 6];         //客户端连接服务器成功后，服务器接收客户端发送的消息
//                         int len = socketSend.Receive(buffer);       //实际接收到的有效字节数
//                         float[] floats = new float[6];
//                         Buffer.BlockCopy(buffer, 0, floats, 0, 24);
//                         data[40425] = floats;
//                     }
//                 }
//                 );
//                 r_thread.IsBackground = true;
//                 r_thread.Start();

//             }
//         }
//         catch (Exception e) { Debug.LogError(e.Message); }

//     }
//     public static void SendData(byte[] bytes, int port, int offSet, int count)
//     {
//         portSock[port].Send(bytes, offSet, count, SocketFlags.None);
//     }

// }
