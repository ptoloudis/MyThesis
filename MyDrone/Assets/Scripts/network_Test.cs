using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class Program : MonoBehaviour
{
    private Thread SendThread;
    UdpClient udpClient = new UdpClient();

    private void Awake()
    {
        SendThread = new Thread(new ThreadStart(test));
        SendThread.IsBackground = true;
        SendThread.Start();
    }

    private void test()
    {
        int port = 5000;

        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), port);

        string message = "hi";
        byte[] binaryMessage = Encoding.ASCII.GetBytes(message);

        // Frequency in Hertz
        int frequency = 500;
        int interval = 1000 / frequency; // Interval in milliseconds

        Debug.Log($"Sending messages to port {port} at {frequency}Hz...");

        while (true)
        {
            udpClient.Send(binaryMessage, binaryMessage.Length, remoteEndPoint);
            Thread.Sleep(interval);
        }
    }

    private void OnDestroy()
    {
        // Close the UDP client and stop receiving thread
        if (udpClient != null)
            udpClient.Close();

        if (SendThread != null)
            SendThread.Abort();
    }
}