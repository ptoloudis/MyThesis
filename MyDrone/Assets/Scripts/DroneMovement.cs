using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class DroneMovement : MonoBehaviour
{
    private Vector3 Location;
    private Semaphore semaphore = new Semaphore(1, 1); // Binary semaphore
    private const int listenPort = 5000; // Change this to your desired port number
    private UdpClient client;
    private Thread receiveThread;



    void Start()
    {
        // Create UDP client and start receiving thread
        client = new UdpClient(listenPort);
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void Update()
    {
        // Use semaphore to ensure thread safety when accessing Location
        semaphore.WaitOne();
        transform.position = Location;
        Debug.Log(Location);
        semaphore.Release();

    }

    private void ReceiveData()
    {
        try
        {
            while (true)
            {
                // Receive UDP packet
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = client.Receive(ref anyIP);

                // Convert bytes to string
                string[] parts = Encoding.UTF8.GetString(data).Split("|");
                if (parts.Length == 3)
                {
                    // ArduPilot body frame coordinates
                    float ardupilot_x = float.Parse(parts[0]);
                    float ardupilot_y = float.Parse(parts[1]);
                    float ardupilot_z = float.Parse(parts[2]);

                    // Convert to Unity coordinates
                    float unity_x = ardupilot_y;
                    float unity_y = -ardupilot_z;
                    float unity_z = ardupilot_x;

                    // Use semaphore to ensure thread safety when accessing Location
                    semaphore.WaitOne();
                    Location = new Vector3(unity_x, unity_y, unity_z);
                    semaphore.Release();
                }               

            }
        }
        catch (Exception e)
        {
            Debug.LogError("UDP Receive Error: " + e.ToString());
        }
    }

    void OnDestroy()
    {
        // Close the UDP client and stop receiving thread
        if (client != null)
        {
            client.Close();
        }
        if (receiveThread != null)
        {
            receiveThread.Abort();
        }
    }

}
