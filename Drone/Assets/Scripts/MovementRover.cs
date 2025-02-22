using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class MovementRover : MonoBehaviour
{
    private UdpClient udpClient;

    // Define the UDP IP address and port to listen on
    [SerializeField]private string udpIP = "127.0.0.1";
    [SerializeField]private int udpPort = 9003;

    // Variable to store the last received data
    private SensorDataRover SensorDataRover;
    private Vector3 StartPosition;

    // Cancellation token source to manage the receive task
    private CancellationTokenSource cancellationTokenSource;

    void Start()
    {        
        StartPosition = transform.position;
        Debug.Log(StartPosition);
        // Initialize UDP client and endpoint
        udpClient = new UdpClient(udpPort);
        cancellationTokenSource = new CancellationTokenSource(); // Initialize the cancellation token source
        Debug.Log($"Listening for UDP packets on {udpIP}:{udpPort}");
        ReceiveData(cancellationTokenSource.Token);
    }

    private async void ReceiveData(CancellationToken cancellationToken)
    {
        try
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                // Await receiving data
                UdpReceiveResult result = await udpClient.ReceiveAsync();
                string ReceivedData = Encoding.UTF8.GetString(result.Buffer); // Store the last received 
                SensorDataRover = JsonUtility.FromJson<SensorDataRover>(ReceivedData); // Deserialize the JSON into the SensorDataRover 
                // Set position
                Vector3 position = new(SensorDataRover.position[1], -SensorDataRover.position[2], SensorDataRover.position[0]);
                position += StartPosition;

                // Set rotation
                Vector3 rotationEuler = new(-SensorDataRover.attitude[1], SensorDataRover.attitude[2], -SensorDataRover.attitude[0]);
                rotationEuler *= Mathf.Rad2Deg;
                Quaternion rotation = Quaternion.Euler(rotationEuler);

                transform.position = Vector3.Lerp(transform.position, position, Time.deltaTime * 5f);
                transform.rotation = Quaternion.Lerp(transform.rotation, rotation, Time.deltaTime * 5f);
            }
        }
        catch (Exception ex)
        {
            if (cancellationToken.IsCancellationRequested)
            {
                Debug.Log("Receiving has been cancelled.");
            }
            else
            {
                Debug.LogError($"Error receiving data: {ex.Message}");
            }
        }
    }

    private void OnApplicationQuit()
    {
        // Clean up
        cancellationTokenSource.Cancel(); // Signal the receive task to cancel
        udpClient.Close(); // Close the UDP client
    }

}


[System.Serializable]
public class SensorDataRover
{
    public float[] position;
    public float[] attitude;
}
