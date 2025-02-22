using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class MovementDrone : MonoBehaviour
{
    private UdpClient udpClient;
    public List<GameObject> propellers;

    // Define the UDP IP address and port to listen on
    [SerializeField]private string udpIP = "127.0.0.1";
    [SerializeField]private int udpPort = 9003;

    // Variable to store the last received data
    private SensorDataDrone SensorDataDrone;

    //The start position of the drone
    private Vector3 StartPosition;

    // Cancellation token source to manage the receive task
    private CancellationTokenSource cancellationTokenSource;

    void Start()
    {
        StartPosition = transform.position;

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
                SensorDataDrone = JsonUtility.FromJson<SensorDataDrone>(ReceivedData); // Deserialize the JSON into the SensorDataDrone 
                // Set position
                Vector3 position = new(SensorDataDrone.position[1], -SensorDataDrone.position[2], SensorDataDrone.position[0]);
                position += StartPosition;

                // Set rotation
                Vector3 rotationEuler = new(-SensorDataDrone.attitude[1], SensorDataDrone.attitude[2], -SensorDataDrone.attitude[0]);
                rotationEuler *= Mathf.Rad2Deg;
                Quaternion rotation = Quaternion.Euler(rotationEuler);

                transform.position = Vector3.Lerp(transform.position, position, Time.deltaTime * 5f);
                transform.rotation = Quaternion.Lerp(transform.rotation, rotation, Time.deltaTime * 5f);

                // Rotate propellers based on RPM
                for (int i = 0; i < propellers.Count; i++)
                 {
                     if (i >= SensorDataDrone.rpms.Length)
                     {
                         Debug.LogWarning("Mismatch between propellers and RPM data.");
                         break;
                     }

                     GameObject propeller = propellers[i];
                     float speed = SensorDataDrone.rpms[i] * Time.fixedDeltaTime;
                     propeller.transform.Rotate(Vector3.forward, speed);
                 }
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
public class SensorDataDrone
{
    public float[] rpms;
    public float[] position;
    public float[] attitude;
}
