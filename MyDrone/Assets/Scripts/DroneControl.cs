using UnityEngine;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Runtime.InteropServices;


[RequireComponent(typeof(Rigidbody))]

public class DroneControl : MonoBehaviour
{
    [Header("Ardupilot Properties")]
    [SerializeField] private int theMagic = 18458;
    [SerializeField] private int TimeOutMax = 10;
    [SerializeField] private float MaxDiff0Pitch = 0.5f;
    [SerializeField] private float MaxDiff0Yaw = 0.5f;

    [SerializeField] private float MaxDiff0Roll = 0.5f;


    private const int listenPort = 9004; // Change this to your desired port number
    private UdpClient client;
    private UdpClient send;
    private IPEndPoint DroneIP;
    private List<I_Engine> engines = new List<I_Engine>();
    protected Rigidbody rb;
    private double[] lastVelocity;
    private Thread receiveThread;
    private DroneInfo drone;
    private bool ArdupilotOnline; 
    private RingBuffer ringBuffer;


    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (!rb)
            Debug.LogError("Not Found Rigidbody");
        drone = new DroneInfo();
        drone.Init();
        rb.mass = drone.copter_mass;
    }

    void Start()
    {
        client = new UdpClient(listenPort);
        client.Client.ReceiveTimeout = 1000; // Timeout 1 second
        send = new UdpClient();
        DroneIP = new IPEndPoint(IPAddress.Any, 0);
       
        ArdupilotOnline = false;

        engines = GetComponentsInChildren<I_Engine>().ToList<I_Engine>();
        for (int i = 0; i < engines.Count; i++)
        {
            engines[i].InitEngine();
        }
        drone.battery_dropped_voltage = 0;

        lastVelocity = new double[3];
        ringBuffer = new RingBuffer(400, 16);

        receiveThread = new Thread(new ThreadStart(ReceivedData));
        receiveThread.IsBackground = true;
        receiveThread.Start();

        StartCoroutine(SendData());
    }

    /*  Ardupilot (NED) / Unity (EUN)
        The Unity to Ardupilot cartensinan
        ardupilot_x = unity_z;
        ardupilot_y = unity_x;
        ardupilot_z = -unity_y;
    */

    void FixedUpdate()
    {
        if (!ArdupilotOnline)
        {
            SimReset();
            return;
        }

        if ( ringBuffer.IsEmpty )
            return;

        ushort[] pwns = ringBuffer.Dequeue();
        
        //drone.battery_dropped_voltage = drone.battery_voltage - drone.battery_resistance * drone.battery_current
        drone.battery_dropped_voltage = drone.battery_voltage;
        float thrust = 0;

        drone.battery_current = 0;
        Vector3 moment = Vector3.zero;

        for (int i = 0; i < engines.Count; i++)
        {
            engines[i].UpdateEngine(pwns[i], drone.battery_voltage);
            drone.battery_current += engines[i].GetCurrent();
            thrust += engines[i].GetThrust();
            moment.x += engines[i].GetPitch();
            // TODO: Check if - is correct
            moment.y += engines[i].GetYaw(); 
            moment.z += engines[i].GetRoll();
        }   
        

        Vector3 engineForce = rb.transform.up * thrust;
        Vector3 angVel = CalculateAV(moment);
        
        // Apply the Angular Velocity
        rb.angularVelocity = angVel;

        // Apply force and change moments
        rb.AddForce(engineForce);
    }


    string BuildJSON()
    {
        double timestamp = Time.realtimeSinceStartup;
        double[] gyro = ToArdupilotCoordinates(rb.angularVelocity);
        double[] position = ToArdupilotCoordinates(rb.position);
        double[] attitude = ToArdupilotCoordinates(rb.rotation.eulerAngles);
        double[] velocity = ToArdupilotCoordinates(rb.velocity);
        double[] accelBody = new double[3];
        accelBody[0] = (velocity[0] - lastVelocity[0])/ Time.fixedDeltaTime;
        accelBody[1] = (velocity[1] - lastVelocity[1])/ Time.fixedDeltaTime;
        accelBody[2] = (velocity[2] - lastVelocity[2])/ Time.fixedDeltaTime - Physics.gravity.magnitude;
        lastVelocity = velocity;

        IMU imu = new IMU(gyro, accelBody);
        IMUData data = new IMUData(timestamp, imu, position, attitude, velocity);

        // Convert to JSON
        return ("\n" + JsonUtility.ToJson(data) + "\n");
    }

    double[] ToArdupilotCoordinates(Vector3 unityCoordinates)
    {
        // Create an array with 3 elements
        double[] array = new double[3];

        // Assign Vector3 components to the array
        array[0] = unityCoordinates.z;
        array[1] = unityCoordinates.x;
        array[2] = -unityCoordinates.y;

        return array;
    }

    public double[] ToArdupilotQuaternion(Quaternion unityQuaternion)
    {
        // Create an array with 4 elements
        double[] array = new double[4];

        // Assign Vector3 components to the array
        array[0] = unityQuaternion.w;
        array[1] = unityQuaternion.z;
        array[2] = unityQuaternion.x;
        array[3] = -unityQuaternion.y;

        return array;
    }

    private void ReceivedData()
    {
        uint Count = 0;
        uint timeout = 0;
        while (true)
        {
            try
            {
                // Receive UDP packet
                byte[] data = client.Receive(ref DroneIP);
                if(!ArdupilotOnline)
                {
                    ArdupilotOnline = true;
                    timeout = 0;
                }

                // Extract data
                if (data.Length >= 12) // Ensure Receive data is of correct length
                {

                    ushort magic = BitConverter.ToUInt16(data, 0);
                    if (magic != theMagic)
                    {
                        // If not magic
                        Debug.LogError("Receive data does not have the correct magic number.");
                        continue;
                    }

                    ushort frameRate = BitConverter.ToUInt16(data, 2);
                    uint frameCount = BitConverter.ToUInt32(data, 4);
                    ushort[] pwm = new ushort[16];

                    if(Count >= frameCount)
                        continue;
                    else if (frameCount != Count + 1)
                        Debug.LogError("Missed packets detected.");

                    Count = frameCount;
                    // Extract pwm array
                    for (int i = 0; i < 16; i++)
                    {
                        pwm[i] = BitConverter.ToUInt16(data, 8 + i * 2);
                        // Debug.Log(i + " " + pwm[i]);
                    }

                    while (ringBuffer.IsFull) { }
                    ringBuffer.Enqueue(pwm); 
                }
                else
                {
                    Debug.LogError("Receive data is incomplete.");
                }
            

            }
            catch (SocketException ex)
            {
                if(ex.SocketErrorCode == SocketError.TimedOut)
                {
                    timeout++;
                    if(timeout > TimeOutMax)
                    {
                        ArdupilotOnline = false;
                        timeout = 0;
                        Debug.LogError("Offline");
                    }
                }
                else 
                {
                    Debug.LogError("UDP Receive Error: " + ex.ToString());
                }
            }
        }
    }
    
    IEnumerator SendData()
    {
        double lastInterval = Time.realtimeSinceStartup;
        while (true)
        {
            yield return new WaitForFixedUpdate();
            if (!ArdupilotOnline)
                continue;

            string json = BuildJSON();
            // byte[] jsonBytes = Encoding.UTF8.GetBytes(json);
            // client.Send(jsonBytes, jsonBytes.Length, DroneIP);
            // WriteJsonToFile(json);
            // Debug.Log("A"+json);
        }
    }

    private void SimReset()
    {
        drone.Reset();
        for (int i = 0; i < engines.Count; i++)
        {
            engines[i].Reset();
        }
        rb.position = Vector3.zero;

    }

    private Vector3 CalculateAV(Vector3 moment)
    {

        // Calculate the Drag in rotation
        Vector3 gyro = rb.angularVelocity;
        float rotDragCoefficient = 0.2f;
        Vector3 rotDrag = Vector3.zero;
        rotDrag.x = rotDragCoefficient * Mathf.Sign(gyro.x) * Mathf.Pow(gyro.x, 2);
        rotDrag.y = rotDragCoefficient * Mathf.Sign(gyro.y) * Mathf.Pow(gyro.y, 2);
        rotDrag.z = rotDragCoefficient * Mathf.Sign(gyro.z) * Mathf.Pow(gyro.z, 2);

        // Calculate the rotation accel
        Vector3 moments = moment - rotDrag;
        Vector3 rotAccel = moments / drone.copter_inertia;

        // Calculate the new value
        // Debug.Log(gyro + " " + rotAccel + " " + rb.position + "#" + rb.rotation);
        gyro = gyro + rotAccel * Time.fixedDeltaTime;
        gyro.x = Mathf.Clamp(gyro.x, -2000 * Mathf.Deg2Rad, 2000 * Mathf.Deg2Rad);
        gyro.y = Mathf.Clamp(gyro.y, -2000 * Mathf.Deg2Rad, 2000 * Mathf.Deg2Rad);  
        gyro.z = Mathf.Clamp(gyro.z, -2000 * Mathf.Deg2Rad, 2000 * Mathf.Deg2Rad);

        // Fix the Unity error
        if(Mathf.Abs(moment.x) < MaxDiff0Pitch)
            gyro.x = 0; 
        if(Mathf.Abs(moment.y) < MaxDiff0Yaw)
            gyro.y = 0; 
        if(Mathf.Abs(moment.z) < MaxDiff0Roll)
            gyro.z = 0;

        return gyro;
    }

     private void OnDestroy()
    {
        // Close the UDP client and stop receiving thread
        if (client != null)
        {
            client.Close();
            send.Close();
        }
        if (receiveThread != null)
            receiveThread.Abort();
        
        StopAllCoroutines();
    }


    static void WriteJsonToFile(string json)
    {
        // Get the path to the user's home directory
        string homeDirectory = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);

        // Construct the full file path
        string filePath = Path.Combine(homeDirectory, "MyDrone","JSON", "unity", Time.realtimeSinceStartup +".json");

        // Write JSON string to the file
        using (StreamWriter streamWriter = File.CreateText(filePath))
        {
            streamWriter.Write(json);
        }
    }

}