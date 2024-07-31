using UnityEngine;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Runtime.InteropServices;


[RequireComponent(typeof(Rigidbody))]

public class DroneControlOr : MonoBehaviour
{
    [Header("Ardupilot Properties")]
    [SerializeField] private int theMagic = 18458;

    private int TimeOutMax = 5000;
    private const float maxGyro = 2000 * Mathf.Deg2Rad;
    private const float rotDragCoefficient = 0.2f;

    private uint count = 0;
    private uint timeout = 0;
    private int x = 0;

    private const int listenPort = 9002; // Change this to your desired port number
    private UdpClient client;
    private IPEndPoint DroneIP;
    private List<I_Engine> engines = new List<I_Engine>();
    protected Rigidbody rb;
    private double[] lastVelocity;
    private Thread SendThread;
    private Thread ReceiveThread;
    private DroneInfo drone;
    private bool ArdupilotOnline;
    private RingString RS;
    private RingBuffer RBpwm;


    // Make the GUI Frame Rate
    private float updateCount = 0;
    private float fixedUpdateCount = 0;
    private float test = 0;
    private float updateUpdateCountPerSecond;
    private float updateFixedUpdateCountPerSecond;
    private float testCountPerSecond;

    private void Awake()
    {
        // For Rigidbody
        rb = GetComponent<Rigidbody>();
        if (!rb)
            Debug.LogError("Not Found Rigidbody");
        drone = new DroneInfo();
        drone.Init();
        rb.mass = drone.copter_mass;

        // UDP 
        client = new UdpClient(listenPort);
        client.Client.ReceiveTimeout = 1; // Timeout 1 ms
        DroneIP = new IPEndPoint(IPAddress.Any, 0);
        ArdupilotOnline = false; // See the Ardupilot is Online

        // Get the engines & init them.
        engines = GetComponentsInChildren<I_Engine>().ToList<I_Engine>();
        for (int i = 0; i < engines.Count; i++)
        {
            engines[i].InitEngine();
        }
        drone.battery_dropped_voltage = 0;

        // Init help var. 
        lastVelocity = new double[3];

        RS = new RingString(400);
        RBpwm = new RingBuffer(400, 16);

        // Create & start the nessasy thread.
        // SendThread = new Thread(new ThreadStart(SendData));
        // SendThread.IsBackground = true;
        // SendThread.Start();

        // ReceiveThread = new Thread(new ThreadStart(ReceivedData));
        // ReceiveThread.IsBackground = true;
        // ReceiveThread.Start();

        StartCoroutine(BuildJson());
        StartCoroutine(Loop());
    }

    /*  Ardupilot (NED) / Unity (EUN)
        The Unity to Ardupilot cartensinan
        ardupilot_x = unity_z;
        ardupilot_y = unity_x;
        ardupilot_z = -unity_y;
    */

    void Update()
    {
        updateCount += 1;
    }

    void FixedUpdate()
    {
        fixedUpdateCount += 1;
        ushort[] pwms = ReceivedData();
        //while (RBpwm.IsEmpty) { }
        //ushort[] pwms = RBpwm.Dequeue();

        if (!ArdupilotOnline)
        {
            SimReset();
            return;
        }

        drone.battery_dropped_voltage = drone.battery_voltage; // If battery resistance is negligible
        float totalThrust = 0;
        drone.battery_current = 0;
        Vector3 totalMoment = Vector3.zero;

        for (int i = 0; i < engines.Count; i++)
        {
            var engine = engines[i];
            engine.UpdateEngine(pwms[i], drone.battery_voltage);
            drone.battery_current += engine.Current();
            totalThrust += engine.Thrust();
            totalMoment.x += engine.Pitch();
            totalMoment.y += engine.Yaw();
            totalMoment.z += engine.Roll();
        }

        Vector3 engineForce = rb.transform.up * totalThrust;
        Vector3 angularVelocity = CalculateAV(totalMoment);

        rb.angularVelocity = angularVelocity;
        rb.AddForce(engineForce);
    }

    private Vector3 CalculateAV(Vector3 moment)
    {
        // Cache angular velocity
        Vector3 gyro = rb.angularVelocity;

        // Drag in rotation
        Vector3 rotDrag = new Vector3(
            rotDragCoefficient * Mathf.Sign(gyro.x) * gyro.x * gyro.x,
            rotDragCoefficient * Mathf.Sign(gyro.y) * gyro.y * gyro.y,
            rotDragCoefficient * Mathf.Sign(gyro.z) * gyro.z * gyro.z
        );

        // Rotation acceleration
        Vector3 moments = moment - rotDrag;
        Vector3 rotAccel = moments / drone.copter_inertia;

        // Update gyro with rotational acceleration
        gyro += rotAccel * Time.fixedDeltaTime;

        // Clamp gyro values
        gyro.x = Mathf.Clamp(gyro.x, -maxGyro, maxGyro);
        gyro.y = Mathf.Clamp(gyro.y, -maxGyro, maxGyro);
        gyro.z = Mathf.Clamp(gyro.z, -maxGyro, maxGyro);

        return gyro;
    }

    private ushort[] ReceivedData() // Not THREAD 
    {
        ushort[] pwm = new ushort[16];

        try
        {
            // Receive UDP packet
            byte[] data = client.Receive(ref DroneIP);
            if (!ArdupilotOnline)
            {
                ArdupilotOnline = true;
                timeout = 0;
            }

            if (BitConverter.ToUInt16(data, 0) != theMagic)
            {
                // If not magic
                Debug.LogError("Received data does not have the correct magic number.");
                return pwm;
            }

            ushort frameRate = BitConverter.ToUInt16(data, 2);
            uint frameCount = BitConverter.ToUInt32(data, 4);

            if (count >= frameCount)
                return pwm;
            else if (frameCount > count + 1)
                Debug.LogError("Missed packets detected.");
            count = frameCount;

            // Extract pwm array
            Buffer.BlockCopy(data, 8, pwm, 0, 32); // 16 * 2 bytes for 16 ushort values
            return pwm;

        }
        catch (SocketException ex)
        {
            if (ex.SocketErrorCode == SocketError.TimedOut)
            {
                timeout++;
                if (timeout > TimeOutMax)
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

        return pwm;
    }

    // private void ReceivedData()
    // {
    //     ushort[] pwm = new ushort[16];

    //     while (true)
    //     {

    //         try
    //         {
    //             // Receive UDP packet
    //             byte[] data = client.Receive(ref DroneIP);
    //             if (!ArdupilotOnline)
    //             {
    //                 ArdupilotOnline = true;
    //                 timeout = 0;
    //             }

    //             if (BitConverter.ToUInt16(data, 0) != theMagic)
    //             {
    //                 // If not magic
    //                 Debug.LogError("Received data does not have the correct magic number.");
    //                 continue;
    //             }

    //             ushort frameRate = BitConverter.ToUInt16(data, 2);
    //             uint frameCount = BitConverter.ToUInt32(data, 4);

    //             if (count >= frameCount)
    //                 continue;
    //             else if (frameCount > count + 1)
    //                 Debug.LogError("Missed packets detected.");
    //             count = frameCount;

    //             // Extract pwm array
    //             Buffer.BlockCopy(data, 8, pwm, 0, 32); // 16 * 2 bytes for 16 ushort values
    //             while (RBpwm.IsFull) { }
    //             RBpwm.Enqueue(pwm);

    //         }
    //         catch (SocketException ex)
    //         {
    //             if (ex.SocketErrorCode == SocketError.TimedOut)
    //             {
    //                 timeout++;
    //                 if (timeout > TimeOutMax)
    //                 {
    //                     ArdupilotOnline = false;
    //                     timeout = 0;
    //                     Debug.LogError("Offline");
    //                 }
    //             }
    //             else
    //             {
    //                 Debug.LogError("UDP Receive Error: " + ex.ToString());
    //             }
    //         }

    //     }
    // }

    private IEnumerator BuildJson()
    {
        double[] gyro = new double[3];
        double[] position = new double[3];
        double[] attitude = new double[3];
        double[] velocity = new double[3];
        double[] accelBody = new double[3];

        while (true)
        {
            yield return new WaitForFixedUpdate();

            if (!ArdupilotOnline)
                continue;

            double timestamp = Time.realtimeSinceStartup;

            ToArdupilotCoordinates(rb.angularVelocity, gyro);
            ToArdupilotCoordinates(rb.position, position);
            ToArdupilotCoordinates(rb.rotation.eulerAngles, attitude);
            ToArdupilotCoordinates(rb.velocity, velocity);

            accelBody[0] = (velocity[0] - lastVelocity[0]) / Time.fixedDeltaTime;
            accelBody[1] = (velocity[1] - lastVelocity[1]) / Time.fixedDeltaTime;
            accelBody[2] = (velocity[2] - lastVelocity[2]) / Time.fixedDeltaTime - Physics.gravity.magnitude;

            Array.Copy(velocity, lastVelocity, velocity.Length);

            IMU imu = new IMU(gyro, accelBody);
            IMUData data = new IMUData(timestamp, imu, position, attitude, velocity);

            // Convert to JSON
            // string json = JsonUtility.ToJson(data);         
            // if (!RS.IsFull)
            //     RS.Enqueue(json);


            string json = "\n" + JsonUtility.ToJson(data) + "\n";
            byte[] jsonBytes = Encoding.UTF8.GetBytes(json);
            client.Send(jsonBytes, jsonBytes.Length, DroneIP);
            test++;

        }
    }

    private void SendData()
    {
        while (true)
        {
            // ushort[] pwms = ReceivedData();
            // while (RBpwm.IsFull) { }
            // RBpwm.Enqueue(pwms);


            while (RS.IsEmpty) { }
            string json = "\n" + RS.Dequeue() + "\n";
            byte[] jsonBytes = Encoding.UTF8.GetBytes(json);
            client.Send(jsonBytes, jsonBytes.Length, DroneIP);
            test++;
        }
    }

    private void ToArdupilotCoordinates(Vector3 unityCoordinates, double[] array)
    {
        // Assign Vector3 components to the array
        array[0] = unityCoordinates.z;
        array[1] = unityCoordinates.x;
        array[2] = -unityCoordinates.y;
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

    private void OnDestroy()
    {
        // Close the UDP client and stop receiving thread
        if (client != null)
            client.Close();

        if (SendThread != null)
            SendThread.Abort();

        if (ReceiveThread != null)
            ReceiveThread.Abort();

        StopAllCoroutines();
    }

    static void WriteJsonToFile(string json, int x)
    {
        // Get the path to the user's home directory
        string homeDirectory = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);

        // Construct the full file path
        string filePath = Path.Combine(homeDirectory, "MyDrone", "JSON", "unity", x + ".json");

        // Write JSON string to the file
        using (StreamWriter streamWriter = File.CreateText(filePath))
        {
            streamWriter.Write(json);
        }
    }

    void OnGUI()
    {
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        fontSize.fontSize = 24;
        GUI.Label(new Rect(100, 100, 200, 50), "Update: " + updateUpdateCountPerSecond.ToString(), fontSize);
        GUI.Label(new Rect(100, 150, 200, 50), "FixedUpdate: " + updateFixedUpdateCountPerSecond.ToString(), fontSize);
        GUI.Label(new Rect(100, 200, 200, 50), "Test: " + testCountPerSecond.ToString(), fontSize);
    }

    IEnumerator Loop()
    {
        while (true)
        {
            yield return new WaitForSeconds(1);
            Debug.Log(updateCount + " " + updateCount + " " + test);
            updateUpdateCountPerSecond = updateCount;
            updateFixedUpdateCountPerSecond = fixedUpdateCount;
            testCountPerSecond = test;

            updateCount = 0;
            fixedUpdateCount = 0;
            test = 0;
        }
    }
}