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
using UnityEngine.UIElements;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;
using Unity.VisualScripting;

[RequireComponent(typeof(Rigidbody))]

public class DroneMatlab : MonoBehaviour
{
    [Header("Ardupilot Properties")]
    [SerializeField] private int theMagic = 18458;

    private int TimeOutMax = 5000;
    private const float maxGyro = 2000 * Mathf.Deg2Rad;
    private const float rotDragCoefficient = 0.2f;
    private const float max_timestep = 1.0f / 50;

    private uint count = 0;
    private uint timeout = 0;
    private int x = 0;

    private const int listenPort = 9002; // Change this to your desired port number
    private UdpClient client;
    private IPEndPoint DroneIP;
    private List<I_Engine> engines = new List<I_Engine>();
    protected Rigidbody rb;
    private Thread MainCal;
    private DroneInfo drone;

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

        // Get the engines & init them.
        engines = GetComponentsInChildren<I_Engine>().ToList<I_Engine>();
        for (int i = 0; i < engines.Count; i++)
        {
            engines[i].InitEngine();
        }
        drone.battery_dropped_voltage = 0;

        MainCal = new Thread(new ThreadStart(MainFun));
        MainCal.IsBackground = true;
        MainCal.Start();

        StartCoroutine(Loop());
    }

    private void OnDestroy()
    {
        // Close the UDP client and stop receiving thread
        if (MainCal != null)
            MainCal.Abort();

        if (client != null)
            client.Close();

        StopAllCoroutines();
    }

    // Make the GUI Frame Rate function
    void Update()
    {
        updateCount += 1;
    }

    // Print on the GUI the Update 
    void OnGUI()
    {
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        fontSize.fontSize = 24;
        GUI.Label(new Rect(100, 100, 200, 50), "Update: " + updateUpdateCountPerSecond.ToString(), fontSize);
        GUI.Label(new Rect(100, 150, 200, 50), "FixedUpdate: " + updateFixedUpdateCountPerSecond.ToString(), fontSize);
        GUI.Label(new Rect(100, 200, 200, 50), "Test: " + testCountPerSecond.ToString(), fontSize);
    }

    // Find the Updates
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

// Make the Calculetion.
    private void MainFun()
    {
        ushort[] pwm = new ushort[16];
        double timestamp = 0;
        IMUData imuData = null;
        STATE state = new STATE();
        state.Init();

        double[] gyro = new double[3];
        double[] position = new double[3];
        double[] attitude = new double[3];
        double[] velocity = new double[3];
        double[] accelBody = new double[3];

        while (true)
        {
            try
            {
                byte[] data = client.Receive(ref DroneIP);
                timeout = 0;

                if (BitConverter.ToUInt16(data, 0) != theMagic)
                {
                    // If not magic
                    Debug.LogError("Received data does not have the correct magic number.");
                }

                ushort frameRate = BitConverter.ToUInt16(data, 2);
                uint frameCount = BitConverter.ToUInt32(data, 4);

                if (frameCount < count)
                    // reset
                    continue;
                else if (frameCount == count)
                {
                    Debug.LogError("Duplicate packets detected.");
                    continue;
                }
                else if (frameCount > count + 1)
                    Debug.LogError($"Missed {frameCount - count - 1} packets detected.");
               
                count = frameCount;
                Debug.Log(frameRate + " " + 1.0f / frameRate);
                state.delta_t = Mathf.Min(1.0f/frameRate, max_timestep);
                timestamp += state.delta_t;

                // Extract pwm array
                Buffer.BlockCopy(data, 8, pwm, 0, 32); // 16 * 2 bytes for 16 ushort values
                physics_function(pwm, state);


                Vector3ToArr(state.gyro, gyro);
                Vector3ToArr(state.position, position);
                Vector3ToArr(state.attitude, attitude);
                Vector3ToArr(state.velocity, velocity);
                Vector3ToArr(state.accel, accelBody);


                IMU imu = new IMU(gyro, accelBody);
                imuData = new IMUData(timestamp, imu, position, attitude, velocity);

                // Create & send the Json.
                string json = "\n" + JsonUtility.ToJson(imuData) + "\n";
                byte[] jsonBytes = Encoding.UTF8.GetBytes(json);
                client.Send(jsonBytes, jsonBytes.Length, DroneIP);
            }
            catch (SocketException ex)
            {
                if (ex.SocketErrorCode == SocketError.TimedOut
                || ex.SocketErrorCode == SocketError.ConnectionReset)
                {
                    timeout++;
                    if (timeout > TimeOutMax)
                    {
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

    public void Vector3ToArr(Vector3 v, double[] a)
    {
        a[0] = v.x;
        a[1] = v.y;
        a[2] = v.z;
    }

    public void physics_function(ushort[] pwm, STATE state)
    {
        drone.battery_dropped_voltage = drone.battery_voltage; // If battery resistance is negligible
        float totalThrust = 0;
        drone.battery_current = 0;
        Vector3 totalMoment = Vector3.zero;

        for (int i = 0; i < engines.Count; i++)
        {
            var engine = engines[i];
            engine.UpdateEngine(pwm[i], drone.battery_voltage, state.delta_t);
            drone.battery_current += engine.Current();
            totalThrust += engine.Thrust();
            totalMoment.y += engine.Pitch();
            totalMoment.z += engine.Yaw();
            totalMoment.x -= engine.Roll();
            Debug.Log("1 " + pwm[i]);
        }

        Vector3 drag = CalculateDrag(state.bf_velo);
        Vector3 force = new Vector3(0, 0, -totalThrust) - drag;
        Vector3 angularVelocity = CalculateAV(totalMoment, state.gyro, state.delta_t);

        //update the dcm and attitude
        RotateDCM(state.dcm, angularVelocity, state.attitude);

        state.accel = force / drone.copter_mass;

        // earth frame accelerations(NED)
        Vector3 accel_ef = state.dcm * state.accel;
        accel_ef.z -= Physics.gravity.y;

        // if we're on the ground, then our vertical acceleration is limited
        // to zero.This effectively adds the force of the ground on the aircraft
        if (state.position.z >= 0 && accel_ef.z > 0)
            accel_ef.z = 0;

        //work out acceleration as seen by the accelerometers.It sees the kinematic
        //acceleration(ie.real movement), plus gravity
        state.accel = state.dcm.transpose.MultiplyVector(accel_ef + new Vector3(0, 0, Physics.gravity.y));

        state.velocity += accel_ef * state.delta_t;
        state.position += state.velocity * state.delta_t;

        // Prevent going underground (NED coordinate system: positive Z is down)
        if (state.position.z >= 0)
        {
            state.position.z = 0;
            state.velocity = Vector3.zero;
            state.gyro = Vector3.zero;
        }

        // Calculate the body frame velocity for drag calculation
        state.bf_velo = state.dcm.transpose.MultiplyVector(state.velocity);
    }

    // Calculate the Drag
    private Vector3 CalculateDrag(Vector3 bf_velo)
    {
        // Sign of the body frame velocity
        Vector3 signBfVelo = new Vector3(Mathf.Sign(bf_velo.x), Mathf.Sign(bf_velo.y), Mathf.Sign(bf_velo.z));

        // Square of the body frame velocity
        Vector3 bfVeloSquared = new Vector3(bf_velo.x * bf_velo.x, bf_velo.y * bf_velo.y, bf_velo.z * bf_velo.z);

        // Calculate drag force component-wise
        Vector3 drag = signBfVelo;
        drag.Scale(drone.copter_cd);
        drag.Scale(drone.copter_cd_ref_area);
        drag *= 0.5f * drone.density;
        drag.Scale(bfVeloSquared);

        return drag;
    }

    // Caclulate the angular velocity of the moment
    private Vector3 CalculateAV(Vector3 moment, Vector3 gyro, float delta)
    { 
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
        gyro += rotAccel * delta;

        // Clamp gyro values
        gyro.x = Mathf.Clamp(gyro.x, -maxGyro, maxGyro);
        gyro.y = Mathf.Clamp(gyro.y, -maxGyro, maxGyro);
        gyro.z = Mathf.Clamp(gyro.z, -maxGyro, maxGyro);

        return gyro;
    }

    public void RotateDCM(Matrix4x4 dcm, Vector3 ang, Vector3 euler)
    {
        // Calculate delta
        Vector3 delta1 = new Vector3(
            dcm.m01 * ang.z - dcm.m02 * ang.y,
            dcm.m02 * ang.x - dcm.m00 * ang.z,
            dcm.m00 * ang.y - dcm.m01 * ang.x
        );
        Vector3 delta2 = new Vector3(
            dcm.m11 * ang.z - dcm.m12 * ang.y,
            dcm.m12 * ang.x - dcm.m10 * ang.z,
            dcm.m10 * ang.y - dcm.m11 * ang.x
        );
        Vector3 delta3 = new Vector3(
            dcm.m21 * ang.z - dcm.m22 * ang.y,
            dcm.m22 * ang.x - dcm.m20 * ang.z,
            dcm.m20 * ang.y - dcm.m21 * ang.x
        );

        // Apply delta to the DCM
        dcm.SetColumn(0, dcm.GetColumn(0) + new Vector4(delta1.x, delta2.x, delta3.x, 0));
        dcm.SetColumn(1, dcm.GetColumn(1) + new Vector4(delta1.y, delta2.y, delta3.y, 0));
        dcm.SetColumn(2, dcm.GetColumn(2) + new Vector4(delta1.z, delta2.z, delta3.z, 0));

        // Normalize
        Vector3 t0 = dcm.GetRow(0);
        Vector3 t1 = dcm.GetRow(1);
        float error = Vector3.Dot(t0, t1);
        t0 = t0 - (t1 * (0.5f * error));
        t1 = t1 - (t0 * (0.5f * error));
        Vector3 t2 = Vector3.Cross(t0, t1);
        t0.Normalize();
        t1.Normalize();
        t2.Normalize();

        dcm.SetRow(0, new Vector4(t0.x, t0.y, t0.z, 0));
        dcm.SetRow(1, new Vector4(t1.x, t1.y, t1.z, 0));
        dcm.SetRow(2, new Vector4(t2.x, t2.y, t2.z, 0));

        // Calculate Euler angles
        euler = new Vector3(
            Mathf.Atan2(dcm.m21, dcm.m22),
            -Mathf.Asin(dcm.m20),
            Mathf.Atan2(dcm.m10, dcm.m00)
        );
    }
}


public class STATE
{
    public float delta_t;
    public Vector3 gyro; // (rad/sec)
    public Matrix4x4 dcm; // Identity matrix initially
    public Vector3 attitude; // (radians)
    public Vector3 accel; // (m/s^2) body frame
    public Vector3 velocity; // (m/s) earth frame
    public Vector3 position; // (m) earth frame
    public Vector3 bf_velo; // (m/s) body frame

    public void Init()
    {
        this.delta_t = 0.0f;
        gyro = Vector3.zero; // [0;0;0] (rad/sec)
        dcm = Matrix4x4.identity; // diag([1,1,1])
        attitude = Vector3.zero; // [0;0;0] (radians)
        accel = Vector3.zero; // [0;0;0] (m/s^2) body frame
        velocity = Vector3.zero; // [0;0;0] (m/s) earth frame
        position = Vector3.zero; // [0;0;0] (m) earth frame
        bf_velo = Vector3.zero; // [0;0;0] (m/s) body frame
    }
}