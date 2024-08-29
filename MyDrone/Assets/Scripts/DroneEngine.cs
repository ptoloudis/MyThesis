using UnityEngine;

public class DroneEngine : MonoBehaviour, I_Engine
{
    [Header("Engine Properties")]
    [SerializeField] public float ServoMin = 1100.0f;
    [SerializeField] public float ServoMax = 1900.0f;
    [SerializeField] public int direction = 1;
    [SerializeField] public Transform Propeller;

    // The Values change
    private float rpm;
    private float current;
    private float thrust;
    private float moment_roll;
    private float moment_pitch;
    private float moment_yaw;

    // Air density
    private const float density = 1.225f; // kg/m^3

    // Motor properties
    private const float electrical_kv = 850; // RPM/Volt 
    private const float electrical_resistance = 0.272f; // Ohms

    // ESC properties
    private const float esc_resistance = 0.01f; // Ohms

    // Propeller properties
    private const float prop_diameter = 0.254f; // m
    private const float prop_pitch = 0.1143f; // m
    private const float prop_PConst = 1.20f;
    private const float prop_TConst = 1;
    private const float prop_mass = 0.02f; // kg only used for inertia
    private float prop_inertia; // rotational inertia(kgm^2) (rod about center)
    private float x;
    private float z;

    // Cache constants
    private const float Kt = 1 / (electrical_kv * ((2 * Mathf.PI) / 60));
    private const float twoPi = 2 * Mathf.PI;

    // Init the engine
    public void InitEngine()
    {
        prop_inertia = prop_mass * Mathf.Pow(prop_diameter, 2) * 0.08f;
        rpm = 0;
        current = 0;
        x = (Mathf.Round(transform.position.x * 100)) / 100.0f;
        z = (Mathf.Round(transform.position.z * 100)) / 100.0f;
        //Debug.Log($"{x} {z}");
    }

    // Reset the value to 0
    public void Reset()
    {
        rpm = 0;
        current = 0;
        thrust = 0;
        moment_roll = 0;
        moment_pitch = 0;
        moment_yaw = 0;
    }

    public void UpdateEngine(ushort pwm, float battery_dropped_voltage, float delta)
    {
        //Debug.Log("delta" + delta);
        // Calculate the throttle
        float throttle = Mathf.Clamp((pwm - ServoMin) / (ServoMax - ServoMin), 0, 1);

        // Effective voltage
        float voltage = throttle * battery_dropped_voltage;

        // Take the RPM from the last step to calculate the torque and current
        current = ((electrical_kv * voltage) - rpm) / ((electrical_resistance + esc_resistance) * electrical_kv);
        float torque = current * Kt;

        // Calculate the prop drag
        float prop_drag = prop_PConst * density * Mathf.Pow(rpm / 60, 2) * Mathf.Pow(prop_diameter, 5);

        float w = rpm * (twoPi / 60); // Convert to rad/s
        float w1 = w + ((torque - prop_drag) / prop_inertia) * delta;
        float rps = Mathf.Max(w1 / twoPi, 0); // For the negative rps

        // Calculate the thrust (with fudge factor)
        thrust = 4.4f * prop_TConst * density * Mathf.Pow(rps, 2) * Mathf.Pow(prop_diameter, 4);
        //Debug.Log($"{thrust} {thrust/4.5f}");

        // Update
        rpm = rps * 60;
        moment_roll = thrust * z;
        moment_pitch = thrust * x;
        moment_yaw = -torque * direction;

        //HandlePropeller(delta);
    }

    // Move Propeller
    private void HandlePropeller(float delta_t)
    {
        if (!Propeller)
            return;

        // Rotate by RPM
        Propeller.Rotate(Vector3.forward, rpm * delta_t);
    }

    // Thread-safe properties for getting values
    public float Roll() => moment_roll;
    public float Pitch() => moment_pitch;
    public float Yaw() => moment_yaw;
    public float Thrust() => thrust;
    public float Current() => current;
}
