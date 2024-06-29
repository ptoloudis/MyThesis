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
    
    // Air denstity
    private float denstity = 1.225f; // kg/m^3

    // Motor properties
    private float electrical_kv = 850; // RPM/Volt 
    private float electrical_resistance = 0.272f; // Ohms

    // ESC properties
    private float esc_resistance = 0.01f; // Ohms

    // Propeller properties
    private float prop_diameter = 0.254f; // m
    private float prop_pitch = 0.1143f; // m
    private float prop_PConst = 1.20f;
    private float prop_TConst = 1;
    private float prop_mass = 0.02f; //  kg only used for inertia
    private float prop_inertia; //  rotational inertia(kgm^2) (rod about center)
    private float x;
    private float z;

    // Init the engine
    public void InitEngine()
    {
        prop_inertia = prop_mass * Mathf.Pow(prop_diameter, 2) * 0.08f;
        rpm = 0;
        current = 0;
        x = transform.position.x;
        z = transform.position.z;
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

    public void UpdateEngine(ushort pwm, float battery_dropped_voltage)
    {
        // Calculate the throttle
        float throttle = (pwm - ServoMin)/(ServoMax - ServoMin);
        throttle = Mathf.Clamp(throttle, 0, 1);

        // effective voltage
        float voltage = throttle * battery_dropped_voltage;

        // Take the RPM from the last step to calculate the torque and current
        float Kt = 1/(electrical_kv * ((2 * Mathf.PI)/60));

        // RPM equation rearranged for current
        current = ((electrical_kv * voltage) - rpm)/((electrical_resistance + esc_resistance) * electrical_kv);
        float torque = current * Kt;

        // Calculate the prop drag
        float prop_drag = prop_PConst * denstity * Mathf.Pow(rpm/60, 2) * Mathf.Pow(prop_diameter, 5);

        float w = rpm * ((2 * Mathf.PI)/60); // Convert to rad/s
        float w1 = w + ((torque - prop_drag)/ prop_inertia) * Time.fixedDeltaTime;
        float rps = w1 * (1/(2 * Mathf.PI));

        // For the negative rps
        rps = Mathf.Max(rps, 0);

        // Calculate the thrust (with fudge factor)
        thrust = 4.4f * prop_TConst * denstity * Mathf.Pow(rps, 2) * Mathf.Pow(prop_diameter, 4);

        // Update
        rpm = rps * 60;
        moment_roll = thrust * z;
        moment_pitch = thrust * x;
        moment_yaw = -torque * direction;

        HandlePropeller();
    }

    // Move Propeller
    private void HandlePropeller()
    {
        if(!Propeller)
            return;
        
        // Rotate by RPM
        Propeller.Rotate(Vector3.forward, rpm);
    }

    public float GetCurrent()
    {
        return current;
    }

    public float GetThrust()
    {
        return thrust;
    }

    public float GetRoll()
    {
        return moment_roll;
    }

    public float GetPitch()
    {
        return moment_pitch;
    }
    
    public float GetYaw() 
    {
        return moment_yaw;
    }


}
