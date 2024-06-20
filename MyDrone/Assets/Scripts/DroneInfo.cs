using UnityEngine;

public class DroneInfo 
{
    // Setup battery
    public float battery_voltage = 3 * 4.2f; // Volts
    public float battery_resistance = 0.00317f; // Ohms
    public float battery_capacity = 3.5f; // Ah
    public float battery_dropped_voltage; // Volts
    public float battery_current; // Volts
    public float copter_mass = 1.282f; // Kg
    public float copter_cd = 0.5f; 
    public float copter_cd_ref_area = Mathf.PI * Mathf.Pow( 0.45f * 0.5f, 2);
    public float denstity = 1.225f; // Kg/m^3
    public float copter_inertia; 
    
    public void Init()
    {
        copter_inertia = 0.2f * copter_mass * Mathf.Pow(0.45f * 0.2f, 2);
    }

    public void Reset()
    {
        battery_dropped_voltage = 0;
        battery_current = 0;
    }

}
