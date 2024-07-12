using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface I_Engine
{
    void InitEngine();
    void UpdateEngine(ushort pwn, float battery_dropped_voltage);
    void Reset();
    float Current();
    float Thrust();
    float Roll();
    float Pitch();
    float Yaw();
}
