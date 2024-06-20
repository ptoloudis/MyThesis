using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface I_Engine
{
    void InitEngine();
    void UpdateEngine(ushort pwn, float battery_dropped_voltage);
    void Reset();
    float GetCurrent();
    float GetThrust();
    float GetRoll();
    float GetPitch();
    float GetYaw();
}
