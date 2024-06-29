[System.Serializable]
public class IMUData
{
    public double timestamp;
    public IMU imu;
    public double[] position;
    public double[] attitude;
    public double[] velocity;

    public IMUData(double timestamp, IMU imu, double[] position, double[] attitude, double[] velocity)
    {
        this.timestamp = timestamp;
        this.imu = imu;
        this.position = position;
        this.attitude = attitude;
        this.velocity = velocity;
    }
}

[System.Serializable]
public class IMU
{
    public double[] gyro;
    public double[] accel_body;

    public IMU(double[] gyro, double[] accel_body)
    {
        this.gyro = gyro;
        this.accel_body = accel_body;
    }
}

