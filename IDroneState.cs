public interface IDroneState
{
    public float Altitude { get; set; }
    public float Latitude { get; set; }
    public float Longitude { get; set; }
    public float Rollspeed { get; set; }
    public float Yawspeed { get; set; }
    public float Pitchspeed { get; set; }
    public float Roll { get; set; }
    public float Pitch { get; set; }
    public float Yaw { get; set; }
    public float BatteryLevel { get; set; }
    EnFlightMode FlightMode { get; set; }
}