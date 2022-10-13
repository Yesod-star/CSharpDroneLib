public interface IDroneState
{
    float Altitude { get; set; }
    float Latitude { get; set; }
    float Longitude { get; set; }
    float Roll { get; set; }
    float Pitch { get; set; }
    float Yaw { get; set; }
    float BatteryLevel { get; set; }
    EnFlightMode FlightMode { get; set; }
}