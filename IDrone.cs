using System;

public interface IDrone
{
    public IDroneState DroneState { get; set; }  
    void SetDroneState();
    void GetDroneState();

    void TakeOff(int altitude);
    void Land();
    void GoToWaypoint(float latitude, float longitude, float altitude);
    void SetFlightMode(EnFlightMode flightMode);
    void Arm();
    void Disarm();
    void Connect(int port, int baudRate, EnConnectionType connType,string ip);
    void Disconnect();
}