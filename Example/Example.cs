using CSharpDroneLib;
using System;

namespace SimpleExample
{
    class Program
    {
        static IDrone DroneAct = new Drone();
        static int port;
        static int frequency;
        static string connection;
        static string mode1;
        static int alt;
        static float coord1;
        static float coord2;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Console.WriteLine("Type the connection model (UDP, TCP or SERIAL):");
            connection = Console.ReadLine();
            Console.WriteLine("Type the number of the connection door:");
            port = Convert.ToInt32(Console.ReadLine());
            Console.WriteLine("Type the frequency of the connection:");
            frequency = Convert.ToInt32(Console.ReadLine());
            Console.WriteLine("Enter to connect");
            Console.ReadLine();
            DroneAct.Connect(port, frequency, EnConnectionType.UDP, "");

            Console.WriteLine("Enter to arm");
            Console.ReadLine();
            DroneAct.Arm();

            Console.WriteLine("Enter to enter flight mode:");
            Console.ReadLine();
            DroneAct.SetFlightMode(EnFlightMode.Guided);

            Console.WriteLine("Type the height for flight:");
            alt = Convert.ToInt32(Console.ReadLine());
            DroneAct.TakeOff(alt);

            Console.WriteLine("Enter the latitude of the waypoint:");
            coord1 = Convert.ToSingle(Console.ReadLine());

            Console.WriteLine("Enter the longitude of the waypoint:");
            coord2 = Convert.ToSingle(Console.ReadLine());

            Console.WriteLine("Drone is in flight mode. Press Enter to go to waypoint.");
            Console.ReadLine();

            DroneAct.GoToWaypoint(coord1, coord2, alt);

            Console.WriteLine("Drone is on its way to the waypoint. Press Enter to disarm and land.");
            Console.ReadLine();

            DroneAct.Disarm();
            DroneAct.Land();

            Console.WriteLine("Drone has landed. Press Enter to disconnect.");
            Console.ReadLine();

            DroneAct.Disconnect();
        }
    }
}