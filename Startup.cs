using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using CSharpDroneLib;

namespace SimpleExample
{

    class Program
    {
        static Drone DroneAct = new Drone();
        static int port;
        static int frequency;
        static string connection;
        static string mode1;
        static int alt;
        static int coord1;
        static int coord2;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {

            Console.WriteLine("Type the connection model(UDP, TCP or SERIAL):");
            connection = Console.ReadLine();
            Console.WriteLine("Type the number of the connection door:");
            port = 14551;
            Console.WriteLine("Type the frequency of the connection:");
            frequency = 47600;
            Console.WriteLine("Enter to connect");
            Console.ReadLine();
            DroneAct.Connect(port, frequency, (EnConnectionType)1, "");
            Console.WriteLine("Enter to arm");
            Console.ReadLine();
            DroneAct.Arm();
            Console.WriteLine("Enter to enter flight mode:");
            Console.ReadLine();
            DroneAct.SetFlightMode((EnFlightMode)3);
            Console.WriteLine("Type height for flight:");
            DroneAct.TakeOff(80);
            Console.ReadLine();

        }

    }
}
