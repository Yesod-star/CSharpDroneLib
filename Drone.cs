using Comms;
using log4net;
using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Net.Sockets;

namespace CSharpDroneLib
{
    public class Drone : IDrone
    {
        MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
        bool armed = false;
        // locking to prevent multiple reads on serial port
        object readlock = new object();
        // our target sysid
        byte sysid;
        // our target compid
        byte compid;
        //Connect via UDP
        Comms.UdpSerial UdpSerialConnect1 = new Comms.UdpSerial();
        //Connect via TCP
        Comms.TcpSerial TcpSerialConnect1 = new Comms.TcpSerial();
        //Connect via SERIAL
        SerialPort serialPort1 = new SerialPort();
        //Tipo de conexao
        EnConnectionType TypeConnect;
        //Check arm
        int stateArm = 0;
        private ILog log = LogManager.GetLogger(typeof(TcpSerial));

        public IDroneState DroneState { 
            get => throw new NotImplementedException(); set => throw new NotImplementedException(); 
        }

        public void GetDroneState()
        {
            throw new NotImplementedException();
        }

        public void SetDroneState()
        {
            throw new NotImplementedException();
        }

        void IDrone.ReadPackage(object sender, EventArgs e)
        {

            switch ((int)TypeConnect)
            {
                case 1:
                    while (UdpSerialConnect1.IsOpen)
                    {
                        try
                        {
                            MAVLink.MAVLinkMessage packet;
                            lock (readlock)
                            {
                                // read any valid packet from the port
                                packet = mavlink.ReadPacket(UdpSerialConnect1.BaseStream);

                                // check its valid
                                if (packet == null || packet.data == null)
                                    continue;
                            }

                            // check to see if its a hb packet from the comport
                            if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                            {
                                var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                                // save the sysid and compid of the seen MAV
                                sysid = packet.sysid;
                                compid = packet.compid;

                                // request streams at 2 hz
                                var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                                    new MAVLink.mavlink_request_data_stream_t()
                                    {
                                        req_message_rate = 2,
                                        req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                        start_stop = 1,
                                        target_component = compid,
                                        target_system = sysid
                                    });

                                UdpSerialConnect1.Write(buffer, 0, buffer.Length);

                                buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

                                UdpSerialConnect1.Write(buffer, 0, buffer.Length);
                            }

                            // from here we should check the the message is addressed to us
                            if (sysid != packet.sysid || compid != packet.compid)
                                continue;

                            Console.WriteLine(packet.msgtypename);

                            if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                            //or
                            //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                            {
                                var att = (MAVLink.mavlink_attitude_t)packet.data;

                                Console.WriteLine(att.pitch * 57.2958 + " " + att.roll * 57.2958);
                            }
                        }
                        catch
                        {
                        }

                        System.Threading.Thread.Sleep(1);
                    }
                    break;
                case 2:
                    while (TcpSerialConnect1.IsOpen)
                    {
                        try
                        {
                            MAVLink.MAVLinkMessage packet;
                            lock (readlock)
                            {
                                // read any valid packet from the port
                                packet = mavlink.ReadPacket(TcpSerialConnect1.BaseStream);

                                // check its valid
                                if (packet == null || packet.data == null)
                                    continue;
                            }

                            // check to see if its a hb packet from the comport
                            if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                            {
                                var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                                // save the sysid and compid of the seen MAV
                                sysid = packet.sysid;
                                compid = packet.compid;

                                // request streams at 2 hz
                                var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                                    new MAVLink.mavlink_request_data_stream_t()
                                    {
                                        req_message_rate = 2,
                                        req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                        start_stop = 1,
                                        target_component = compid,
                                        target_system = sysid
                                    });

                                TcpSerialConnect1.Write(buffer, 0, buffer.Length);

                                buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

                                TcpSerialConnect1.Write(buffer, 0, buffer.Length);
                            }

                            // from here we should check the the message is addressed to us
                            if (sysid != packet.sysid || compid != packet.compid)
                                continue;

                            Console.WriteLine(packet.msgtypename);

                            if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                            //or
                            //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                            {
                                var att = (MAVLink.mavlink_attitude_t)packet.data;

                                Console.WriteLine(att.pitch * 57.2958 + " " + att.roll * 57.2958);
                            }
                        }
                        catch
                        {
                        }

                        System.Threading.Thread.Sleep(1);
                    }
                    break;
                case 3:
                    while (serialPort1.IsOpen)
                    {
                        try
                        {
                            MAVLink.MAVLinkMessage packet;
                            lock (readlock)
                            {
                                // read any valid packet from the port
                                packet = mavlink.ReadPacket(serialPort1.BaseStream);

                                // check its valid
                                if (packet == null || packet.data == null)
                                    continue;
                            }

                            // check to see if its a hb packet from the comport
                            if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                            {
                                var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                                // save the sysid and compid of the seen MAV
                                sysid = packet.sysid;
                                compid = packet.compid;

                                // request streams at 2 hz
                                var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                                    new MAVLink.mavlink_request_data_stream_t()
                                    {
                                        req_message_rate = 2,
                                        req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                        start_stop = 1,
                                        target_component = compid,
                                        target_system = sysid
                                    });

                                serialPort1.Write(buffer, 0, buffer.Length);

                                buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

                                serialPort1.Write(buffer, 0, buffer.Length);
                            }

                            // from here we should check the the message is addressed to us
                            if (sysid != packet.sysid || compid != packet.compid)
                                continue;

                            Console.WriteLine(packet.msgtypename);

                            if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                            //or
                            //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                            {
                                var att = (MAVLink.mavlink_attitude_t)packet.data;

                                Console.WriteLine(att.pitch * 57.2958 + " " + att.roll * 57.2958);
                            }
                        }
                        catch
                        {
                        }

                        System.Threading.Thread.Sleep(1);
                    }
                    break;
                default:
                    Console.WriteLine("Connection Value invalid");
                    break;
            }

        }

        T readsomedata<T>(byte sysid, byte compid, int timeout = 2000)
        {
            DateTime deadline = DateTime.Now.AddMilliseconds(timeout);

            lock (readlock)
            {
                // read the current buffered bytes
                while (DateTime.Now < deadline)
                {
                    var packet = mavlink.ReadPacket(UdpSerialConnect1.BaseStream);

                    // check its not null, and its addressed to us
                    if (packet == null || sysid != packet.sysid || compid != packet.compid)
                        continue;

                    Console.WriteLine(packet);


                    if (packet.data.GetType() == typeof(T))
                    {
                        return (T)packet.data;
                    }
                }
            }

            throw new Exception("No packet match found");
        }

        private void bgw_DoWork(object sender, EventArgs e)
        {
            switch ((int)TypeConnect)
            {
                case 1:
                    while (UdpSerialConnect1.IsOpen)
                    {
                        try
                        {
                            MAVLink.MAVLinkMessage packet;
                            lock (readlock)
                            {
                                // read any valid packet from the port
                                packet = mavlink.ReadPacket(UdpSerialConnect1.BaseStream);

                                // check its valid
                                if (packet == null || packet.data == null)
                                    continue;
                            }

                            // check to see if its a hb packet from the comport
                            if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                            {
                                var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                                // save the sysid and compid of the seen MAV
                                sysid = packet.sysid;
                                compid = packet.compid;

                                // request streams at 2 hz
                                var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                                    new MAVLink.mavlink_request_data_stream_t()
                                    {
                                        req_message_rate = 2,
                                        req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                        start_stop = 1,
                                        target_component = compid,
                                        target_system = sysid
                                    });

                                UdpSerialConnect1.Write(buffer, 0, buffer.Length);

                                buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

                                UdpSerialConnect1.Write(buffer, 0, buffer.Length);
                            }

                            // from here we should check the the message is addressed to us
                            if (sysid != packet.sysid || compid != packet.compid)
                                continue;

                            Console.WriteLine(packet.msgtypename);

                            if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                            //or
                            //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                            {
                                var att = (MAVLink.mavlink_attitude_t)packet.data;

                                Console.WriteLine(att.pitch * 57.2958 + " " + att.roll * 57.2958);
                            }
                        }
                        catch
                        {
                        }

                        System.Threading.Thread.Sleep(1);
                    }
                    break;
                case 2:
                    while (TcpSerialConnect1.IsOpen)
                    {
                        try
                        {
                            MAVLink.MAVLinkMessage packet;
                            lock (readlock)
                            {
                                // read any valid packet from the port
                                packet = mavlink.ReadPacket(TcpSerialConnect1.BaseStream);

                                // check its valid
                                if (packet == null || packet.data == null)
                                    continue;
                            }

                            // check to see if its a hb packet from the comport
                            if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                            {
                                var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                                // save the sysid and compid of the seen MAV
                                sysid = packet.sysid;
                                compid = packet.compid;

                                // request streams at 2 hz
                                var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                                    new MAVLink.mavlink_request_data_stream_t()
                                    {
                                        req_message_rate = 2,
                                        req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                        start_stop = 1,
                                        target_component = compid,
                                        target_system = sysid
                                    });

                                TcpSerialConnect1.Write(buffer, 0, buffer.Length);

                                buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

                                TcpSerialConnect1.Write(buffer, 0, buffer.Length);
                            }

                            // from here we should check the the message is addressed to us
                            if (sysid != packet.sysid || compid != packet.compid)
                                continue;

                            Console.WriteLine(packet.msgtypename);

                            if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                            //or
                            //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                            {
                                var att = (MAVLink.mavlink_attitude_t)packet.data;

                                Console.WriteLine(att.pitch * 57.2958 + " " + att.roll * 57.2958);
                            }
                        }
                        catch
                        {
                        }

                        System.Threading.Thread.Sleep(1);
                    }
                    break;
                case 3:
                    while (serialPort1.IsOpen)
                    {
                        try
                        {
                            MAVLink.MAVLinkMessage packet;
                            lock (readlock)
                            {
                                // read any valid packet from the port
                                packet = mavlink.ReadPacket(serialPort1.BaseStream);

                                // check its valid
                                if (packet == null || packet.data == null)
                                    continue;
                            }

                            // check to see if its a hb packet from the comport
                            if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                            {
                                var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                                // save the sysid and compid of the seen MAV
                                sysid = packet.sysid;
                                compid = packet.compid;

                                // request streams at 2 hz
                                var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                                    new MAVLink.mavlink_request_data_stream_t()
                                    {
                                        req_message_rate = 2,
                                        req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                        start_stop = 1,
                                        target_component = compid,
                                        target_system = sysid
                                    });

                                serialPort1.Write(buffer, 0, buffer.Length);

                                buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

                                serialPort1.Write(buffer, 0, buffer.Length);
                            }

                            // from here we should check the the message is addressed to us
                            if (sysid != packet.sysid || compid != packet.compid)
                                continue;

                            Console.WriteLine(packet.msgtypename);

                            if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                            //or
                            //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                            {
                                var att = (MAVLink.mavlink_attitude_t)packet.data;

                                Console.WriteLine(att.pitch * 57.2958 + " " + att.roll * 57.2958);
                            }
                        }
                        catch
                        {
                        }

                        System.Threading.Thread.Sleep(1);
                    }
                    break;
                default:
                    Console.WriteLine("Connection Value invalid");
                    break;
            }

        }

        public void Arm()
        {
            if (stateArm == 0)
            {
                //Variavel para armazenar o comando
                MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();


                //Geracao do comando
                req.target_system = 1;
                req.target_component = 1;

                req.command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM;

                req.param1 = armed ? 0 : 1;
                armed = !armed;




                //Armazenamento do comando
                byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

                switch ((int)TypeConnect)
                {
                    case 1:
                        //Envio do comando
                        UdpSerialConnect1.Write(packet, 0, packet.Length);
                        break;
                    case 2:
                        //Envio do comando
                        TcpSerialConnect1.Write(packet, 0, packet.Length);
                        break;
                    case 3:
                        //Envio do comando
                        serialPort1.Write(packet, 0, packet.Length);
                        break;
                    default:
                        log.InfoFormat("Connection Value invalid");
                        break;
                }


                //Ver se esta sendo aceito o comando
                try
                {
                    var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                    if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                    {
                        stateArm = 1;
                    }
                }
                catch
                {
                }
            }
            else
            {
                log.InfoFormat("Drone já armado");
            }
        }

        public void Disarm()
        {
            {
                if (stateArm == 0)
                {
                    //Variavel para armazenar o comando
                    MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();


                    //Geracao do comando
                    req.target_system = 1;
                    req.target_component = 1;

                    req.command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM;

                    req.param1 = armed ? 0 : 1;
                    armed = !armed;




                    //Armazenamento do comando
                    byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

                    switch ((int)TypeConnect)
                    {
                        case 1:
                            //Envio do comando
                            UdpSerialConnect1.Write(packet, 0, packet.Length);
                            break;
                        case 2:
                            //Envio do comando
                            TcpSerialConnect1.Write(packet, 0, packet.Length);
                            break;
                        case 3:
                            //Envio do comando
                            serialPort1.Write(packet, 0, packet.Length);
                            break;
                        default:
                            log.InfoFormat("Connection Value invalid");
                            break;
                    }


                    //Ver se esta sendo aceito o comando
                    try
                    {
                        var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                        if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                        {
                            stateArm = 0;
                        }
                    }
                    catch
                    {
                    }
                }
            }
        }

        public void Connect(int port, int baudRate, EnConnectionType TypeConnect, string ip)
        {
            if (ip == null || ip == "")
            {
                ip = "127.0.0.1";
            }
            switch ((int)TypeConnect)
            {
                case 1:
                    log = LogManager.GetLogger(typeof(UdpSerial));
                    TypeConnect = (EnConnectionType)1;
                    UdpSerialConnect1.Port = "" + port;
                    UdpSerialConnect1.client = new UdpClient(ip, port);

                    //if the port is open close it
                    if (UdpSerialConnect1.IsOpen)
                    {
                        UdpSerialConnect1.Close();
                        return;
                    }


                    // set the comport options
                    UdpSerialConnect1.PortName = "UDP" + port;
                    UdpSerialConnect1.BaudRate = baudRate;
                    // open the comport
                    UdpSerialConnect1.Open();
                    // set timeout to 2 seconds
                    UdpSerialConnect1.ReadTimeout = 2000;
                    break;
                case 2:
                    log = LogManager.GetLogger(typeof(TcpSerial));
                    TypeConnect = (EnConnectionType)2;
                    TcpSerialConnect1.Port = "" + port;
                    TcpSerialConnect1.client = new TcpClient(ip, port);

                    //if the port is open close it
                    if (TcpSerialConnect1.IsOpen)
                    {
                        TcpSerialConnect1.Close();
                        return;
                    }


                    // set the comport options
                    TcpSerialConnect1.PortName = "TCP" + port;
                    TcpSerialConnect1.BaudRate = baudRate;
                    // open the comport
                    TcpSerialConnect1.Open();
                    // set timeout to 2 seconds
                    TcpSerialConnect1.ReadTimeout = 2000;
                    break;
                case 3:
                    log = LogManager.GetLogger(typeof(SerialPort));
                    TypeConnect = (EnConnectionType)3;
                    // if the port is open close it
                    if (serialPort1.IsOpen)
                    {
                        serialPort1.Close();
                        return;
                    }

                    // set the comport options
                    serialPort1.PortName = "COM" + port;
                    serialPort1.BaudRate = baudRate;

                    // open the comport
                    serialPort1.Open();

                    // set timeout to 2 seconds
                    serialPort1.ReadTimeout = 2000;

                    break;
                default:
                    log.InfoFormat("Connection Value invalid");
                    break;
            }

            BackgroundWorker bgw = new BackgroundWorker();

            bgw.DoWork += bgw_DoWork;

            bgw.RunWorkerAsync();
        }

        public void Disconnect()
        {
            switch ((int)TypeConnect)
            {
                case 1:
                    if (UdpSerialConnect1.IsOpen)
                    {
                        UdpSerialConnect1.Close();
                        log.InfoFormat("Connection closed");
                        return;
                    }
                    else
                    {
                        log.InfoFormat("Connection is not open");
                    }
                    break;
                case 2:
                    if (TcpSerialConnect1.IsOpen)
                    {
                        TcpSerialConnect1.Close();
                        log.InfoFormat("Connection closed");
                        return;
                    }
                    else
                    {
                        log.InfoFormat("Connection is not open");
                    }
                    break;
                case 3:
                    if (serialPort1.IsOpen)
                    {
                        serialPort1.Close();
                        log.InfoFormat("Connection closed");
                        return;
                    }
                    else
                    {
                        log.InfoFormat("Connection is not open");
                    }
                    break;
                default:
                    log.InfoFormat("Connection Value invalid");
                    break;
            }
        }

        public void GoToWaypoint(float latitude, float longitude, float altitude)
        {
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            //Geracao do comando
            req.target_system = 255;
            req.target_component = 190;

            req.command = (ushort)MAVLink.MAV_CMD.WAYPOINT;
            req.param5 = latitude;
            req.param6 = longitude;
            req.param7 = altitude;

            //Armazenamento do comando
            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            switch ((int)TypeConnect)
            {
                case 1:
                    //Envio do comando
                    UdpSerialConnect1.Write(packet, 0, packet.Length);
                    break;
                case 2:
                    //Envio do comando
                    TcpSerialConnect1.Write(packet, 0, packet.Length);
                    break;
                case 3:
                    //Envio do comando
                    serialPort1.Write(packet, 0, packet.Length);
                    break;
                default:
                    log.InfoFormat("Connection Value invalid");
                    break;
            }

            //Ver se esta sendo aceito o comando
            try
            {
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                {

                }
            }
            catch
            {
            }
        }

        public void Land()
        {
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            //Geracao do comando
            req.target_system = 1;
            req.target_component = 1;

            req.command = (ushort)MAVLink.MAV_CMD.LAND;

            //Armazenamento do comando
            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            switch ((int)TypeConnect)
            {
                case 1:
                    //Envio do comando
                    UdpSerialConnect1.Write(packet, 0, packet.Length);
                    break;
                case 2:
                    //Envio do comando
                    TcpSerialConnect1.Write(packet, 0, packet.Length);
                    break;
                case 3:
                    //Envio do comando
                    serialPort1.Write(packet, 0, packet.Length);
                    break;
                default:
                    log.InfoFormat("Connection Value invalid");
                    break;
            }

            //Ver se esta sendo aceito o comando
            try
            {
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                {

                }
            }
            catch
            {
            }
        }

        public void TakeOff(int altitude)
        {
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            //Geracao do comando
            req.target_system = 1;
            req.target_component = 1;

            req.command = (ushort)MAVLink.MAV_CMD.TAKEOFF;
            req.param1 = 0;
            req.param2 = 0;
            req.param3 = 0;
            req.param4 = 0;
            req.param5 = 0;
            req.param6 = 0;
            req.param7 = 75;

            //Armazenamento do comando
            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            switch ((int)TypeConnect)
            {
                case 1:
                    //Envio do comando
                    UdpSerialConnect1.Write(packet, 0, packet.Length);
                    break;
                case 2:
                    //Envio do comando
                    TcpSerialConnect1.Write(packet, 0, packet.Length);
                    break;
                case 3:
                    //Envio do comando
                    serialPort1.Write(packet, 0, packet.Length);
                    break;
                default:
                    log.InfoFormat("Connection Value invalid");
                    break;
            }

            //Ver se esta sendo aceito o comando
            try
            {
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                {

                }
            }
            catch
            {
            }
        }

        public void SetFlightMode(EnFlightMode flightMode)
        {
            //Variavel para armazenar o comando
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            //Geracao do comando
            req.target_system = 1;
            req.target_component = 1;

            req.command = (ushort)MAVLink.MAV_CMD.DO_SET_MODE;
            switch ((int)flightMode)
            {
                case 1:
                    req.param1 = 7;
                    req.param2 = 0;
                    break;
                case 2:
                    req.param1 = 7;
                    req.param2 = 2;
                    break;
                case 3:
                    req.param1 = 5;
                    req.param2 = 5;
                    break;
                case 4:
                    req.param1 = 5;
                    req.param2 = 4;
                    break;
                case 5:
                    req.param1 = 3;
                    req.param2 = 3;
                    break;
                case 6:
                    req.param1 = 9;
                    req.param2 = 9;
                    break;
                case 7:
                    req.param1 = 1;
                    req.param2 = 6;
                    break;
                default:
                    Console.WriteLine("Model of flight unknown");
                    break;
            }

            //Armazenamento do comando
            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            switch ((int)TypeConnect)
            {
                case 1:
                    //Envio do comando
                    UdpSerialConnect1.Write(packet, 0, packet.Length);
                    break;
                case 2:
                    //Envio do comando
                    TcpSerialConnect1.Write(packet, 0, packet.Length);
                    break;
                case 3:
                    //Envio do comando
                    serialPort1.Write(packet, 0, packet.Length);
                    break;
                default:
                    Console.WriteLine("Connection Value invalid");
                    break;
            }

            //Ver se esta sendo aceito o comando
            try
            {
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                {

                }
            }
            catch
            {
            }
        }

    }
}