using Cepton.SDK;
using System;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;

namespace Cepton.SDKSample
{
    class Program
    {
        static void ErrorCallback(IntPtr handle, SensorErrorCode error_code, String error_msg, IntPtr error_data, IntPtr user_data)
        {
            float[] float_data = new float[2];
            switch (error_code)
            {
                case SensorErrorCode.FAULT_EXTREME_TEMPERATURE:
                    Marshal.Copy(error_data, float_data, 0, 1); // Copy data from caller
                    Console.WriteLine("{0}: {1} (T={2})", error_code, error_msg, float_data[0]);
                    break;
                default:
                    Console.WriteLine("{0}: {1}", error_code, error_msg);
                    break;
            }
        }

        static void FrameCallback(IntPtr handle, int n_points, SensorImagePoint[] points, IntPtr user_data)
        {
            Console.WriteLine("Frame {0}", n_points);
        }

        static void TestSensorOjbect()
        {
            CeptonSDK.Initialize(ErrorCallback);
            Thread.Sleep(1000);
            foreach (var s in CeptonSDK.Sensors)
            {
                var points = s.GetImagePoints(1000);
                foreach (var p in points)
                    Console.WriteLine(p);
            }
            CeptonSDK.DeInitialize();
        }

        static void TestReplay(string path)
        {
            CeptonSDK.Initialize(ErrorCallback);
            CaptureReplay.Open(path);
            Thread.Sleep(1000);
            CeptonSDK.DeInitialize();
        }

        static void Main(string[] args)
        {
            // TestSensorOjbect();
            // TestReplay(args[0]);
            CeptonSDK.Initialize(ErrorCallback);
            Console.WriteLine("SDK initialized");
            Thread.Sleep(2000); // Wait for 2 seconds for sensor discovery
            Sensor s = CeptonSDK.Sensors.FirstOrDefault();
            if (s != null)
            {
                Console.WriteLine("Found sensor #{0}", s.SerialNumber);
                for (int i = 0; i < 10; i++)
                {
                    var image_points = s.GetImagePoints(100);
                    Console.WriteLine("Got {0} points", image_points.Length);
                }
            }

            CeptonSDK.DeInitialize();
            Console.WriteLine("SDK deinitialized");
        }
    }
}
