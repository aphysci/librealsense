using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;

namespace Intel.RealSense
{
    class Program
    {
        private const string Path = "c:\\temp\\cs_mapfile";

        static void Main(string[] args) {
            if (File.Exists(Path)) {
                Console.WriteLine("Relocalizing...");
                relocalize();
            } else {
                Console.WriteLine("Making the map...");
                makemap();
            }
        }

        static void makemap() { 
            using (var ctx = new Context())
            {
                var devices = ctx.QueryDevices();

                Console.WriteLine("There are {0} connected RealSense devices.", devices.Count);
                if (devices.Count == 0) return;
                var dev = devices[0];

                Console.WriteLine("\nUsing device 0, an {0}", dev.Info[CameraInfo.Name]);
                Console.WriteLine("    Serial number: {0}", dev.Info[CameraInfo.SerialNumber]);
                Console.WriteLine("    Firmware version: {0}", dev.Info[CameraInfo.FirmwareVersion]);

                var poseSensor = dev.Sensors[0];


                var sp = poseSensor.StreamProfiles.Where(p => p.Stream == Stream.Pose).First();

                poseSensor.Open(sp);

                bool stopit = false;
                poseSensor.Start<PoseFrame>(f => {
                    if (!stopit)
                      Console.Write("Translation = {0} {1} {2}\r" , f.PoseData.translation.x, f.PoseData.translation.y, f.PoseData.translation.z);
                });

                AutoResetEvent stop = new AutoResetEvent(false);
                Console.CancelKeyPress += (s, e) =>
                {
                    e.Cancel = true;
                    stop.Set();
                };
                stop.WaitOne();
                stopit = true;
                foreach (var o in poseSensor.Options) {
                    Console.WriteLine(o.Key.ToString() + o.Value);
                }
                Math.Vector pos = new Math.Vector();
                pos.x = pos.y = pos.z = 0;
                Math.Quaternion rot = new Math.Quaternion();
                rot.x = rot.y = rot.z = 0;
                rot.w = 1;
                bool ok = poseSensor.SetStaticNode("origin", pos, rot);
                Console.WriteLine("SetStatic node returns: " + ok);
                poseSensor.Stop();
                Thread.Sleep(1000);
                var map = poseSensor.ExportLocalizationMap();
                Console.WriteLine("Got map: " + map + " num " + map.Count());
                Console.WriteLine("bytes = " + map[0] + ":" + map[1] + ":" + map[2] + ":");

                poseSensor.Close();
                File.WriteAllBytes(Path, map);
            }
        }

        static void relocalize() {
            using (var ctx = new Context()) {
                var devices = ctx.QueryDevices();

                Console.WriteLine("There are {0} connected RealSense devices.", devices.Count);
                if (devices.Count == 0) return;
                var dev = devices[0];

                Console.WriteLine("\nUsing device 0, an {0}", dev.Info[CameraInfo.Name]);
                Console.WriteLine("    Serial number: {0}", dev.Info[CameraInfo.SerialNumber]);
                Console.WriteLine("    Firmware version: {0}", dev.Info[CameraInfo.FirmwareVersion]);

                var poseSensor = dev.Sensors[0];


                var sp = poseSensor.StreamProfiles.Where(p => p.Stream == Stream.Pose).First();

                poseSensor.Open(sp);

                byte[] b = File.ReadAllBytes(Path);
                poseSensor.ImportLocalizationMap(b);
                bool stopit = false;
                poseSensor.Start<PoseFrame>(f => {
                    if (!stopit)
                        Console.Write("Translation = {0} {1} {2}\r", f.PoseData.translation.x, f.PoseData.translation.y, f.PoseData.translation.z);
                });



                ConsoleKeyInfo cki;
                do {
                    Console.WriteLine("\nPress a key to display; press the 'x' key to quit.");

                    while (Console.KeyAvailable == false)
                        Thread.Sleep(250); // Loop until input is entered.

                    cki = Console.ReadKey(true);
                    //Console.WriteLine("You pressed the '{0}' key.", cki.Key);
                    Math.Vector posO = new Math.Vector();
                    Math.Quaternion rotO = new Math.Quaternion();
                    bool hasIt = poseSensor.GetStaticNode("origin", out posO, out rotO);
                    Console.WriteLine("\n\nOrigin: present: {0} ({1}:{2}:{3})\n", hasIt, posO.x, posO.y, posO.z);
                } while (cki.Key != ConsoleKey.X);

                /*
                AutoResetEvent stop = new AutoResetEvent(false);
                Console.CancelKeyPress += (s, e) => {
                    e.Cancel = true;
                    stop.Set();
                };

                stop.WaitOne();
                */
                stopit = true;
                foreach (var o in poseSensor.Options) {
                    Console.WriteLine(o.Key.ToString() + " " + o.Value);
                }
                poseSensor.Stop();
                Thread.Sleep(1000);
                poseSensor.Close();
            }
        }

    }
}
