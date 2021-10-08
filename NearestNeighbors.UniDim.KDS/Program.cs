#nullable enable

using KDS;
using KDS.Interfaces;
using NearestNeighbors.UniDim.KDS.Algorithm;
using NearestNeighbors.UniDim.KDS.Algorithm.Certificates;
using NearestNeighbors.UniDim.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;

namespace NearestNeighbors.UniDim.KDS
{
    internal static class Program
    {
        private static Point[] SimulationPoints = Array.Empty<Point>();

        /// <summary>
        /// This function gets all data from the data set we are working with and normalizes it so we have a maximum speed of 1.
        /// </summary>
        /// <returns>Normalized parsed dataset</returns>
        private static Point[] Compute()
        {
            IEnumerable<string>? csvs = Directory.EnumerateFiles(@"C:\Users\Gus\Documents\GitHub\KDS_1D\DataSet4", "*.csv");

            Point[]? data = DataSetReader.ReadDataSet(csvs);
            double maxspeed = DataSetReader.GetMaxSpeed(data);

            foreach (Point? pt in data)
            {
                foreach (Data? el in pt.Data)
                {
                    el.X /= maxspeed;
                    el.Y /= maxspeed;
                }
            }

            return data;
        }

        private static async Task Main(string[] _)
        {
            SimulationPoints = Compute();

            // Provide a Node Initialize class capabable of performing initialization of the data structure
            // This will get called by the simulator when needed
            INodeInitializer<Node> nodeInitializer = new NodeInitializer();

            // Provide a Code class implementing the ICode interface. This is our algorithm designed to work with certificates
            IAlgorithmCode<Node> algorithmCode = new AlgorithmCode();

            // This starts the simulation
            // We provide a list of files to make accessible from the SimulationPoint<Node> class so we can move points,
            // As well as the move point function that will move said points into place
            Simulator<Node>? sim = new(EnablePredictions: false, EndTime: SimulationPoints.Max(x => x.Data.Max(x => x.T)));
            //sim.SimulationPointsChanged += Sim_SimulationPointsChanged;
            sim.SimulationTick += Sim_SimulationTick;
            await sim.StartSimulationAsync(algorithmCode,
                nodeInitializer,
                new CertificateGenerator(),
                null,
                SimulationPoints.Select(x => x.File), MovePoints);
        }

        private static void Sim_SimulationTick(double CurrentTime, IEnumerable<SimulationPoint<Node>> Points)
        {
            foreach (SimulationPoint<Node>? point in Points)
            {
                var closenodes = point.Node.Neighbors.OrderBy(x => x.Identifier).Select(x => x.Identifier.ToString());

                // Real current simulation data
                Console.ForegroundColor = ConsoleColor.Blue;
                Console.WriteLine($"[RE][{point.Identifier}] Nu: {string.Join(", ", closenodes)}");

                File.AppendAllLines("simulation.csv", new string[] { $"{CurrentTime},{point.Identifier},{string.Join(" ", closenodes)}" +
                    $",{point.ExternalEvents},{point.InternalEvents},{point.Node.Cost.Certificate},{point.Node.Cost.Structure},{point.Node.Cost.Queries},{point.SentMessages},{point.ReceivedMessages}" });

                Console.ForegroundColor = ConsoleColor.White;

                Console.Write($"External Events: {point.ExternalEvents}");
                Console.Write($" Internal Events: {point.InternalEvents}");
                Console.Write($" Certificate Cost: {point.Node.Cost.Certificate}");
                Console.Write($" Structure Cost: {point.Node.Cost.Structure}");
                Console.Write($" Queries Cost: {point.Node.Cost.Queries}");
                Console.Write($" Sent Messages: {point.SentMessages}");
                Console.Write($" Received Messages: {point.ReceivedMessages}");
                Console.WriteLine($" Recomputed Pol Count: {point.RecomputedPolynomialCount}");
                Console.WriteLine();
            }
        }

        private static void Sim_SimulationPointsChanged(double CurrentTime, IEnumerable<SimulationPoint<Node>> Points, IEnumerable<SimulationPoint<Node>> ChangedPoints)
        {
            Console.WriteLine("================");

            foreach (SimulationPoint<Node>? point in ChangedPoints)
            {
                var closenodes = point.Node.Neighbors.OrderBy(x => x.Identifier).Select(x => x.Identifier.ToString());

                // Real current simulation data
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"[RE][{point.Identifier}] Nu: {string.Join(", ", closenodes)}");
                //Console.WriteLine($" Nu: {string.Join(", ", point.Node.Neighbors.Select(x => x.Identifier).OrderBy(x => x))}");

                File.AppendAllLines("simulation.csv", new string[] { $"{point.Identifier},{string.Join(" ", closenodes)}" +
                    $",{point.ExternalEvents},{point.InternalEvents},{point.Node.Cost.Certificate},{point.Node.Cost.Structure},{point.Node.Cost.Queries},{point.SentMessages},{point.ReceivedMessages}" });

                // Theoritical data
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine($"[TH]|{point.Identifier}| Nu: {string.Join(", ", closenodes)}");
                //Console.WriteLine($" Nu: {string.Join(", ", point.OldData.Node.Neighbors.Select(x => x.Identifier))}");
                Console.ForegroundColor = ConsoleColor.White;

                Console.Write($"External Events: {point.ExternalEvents}");
                Console.Write($" Internal Events: {point.InternalEvents}");
                Console.Write($" Certificate Cost: {point.Node.Cost.Certificate}");
                Console.Write($" Structure Cost: {point.Node.Cost.Structure}");
                Console.Write($" Queries Cost: {point.Node.Cost.Queries}");
                Console.Write($" Sent Messages: {point.SentMessages}");
                Console.Write($" Received Messages: {point.ReceivedMessages}");
                Console.WriteLine($" Recomputed Pol Count: {point.RecomputedPolynomialCount}");
                Console.WriteLine();
            }
            Console.ReadLine();
        }

        /// <summary>
        /// This function moves a point to a new location depending on the current and previous time of the simulation
        /// </summary>
        /// <param name="Points">The list of points currently present in the simulation</param>
        /// <param name="CurrentTime">The current time of the simulation</param>
        /// <param name="OldCurrentTime">The previous time of the simulation, if available</param>
        /// <returns></returns>
        private static async Task MovePoints(IEnumerable<SimulationPoint<Node>> Points, double CurrentTime, double? OldCurrentTime)
        {
            foreach (SimulationPoint<Node>? point in Points)
            {
                bool wrotePosition = false;
                Point pt = SimulationPoints.First(x => x.File == point.DataFile);
                foreach (Data d in pt.Data)
                {
                    if (OldCurrentTime != null && d.T <= OldCurrentTime)
                    {
                        continue;
                    }

                    if (d.T <= CurrentTime)
                    {
                        point.AddLastPosition(new double[] { d.X, d.Y }, d.T);
                        wrotePosition = true;
                    }
                }
                // The point may not actually move anymore because it is dead, we need the data to get that it's a point now, hence this check.
                // As a result we will generate a polynomial that is a constant because of the following code.
                if (!wrotePosition)
                {
                    point.AddLastPosition(new double[] { point.X.Static, point.Y.Static }, CurrentTime);
                }
            }
        }
    }
}
