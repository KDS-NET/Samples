using KDS;
using KDS.Certificates;
using KDS.Interfaces;
using NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm.Certificates;
using NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm
{
    public class AlgorithmCode : IAlgorithmCode<Node>
    {
        public int GetMaxIterationCount()
        {
            return 7;
        }

        #region Algorithm
        public void RunAlgorithmAfterAllPointsMovedPerPoint(int i, IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u, double CurrentTime)
        {
            switch (i)
            {
                case 0:
                    if (u.Node.PreviousData == null)
                        u.Node.PreviousData = new();
                    u.Node.CopyTo(u.Node.PreviousData);
                    Correct_Legitimate_and_Distant_Lookout_round1X(Fail, u);
                    Correct_Legitimate_and_Distant_Lookout_round1Y(Fail, u);
                    break;
                case 1:
                    Correct_Legitimate_and_Distant_Lookout_round2X(u);
                    Correct_Legitimate_and_Distant_Lookout_round2Y(u);
                    break;
                case 2:
                    Correct_Legitimate_and_Distant_Lookout_round3X(u);
                    Correct_Legitimate_and_Distant_Lookout_round3Y(u);
                    break;
                case 3:
                    Correct_Legitimate_and_Distant_Lookout_round4X(u);
                    Correct_Legitimate_and_Distant_Lookout_round4Y(u);
                    break;
                case 4:
                    Correct_Legitimate_and_Distant_Lookout_round5X(u);
                    Correct_Close_Neighbor_round5X(Fail, u);
                    Correct_Legitimate_and_Distant_Lookout_round5Y(u);
                    Correct_Close_Neighbor_round5Y(Fail, u);
                    break;
                case 5:
                    Correct_Close_Neighbor_round6X(u);
                    Correct_Potential_Neighbor_round6X(Fail, u);
                    Correct_Close_Neighbor_round6Y(u);
                    Correct_Potential_Neighbor_round6Y(Fail, u);
                    break;
                case 6:
                    Correct_Potential_Neighbor_round7X(u);
                    Correct_Potential_Neighbor_round7Y(u);

                    // Intersection between both lists is all neighbors to the point
                    u.Node.Neighbors = u.Node.NeighborsX.Intersect(u.Node.NeighborsY).ToList();
                    u.Node.Cost.Certificate += Fail.Count();
                    break;
            }
        }

        private void Correct_Legitimate_and_Distant_Lookout_round1X(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            u.Node.newLookoutPlusX = null;
            u.Node.newLookoutMinusX = null;

            // C <- v: (LegitimateLookout+,u,v) € Fail
            List<SimulationPoint<Node>> C = Fail.Where(x => x is LegitimateLookoutPlusXCertificate)
                .Select(x => x.GetV()).ToList();

            if (C.Count > 0)
            {
                // argmin pos(x) x€C
                foreach (SimulationPoint<Node> x in C)
                {
                    if (u.Node.newLookoutPlusX == null)
                    {
                        u.Node.newLookoutPlusX = x;
                    }
                    else
                    {
                        if (u.Node.newLookoutPlusX.X.Position > x.X.Position)
                        {
                            u.Node.newLookoutPlusX = x;
                        }
                    }
                }
            }
            else
            {
                bool distantLookoutLuPlusFailed = false;
                foreach (ISimulationCertificate<Node> @event in Fail)
                {
                    if (@event is DistantLookoutXCertificate && u.Node.LookoutPointPlusX != null && @event.GetV() == u.Node.LookoutPointPlusX)
                    {
                        distantLookoutLuPlusFailed = true;
                        break;
                    }
                }

                if (distantLookoutLuPlusFailed)
                {
                    u.Node.LookoutPointPlusX!.SendMessage(u, (int)SimulationPointMessageType.REQPlusX, u);
                }
                else
                {
                    u.Node.newLookoutPlusX = u.Node.LookoutPointPlusX;
                }
            }

            // C <- v: (LegitimateLookout-,u,v) € Fail
            C = Fail.Where(x => x is LegitimateLookoutMinusXCertificate)
                .Select(x => x.GetV()).ToList();

            if (C.Count > 0)
            {
                // argmax pos(x) x€C
                foreach (SimulationPoint<Node> x in C)
                {
                    if (u.Node.newLookoutMinusX == null)
                    {
                        u.Node.newLookoutMinusX = x;
                    }
                    else
                    {
                        if (u.Node.newLookoutMinusX.X.Position < x.X.Position)
                        {
                            u.Node.newLookoutMinusX = x;
                        }
                    }
                }
            }
            else
            {
                bool distantLookoutLuMinusFailed = false;
                foreach (ISimulationCertificate<Node> @event in Fail)
                {
                    if (@event is DistantLookoutXCertificate &&
                        u.Node.LookoutPointMinusX != null &&
                        @event.GetV() == u.Node.LookoutPointMinusX)
                    {
                        distantLookoutLuMinusFailed = true;
                        break;
                    }
                }

                if (distantLookoutLuMinusFailed)
                {
                    u.Node.LookoutPointMinusX!.SendMessage(u, (int)SimulationPointMessageType.REQMinusX, u);
                }
                else
                {
                    u.Node.newLookoutMinusX = u.Node.LookoutPointMinusX;
                }
            }
        }

        private void Correct_Legitimate_and_Distant_Lookout_round1Y(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            u.Node.newLookoutPlusY = null;
            u.Node.newLookoutMinusY = null;

            // C <- v: (LegitimateLookout+,u,v) € Fail
            List<SimulationPoint<Node>> C = Fail.Where(x => x is LegitimateLookoutPlusYCertificate)
                .Select(x => x.GetV()).ToList();

            if (C.Count > 0)
            {
                // argmin pos(x) x€C
                foreach (SimulationPoint<Node> x in C)
                {
                    if (u.Node.newLookoutPlusY == null)
                    {
                        u.Node.newLookoutPlusY = x;
                    }
                    else
                    {
                        if (u.Node.newLookoutPlusY.Y.Position > x.Y.Position)
                        {
                            u.Node.newLookoutPlusY = x;
                        }
                    }
                }
            }
            else
            {
                bool distantLookoutLuPlusFailed = false;
                foreach (ISimulationCertificate<Node> @event in Fail)
                {
                    if (@event is DistantLookoutYCertificate && u.Node.LookoutPointPlusY != null && @event.GetV() == u.Node.LookoutPointPlusY)
                    {
                        distantLookoutLuPlusFailed = true;
                        break;
                    }
                }

                if (distantLookoutLuPlusFailed)
                {
                    u.Node.LookoutPointPlusY!.SendMessage(u, (int)SimulationPointMessageType.REQPlusY, u);
                }
                else
                {
                    u.Node.newLookoutPlusY = u.Node.LookoutPointPlusY;
                }
            }

            // C <- v: (LegitimateLookout-,u,v) € Fail
            C = Fail.Where(x => x is LegitimateLookoutMinusYCertificate)
                .Select(x => x.GetV()).ToList();

            if (C.Count > 0)
            {
                // argmax pos(x) x€C
                foreach (SimulationPoint<Node> x in C)
                {
                    if (u.Node.newLookoutMinusY == null)
                    {
                        u.Node.newLookoutMinusY = x;
                    }
                    else
                    {
                        if (u.Node.newLookoutMinusY.Y.Position < x.Y.Position)
                        {
                            u.Node.newLookoutMinusY = x;
                        }
                    }
                }
            }
            else
            {
                bool distantLookoutLuMinusFailed = false;
                foreach (ISimulationCertificate<Node> @event in Fail)
                {
                    if (@event is DistantLookoutYCertificate &&
                        u.Node.LookoutPointMinusY != null &&
                        @event.GetV() == u.Node.LookoutPointMinusY)
                    {
                        distantLookoutLuMinusFailed = true;
                        break;
                    }
                }

                if (distantLookoutLuMinusFailed)
                {
                    u.Node.LookoutPointMinusY!.SendMessage(u, (int)SimulationPointMessageType.REQMinusY, u);
                }
                else
                {
                    u.Node.newLookoutMinusY = u.Node.LookoutPointMinusY;
                }
            }
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round2X(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> w in u.ReceiveMessages((int)SimulationPointMessageType.REQPlusX))
            {
                // C <- Nu u Lu+
                List<SimulationPoint<Node>> C = u.Node.NeighborsX.Union(u.Node.LookoutPointPlusNeighborsX)
                    .Where(x => Math.Abs(w.X.Position - x.X.Position) > Constants.R).ToList();

                // c <- argmin x€C (pos(x))
                SimulationPoint<Node>? c = null;
                foreach (SimulationPoint<Node> x in C)
                {
                    if (c == null)
                    {
                        c = x;
                    }
                    else if (c.X.Position > x.X.Position)
                    {
                        c = x;
                    }
                }

                w.SendMessage(u, (int)SimulationPointMessageType.REPLYPlusX, c!);
            }

            foreach (SimulationPoint<Node> w in u.ReceiveMessages((int)SimulationPointMessageType.REQMinusX))
            {
                // C <- Nu u Lu-
                List<SimulationPoint<Node>> C = u.Node.NeighborsX.Union(u.Node.LookoutPointMinusNeighborsX
                    .Where(x => Math.Abs(w.X.Position - x.X.Position) > Constants.R)).ToList();

                // c <- argmax x€C (pos(x))
                SimulationPoint<Node>? c = null;
                foreach (SimulationPoint<Node> x in C)
                {
                    if (c == null)
                    {
                        c = x;
                    }
                    else if (c.X.Position < x.X.Position)
                    {
                        c = x;
                    }
                }

                w.SendMessage(u, (int)SimulationPointMessageType.REPLYMinusX, c!);
            }
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round2Y(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> w in u.ReceiveMessages((int)SimulationPointMessageType.REQPlusY))
            {
                // C <- Nu u Lu+
                List<SimulationPoint<Node>> C = u.Node.NeighborsY.Union(u.Node.LookoutPointPlusNeighborsY)
                    .Where(x => Math.Abs(w.Y.Position - x.Y.Position) > Constants.R).ToList();

                // c <- argmin x€C (pos(x))
                SimulationPoint<Node>? c = null;
                foreach (SimulationPoint<Node> x in C)
                {
                    if (c == null)
                    {
                        c = x;
                    }
                    else if (c.Y.Position > x.Y.Position)
                    {
                        c = x;
                    }
                }

                w.SendMessage(u, (int)SimulationPointMessageType.REPLYPlusY, c!);
            }

            foreach (SimulationPoint<Node> w in u.ReceiveMessages((int)SimulationPointMessageType.REQMinusY))
            {
                // C <- Nu u Lu-
                List<SimulationPoint<Node>> C = u.Node.NeighborsY.Union(u.Node.LookoutPointMinusNeighborsY
                    .Where(x => Math.Abs(w.Y.Position - x.Y.Position) > Constants.R)).ToList();

                // c <- argmax x€C (pos(x))
                SimulationPoint<Node>? c = null;
                foreach (SimulationPoint<Node> x in C)
                {
                    if (c == null)
                    {
                        c = x;
                    }
                    else if (c.Y.Position < x.Y.Position)
                    {
                        c = x;
                    }
                }

                w.SendMessage(u, (int)SimulationPointMessageType.REPLYMinusY, c!);
            }
        }

        private void Correct_Legitimate_and_Distant_Lookout_round3X(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REPLYPlusX))
            {
                u.Node.newLookoutPlusX = v;
            }

            if (u.Node.newLookoutPlusX != u.Node.LookoutPointPlusX)
            {
                u.Node.LookoutPointPlusX?.SendMessage(u, (int)SimulationPointMessageType.REMOVEPlusX, u);

                u.Node.LookoutPointPlusX = u.Node.newLookoutPlusX;
                if (u.Node.newLookoutPlusX == null)
                {
                    u.Node.LookoutPointPlusNeighborsX.Clear();
                }
                else
                {
                    u.Node.newLookoutPlusX.SendMessage(u, (int)SimulationPointMessageType.ADDPlusX, u);
                }
            }

            u.Node.newLookoutPlusX = null;

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REPLYMinusX))
            {
                u.Node.newLookoutMinusX = v;
            }

            if (u.Node.newLookoutMinusX != u.Node.LookoutPointMinusX)
            {
                u.Node.LookoutPointMinusX?.SendMessage(u, (int)SimulationPointMessageType.REMOVEMinusX, u);

                u.Node.LookoutPointMinusX = u.Node.newLookoutMinusX;
                if (u.Node.newLookoutMinusX == null)
                {
                    u.Node.LookoutPointMinusNeighborsX.Clear();
                }
                else
                {
                    u.Node.newLookoutMinusX.SendMessage(u, (int)SimulationPointMessageType.ADDMinusX, u);
                }
            }

            u.Node.newLookoutMinusX = null;
        }

        private void Correct_Legitimate_and_Distant_Lookout_round3Y(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REPLYPlusY))
            {
                u.Node.newLookoutPlusY = v;
            }

            if (u.Node.newLookoutPlusY != u.Node.LookoutPointPlusY)
            {
                u.Node.LookoutPointPlusY?.SendMessage(u, (int)SimulationPointMessageType.REMOVEPlusY, u);

                u.Node.LookoutPointPlusY = u.Node.newLookoutPlusY;
                if (u.Node.newLookoutPlusY == null)
                {
                    u.Node.LookoutPointPlusNeighborsY.Clear();
                }
                else
                {
                    u.Node.newLookoutPlusY.SendMessage(u, (int)SimulationPointMessageType.ADDPlusY, u);
                }
            }

            u.Node.newLookoutPlusY = null;

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REPLYMinusY))
            {
                u.Node.newLookoutMinusY = v;
            }

            if (u.Node.newLookoutMinusY != u.Node.LookoutPointMinusY)
            {
                u.Node.LookoutPointMinusY?.SendMessage(u, (int)SimulationPointMessageType.REMOVEMinusY, u);

                u.Node.LookoutPointMinusY = u.Node.newLookoutMinusY;
                if (u.Node.newLookoutMinusY == null)
                {
                    u.Node.LookoutPointMinusNeighborsY.Clear();
                }
                else
                {
                    u.Node.newLookoutMinusY.SendMessage(u, (int)SimulationPointMessageType.ADDMinusY, u);
                }
            }

            u.Node.newLookoutMinusY = null;
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round4X(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVEPlusX))
            {
                u.Node.SupervisingPlusX.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDPlusX))
            {
                if (!u.Node.SupervisingPlusX.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.SupervisingPlusX.Add(v);
                }
                foreach (SimulationPoint<Node> w in u.Node.NeighborsX)
                {
                    v.SendMessage(u, (int)SimulationPointMessageType.MYNEIGHBORSPlusX, w);
                }
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVEMinusX))
            {
                u.Node.SupervisingMinusX.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDMinusX))
            {
                if (!u.Node.SupervisingMinusX.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.SupervisingMinusX.Add(v);
                }
                foreach (SimulationPoint<Node> w in u.Node.NeighborsX)
                {
                    v.SendMessage(u, (int)SimulationPointMessageType.MYNEIGHBORSMinusX, w);
                }
            }
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round4Y(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVEPlusY))
            {
                u.Node.SupervisingPlusY.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDPlusY))
            {
                if (!u.Node.SupervisingPlusY.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.SupervisingPlusY.Add(v);
                }
                foreach (SimulationPoint<Node> w in u.Node.NeighborsY)
                {
                    v.SendMessage(u, (int)SimulationPointMessageType.MYNEIGHBORSPlusY, w);
                }
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVEMinusY))
            {
                u.Node.SupervisingMinusY.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDMinusY))
            {
                if (!u.Node.SupervisingMinusY.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.SupervisingMinusY.Add(v);
                }
                foreach (SimulationPoint<Node> w in u.Node.NeighborsY)
                {
                    v.SendMessage(u, (int)SimulationPointMessageType.MYNEIGHBORSMinusY, w);
                }
            }
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round5X(SimulationPoint<Node> u)
        {
            SimulationPoint<Node>[]? MYNEIGHBORSPlus = u.ReceiveMessages((int)SimulationPointMessageType.MYNEIGHBORSPlusX);
            if (MYNEIGHBORSPlus.Length != 0)
            {
                u.Node.LookoutPointPlusNeighborsX.Clear();
            }
            foreach (SimulationPoint<Node> w in MYNEIGHBORSPlus)
            {
                if (!u.Node.LookoutPointPlusNeighborsX.Any(x => x.Identifier == w.Identifier))
                {
                    u.Node.LookoutPointPlusNeighborsX.Add(w);
                }
            }

            SimulationPoint<Node>[]? MYNEIGHBORSMinus = u.ReceiveMessages((int)SimulationPointMessageType.MYNEIGHBORSMinusX);
            if (MYNEIGHBORSMinus.Length != 0)
            {
                u.Node.LookoutPointMinusNeighborsX.Clear();
            }
            foreach (SimulationPoint<Node> w in MYNEIGHBORSMinus)
            {
                if (!u.Node.LookoutPointMinusNeighborsX.Any(x => x.Identifier == w.Identifier))
                {
                    u.Node.LookoutPointMinusNeighborsX.Add(w);
                }
            }
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round5Y(SimulationPoint<Node> u)
        {
            SimulationPoint<Node>[]? MYNEIGHBORSPlus = u.ReceiveMessages((int)SimulationPointMessageType.MYNEIGHBORSPlusY);
            if (MYNEIGHBORSPlus.Length != 0)
            {
                u.Node.LookoutPointPlusNeighborsY.Clear();
            }
            foreach (SimulationPoint<Node> w in MYNEIGHBORSPlus)
            {
                if (!u.Node.LookoutPointPlusNeighborsY.Any(x => x.Identifier == w.Identifier))
                {
                    u.Node.LookoutPointPlusNeighborsY.Add(w);
                }
            }

            SimulationPoint<Node>[]? MYNEIGHBORSMinus = u.ReceiveMessages((int)SimulationPointMessageType.MYNEIGHBORSMinusY);
            if (MYNEIGHBORSMinus.Length != 0)
            {
                u.Node.LookoutPointMinusNeighborsY.Clear();
            }
            foreach (SimulationPoint<Node> w in MYNEIGHBORSMinus)
            {
                if (!u.Node.LookoutPointMinusNeighborsY.Any(x => x.Identifier == w.Identifier))
                {
                    u.Node.LookoutPointMinusNeighborsY.Add(w);
                }
            }
        }

        private static void Correct_Close_Neighbor_round5X(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            List<ISimulationCertificate<Node>> T = new();

            foreach (ISimulationCertificate<Node> @event in Fail)
            {
                if (@event is CloseNeighborXCertificate)
                {
                    T.Add(@event);
                }
            }

            foreach (ISimulationCertificate<Node> @event in T)
            {
                //Console.WriteLine($"[{Identifier}] Removing {@event.GetV().Identifier}");
                u.Node.NeighborsX.Remove(@event.GetV());
                foreach (SimulationPoint<Node> w in u.Node.SupervisingPlusX)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.REMOVELookoutPlusX, @event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingMinusX)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.REMOVELookoutMinusX, @event.GetV());
                }
            }
        }

        private static void Correct_Close_Neighbor_round5Y(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            List<ISimulationCertificate<Node>> T = new();

            foreach (ISimulationCertificate<Node> @event in Fail)
            {
                if (@event is CloseNeighborYCertificate)
                {
                    T.Add(@event);
                }
            }

            foreach (ISimulationCertificate<Node> @event in T)
            {
                //Console.WriteLine($"[{Identifier}] Removing {@event.GetV().Identifier}");
                u.Node.NeighborsY.Remove(@event.GetV());
                foreach (SimulationPoint<Node> w in u.Node.SupervisingPlusY)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.REMOVELookoutPlusY, @event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingMinusY)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.REMOVELookoutMinusY, @event.GetV());
                }
            }
        }

        private static void Correct_Close_Neighbor_round6X(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVELookoutPlusX))
            {
                u.Node.LookoutPointPlusNeighborsX.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVELookoutMinusX))
            {
                u.Node.LookoutPointMinusNeighborsX.Remove(v);
            }
        }

        private static void Correct_Close_Neighbor_round6Y(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVELookoutPlusY))
            {
                u.Node.LookoutPointPlusNeighborsY.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVELookoutMinusY))
            {
                u.Node.LookoutPointMinusNeighborsY.Remove(v);
            }
        }

        private static void Correct_Potential_Neighbor_round6X(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            List<ISimulationCertificate<Node>> T = new();

            foreach (ISimulationCertificate<Node> @event in Fail)
            {
                if (@event is PotentialNeighborXCertificate)
                {
                    T.Add(@event);
                }
            }

            foreach (ISimulationCertificate<Node> @event in T)
            {
                //Console.WriteLine($"[{Identifier}] Adding {@event.GetV().Identifier}");
                if (!u.Node.NeighborsX.Any(x => x.Identifier == @event.GetV().Identifier))
                {
                    u.Node.NeighborsX.Add(@event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingPlusX)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.ADDLookoutPlusX, @event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingMinusX)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.ADDLookoutMinusX, @event.GetV());
                }
            }
        }

        private static void Correct_Potential_Neighbor_round6Y(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            List<ISimulationCertificate<Node>> T = new();

            foreach (ISimulationCertificate<Node> @event in Fail)
            {
                if (@event is PotentialNeighborYCertificate)
                {
                    T.Add(@event);
                }
            }

            foreach (ISimulationCertificate<Node> @event in T)
            {
                //Console.WriteLine($"[{Identifier}] Adding {@event.GetV().Identifier}");
                if (!u.Node.NeighborsY.Any(x => x.Identifier == @event.GetV().Identifier))
                {
                    u.Node.NeighborsY.Add(@event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingPlusY)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.ADDLookoutPlusY, @event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingMinusY)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.ADDLookoutMinusY, @event.GetV());
                }
            }
        }

        private static void Correct_Potential_Neighbor_round7X(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDLookoutPlusX))
            {
                if (!u.Node.LookoutPointPlusNeighborsX.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.LookoutPointPlusNeighborsX.Add(v);
                }
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDLookoutMinusX))
            {
                if (!u.Node.LookoutPointMinusNeighborsX.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.LookoutPointMinusNeighborsX.Add(v);
                }
            }
        }

        private static void Correct_Potential_Neighbor_round7Y(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDLookoutPlusY))
            {
                if (!u.Node.LookoutPointPlusNeighborsY.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.LookoutPointPlusNeighborsY.Add(v);
                }
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDLookoutMinusY))
            {
                if (!u.Node.LookoutPointMinusNeighborsY.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.LookoutPointMinusNeighborsY.Add(v);
                }
            }
        }

        public void RunAlgorithmAfterAllPointsMoved(IEnumerable<ISimulationCertificate<Node>> Fail, IEnumerable<SimulationPoint<Node>> Points, double CurrentTime)
        {
            throw new NotImplementedException();
        }

        public void RunAlgorithmAfterSinglePointMoved(IEnumerable<ISimulationCertificate<Node>> Fail, IEnumerable<SimulationPoint<Node>> Points, double CurrentTime)
        {
            throw new NotImplementedException();
        }
        #endregion
    }
}
