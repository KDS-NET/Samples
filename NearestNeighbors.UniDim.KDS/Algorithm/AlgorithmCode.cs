using KDS;
using KDS.Certificates;
using KDS.Interfaces;
using NearestNeighbors.UniDim.KDS.Algorithm.Certificates;
using NearestNeighbors.UniDim.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.UniDim.KDS.Algorithm
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
                    Correct_Legitimate_and_Distant_Lookout_round1(Fail, u);
                    break;
                case 1:
                    Correct_Legitimate_and_Distant_Lookout_round2(u);
                    break;
                case 2:
                    Correct_Legitimate_and_Distant_Lookout_round3(u);
                    break;
                case 3:
                    Correct_Legitimate_and_Distant_Lookout_round4(u);
                    break;
                case 4:
                    Correct_Legitimate_and_Distant_Lookout_round5(u);
                    Correct_Close_Neighbor_round5(Fail, u);
                    break;
                case 5:
                    Correct_Close_Neighbor_round6(u);
                    Correct_Potential_Neighbor_round6(Fail, u);
                    break;
                case 6:
                    Correct_Potential_Neighbor_round7(u);
                    u.Node.Cost.Certificate += Fail.Count();
                    break;
            }
        }

        private void Correct_Legitimate_and_Distant_Lookout_round1(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            u.Node.newLookoutPlus = null;
            u.Node.newLookoutMinus = null;

            // C <- v: (LegitimateLookout+,u,v) € Fail
            List<SimulationPoint<Node>> C = Fail.Where(x => x is LegitimateLookoutPlusCertificate)
                .Select(x => x.GetV()).ToList();

            if (C.Count > 0)
            {
                // argmin pos(x) x€C
                foreach (SimulationPoint<Node> x in C)
                {
                    if (u.Node.newLookoutPlus == null)
                    {
                        u.Node.newLookoutPlus = x;
                    }
                    else
                    {
                        if (u.Node.newLookoutPlus.X.Position > x.X.Position)
                        {
                            u.Node.newLookoutPlus = x;
                        }
                    }
                }
            }
            else
            {
                bool distantLookoutLuPlusFailed = false;
                foreach (ISimulationCertificate<Node> @event in Fail)
                {
                    if (@event is DistantLookoutCertificate && u.Node.LookoutPointPlus != null && @event.GetV() == u.Node.LookoutPointPlus)
                    {
                        distantLookoutLuPlusFailed = true;
                        break;
                    }
                }

                if (distantLookoutLuPlusFailed)
                {
                    u.Node.LookoutPointPlus!.SendMessage(u, (int)SimulationPointMessageType.REQPlus, u);
                }
                else
                {
                    u.Node.newLookoutPlus = u.Node.LookoutPointPlus;
                }
            }

            // C <- v: (LegitimateLookout-,u,v) € Fail
            C = Fail.Where(x => x is LegitimateLookoutMinusCertificate)
                .Select(x => x.GetV()).ToList();

            if (C.Count > 0)
            {
                // argmax pos(x) x€C
                foreach (SimulationPoint<Node> x in C)
                {
                    if (u.Node.newLookoutMinus == null)
                    {
                        u.Node.newLookoutMinus = x;
                    }
                    else
                    {
                        if (u.Node.newLookoutMinus.X.Position < x.X.Position)
                        {
                            u.Node.newLookoutMinus = x;
                        }
                    }
                }
            }
            else
            {
                bool distantLookoutLuMinusFailed = false;
                foreach (ISimulationCertificate<Node> @event in Fail)
                {
                    if (@event is DistantLookoutCertificate &&
                        u.Node.LookoutPointMinus != null &&
                        @event.GetV() == u.Node.LookoutPointMinus)
                    {
                        distantLookoutLuMinusFailed = true;
                        break;
                    }
                }

                if (distantLookoutLuMinusFailed)
                {
                    u.Node.LookoutPointMinus!.SendMessage(u, (int)SimulationPointMessageType.REQMinus, u);
                }
                else
                {
                    u.Node.newLookoutMinus = u.Node.LookoutPointMinus;
                }
            }
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round2(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> w in u.ReceiveMessages((int)SimulationPointMessageType.REQPlus))
            {
                // C <- Nu u Lu+
                List<SimulationPoint<Node>> C = u.Node.Neighbors.Union(u.Node.LookoutPointPlusNeighbors)
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

                w.SendMessage(u, (int)SimulationPointMessageType.REPLYPlus, c!);
            }

            foreach (SimulationPoint<Node> w in u.ReceiveMessages((int)SimulationPointMessageType.REQMinus))
            {
                // C <- Nu u Lu-
                List<SimulationPoint<Node>> C = u.Node.Neighbors.Union(u.Node.LookoutPointMinusNeighbors
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

                w.SendMessage(u, (int)SimulationPointMessageType.REPLYMinus, c!);
            }
        }

        private void Correct_Legitimate_and_Distant_Lookout_round3(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REPLYPlus))
            {
                u.Node.newLookoutPlus = v;
            }

            if (u.Node.newLookoutPlus != u.Node.LookoutPointPlus)
            {
                u.Node.LookoutPointPlus?.SendMessage(u, (int)SimulationPointMessageType.REMOVEPlus, u);

                u.Node.LookoutPointPlus = u.Node.newLookoutPlus;
                if (u.Node.newLookoutPlus == null)
                {
                    u.Node.LookoutPointPlusNeighbors.Clear();
                }
                else
                {
                    u.Node.newLookoutPlus.SendMessage(u, (int)SimulationPointMessageType.ADDPlus, u);
                }
            }

            u.Node.newLookoutPlus = null;

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REPLYMinus))
            {
                u.Node.newLookoutMinus = v;
            }

            if (u.Node.newLookoutMinus != u.Node.LookoutPointMinus)
            {
                u.Node.LookoutPointMinus?.SendMessage(u, (int)SimulationPointMessageType.REMOVEMinus, u);

                u.Node.LookoutPointMinus = u.Node.newLookoutMinus;
                if (u.Node.newLookoutMinus == null)
                {
                    u.Node.LookoutPointMinusNeighbors.Clear();
                }
                else
                {
                    u.Node.newLookoutMinus.SendMessage(u, (int)SimulationPointMessageType.ADDMinus, u);
                }
            }

            u.Node.newLookoutMinus = null;
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round4(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVEPlus))
            {
                u.Node.SupervisingPlus.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDPlus))
            {
                if (!u.Node.SupervisingPlus.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.SupervisingPlus.Add(v);
                }
                foreach (SimulationPoint<Node> w in u.Node.Neighbors)
                {
                    v.SendMessage(u, (int)SimulationPointMessageType.MYNEIGHBORSPlus, w);
                }
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVEMinus))
            {
                u.Node.SupervisingMinus.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDMinus))
            {
                if (!u.Node.SupervisingMinus.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.SupervisingMinus.Add(v);
                }
                foreach (SimulationPoint<Node> w in u.Node.Neighbors)
                {
                    v.SendMessage(u, (int)SimulationPointMessageType.MYNEIGHBORSMinus, w);
                }
            }
        }

        private static void Correct_Legitimate_and_Distant_Lookout_round5(SimulationPoint<Node> u)
        {
            SimulationPoint<Node>[]? MYNEIGHBORSPlus = u.ReceiveMessages((int)SimulationPointMessageType.MYNEIGHBORSPlus);
            if (MYNEIGHBORSPlus.Length != 0)
            {
                u.Node.LookoutPointPlusNeighbors.Clear();
            }
            foreach (SimulationPoint<Node> w in MYNEIGHBORSPlus)
            {
                if (!u.Node.LookoutPointPlusNeighbors.Any(x => x.Identifier == w.Identifier))
                {
                    u.Node.LookoutPointPlusNeighbors.Add(w);
                }
            }

            SimulationPoint<Node>[]? MYNEIGHBORSMinus = u.ReceiveMessages((int)SimulationPointMessageType.MYNEIGHBORSMinus);
            if (MYNEIGHBORSMinus.Length != 0)
            {
                u.Node.LookoutPointMinusNeighbors.Clear();
            }
            foreach (SimulationPoint<Node> w in MYNEIGHBORSMinus)
            {
                if (!u.Node.LookoutPointMinusNeighbors.Any(x => x.Identifier == w.Identifier))
                {
                    u.Node.LookoutPointMinusNeighbors.Add(w);
                }
            }
        }

        private static void Correct_Close_Neighbor_round5(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            List<ISimulationCertificate<Node>> T = new();

            foreach (ISimulationCertificate<Node> @event in Fail)
            {
                if (@event is CloseNeighborCertificate)
                {
                    T.Add(@event);
                }
            }

            foreach (ISimulationCertificate<Node> @event in T)
            {
                //Console.WriteLine($"[{Identifier}] Removing {@event.GetV().Identifier}");
                u.Node.Neighbors.Remove(@event.GetV());
                foreach (SimulationPoint<Node> w in u.Node.SupervisingPlus)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.REMOVELookoutPlus, @event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingMinus)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.REMOVELookoutMinus, @event.GetV());
                }
            }
        }

        private static void Correct_Close_Neighbor_round6(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVELookoutPlus))
            {
                u.Node.LookoutPointPlusNeighbors.Remove(v);
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.REMOVELookoutMinus))
            {
                u.Node.LookoutPointMinusNeighbors.Remove(v);
            }
        }

        private static void Correct_Potential_Neighbor_round6(IEnumerable<ISimulationCertificate<Node>> Fail, SimulationPoint<Node> u)
        {
            List<ISimulationCertificate<Node>> T = new();

            foreach (ISimulationCertificate<Node> @event in Fail)
            {
                if (@event is PotentialNeighborCertificate)
                {
                    T.Add(@event);
                }
            }

            foreach (ISimulationCertificate<Node> @event in T)
            {
                //Console.WriteLine($"[{Identifier}] Adding {@event.GetV().Identifier}");
                if (!u.Node.Neighbors.Any(x => x.Identifier == @event.GetV().Identifier))
                {
                    u.Node.Neighbors.Add(@event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingPlus)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.ADDLookoutPlus, @event.GetV());
                }
                foreach (SimulationPoint<Node> w in u.Node.SupervisingMinus)
                {
                    w.SendMessage(u, (int)SimulationPointMessageType.ADDLookoutMinus, @event.GetV());
                }
            }
        }

        private static void Correct_Potential_Neighbor_round7(SimulationPoint<Node> u)
        {
            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDLookoutPlus))
            {
                if (!u.Node.LookoutPointPlusNeighbors.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.LookoutPointPlusNeighbors.Add(v);
                }
            }

            foreach (SimulationPoint<Node> v in u.ReceiveMessages((int)SimulationPointMessageType.ADDLookoutMinus))
            {
                if (!u.Node.LookoutPointMinusNeighbors.Any(x => x.Identifier == v.Identifier))
                {
                    u.Node.LookoutPointMinusNeighbors.Add(v);
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
