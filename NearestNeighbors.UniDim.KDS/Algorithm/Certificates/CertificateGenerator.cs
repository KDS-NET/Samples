using KDS;
using KDS.Certificates;
using KDS.Interfaces;
using NearestNeighbors.UniDim.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.UniDim.KDS.Algorithm.Certificates
{
    public class CertificateGenerator : ICertificateGenerator<Node>
    {
        public void RebuildCertificates(SimulationPoint<Node> u, Node Node/*, Node OldNode*/, double CurrentTime)
        {
            bool flag = Node.PreviousData == null;
            if (flag)
            {
                Node.PreviousData = new();
            }
            RebuildCertificatesX(u, Node/*, OldNode*/, CurrentTime);
            u.RemovedCertificatesList.Clear();
        }

        public void RebuildCertificates(IEnumerable<SimulationPoint<Node>> Points, double CurrentTime)
        {
            throw new NotImplementedException();
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        private static void RebuildCertificatesX(SimulationPoint<Node> u, Node Node, double CurrentTime)
        {
            Node OldNode = Node.PreviousData ?? new();
            List<SimulationPoint<Node>>? concernedPoints = Node.LookoutPointPlusNeighbors.Union(Node.LookoutPointMinusNeighbors).Except(Node.Neighbors).ToList();
            List<SimulationPoint<Node>>? oldConcernedPoints = OldNode.LookoutPointPlusNeighbors.Union(OldNode.LookoutPointMinusNeighbors).Except(OldNode.Neighbors).ToList();

            List<SimulationPoint<Node>>? nConcernedPoints = Node.Neighbors.Union(Node.LookoutPointPlusNeighbors).ToList();
            List<SimulationPoint<Node>>? nOldConcernedPoints = OldNode.Neighbors.Union(OldNode.LookoutPointPlusNeighbors).ToList();

            List<SimulationPoint<Node>>? n2ConcernedPoints = Node.Neighbors.Union(Node.LookoutPointMinusNeighbors).ToList();
            List<SimulationPoint<Node>>? n2OldConcernedPoints = OldNode.Neighbors.Union(OldNode.LookoutPointMinusNeighbors).ToList();

            (HashSet<SimulationPoint<Node>> addedNeighbors, HashSet<SimulationPoint<Node>> removedNeighbors, HashSet<SimulationPoint<Node>> unchangedNeighbors) = Utils.CompareLists(OldNode.Neighbors, Node.Neighbors);

            (HashSet<SimulationPoint<Node>> addedConcernedPoints, HashSet<SimulationPoint<Node>> removedConcernedPoints, HashSet<SimulationPoint<Node>> unchangedConcernedPoints) = Utils.CompareLists(oldConcernedPoints, concernedPoints);

            (HashSet<SimulationPoint<Node>> nAddedConcernedPoints, HashSet<SimulationPoint<Node>> nRemovedConcernedPoints, HashSet<SimulationPoint<Node>> nUnchangedConcernedPoints) = Utils.CompareLists(nOldConcernedPoints, nConcernedPoints);

            (HashSet<SimulationPoint<Node>> n2AddedConcernedPoints, HashSet<SimulationPoint<Node>> n2RemovedConcernedPoints, HashSet<SimulationPoint<Node>> n2UnchangedConcernedPoints) = Utils.CompareLists(n2OldConcernedPoints, n2ConcernedPoints);

            foreach (SimulationPoint<Node>? v in removedNeighbors)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is CloseNeighborCertificate) && x.GetV() == v);
                foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                {
                    u.Node.Cost.Certificate++;

                    bool result = u.RemoveCertificate(itm);
                    //if (!result)
                    //    throw new Exception("wut");
                }
            }

            foreach (SimulationPoint<Node>? v in addedNeighbors)
            {
                u.Node.Cost.Structure++;

                u.Node.Cost.Certificate++;

                u.Certificates.Add(new CloseNeighborCertificate(u, v, CurrentTime));
            }

            foreach (SimulationPoint<Node>? v in removedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is PotentialNeighborCertificate) && x.GetV() == v);
                foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                {
                    u.Node.Cost.Certificate++;

                    //Console.WriteLine($"[{Identifier}] Removing PotentialNeighborEvent {itm.GetV().Identifier}");
                    bool result = u.RemoveCertificate(itm);
                    //if (!result)
                    //    throw new Exception("wut");
                }
            }

            foreach (SimulationPoint<Node>? v in addedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                u.Node.Cost.Certificate++;

                u.Certificates.Add(new PotentialNeighborCertificate(u, v, CurrentTime));
            }

            List<ISimulationCertificate<Node>>? distantLookoutEvents = u.Certificates.Where(x => x is DistantLookoutCertificate).ToList();

            if (OldNode.LookoutPointPlus != Node.LookoutPointPlus)
            {
                u.Node.Cost.Structure++;

                if (Node.LookoutPointPlus == null)
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointPlus);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                }
                else if (OldNode.LookoutPointPlus == null)
                {
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutCertificate(u, Node.LookoutPointPlus, CurrentTime));
                }
                else
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointPlus);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutCertificate(u, Node.LookoutPointPlus, CurrentTime));
                }
            }

            if (OldNode.LookoutPointMinus != Node.LookoutPointMinus)
            {
                u.Node.Cost.Structure++;

                if (Node.LookoutPointMinus == null)
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointMinus);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                }
                else if (OldNode.LookoutPointMinus == null)
                {
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutCertificate(u, Node.LookoutPointMinus, CurrentTime));
                }
                else
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointMinus);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutCertificate(u, Node.LookoutPointMinus, CurrentTime));
                }
            }

            foreach (SimulationPoint<Node>? v in nRemovedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is LegitimateLookoutPlusCertificate) && x.GetV() == v);
                foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                {
                    u.Node.Cost.Certificate++;

                    bool result = u.RemoveCertificate(itm);
                    //if (!result)
                    //    throw new Exception("wut");
                }
            }

            foreach (SimulationPoint<Node>? v in nAddedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                u.Node.Cost.Certificate++;

                u.Certificates.Add(new LegitimateLookoutPlusCertificate(u, v, CurrentTime));
            }

            foreach (SimulationPoint<Node>? v in n2RemovedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is LegitimateLookoutMinusCertificate) && x.GetV() == v);
                foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                {
                    u.Node.Cost.Certificate++;

                    bool result = u.RemoveCertificate(itm);
                    //if (!result)
                    //    throw new Exception("wut");
                }
            }

            foreach (SimulationPoint<Node>? v in n2AddedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                u.Node.Cost.Certificate++;

                u.Certificates.Add(new LegitimateLookoutMinusCertificate(u, v, CurrentTime));
            }
        }
    }
}
