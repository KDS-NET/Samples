using KDS;
using KDS.Certificates;
using KDS.Interfaces;
using NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm.Certificates
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
            RebuildCertificatesY(u, Node/*, OldNode*/, CurrentTime);
            u.RemovedCertificatesList.Clear();
            Node.CopyTo(Node.PreviousData);
            if (flag)
            {
                Node.Cost.Queries = 0;
                Node.Cost.Certificate = 0;
                Node.Cost.Structure = 0;
            }
        }

        public void RebuildCertificates(IEnumerable<SimulationPoint<Node>> Points, double CurrentTime)
        {
            throw new NotImplementedException();
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        private static void RebuildCertificatesX(SimulationPoint<Node> u, Node Node, double CurrentTime)
        {
            Node OldNode = Node.PreviousData;
            List<SimulationPoint<Node>>? concernedPoints = Node.LookoutPointPlusNeighborsX.Union(Node.LookoutPointMinusNeighborsX).Except(Node.NeighborsX).ToList();
            List<SimulationPoint<Node>>? oldConcernedPoints = OldNode.LookoutPointPlusNeighborsX.Union(OldNode.LookoutPointMinusNeighborsX).Except(OldNode.NeighborsX).ToList();

            List<SimulationPoint<Node>>? nConcernedPoints = Node.NeighborsX.Union(Node.LookoutPointPlusNeighborsX).ToList();
            List<SimulationPoint<Node>>? nOldConcernedPoints = OldNode.NeighborsX.Union(OldNode.LookoutPointPlusNeighborsX).ToList();

            List<SimulationPoint<Node>>? n2ConcernedPoints = Node.NeighborsX.Union(Node.LookoutPointMinusNeighborsX).ToList();
            List<SimulationPoint<Node>>? n2OldConcernedPoints = OldNode.NeighborsX.Union(OldNode.LookoutPointMinusNeighborsX).ToList();

            (HashSet<SimulationPoint<Node>> addedNeighbors, HashSet<SimulationPoint<Node>> removedNeighbors, HashSet<SimulationPoint<Node>> unchangedNeighbors) = Utils.CompareLists(OldNode.NeighborsX, Node.NeighborsX);

            (HashSet<SimulationPoint<Node>> addedConcernedPoints, HashSet<SimulationPoint<Node>> removedConcernedPoints, HashSet<SimulationPoint<Node>> unchangedConcernedPoints) = Utils.CompareLists(oldConcernedPoints, concernedPoints);

            (HashSet<SimulationPoint<Node>> nAddedConcernedPoints, HashSet<SimulationPoint<Node>> nRemovedConcernedPoints, HashSet<SimulationPoint<Node>> nUnchangedConcernedPoints) = Utils.CompareLists(nOldConcernedPoints, nConcernedPoints);

            (HashSet<SimulationPoint<Node>> n2AddedConcernedPoints, HashSet<SimulationPoint<Node>> n2RemovedConcernedPoints, HashSet<SimulationPoint<Node>> n2UnchangedConcernedPoints) = Utils.CompareLists(n2OldConcernedPoints, n2ConcernedPoints);

            foreach (SimulationPoint<Node>? v in removedNeighbors)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is CloseNeighborXCertificate) && x.GetV() == v);
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

                u.Certificates.Add(new CloseNeighborXCertificate(u, v, CurrentTime));
            }

            foreach (SimulationPoint<Node>? v in removedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is PotentialNeighborXCertificate) && x.GetV() == v);
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

                u.Certificates.Add(new PotentialNeighborXCertificate(u, v, CurrentTime));
            }

            List<ISimulationCertificate<Node>>? distantLookoutEvents = u.Certificates.Where(x => x is DistantLookoutXCertificate).ToList();

            if (OldNode.LookoutPointPlusX != Node.LookoutPointPlusX)
            {
                u.Node.Cost.Structure++;

                if (Node.LookoutPointPlusX == null)
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointPlusX);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                }
                else if (OldNode.LookoutPointPlusX == null)
                {
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutXCertificate(u, Node.LookoutPointPlusX, CurrentTime));
                }
                else
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointPlusX);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutXCertificate(u, Node.LookoutPointPlusX, CurrentTime));
                }
            }

            if (OldNode.LookoutPointMinusX != Node.LookoutPointMinusX)
            {
                u.Node.Cost.Structure++;

                if (Node.LookoutPointMinusX == null)
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointMinusX);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                }
                else if (OldNode.LookoutPointMinusX == null)
                {
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutXCertificate(u, Node.LookoutPointMinusX, CurrentTime));
                }
                else
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointMinusX);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutXCertificate(u, Node.LookoutPointMinusX, CurrentTime));
                }
            }

            foreach (SimulationPoint<Node>? v in nRemovedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is LegitimateLookoutPlusXCertificate) && x.GetV() == v);
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

                u.Certificates.Add(new LegitimateLookoutPlusXCertificate(u, v, CurrentTime));
            }

            foreach (SimulationPoint<Node>? v in n2RemovedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is LegitimateLookoutMinusXCertificate) && x.GetV() == v);
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

                u.Certificates.Add(new LegitimateLookoutMinusXCertificate(u, v, CurrentTime));
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        private static void RebuildCertificatesY(SimulationPoint<Node> u, Node Node, double CurrentTime)
        {
            Node OldNode = Node.PreviousData;
            List<SimulationPoint<Node>>? concernedPoints = Node.LookoutPointPlusNeighborsY.Union(Node.LookoutPointMinusNeighborsY).Except(Node.NeighborsY).ToList();
            List<SimulationPoint<Node>>? oldConcernedPoints = OldNode.LookoutPointPlusNeighborsY.Union(OldNode.LookoutPointMinusNeighborsY).Except(OldNode.NeighborsY).ToList();

            List<SimulationPoint<Node>>? nConcernedPoints = Node.NeighborsY.Union(Node.LookoutPointPlusNeighborsY).ToList();
            List<SimulationPoint<Node>>? nOldConcernedPoints = OldNode.NeighborsY.Union(OldNode.LookoutPointPlusNeighborsY).ToList();

            List<SimulationPoint<Node>>? n2ConcernedPoints = Node.NeighborsY.Union(Node.LookoutPointMinusNeighborsY).ToList();
            List<SimulationPoint<Node>>? n2OldConcernedPoints = OldNode.NeighborsY.Union(OldNode.LookoutPointMinusNeighborsY).ToList();

            (HashSet<SimulationPoint<Node>> addedNeighbors, HashSet<SimulationPoint<Node>> removedNeighbors, HashSet<SimulationPoint<Node>> unchangedNeighbors) = Utils.CompareLists(OldNode.NeighborsY, Node.NeighborsY);

            (HashSet<SimulationPoint<Node>> addedConcernedPoints, HashSet<SimulationPoint<Node>> removedConcernedPoints, HashSet<SimulationPoint<Node>> unchangedConcernedPoints) = Utils.CompareLists(oldConcernedPoints, concernedPoints);

            (HashSet<SimulationPoint<Node>> nAddedConcernedPoints, HashSet<SimulationPoint<Node>> nRemovedConcernedPoints, HashSet<SimulationPoint<Node>> nUnchangedConcernedPoints) = Utils.CompareLists(nOldConcernedPoints, nConcernedPoints);

            (HashSet<SimulationPoint<Node>> n2AddedConcernedPoints, HashSet<SimulationPoint<Node>> n2RemovedConcernedPoints, HashSet<SimulationPoint<Node>> n2UnchangedConcernedPoints) = Utils.CompareLists(n2OldConcernedPoints, n2ConcernedPoints);

            foreach (SimulationPoint<Node>? v in removedNeighbors)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is CloseNeighborYCertificate) && x.GetV() == v);
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

                u.Certificates.Add(new CloseNeighborYCertificate(u, v, CurrentTime));
            }

            foreach (SimulationPoint<Node>? v in removedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is PotentialNeighborYCertificate) && x.GetV() == v);
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

                u.Certificates.Add(new PotentialNeighborYCertificate(u, v, CurrentTime));
            }

            List<ISimulationCertificate<Node>>? distantLookoutEvents = u.Certificates.Where(x => x is DistantLookoutYCertificate).ToList();

            if (OldNode.LookoutPointPlusY != Node.LookoutPointPlusY)
            {
                u.Node.Cost.Structure++;

                if (Node.LookoutPointPlusY == null)
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointPlusY);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                }
                else if (OldNode.LookoutPointPlusY == null)
                {
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutYCertificate(u, Node.LookoutPointPlusY, CurrentTime));
                }
                else
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointPlusY);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutYCertificate(u, Node.LookoutPointPlusY, CurrentTime));
                }
            }

            if (OldNode.LookoutPointMinusY != Node.LookoutPointMinusY)
            {
                u.Node.Cost.Structure++;

                if (Node.LookoutPointMinusY == null)
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointMinusY);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                }
                else if (OldNode.LookoutPointMinusY == null)
                {
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutYCertificate(u, Node.LookoutPointMinusY, CurrentTime));
                }
                else
                {
                    IEnumerable<ISimulationCertificate<Node>>? toRemove = distantLookoutEvents.Where(x => x.GetV() == OldNode.LookoutPointMinusY);
                    foreach (ISimulationCertificate<Node>? itm in toRemove.ToList())
                    {
                        u.Node.Cost.Certificate++;

                        bool result = u.RemoveCertificate(itm);
                        //if (!result)
                        //    throw new Exception("wut");
                    }
                    u.Node.Cost.Certificate++;

                    u.Certificates.Add(new DistantLookoutYCertificate(u, Node.LookoutPointMinusY, CurrentTime));
                }
            }

            foreach (SimulationPoint<Node>? v in nRemovedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is LegitimateLookoutPlusYCertificate) && x.GetV() == v);
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

                u.Certificates.Add(new LegitimateLookoutPlusYCertificate(u, v, CurrentTime));
            }

            foreach (SimulationPoint<Node>? v in n2RemovedConcernedPoints)
            {
                u.Node.Cost.Structure++;

                IEnumerable<ISimulationCertificate<Node>>? toRemove = u.Certificates.Where(x => (x is LegitimateLookoutMinusYCertificate) && x.GetV() == v);
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

                u.Certificates.Add(new LegitimateLookoutMinusYCertificate(u, v, CurrentTime));
            }
        }
    }
}
