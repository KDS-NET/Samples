using KDS;
using KDS.Certificates;
using KDS.Interfaces;
using NearestNeighbors.MultiDim.KDS.Algorithm.Certificates;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.MultiDim.KDS.Algorithm.Data
{
    public class NodeInitializer : INodeInitializer<Node>
    {
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Wrong Usage", "DF0010:Marks undisposed local variables.", Justification = "<Pending>")]
        public void GenerateInitialCertificates(IEnumerable<SimulationPoint<Node>> Points)
        {
            if (RanOnce)
            {
                return;
            }

            RanOnce = true;

            SimulationPoint<Node>? root = Points.MaxBy(x => x.Node.InitialData.MaximumLevel);
            int kmax = root.Node.InitialData.MaximumLevel;

            foreach (SimulationPoint<Node> u in Points)
            {
                for (int k = 1; k <= kmax; k++)
                {
                    IEnumerable<(SimulationPoint<Node> w, int kp1)>? W = u.Node.InitialData.Neighbors.Where(x => x.k == k + 1);
                    IEnumerable<(SimulationPoint<Node> pn, int k)>? PN = W
                        .SelectMany(x => x.w.Node.InitialData.Children
                                        .Where(x => x.Node.InitialData.MaximumLevel == k)
                                        .Union(W.Select(x => x.w))
                                        .Select(x => (x, k)));
                    u.Node.InitialData.PotentialNeighbors = u.Node.InitialData.PotentialNeighbors.Union(PN.Where(c => c.pn.Distance(u) >= 2 * Math.Pow(Constants.b, k))).ToHashSet();
                }

                /*int k = u.Node.InitialData.MaximumLevel;
                IEnumerable<(SimulationPoint<Node> w, int kp1)>? W = u.Node.InitialData.Neighbors.Where(x => x.k == k + 1);
                IEnumerable<(SimulationPoint<Node> pn, int k)>? PN = W.SelectMany(x => x.w.Node.InitialData.Children.Select(x => (x, x.Node.InitialData.MaximumLevel))).Union(W.Select(x => x));
                u.Node.InitialData.PotentialNeighbors = PN.Where(c => c.pn.Distance(u) > 2 * Math.Pow(Constants.b, k)).ToHashSet();*/
            }

            foreach (SimulationPoint<Node> u in Points)
            {
                foreach ((SimulationPoint<Node>, int) v in u.Node.InitialData.Neighbors)
                {
                    if (u == v.Item1)
                    {
                        continue;
                    }

                    ISimulationCertificate<Node> cert = new ShortEdgeCertificate(u, v.Item1, 0, v.Item2);
                    u.AddCertificate(cert);
                    v.Item1.AddCertificate(cert);

                    cert = new SeparationCertificate(u, v.Item1, 0, v.Item2);
                    u.AddCertificate(cert);
                    v.Item1.AddCertificate(cert);
                }

                //IEnumerable<(SimulationPoint<Node> w, int kp1)>? W = u.Node.Neighbors.Where(x => x.k == u.Node.MaximumLevel + 1);
                //IEnumerable<(SimulationPoint<Node> pn, int k)>? PN = W.SelectMany(x => x.w.Node.Children.Select(x => (x, x.Node.MaximumLevel))).Union(W.Select(x => x));
                //IEnumerable<(SimulationPoint<Node> pn, int k)> PotentialNeighbors = PN.Where(c => c.pn.Distance(u) > 2 * Math.Pow(Constants.b, u.Node.MaximumLevel));

                foreach ((SimulationPoint<Node>, int) v in u.Node.InitialData.PotentialNeighbors)
                {
                    if (u == v.Item1 || v.Item2 == 0)
                    {
                        continue;
                    }

                    LongEdgeCertificate? cert = new(u, v.Item1, 0, v.Item2);

                    u.AddCertificate(cert);
                    v.Item1.AddCertificate(cert);
                }

                if (u.Node.InitialData.Parent != null && u.Node.InitialData.Parent != u)
                {
                    SimulationPoint<Node>? fu = u.Node.InitialData.Parent;
                    NodeLinkLabel label = u.RoundDistance(fu, u.Node.InitialData.MaximumLevel);
                    u.AddCertificate(new CoverCertificate(u, fu, 0, u.Node.InitialData.MaximumLevel, label));
                    if (label != NodeLinkLabel.Gamma)
                    {
                        u.AddCertificate(new SCoverCertificate(u, fu, 0, u.Node.InitialData.MaximumLevel, label - 1));
                    }
                }
            }
        }

        #region Initialize structure values

        private bool RanOnce = false;

        public void ComputeNodeValues(IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            foreach (SimulationPoint<Node> u in PointStructureList)
            {
                u.Node.InitialData = new();
                u.Node.InitialData.MaximumLevel = 0;
                u.Node.InitialData.Parent = u;
                u.Node.InitialData.Children.Clear();
                u.Node.InitialData.Neighbors.Clear();
            }

            int k = 0;
            List<SimulationPoint<Node>> S = new(PointStructureList);

            foreach (SimulationPoint<Node>? element in S)
            {
                element.Node.InitialData.MaximumLevel = 0;
            }

            while (S.Count > 1)
            {
                k++;
                List<SimulationPoint<Node>> removed = new();
                foreach (SimulationPoint<Node> u in S)
                {
                    if (removed.Contains(u))
                    {
                        continue;
                    }

                    double d = Constants.Gamma * Math.Pow(Constants.b, k);
                    foreach (SimulationPoint<Node> v in (IEnumerable<SimulationPoint<Node>>)S.Where(t => t != u && t.StaticDistance(u) < d))
                    {
                        if (removed.Contains(v))
                        {
                            continue;
                        }

                        removed.Add(v);
                        v.Node.InitialData.Parent = u;
                        u.Node.InitialData.Children.Add(v);
                    }

                    u.Node.InitialData.MaximumLevel = k;
                }

                foreach (SimulationPoint<Node>? wasRemoved in removed)
                {
                    S.Remove(wasRemoved);
                }
            }

            SimulationPoint<Node>? root = PointStructureList.MaxBy(x => x.Node.InitialData.MaximumLevel);
            int kmax = root.Node.InitialData.MaximumLevel;
            if (kmax > 0)
            {
                for (int i = kmax; i > 0; i--)
                {
                    root.Node.InitialData.Neighbors.Add((root, i));
                }
            }

            for (k = kmax; k >= 2; k--)
            {
                foreach (SimulationPoint<Node>? u in PointStructureList.Where(x => x.Node.InitialData.MaximumLevel >= k))
                {
                    List<SimulationPoint<Node>> PN = new();
                    foreach ((SimulationPoint<Node>, int) v in u.Node.InitialData.Neighbors)
                    {
                        foreach (SimulationPoint<Node>? w in v.Item1.Node.InitialData.Children.Where(x => x.Node.InitialData.MaximumLevel == k - 1).Union(new[] { v.Item1 }))
                        {
                            PN.Add(w);
                        }
                    }

                    foreach (SimulationPoint<Node>? wp in u.Node.InitialData.Children.Where(x => x.Node.InitialData.MaximumLevel == k - 1).Union(new[] { u }))
                    {
                        foreach (SimulationPoint<Node>? w in PN)
                        {
                            if (wp.StaticDistance(w) < 2 * Math.Pow(Constants.b, k - 1))
                            {
                                if (!wp.Node.InitialData.Neighbors.Any(x => x.v == w && x.k == k - 1))
                                {
                                    wp.Node.InitialData.Neighbors.Add((w, k - 1));

                                    if (wp == w)
                                    {
                                        for (int i = k - 2; i > 0; i--)
                                        {
                                            if (!wp.Node.InitialData.Neighbors.Any(x => x.v == w && x.k == i))
                                            {
                                                wp.Node.InitialData.Neighbors.Add((w, i));
                                            }
                                        }
                                    }
                                }
                            }
                            /*else
                            {
                                if (!wp.Node.InitialData.PotentialNeighbors.Any(x => x.v == w && x.k == k - 1))
                                {
                                    wp.Node.InitialData.PotentialNeighbors.Add((w, k - 1));

                                    if (wp == w)
                                    {
                                        for (int i = k - 2; i > 0; i--)
                                        {
                                            if (!wp.Node.InitialData.PotentialNeighbors.Any(x => x.v == w && x.k == i))
                                            {
                                                wp.Node.InitialData.PotentialNeighbors.Add((w, i));
                                            }
                                        }
                                    }
                                }
                            }*/
                        }
                    }
                }
            }

            GenerateInitialCertificates(PointStructureList);

            foreach (var pt in PointStructureList)
            {
                pt.Node.InitialData.CopyTo(pt.Node);
                pt.Node.Cost.Certificate = 0;
                pt.Node.Cost.Queries = 0;
                pt.Node.Cost.Structure = 0;
            }
        }
        #endregion
    }
}
