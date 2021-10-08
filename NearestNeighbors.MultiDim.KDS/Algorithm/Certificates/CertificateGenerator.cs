using KDS;
using KDS.Certificates;
using KDS.Interfaces;
using NearestNeighbors.MultiDim.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.MultiDim.KDS.Algorithm.Certificates
{
    public class CertificateGenerator : ICertificateGenerator<Node>
    {
        public void RebuildCertificates(SimulationPoint<Node> u, Node Node, double CurrentTime)
        {
            throw new NotImplementedException();
        }

        private bool RanOnce;

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Wrong Usage", "DF0010:Marks undisposed local variables.", Justification = "<Pending>")]
        public void RebuildCertificates(IEnumerable<SimulationPoint<Node>> Points, double CurrentTime)
        {
            if (RanOnce)
            {
                return;
            }

            RanOnce = true;

            SimulationPoint<Node>? root = Points.MaxBy(x => x.Node.MaximumLevel);
            int kmax = root.Node.MaximumLevel;

            foreach (SimulationPoint<Node> u in Points)
            {
                for (int k = 0; k <= kmax; k++)
                {
                    IEnumerable<(SimulationPoint<Node> w, int kp1)>? W = u.Node.Neighbors.Where(x => x.k == k + 1);
                    IEnumerable<(SimulationPoint<Node> pn, int k)>? PN = W.SelectMany(x => x.w.Node.Children.Select(x => (x, x.Node.MaximumLevel))).Union(W.Select(x => x));
                    u.Node.PotentialNeighbors = PN.Where(c => c.pn.Distance(u) > 2 * Math.Pow(Constants.b, k)).ToHashSet();
                }
            }

            foreach (SimulationPoint<Node> u in Points)
            {
                foreach ((SimulationPoint<Node>, int) v in u.Node.Neighbors)
                {
                    ISimulationCertificate<Node> cert = new ShortEdgeCertificate(u, v.Item1, 0, v.Item2);
                    u.AddCertificate(cert);
                    v.Item1.AddCertificate(cert);

                    if (u == v.Item1)
                    {
                        continue;
                    }

                    cert = new SeparationCertificate(u, v.Item1, 0, v.Item2);
                    u.AddCertificate(cert);
                    v.Item1.AddCertificate(cert);
                }

                //IEnumerable<(SimulationPoint<Node> w, int kp1)>? W = u.Node.Neighbors.Where(x => x.k == u.Node.MaximumLevel + 1);
                //IEnumerable<(SimulationPoint<Node> pn, int k)>? PN = W.SelectMany(x => x.w.Node.Children.Select(x => (x, x.Node.MaximumLevel))).Union(W.Select(x => x));
                //IEnumerable<(SimulationPoint<Node> pn, int k)> PotentialNeighbors = PN.Where(c => c.pn.Distance(u) > 2 * Math.Pow(Constants.b, u.Node.MaximumLevel));

                foreach ((SimulationPoint<Node>, int) v in u.Node.PotentialNeighbors)
                {
                    if (u == v.Item1 || v.Item2 == 0)
                    {
                        continue;
                    }

                    LongEdgeCertificate? cert = new(u, v.Item1, 0, v.Item2);

                    u.AddCertificate(cert);
                    v.Item1.AddCertificate(cert);
                }

                if (u.Node.Parent != null && u.Node.Parent != u)
                {
                    SimulationPoint<Node>? fu = u.Node.Parent;
                    NodeLinkLabel label = u.RoundDistance(fu, u.Node.MaximumLevel);
                    u.AddCertificate(new CoverCertificate(u, fu, 0, u.Node.MaximumLevel, label));
                    if (label != NodeLinkLabel.Gamma)
                    {
                        u.AddCertificate(new SCoverCertificate(u, fu, 0, u.Node.MaximumLevel, label - 1));
                    }
                }
            }
        }
    }
}
