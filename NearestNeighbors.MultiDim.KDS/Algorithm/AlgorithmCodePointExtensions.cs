using KDS;
using NearestNeighbors.MultiDim.KDS.Algorithm.Certificates;
using NearestNeighbors.MultiDim.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;

#nullable enable

namespace NearestNeighbors.MultiDim.KDS.Algorithm
{
    internal static class AlgorithmCodePointExtensions
    {
        [SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        internal static void AddNeighbor(this SimulationPoint<Node> u, IEnumerable<SimulationPoint<Node>> Points, double CurrentTime, (SimulationPoint<Node> v, int k) v)
        {
            if (u.GetNeighbors().Any(x => x.v == v.v && x.k == v.k))
            {
                return;
            }

            //Console.WriteLine($"[NEIGHBORS][BEGIN] Adding ({v.v.Identifier}) to ({u.Identifier}) neighbors at level {v.k} - {v.v.Node} {u.Node}");

            if (v.k > v.v.Node.MaximumLevel)
            {
                throw new Exception("Neighbor at a level higher than its maximum level");
            }

            u.GetNeighbors().Add(v);
            u.Node.Cost.Structure++;

            if (v.v != u)
            {
                u.AddCertificate(new SeparationCertificate(u, v.v, CurrentTime, v.k));
                u.AddCertificate(new ShortEdgeCertificate(u, v.v, CurrentTime, v.k));

                Points.RemoveCertificates(Points.GetCertificates<LongEdgeCertificate>(u, v.v).Union(Points.GetCertificates<LongEdgeCertificate>(v.v, u))
                    .Where(x => x.K == v.k));
            }

            foreach (SimulationPoint<Node>? up in u.GetChildren().Where(up => up.GetMaximumLevel() == v.k - 1).AppendWithoutAdding(u))
            {
                foreach (SimulationPoint<Node>? vp in v.v.GetChildren().Where(vp => vp.GetMaximumLevel() == v.k - 1).AppendWithoutAdding(v.v))
                {
                    // We check for K > 1 because we are creating a longedge certificate at level K - 1, which should be > 0.
                    if (v.k > 1 && !up.GetNeighbors().Any(x => x.v != up && x.v == vp && x.k == v.k - 1) && up != vp)
                    {
                        LongEdgeCertificate? failedCertificate = new(up, vp, CurrentTime, v.k - 1);
                        up.AddCertificate(failedCertificate);
                    }
                }
            }

            //Console.WriteLine($"[NEIGHBORS][END] Adding ({v.v.Identifier}) to ({u.Identifier}) neighbors at level {v.k} - {v.v.Node} {u.Node}");
        }

        internal static void RemoveNeighbor(this SimulationPoint<Node> u, IEnumerable<SimulationPoint<Node>> Points, double CurrentTime, (SimulationPoint<Node> v, int k) v)
        {
            if (u.GetNeighbors().RemoveWhere(x => x == v) == 0)
            {
                return;
            }

            u.Node.Cost.Structure++;

            //Console.WriteLine($"[NEIGHBORS][BEGIN] Removing ({v.v.Identifier}) from ({u.Identifier}) neighbors at level {v.k} - {v.v.Node} {u.Node}");

            if (u != v.v)
            {
                Points.RemoveCertificates(Points.GetCertificates(u, v.v).Union(Points.GetCertificates(v.v, u))
                    .Where(x => (x is SeparationCertificate c && c.K == v.k) || (x is ShortEdgeCertificate cp && cp.K == v.k)));

                LongEdgeCertificate? failedCertificate = new(u, v.v, CurrentTime, v.k);
                u.AddCertificate(failedCertificate);
            }

            foreach (SimulationPoint<Node>? up in u.GetChildren().Where(up => up.GetMaximumLevel() == v.k - 1).AppendWithoutAdding(u))
            {
                foreach (SimulationPoint<Node>? vp in v.v.GetChildren().Where(vp => vp.GetMaximumLevel() == v.k - 1).AppendWithoutAdding(v.v))
                {
                    Points.RemoveCertificates(Points.GetCertificates<LongEdgeCertificate>(up, vp).Union(Points.GetCertificates<LongEdgeCertificate>(vp, up)).Where(x => x.K == v.k - 1));
                }
            }

            //Console.WriteLine($"[NEIGHBORS][END] Removing ({v.v.Identifier}) from ({u.Identifier}) neighbors at level {v.k} - {v.v.Node} {u.Node}");
        }

        internal static void RemoveNeighborWhere(this SimulationPoint<Node> u, IEnumerable<SimulationPoint<Node>> Points, double CurrentTime, Func<(SimulationPoint<Node> v, int k), bool> predicate)
        {
            foreach ((SimulationPoint<Node> v, int k) v in u.GetNeighbors().Where(predicate))
            {
                u.RemoveNeighbor(Points, CurrentTime, v);
            }
        }

        internal static void AddChildren(this SimulationPoint<Node> u, double CurrentTime, SimulationPoint<Node> v)
        {
            if (u == v)
            {
                return;
            }

            //Console.WriteLine($"[CHILDREN][BEGIN] Adding ({v.Identifier}) to ({u.Identifier}) children - {v.Node} {u.Node}");

            u.GetChildren().Add(v);
            u.Node.Cost.Structure++;

            int k = v.GetMaximumLevel();

            foreach (SimulationPoint<Node>? vp in u.GetNeighbors()
                .Where(w => w.k == k + 1)
                .SelectMany(w => w.v
                    .GetChildren()
                    .Where(w => w.GetMaximumLevel() == k)
                    .AppendWithoutAdding(w.v)))
            {
                // We check for K >= 1 because we are creating a longedge certificate at level K, which should be > 0.
                if (k >= 1 && !v.GetNeighbors().Any(x => x.v != v && x.v == vp && x.k == k) && v != vp)
                {
                    LongEdgeCertificate? failedCertificate = new(v, vp, CurrentTime, k);
                    v.AddCertificate(failedCertificate);
                }
            }

            //Console.WriteLine($"[CHILDREN][END] Adding ({v.Identifier}) to ({u.Identifier}) children - {v.Node} {u.Node}");
        }

        internal static void RemoveChildren(this SimulationPoint<Node> u, IEnumerable<SimulationPoint<Node>> Points, SimulationPoint<Node> v)
        {
            if (u.GetChildren().RemoveWhere(x => x == v) == 0)
            {
                return;
            }
            u.Node.Cost.Structure++;

            //Console.WriteLine($"[CHILDREN][BEGIN] Removing ({v.Identifier}) from ({u.Identifier}) children - {v.Node} {u.Node}");

            int k = v.GetMaximumLevel();

            foreach (SimulationPoint<Node>? vp in u.GetNeighbors()
                .Where(w => w.k == k + 1)
                .SelectMany(w => w.v
                    .GetChildren()
                    .Where(w => w.GetMaximumLevel() == k)
                    .AppendWithoutAdding(w.v)))
            {
                Points.RemoveCertificates(Points.GetCertificates<LongEdgeCertificate>(v, vp).Union(Points.GetCertificates<LongEdgeCertificate>(vp, v)).Where(x => x.K == k));
            }

            //Console.WriteLine($"[CHILDREN][END] Removing ({v.Identifier}) from ({u.Identifier}) children - {v.Node} {u.Node}");
        }
    }
}
