using KDS;
using KDS.Interfaces;
using NearestNeighbors.MultiDim.KDS.Algorithm.Certificates;
using NearestNeighbors.MultiDim.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Linq;

namespace NearestNeighbors.MultiDim.KDS.Algorithm
{
    /// <summary>
    /// This class audits the data structure to ensure nothing unexpected happened during the execution of the algorithm
    /// </summary>
    public class Audit : IAudit<Node>
    {
        /// <summary>
        /// This function audits all data structures, an exception must be thrown in case of an inconsistency
        /// </summary>
        /// <param name="Points">The points containing the data structures to verify</param>
        public void AuditDataStructures(IEnumerable<SimulationPoint<Node>> Points)
        {
            /*foreach (var point in Points)
            {
                Console.WriteLine(point.CurrentData.Node);
            }*/

            // Iterate through all points
            foreach (SimulationPoint<Node> point in Points)
            {
                // Each parent must be at a level strictly higher than their child
                if (point.Node.Parent != point && point.Node.Parent.GetMaximumLevel() <= point.GetMaximumLevel())
                {
                    throw new Exception("Invalid parent");
                }

                // Each point must have itself as neighbor for every level 1 to its Maximum level
                for (int i = 1; i < point.GetMaximumLevel(); i++)
                {
                    if (!point.GetNeighbors().Any(x => x.k == i && x.v == point))
                    {
                        throw new Exception("The point does not have itself as neighbor at every level");
                    }
                }

                // Each point must not have a circular parent/children relationship
                point.EnsureNoCircularRelationship();

                // Each point must have neighbors at a level lower or equal than the maximum of said neighbor
                foreach ((SimulationPoint<Node> v, int k) in point.Node.Neighbors)
                {
                    if (k > v.Node.MaximumLevel)
                    {
                        throw new Exception("Neighbor at a level higher than its maximum level");
                    }
                }

                // Each point must have a neighbor at a level lower or equal to the point maximum level.
                if (point.Node.Neighbors.Any(x => x.k > point.Node.MaximumLevel))
                {
                    (SimulationPoint<Node> v, int k) v = point.Node.Neighbors.First(x => x.k > point.Node.MaximumLevel);
                    throw new Exception($"Invalid neighbor levels between {point.Node} ({point.GetMaximumLevel()}) and {v.v.Node} ({v.k})");
                }

                // Each point must have a longedge certificate (potential neighbor) at a level lower or equal to its maximum
                if (point.Certificates.Any(x => x is LongEdgeCertificate c && c.K > point.Node.MaximumLevel))
                {
                    LongEdgeCertificate failedCertificate = (LongEdgeCertificate)point.Certificates.First(x => x is LongEdgeCertificate c && c.K > point.Node.MaximumLevel);
                    throw new Exception($"Invalid neighbor levels for Lu. Certificate: {failedCertificate.GetCertificateString()}");
                }

                // Each point must have a longedge certificate (potential neighbor) at a level lower or equal to its maximum
                if (point.Certificates.Any(x => x is SeparationCertificate c && c.GetU() == point && (c.K != c.GetU().GetMaximumLevel() && c.K != c.GetV().GetMaximumLevel())))
                {
                    SeparationCertificate failedCertificate = (SeparationCertificate)point.Certificates.First(x => x is SeparationCertificate c && (c.K != c.GetU().GetMaximumLevel() && c.K != c.GetV().GetMaximumLevel()));
                    throw new Exception($"Invalid neighbor levels for Lu. Certificate: {failedCertificate.GetCertificateString()}");
                }

                // Iterate through each neighbor
                foreach ((SimulationPoint<Node> v, int k) in point.GetNeighbors())
                {
                    // A neighbor between U,V must be present in both U's list of neighbors, and V's list of neighbors
                    if (!v.GetNeighbors().Any(x => x.v == point && x.k == k))
                    {
                        throw new Exception("Invalid neighbor levels / not symetrical");
                    }

                    // Neighbors must be valid
                    double D1 = point.Distance(v);
                    double D2 = 2 * Math.Pow(Constants.b, k);
                    if (D1 >= D2)
                    {
                        throw new Exception("Points shouldn't be neighbors");
                    }
                }
            }
        }
    }
}
