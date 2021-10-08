using KDS;
using KDS.Certificates;
using NearestNeighbors.MultiDim.KDS.Algorithm.Certificates;
using NearestNeighbors.MultiDim.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.MultiDim.KDS.Algorithm
{
    /// <summary>
    /// This class contains helper extension methods that act on point objects
    /// </summary>
    public static class PointExtensions
    {
        /// <summary>
        /// This function appends an element to the list without adding it to the collection
        /// </summary>
        /// <typeparam name="TSource">The type of object in the collection</typeparam>
        /// <param name="source">The list to append the element to</param>
        /// <param name="element">The element to append to the list</param>
        /// <returns>The same collection but with the new element added</returns>
        public static IEnumerable<TSource> AppendWithoutAdding<TSource>(this IEnumerable<TSource> source, TSource element)
        {
            return source.Union(new TSource[] { element });
        }

        /// <summary>
        /// This function gets all children of a given point.
        /// </summary>
        /// <param name="v">The point to retrieve children from</param>
        /// <returns>The list of Simulation Points that are a child of v</returns>
        private static IEnumerable<SimulationPoint<Node>> GetAllDescendents(this SimulationPoint<Node> v)
        {
            if (v.Node.Children.Count == 0)
            {
                return Array.Empty<SimulationPoint<Node>>();
            }

            return v.Node.Children.SelectMany(x => GetAllDescendents(x)).Union(v.Node.Children);
        }

        /// <summary>
        /// This function gets all the nodes close at a distance R of a given point, using the KDS structure
        /// </summary>
        /// <param name="u">The point to retrieve close nodes from</param>
        /// <param name="r">The distance between the close nodes and the point u</param>
        /// <returns>The list of Simulation Points that are close at a distance r of u</returns>
        public static SimulationPoint<Node>[] GetCloseNodesUsingStructure(this SimulationPoint<Node> u,
            double r, bool CountCost = true)
        {
            int L = 1 + (int)Math.Floor(Math.Log(r / 2, Constants.b));

            SimulationPoint<Node>? x = u;
            while (x != null && x != x.Node.Parent && x.Node.MaximumLevel < L)
            {
                x = x.Node.Parent;
            }

            // TODO: See if this is ok.
            //L = x.Node.MaximumLevel;

            if (x == null)
            {
                return Array.Empty<SimulationPoint<Node>>();
            }

            HashSet<SimulationPoint<Node>> C = new();
            foreach ((SimulationPoint<Node> vp, int Lp) el in x.Node.Neighbors.Where(vp => vp.k == L))
            {
                //if (el.Lp <= L)
                //if (el.Lp == L)
                {
                    foreach (SimulationPoint<Node> v in el.vp.GetAllDescendents())
                    {
                        C.Add(v);
                    }
                    // TODO: See if this is ok
                    C.Add(el.vp);
                }
            }
            C.Add(u);
            if (CountCost)
                u.Node.Cost.Queries += C.Count;
            return C.Where(v => v.StaticDistance(u) <= r).ToArray();
        }

        /// <summary>
        /// This function gets all the nodes close at a distance R of a given point, using a known good, unoptimized, algorithm
        /// </summary>
        /// <param name="u">The point to retrieve close nodes from</param>
        /// <param name="PointStructureList">The list of all points in the simulation</param>
        /// <param name="r">The distance between the close nodes and the point u</param>
        /// <returns>The list of Simulation Points that are close at a distance r of u</returns>
        public static SimulationPoint<Node>[] GetCloseNodesUsingAllPoints(this SimulationPoint<Node> u,
            IEnumerable<SimulationPoint<Node>> PointStructureList,
            double r)
        {
            List<SimulationPoint<Node>> neighbors = new();
            foreach (SimulationPoint<Node>? pt in PointStructureList)
            {
                double d = u.Distance(pt);
                if (d <= r)
                {
                    neighbors.Add(pt);
                }
            }
            return neighbors.ToArray();
        }

        /// <summary>
        /// This function determines the label between two points
        /// </summary>
        /// <param name="u">The first point</param>
        /// <param name="v">The second point</param>
        /// <param name="k">The level</param>
        /// <returns>The distance label between both points</returns>
        public static NodeLinkLabel RoundDistance(this SimulationPoint<Node> u,
            SimulationPoint<Node> v,
            int k)
        {
            double D = u.StaticDistance(v);
            if (D < Constants.Gamma * Math.Pow(Constants.b, k + 1))
            {
                return NodeLinkLabel.Gamma;
            }
            if (Constants.Gamma * Math.Pow(Constants.b, k + 1) <= D && D < Constants.Beta * Math.Pow(Constants.b, k + 1))
            {
                return NodeLinkLabel.Beta;
            }
            if (Constants.Beta * Math.Pow(Constants.b, k + 1) <= D && D < Constants.Alpha * Math.Pow(Constants.b, k + 1))
            {
                return NodeLinkLabel.Alpha;
            }
            throw new Exception("Should never happen, Round Distance return Undefined");
            //return NodeLinkLabel.Undefined;
        }

        /// <summary>
        /// This function gets the first ancestor of a point. If no ancestor is found, this function returns null.
        /// </summary>
        /// <param name="u">The point to get the ancestor from</param>
        /// <param name="Condition">The condition to match</param>
        /// <returns>The ancestor or null</returns>
        private static SimulationPoint<Node>? GetAncestorWhere(this SimulationPoint<Node> u, Func<SimulationPoint<Node>, double, bool> Condition)
        {
            while (true)
            {
                //Console.WriteLine("GetAncestorWhere loop");

                SimulationPoint<Node>? v = u.GetParent();
                double d = u.Distance(v);
                if (u == v)
                {
                    return null;
                }

                u = v;
                if (Condition(u, d))
                {
                    break;
                }
            }
            return u;
        }

        /// <summary>
        /// This function gets the first alpha ancestor of a point. If no alpha ancestor is found, this function returns null.
        /// </summary>
        /// <param name="u">The point to get the alpha ancestor from</param>
        /// <returns>The alpha ancestor or null</returns>
        public static SimulationPoint<Node>? GetAlphaAncestor(this SimulationPoint<Node> u)
        {
            //Console.WriteLine("+GetAlphaAncestor");

            //Console.WriteLine("-GetAlphaAncestor");
            return u.GetAncestorWhere((u, d) =>
            {
                double c = Math.Pow(Constants.b, u.GetMaximumLevel() + 1);
                return Constants.Beta * c <= d && d < Constants.Alpha * c;
            });
        }

        /// <summary>
        /// This function gets the first gamma ancestor of a point. If no gamma ancestor is found, this function returns null.
        /// </summary>
        /// <param name="u">The point to get the gamma ancestor from</param>
        /// <returns>The gamma ancestor or null</returns>
        public static SimulationPoint<Node>? GetGammaAncestor(this SimulationPoint<Node> u)
        {
            //Console.WriteLine("+GetGammaAncestor");

            //Console.WriteLine("-GetGammaAncestor");
            return u.GetAncestorWhere((u, d) =>
            {
                double c = Math.Pow(Constants.b, u.GetMaximumLevel() + 1);
                return d < Constants.Gamma * c;
            });
        }

        /// <summary>
        /// This function gets the parent point, if the point is the root, then the parent is itself
        /// </summary>
        /// <param name="simulationPoint">The point to get the parent from</param>
        /// <returns>The parent point</returns>
        public static SimulationPoint<Node> GetParent(this SimulationPoint<Node> simulationPoint)
        {
            return simulationPoint.Node.Parent;
        }

        /// <summary>
        /// This function gets the neighbors of a point
        /// </summary>
        /// <param name="simulationPoint">The point to get neighbors from</param>
        /// <returns>The list of neighbors and their level with said point</returns>
        public static HashSet<(SimulationPoint<Node> v, int k)> GetNeighbors(this SimulationPoint<Node> simulationPoint)
        {
            return simulationPoint.Node.Neighbors;
        }

        /// <summary>
        /// This function gets the level of a point
        /// </summary>
        /// <param name="simulationPoint">The point to get the level from</param>
        /// <returns>The level of said point</returns>
        public static int GetMaximumLevel(this SimulationPoint<Node> simulationPoint)
        {
            return simulationPoint.Node.MaximumLevel;
        }

        /// <summary>
        /// This function gets all children of a given point
        /// </summary>
        /// <param name="simulationPoint">The point to get all children from</param>
        /// <returns>The direct children of a point</returns>
        public static HashSet<SimulationPoint<Node>> GetChildren(this SimulationPoint<Node> simulationPoint)
        {
            return simulationPoint.Node.Children;
        }

        /// <summary>
        /// This function ensures no circular relationship is present for the given point, an exception is thrown in case it is the case
        /// </summary>
        /// <param name="point">The point to verify</param>
        public static void EnsureNoCircularRelationship(this SimulationPoint<Node> point)
        {
            List<uint> identifiers = new();
            SimulationPoint<Node>? fv = point.GetParent();
            identifiers.Add(fv.Identifier);
            while (fv != fv.GetParent())
            {
                fv = fv.GetParent();
                if (identifiers.Contains(fv.Identifier))
                {
                    throw new Exception("Circular Parent/Children relationship detected");
                }

                identifiers.Add(fv.Identifier);
            }
        }

        /// <summary>
        /// Gets a string representing the certificate provided
        /// </summary>
        /// <param name="cert">The certificate to get a string representation from</param>
        /// <returns>A string representation of the certificate</returns>
        public static string GetCertificateString(this ISimulationCertificate<Node> cert)
        {
            switch (cert.GetType().Name)
            {
                case "CoverCertificate":
                    {
                        CoverCertificate? failedCertificate = (CoverCertificate)cert;
                        return $"Cover_L={failedCertificate.K},T={failedCertificate.Label}({failedCertificate.GetU().Identifier}, {failedCertificate.GetV().Identifier}) - {failedCertificate.GetU().Node} {failedCertificate.GetV().Node}";
                    }
                case "SCoverCertificate":
                    {
                        SCoverCertificate? failedCertificate = (SCoverCertificate)cert;
                        return $"SCover_L={failedCertificate.K},T={failedCertificate.Label}({failedCertificate.GetU().Identifier}, {failedCertificate.GetV().Identifier}) - {failedCertificate.GetU().Node} {failedCertificate.GetV().Node}";
                    }
                case "SeparationCertificate":
                    {
                        SeparationCertificate? failedCertificate = (SeparationCertificate)cert;
                        return $"Separation_L={failedCertificate.K}({failedCertificate.GetU().Identifier}, {failedCertificate.GetV().Identifier}) - {failedCertificate.GetU().Node} {failedCertificate.GetV().Node}";
                    }
                case "ShortEdgeCertificate":
                    {
                        ShortEdgeCertificate? failedCertificate = (ShortEdgeCertificate)cert;
                        return $"ShortEdge_L={failedCertificate.K}({failedCertificate.GetU().Identifier}, {failedCertificate.GetV().Identifier}) - {failedCertificate.GetU().Node} {failedCertificate.GetV().Node}";
                    }
                case "LongEdgeCertificate":
                    {
                        LongEdgeCertificate? failedCertificate = (LongEdgeCertificate)cert;
                        return $"LongEdge_L={failedCertificate.K}({failedCertificate.GetU().Identifier}, {failedCertificate.GetV().Identifier}) - {failedCertificate.GetU().Node} {failedCertificate.GetV().Node}";
                    }
                default:
                    {
                        return "Unknown";
                    }
            }
        }

        /// <summary>
        /// Adds a certificate
        /// </summary>
        /// <param name="u">The point to add a certificate into</param>
        /// <param name="cert">The certificate to add</param>
        public static void AddCertificate(this SimulationPoint<Node> u, ISimulationCertificate<Node> cert)
        {
            if (u.Certificates.Any(x => x.GetCertificateString() == cert.GetCertificateString()))
            {
                return;
            }

            //Console.WriteLine($"[CERTIFICATE] Adding certificate to ({u.Identifier}) of type {cert.GetCertificateString()} - {u.Node}");
            u.Certificates.Add(cert);
            u.Node.Cost.Certificate++;
        }

        /// <summary>
        /// Removes a certificate specified from all points
        /// </summary>
        /// <param name="Points">The points to remove the certificate from</param>
        /// <param name="cert">The certificate to remove</param>
        public static void RemoveCertificate(this IEnumerable<SimulationPoint<Node>> Points, ISimulationCertificate<Node> cert)
        {
            foreach (SimulationPoint<Node>? point in Points.Where(point => point.Certificates.Contains(cert)).ToList())
            {
                //Console.WriteLine($"[CERTIFICATE] Removing certificate from ({point.Identifier}) of type {cert.GetCertificateString()} - {point.Node}");
                point.RemoveCertificate(cert);
                cert.Dispose();
            }
        }

        /// <summary>
        /// Removes all certificates specified from all points
        /// </summary>
        /// <param name="Points">The points to remove certificates from</param>
        /// <param name="certs">The list of certificates to remove</param>
        public static void RemoveCertificates(this IEnumerable<SimulationPoint<Node>> Points, IEnumerable<ISimulationCertificate<Node>> certs)
        {
            foreach (ISimulationCertificate<Node>? cert in certs.ToList())
            {
                Points.RemoveCertificate(cert);
            }
        }

        /// <summary>
        /// Gets all certificates
        /// </summary>
        /// <param name="Points">The points to get the certificates from</param>
        /// <returns>A list of certificates matching above's criteria</returns>
        public static IEnumerable<ISimulationCertificate<Node>> GetCertificates(this IEnumerable<SimulationPoint<Node>> Points)
        {
            return Points.SelectMany(x => x.Certificates);
        }

        /// <summary>
        /// Gets all certificates of type T bound to U and V
        /// </summary>
        /// <typeparam name="T">The type of certificate to get</typeparam>
        /// <param name="Points">The points to get the certificates from</param>
        /// <param name="u">The point U to match in the certificate</param>
        /// <param name="v">The point V to match in the certificate</param>
        /// <returns>A list of certificates matching above's criteria</returns>
        public static IEnumerable<T> GetCertificates<T>(this IEnumerable<SimulationPoint<Node>> Points, SimulationPoint<Node> u, SimulationPoint<Node> v)
        {
            return Points.SelectMany(x => x.Certificates).Where(x => x is T && (x.GetU() == u && x.GetV() == v)).Cast<T>();
        }

        /// <summary>
        /// Gets all certificates of type T bound to U
        /// </summary>
        /// <typeparam name="T">The type of certificate to get</typeparam>
        /// <param name="Points">The points to get the certificates from</param>
        /// <param name="u">The point U to match in the certificate</param>
        /// <returns>A list of certificates matching above's criteria</returns>
        public static IEnumerable<T> GetCertificates<T>(this IEnumerable<SimulationPoint<Node>> Points, SimulationPoint<Node> u)
        {
            return Points.SelectMany(x => x.Certificates).Where(x => x is T && (x.GetU() == u || x.GetV() == u)).Cast<T>();
        }

        /// <summary>
        /// Gets all certificates of type T
        /// </summary>
        /// <typeparam name="T">The type of certificate to get</typeparam>
        /// <param name="Points">The points to get the certificates from</param>
        /// <returns>A list of certificates matching above's criteria</returns>
        public static IEnumerable<T> GetCertificates<T>(this IEnumerable<SimulationPoint<Node>> Points)
        {
            return Points.SelectMany(x => x.Certificates).OfType<T>();
        }

        /// <summary>
        /// Gets all certificates bound to U
        /// </summary>
        /// <param name="Points">The points to get the certificates from</param>
        /// <param name="u">The point U to match in the certificate</param>
        /// <returns>A list of certificates matching above's criteria</returns>
        public static IEnumerable<ISimulationCertificate<Node>> GetCertificates(this IEnumerable<SimulationPoint<Node>> Points, SimulationPoint<Node> u)
        {
            return Points.SelectMany(x => x.Certificates).Where(x => x.GetU() == u || x.GetV() == u);
        }

        /// <summary>
        /// Gets all certificates bound to U and V
        /// </summary>
        /// <param name="Points">The points to get the certificates from</param>
        /// <param name="u">The point U to match in the certificate</param>
        /// <param name="v">The point V to match in the certificate</param>
        /// <returns>A list of certificates matching above's criteria</returns>
        public static IEnumerable<ISimulationCertificate<Node>> GetCertificates(this IEnumerable<SimulationPoint<Node>> Points, SimulationPoint<Node> u, SimulationPoint<Node> v)
        {
            return Points.SelectMany(x => x.Certificates).Where(x => x.GetU() == u && x.GetV() == v);
        }
    }
}
