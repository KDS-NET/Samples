using KDS;
using KDS.Certificates;
using KDS.Interfaces;
using NearestNeighbors.MultiDim.KDS.Algorithm.Certificates;
using NearestNeighbors.MultiDim.KDS.Algorithm.Data;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;

#nullable enable

namespace NearestNeighbors.MultiDim.KDS.Algorithm
{
    public class AlgorithmCode : IAlgorithmCode<Node>
    {
        private static readonly List<TimeSpan>[] ExecutionTimes = new List<TimeSpan>[10] { new List<TimeSpan>(), new List<TimeSpan>(), new List<TimeSpan>(), new List<TimeSpan>(), new List<TimeSpan>(), new List<TimeSpan>(), new List<TimeSpan>(), new List<TimeSpan>(), new List<TimeSpan>(), new List<TimeSpan>() };

        public static string FunctionNameFromId(int id)
        {
            switch (id)
            {
                case 0:
                    {
                        return "Redirect";
                    }
                case 1:
                    {
                        return "Promote";
                    }
                case 2:
                    {
                        return "HandleSeparationCertificates";
                    }
                case 3:
                    {
                        return "HandleShortEdgeCertificates";
                    }
                case 4:
                    {
                        return "HandleLongEdgeCertificates";
                    }
                case 5:
                    {
                        return "HandleSCoverCertificates";
                    }
                case 6:
                    {
                        return "HandleCoverCertificates";
                    }
                case 7:
                    {
                        return "HandleAlphaCoverCertificates";
                    }
                case 9:
                    {
                        return "RunAlgorithmAfterSinglePointMoved";
                    }
                default:
                    {
                        return "Unknown";
                    }
            }
        }

        public static void PrintTimes()
        {
            for (int i = 0; i < ExecutionTimes.Length; i++)
            {
                string func = FunctionNameFromId(i);
                IEnumerable<double>? ms = ExecutionTimes[i].Select(x => x.TotalMilliseconds);
                double avg = ms.Any() ? ms.Average() : 0;
                double max = ms.Any() ? ms.Max() : 0;
                double min = ms.Any() ? ms.Min() : 0;
                int c = ms.Count();
                Console.WriteLine(func + " - Average: " + avg + " - Max: " + max + " - Min: " + min + " - Count: " + c);
            }
        }

        /// <summary>
        /// Gets the number of rounds this algorithm expects. This is helpful for synchronism if required
        /// </summary>
        /// <returns></returns>
        public int GetMaxIterationCount()
        {
            return 0;
        }

        /// <summary>
        /// Runs the algorithm at a given round with a list of failed certificate and the current point we are handling
        /// </summary>
        /// <param name="i">The current round number, starts from 0</param>
        /// <param name="Fail">The list of failed certificates right now</param>
        /// <param name="u">The current point</param>
        /// <param name="CurrentTime">The current time</param>
        public void RunAlgorithmAfterAllPointsMovedPerPoint(int i, IEnumerable<ISimulationCertificate<Node>> Fail,
            SimulationPoint<Node> u,
            double CurrentTime)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Runs the algorithm at a given round with a list of failed certificate and all the points we are handling
        /// </summary>
        /// <param name="Fail">The list of failed certificates right now</param>
        /// <param name="Points">All the points</param>
        /// <param name="CurrentTime">The current time</param>
        public void RunAlgorithmAfterAllPointsMoved(IEnumerable<ISimulationCertificate<Node>> Fail,
            IEnumerable<SimulationPoint<Node>> Points,
            double CurrentTime)
        {
            throw new NotImplementedException();
        }

        [SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        private static void Redirect(IEnumerable<SimulationPoint<Node>> Points, double CurrentTime,
            SimulationPoint<Node> v,
            SimulationPoint<Node> oldf,
            SimulationPoint<Node> newf)
        {
            //Console.WriteLine();
            //Console.WriteLine("////////////////////////////////////////////////////////");
            //Console.WriteLine();

            //Console.WriteLine($"[Redirect][BEGIN]({v.Node}, {oldf.Node}, {newf.Node})");
            Stopwatch sw = new();
            sw.Start();

            oldf.RemoveChildren(Points, v);
            Points.RemoveCertificates(Points
                .GetCertificates(v, oldf).Where(x => x is CoverCertificate || x is SCoverCertificate));

            v.Node.Parent = newf;
            newf.AddChildren(CurrentTime, v);
            v.AddCertificate(new CoverCertificate(v, newf, CurrentTime, v.GetMaximumLevel(), NodeLinkLabel.Gamma));

            sw.Stop();
            ExecutionTimes[0].Add(sw.Elapsed);
            //Console.WriteLine($"[Redirect][END]({v.Node}, {oldf.Node}, {newf.Node})");

            //Console.WriteLine();
            //Console.WriteLine("////////////////////////////////////////////////////////");
            //Console.WriteLine();
        }

        [SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        private static void Promote(IEnumerable<SimulationPoint<Node>> Points, double CurrentTime,
            SimulationPoint<Node> v,
            SimulationPoint<Node> oldf)
        {
            //Console.WriteLine();
            //Console.WriteLine("////////////////////////////////////////////////////////");
            //Console.WriteLine();

            //Console.WriteLine($"[Promote][BEGIN]({v.Node}, {oldf.Node})");
            Stopwatch sw = new();
            sw.Start();

            SimulationPoint<Node> newf;
            if (oldf.GetMaximumLevel() >= v.GetMaximumLevel() + 2)
            {
                newf = oldf;
            }
            else
            {
                newf = oldf.GetParent();
            }

            NodeLinkLabel T;

            if (v.GetMaximumLevel() != 0)
            {
                if (oldf.Distance(v) >= Constants.Beta * Math.Pow(Constants.b, v.GetMaximumLevel() + 1))
                {
                    T = v.RoundDistance(newf, v.GetMaximumLevel() + 1);
                }
                else
                {
                    T = NodeLinkLabel.Beta;
                }
            }
            else
            {
                T = oldf.RoundDistance(newf, oldf.GetMaximumLevel());
            }

            oldf.RemoveChildren(Points, v);

            Points.RemoveCertificates(Points
                .GetCertificates<CoverCertificate>(v, oldf)
                .Where(x => x.K == v.GetMaximumLevel()));

            v.AddCertificate(new CoverCertificate(v, newf, CurrentTime, v.GetMaximumLevel() + 1, T));

            Points.RemoveCertificates(Points
                .GetCertificates<SCoverCertificate>(v, oldf)
                .Where(x => x.K == v.GetMaximumLevel()));

            if (T - 1 >= NodeLinkLabel.Gamma)
            {
                v.AddCertificate(new SCoverCertificate(v, newf, CurrentTime, v.GetMaximumLevel() + 1, T - 1));
            }

            //Console.WriteLine($"[Promote] Increasing {v.Node} from {v.GetMaximumLevel()} to {v.GetMaximumLevel() + 1}");
            v.Node.MaximumLevel++;
            v.Node.Parent = newf;

            SimulationPoint<Node>? fv = v.GetParent();
            HashSet<(SimulationPoint<Node>, int)>? Nfv = fv.GetNeighbors();
            int Lv = v.GetMaximumLevel();
            foreach (SimulationPoint<Node>? wp in Nfv.Where(x => x.Item2 == Lv + 1).Select(x => x.Item1).ToList()) // Collection modified!
            {
                foreach (SimulationPoint<Node>? w in (IEnumerable<SimulationPoint<Node>>)wp.GetChildren().Where(w => w.GetMaximumLevel() == Lv).AppendWithoutAdding(fv))
                {
                    if (w.Distance(v) < 2 * Math.Pow(Constants.b, Lv))
                    {
                        v.AddNeighbor(Points, CurrentTime, (w, Lv));
                        w.AddNeighbor(Points, CurrentTime, (v, Lv));
                    }
                }
            }
            v.AddNeighbor(Points, CurrentTime, (v, Lv));
            newf.AddChildren(CurrentTime, v);

            sw.Stop();
            ExecutionTimes[1].Add(sw.Elapsed);
            //Console.WriteLine($"[Promote][END]({v.Node}, {oldf.Node})");

            //Console.WriteLine();
            //Console.WriteLine("////////////////////////////////////////////////////////");
            //Console.WriteLine();
        }

        #region Separation Certificate Handling
        [SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        private static void HandleSeparationCertificates(IEnumerable<SeparationCertificate> failedSeparationCertificates,
            IEnumerable<SimulationPoint<Node>> Points,
            double CurrentTime)
        {
            // We store the list of points we already handled below,
            // because during the execution of the function, we want to remove
            // all other certificates linked to the same point U
            // and we can't modify the collection in any way during a foreach statement
            // otherwise we get a Collection Modified exception.
            List<uint> PointsAlreadyHandledForSeparation = new();

            foreach (SeparationCertificate? failedCertificate in failedSeparationCertificates)
            {
                if (PointsAlreadyHandledForSeparation.Contains(failedCertificate.GetU().Identifier) || PointsAlreadyHandledForSeparation.Contains(failedCertificate.GetV().Identifier))
                {
                    continue;
                }
                //Console.WriteLine();
                //Console.WriteLine("////////////////////////////////////////////////////////");
                //Console.WriteLine();

                //Console.WriteLine($"[CERTIFICATE FAILURE][BEGIN] {failedCertificate.GetCertificateString()}");

                if (failedCertificate.GetU().GetMaximumLevel() != failedCertificate.K)
                {
                    throw new Exception("Levels mismatch");
                }

                Stopwatch sw = new();
                sw.Start();

                PointsAlreadyHandledForSeparation.Add(failedCertificate.GetU().Identifier);
                PointsAlreadyHandledForSeparation.Add(failedCertificate.GetV().Identifier);

                List<SimulationPoint<Node>>? PF = failedCertificate
                    .GetU()
                    .GetNeighbors()
                    .Where(w => w.k == failedCertificate.K)
                    .Select(w => w.v)
                    .Where(w => w != failedCertificate.GetU()).ToList();

                /*Console.WriteLine("-");
                foreach (var pf in PF)
                {
                    Console.WriteLine(pf.CurrentData.Node);
                }
                Console.WriteLine("-");*/

                foreach (SimulationPoint<Node>? w in failedCertificate
                    .GetU()
                    .GetChildren()
                    .Where(w => w.GetMaximumLevel() == failedCertificate.K - 1)
                    .ToList()) // Collection modified
                {
                    if (PF.Any(wp => wp != w && wp.Distance(w) < Constants.Gamma * Math.Pow(Constants.b, failedCertificate.K)))
                    {
                        Redirect(Points, CurrentTime, w, failedCertificate.GetU(), PF
                            .First(wp => wp != w && wp.Distance(w) < Constants.Gamma * Math.Pow(Constants.b, failedCertificate.K)));
                    }
                    else
                    {
                        Promote(Points, CurrentTime, w, failedCertificate.GetU());
                        if (!PF.Contains(w))
                        {
                            PF.Add(w);
                        }
                    }
                }

                // TODO check point 1, 7

                Points.RemoveCertificates(Points
                    .GetCertificates(failedCertificate.GetU(), failedCertificate.GetU().GetParent())
                    .Where(x => x is CoverCertificate || x is SCoverCertificate));

                failedCertificate.GetU().GetParent().RemoveChildren(Points, failedCertificate.GetU());

                // Note: change with pdf, this must be done before otherwise we already removed the elements
                foreach ((SimulationPoint<Node> w, int lu) in failedCertificate.GetU().GetNeighbors().Where(w => w.k == failedCertificate.GetU().GetMaximumLevel()))
                {
                    w.RemoveNeighborWhere(Points, CurrentTime, t => t.v == failedCertificate.GetU() && t.k == failedCertificate.GetU().GetMaximumLevel());
                }
                failedCertificate.GetU().RemoveNeighborWhere(Points, CurrentTime, w => w.k == failedCertificate.GetU().GetMaximumLevel());

                //Console.WriteLine($"[HandleSeparationCertificates] Lowering {failedCertificate.GetU().Node} from {failedCertificate.GetU().GetMaximumLevel()} to {failedCertificate.GetU().GetMaximumLevel()}");
                failedCertificate.GetU().Node.MaximumLevel--;
                failedCertificate.GetU().Node.Parent = failedCertificate.GetV();
                failedCertificate.GetV().AddChildren(CurrentTime, failedCertificate.GetU());

                failedCertificate.GetU().AddCertificate(new CoverCertificate(failedCertificate.GetU(), failedCertificate.GetV(), CurrentTime, failedCertificate.GetU().GetMaximumLevel(), NodeLinkLabel.Gamma));

                Points.RemoveCertificates(Points
                    .GetCertificates<SeparationCertificate>(failedCertificate.GetU())
                    .Where(x => x.K == failedCertificate.K));

                Points.RemoveCertificates(Points
                       .GetCertificates<LongEdgeCertificate>(failedCertificate.GetU())
                       .Where(x => x.K == failedCertificate.K));

                sw.Stop();
                ExecutionTimes[2].Add(sw.Elapsed);
                //Console.WriteLine($"[CERTIFICATE FAILURE][END] {failedCertificate.GetCertificateString()}");

                //Console.WriteLine();
                //Console.WriteLine("////////////////////////////////////////////////////////");
                //Console.WriteLine();
            }
        }
        #endregion

        #region Edge Certificate Handling
        private static void HandleEdgeCertificates(IEnumerable<ISimulationCertificate<Node>> failedEdgeCertificates,
            IEnumerable<SimulationPoint<Node>> Points,
            double CurrentTime)
        {
            foreach (ISimulationCertificate<Node> failedEdgeCertificate in failedEdgeCertificates)
            {
                if (failedEdgeCertificate is ShortEdgeCertificate failedCertificate)
                {
                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();

                    //Console.WriteLine($"[CERTIFICATE FAILURE][BEGIN] {failedCertificate.GetCertificateString()}");
                    Stopwatch sw = new();
                    sw.Start();

                    failedCertificate.GetU()
                        .RemoveNeighborWhere(Points, CurrentTime, x => x.v == failedCertificate.GetV() && x.k == failedCertificate.K);
                    failedCertificate.GetV()
                        .RemoveNeighborWhere(Points, CurrentTime, x => x.v == failedCertificate.GetU() && x.k == failedCertificate.K);

                    sw.Stop();
                    ExecutionTimes[3].Add(sw.Elapsed);
                    //Console.WriteLine($"[CERTIFICATE FAILURE][END] {failedCertificate.GetCertificateString()}");

                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();
                }
                else if (failedEdgeCertificate is LongEdgeCertificate failedCertificate2)
                {
                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();

                    //Console.WriteLine($"[CERTIFICATE FAILURE][BEGIN] {failedCertificate2.GetCertificateString()}");
                    Stopwatch sw = new();
                    sw.Start();

                    failedCertificate2.GetU().AddNeighbor(Points, CurrentTime, (failedCertificate2.GetV(), failedCertificate2.K));
                    failedCertificate2.GetV().AddNeighbor(Points, CurrentTime, (failedCertificate2.GetU(), failedCertificate2.K));

                    sw.Stop();
                    ExecutionTimes[4].Add(sw.Elapsed);
                    //Console.WriteLine($"[CERTIFICATE FAILURE][END] {failedCertificate2.GetCertificateString()}");

                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();
                }
            }
        }
        #endregion

        #region Cover Certificate Handling
        [SuppressMessage("Wrong Usage", "DF0000:Marks undisposed anonymous objects from object creations.", Justification = "<Pending>")]
        private static void HandleCoverCertificates(IEnumerable<ISimulationCertificate<Node>> failedCoverCertificates,
            IEnumerable<SimulationPoint<Node>> Points,
            double CurrentTime)
        {
            foreach (ISimulationCertificate<Node>? failedCoverCertificate in failedCoverCertificates)
            {
                if (failedCoverCertificate is SCoverCertificate failedCertificate &&
                    (failedCertificate.Label == NodeLinkLabel.Gamma || failedCertificate.Label == NodeLinkLabel.Beta))
                {
                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();

                    //Console.WriteLine($"[CERTIFICATE FAILURE][BEGIN] {failedCertificate.GetCertificateString()}");
                    Stopwatch sw = new();
                    sw.Start();

                    if (failedCertificate.Label == NodeLinkLabel.Beta)
                    {
                        Points.RemoveCertificate(failedCertificate);
                        failedCertificate.GetU().AddCertificate(new SCoverCertificate(failedCertificate.GetU(), failedCertificate.GetV(), CurrentTime, failedCertificate.K, NodeLinkLabel.Gamma));
                    }

                    Points.RemoveCertificates(Points
                        .GetCertificates<CoverCertificate>(failedCertificate.GetU(), failedCertificate.GetV())
                        .Where(x => x.Label == failedCertificate.Label + 1 && x.K == failedCertificate.K));

                    failedCertificate.GetU().AddCertificate(new CoverCertificate(failedCertificate.GetU(), failedCertificate.GetV(), CurrentTime, failedCertificate.K, failedCertificate.Label));

                    sw.Stop();
                    ExecutionTimes[5].Add(sw.Elapsed);
                    //Console.WriteLine($"[CERTIFICATE FAILURE][END] {failedCertificate.GetCertificateString()}");

                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();
                }
                else if (failedCoverCertificate is CoverCertificate failedCertificate2 &&
                    (failedCertificate2.Label == NodeLinkLabel.Gamma || failedCertificate2.Label == NodeLinkLabel.Beta) &&
                    failedCertificate2.K >= 1)
                {
                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();

                    //Console.WriteLine($"[CERTIFICATE FAILURE][BEGIN] {failedCertificate2.GetCertificateString()}");
                    Stopwatch sw = new();
                    sw.Start();

                    SimulationPoint<Node>? xa = failedCertificate2.GetU().GetAlphaAncestor();
                    SimulationPoint<Node>? xy = failedCertificate2.GetU().GetGammaAncestor();
                    if (xa != null && xy != null && xa.GetMaximumLevel() < xy.GetMaximumLevel())
                    {
                        if (xa.GetParent().GetNeighbors().Where(x => x.v != xa.GetParent())
                            .Any(fp => fp.k == xa.GetMaximumLevel() + 1 && xa.Distance(fp.v) <= Constants.Gamma * Math.Pow(Constants.b, failedCertificate2.K + 1)))
                        {
                            (SimulationPoint<Node>, int) fp = xa.GetParent().GetNeighbors()
                                .First(fp => fp.k == xa.GetMaximumLevel() + 1 && xa.Distance(fp.v) <= Constants.Gamma * Math.Pow(Constants.b, failedCertificate2.K + 1));
                            Redirect(Points, CurrentTime, xa, xa.GetParent(), fp.Item1);
                        }
                        else
                        {
                            Promote(Points, CurrentTime, xa, xa.GetParent());
                        }
                    }

                    Points.RemoveCertificates(Points
                        .GetCertificates<CoverCertificate>(failedCertificate2.GetU(), failedCertificate2.GetV())
                        .Where(x => x.Label == failedCertificate2.Label && x.K == failedCertificate2.K));

                    failedCertificate2.GetU().AddCertificate(new CoverCertificate(failedCertificate2.GetU(), failedCertificate2.GetV(), CurrentTime, failedCertificate2.K, failedCertificate2.Label + 1));

                    Points.RemoveCertificates(Points
                        .GetCertificates<SCoverCertificate>(failedCertificate2.GetU(), failedCertificate2.GetV())
                        .Where(x => x.Label == failedCertificate2.Label - 1 && x.K == failedCertificate2.K));

                    failedCertificate2.GetU().AddCertificate(new SCoverCertificate(failedCertificate2.GetU(), failedCertificate2.GetV(), CurrentTime, failedCertificate2.K, failedCertificate2.Label));

                    sw.Stop();
                    ExecutionTimes[6].Add(sw.Elapsed);
                    //Console.WriteLine($"[CERTIFICATE FAILURE][END] {failedCertificate2.GetCertificateString()}");

                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();
                }
                else if (failedCoverCertificate is CoverCertificate failedCertificate3 &&
                    (failedCertificate3.Label == NodeLinkLabel.Alpha || (failedCertificate3.Label == NodeLinkLabel.Gamma && failedCertificate3.K == 0)))
                {
                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();

                    //Console.WriteLine($"[CERTIFICATE FAILURE][BEGIN] {failedCertificate3.GetCertificateString()}");
                    Stopwatch sw = new();
                    sw.Start();

                    Points.RemoveCertificates(Points
                        .GetCertificates<SCoverCertificate>(failedCertificate3.GetU(), failedCertificate3.GetV())
                        .Where(x => x.Label == NodeLinkLabel.Beta && x.K == failedCertificate3.K));

                    if (failedCertificate3.GetV().GetNeighbors().Where(x => x.v != failedCertificate3.GetV())
                        .Any(vp => vp.k == failedCertificate3.K + 1 && failedCertificate3.GetU().Distance(vp.v) <= Constants.Gamma * Math.Pow(Constants.b, failedCertificate3.K + 1)))
                    {
                        (SimulationPoint<Node>, int) vp = failedCertificate3.GetV().GetNeighbors()
                            .First(vp => vp.k == failedCertificate3.K + 1 && failedCertificate3.GetU().Distance(vp.v) <= Constants.Gamma * Math.Pow(Constants.b, failedCertificate3.K + 1));
                        Redirect(Points, CurrentTime, failedCertificate3.GetU(), failedCertificate3.GetV(), vp.Item1);
                    }
                    else
                    {
                        Promote(Points, CurrentTime, failedCertificate3.GetU(), failedCertificate3.GetV());
                    }

                    sw.Stop();
                    ExecutionTimes[7].Add(sw.Elapsed);
                    //Console.WriteLine($"[CERTIFICATE FAILURE][END] {failedCertificate3.GetCertificateString()}");

                    //Console.WriteLine();
                    //Console.WriteLine("////////////////////////////////////////////////////////");
                    //Console.WriteLine();
                }
            }
        }
        #endregion

        /// <summary>
        /// Runs the algorithm at a given round with a list of failed certificate and all the points we are handling everytime a point moves
        /// </summary>
        /// <param name="Fail">The list of failed certificates right now</param>
        /// <param name="Points">All the points</param>
        /// <param name="CurrentTime">The current time</param>
        public void RunAlgorithmAfterSinglePointMoved(IEnumerable<ISimulationCertificate<Node>> Fail,
            IEnumerable<SimulationPoint<Node>> Points,
            double CurrentTime)
        {
            //Console.WriteLine();
            //Console.WriteLine();
            //Console.WriteLine("========================================================");
            //Console.WriteLine();
            //Console.WriteLine();

            //Console.WriteLine("RunAlgorithmAfterSinglePointMoved - BEGIN");
            Stopwatch sw = new();
            sw.Start();

            SeparationCertificate[] failedSeparationCertificates;
            ISimulationCertificate<Node>[] failedEdgeCertificates;
            ISimulationCertificate<Node>[] failedCoverCertificates;

            List<ISimulationCertificate<Node>> HandledCertificates = new();

            failedSeparationCertificates = Fail
                 .OfType<SeparationCertificate>()
                 .Where(x => x.GetU().GetMaximumLevel() <= x.GetV().GetMaximumLevel() && !HandledCertificates.Contains(x))
                 .OrderByDescending(x => x.K).ToArray();

            while (failedSeparationCertificates.Length > 0)
            {
                HandledCertificates.AddRange(failedSeparationCertificates);

                //Console.WriteLine();
                //Console.WriteLine("--------------------------------------------------------");
                //Console.WriteLine();

                //Console.WriteLine("HandleSeparationCertificates");

                HandleSeparationCertificates(failedSeparationCertificates, Points, CurrentTime);

                // In case we added certificates that failed immediately, recompute, TODO: optimize
                Fail = Points.SelectMany(x => x.GetFailedCertificates());

                failedSeparationCertificates = Fail
                     .OfType<SeparationCertificate>()
                     .Where(x => x.GetU().GetMaximumLevel() <= x.GetV().GetMaximumLevel() && !HandledCertificates.Contains(x))
                     .OrderByDescending(x => x.K).ToArray();
            }

            failedEdgeCertificates = Fail
                    .Where(x => (x is ShortEdgeCertificate || x is LongEdgeCertificate) && x.GetU().GetMaximumLevel() <= x.GetV().GetMaximumLevel() && !HandledCertificates.Contains(x))
                    .OrderByDescending(x => x is ShortEdgeCertificate c ? c.K : ((LongEdgeCertificate)x).K).ToArray();

            while (failedEdgeCertificates.Length > 0)
            {
                HandledCertificates.AddRange(failedEdgeCertificates);

                //Console.WriteLine();
                //Console.WriteLine("--------------------------------------------------------");
                //Console.WriteLine();

                //Console.WriteLine("HandleEdgeCertificates");

                HandleEdgeCertificates(failedEdgeCertificates, Points, CurrentTime);

                // In case we added certificates that failed immediately, recompute, TODO: optimize
                Fail = Points.SelectMany(x => x.GetFailedCertificates());

                failedEdgeCertificates = Fail
                    .Where(x => (x is ShortEdgeCertificate || x is LongEdgeCertificate) && x.GetU().GetMaximumLevel() <= x.GetV().GetMaximumLevel() && !HandledCertificates.Contains(x))
                    .OrderByDescending(x => x is ShortEdgeCertificate c ? c.K : ((LongEdgeCertificate)x).K).ToArray();
            }

            failedCoverCertificates = Fail
                .Where(x => (x is SCoverCertificate || x is CoverCertificate) && !HandledCertificates.Contains(x))
                .OrderByDescending(x => x is SCoverCertificate c ? c.K : ((CoverCertificate)x).K).ToArray();

            while (failedCoverCertificates.Length > 0)
            {
                HandledCertificates.AddRange(failedCoverCertificates);

                //Console.WriteLine();
                //Console.WriteLine("--------------------------------------------------------");
                //Console.WriteLine();

                //Console.WriteLine("HandleCoverCertificates");

                HandleCoverCertificates(failedCoverCertificates, Points, CurrentTime);

                // In case we added certificates that failed immediately, recompute, TODO: optimize
                Fail = Points.SelectMany(x => x.GetFailedCertificates());

                failedCoverCertificates = Fail
                    .Where(x => (x is SCoverCertificate || x is CoverCertificate) && !HandledCertificates.Contains(x))
                    .OrderByDescending(x => x is SCoverCertificate c ? c.K : ((CoverCertificate)x).K).ToArray();
            }

            //Console.WriteLine();
            //Console.WriteLine("--------------------------------------------------------");
            //Console.WriteLine();

            sw.Stop();
            ExecutionTimes[9].Add(sw.Elapsed);
            //Console.WriteLine("RunAlgorithmAfterSinglePointMoved - END");

            foreach (var point in Points)
            {
                point.Node.Cost.Certificate += point.RemovedCertificatesList.Count;
            }
        }
    }
}