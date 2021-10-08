using KDS;
using KDS.Certificates;
using NearestNeighbors.MultiDim.KDS.Algorithm.Data;
using System;
using System.Linq;

namespace NearestNeighbors.MultiDim.KDS.Algorithm.Certificates
{
    public class SCoverCertificate : BaseCertificate<Node>
    {
        public NodeLinkLabel Label { get; }

        public int K { get; }

        public SCoverCertificate(SimulationPoint<Node> u, SimulationPoint<Node> v, double t, int k, NodeLinkLabel label) : base(u, v, t)
        {
            K = k; Label = label; if (k <= 0 || label < 0)
            {
                throw new Exception("Forbidden");
            }
            if (u == v)
            {
                throw new Exception("Forbidden");
            }
        }

        public override bool EvaluateValidity(double CurrentTime)
        {
            if (GetFailureTimeAtCreation() != null)
            {
                return CurrentTime < GetFailureTimeAtCreation();
            }

            double D = GetU().Distance(GetV());
            return D >= Constants.LabelValueDict[Label] * Math.Pow(Constants.b, K + 1);
        }

        public override double? GetFailureTime(double CurrentTime)
        {
            MathNet.Numerics.Polynomial D = GetU().SquareDistance(GetV());
            double T = Constants.LabelValueDict[Label] * Math.Pow(Constants.b, K + 1);

            MathNet.Numerics.Polynomial expr = D - (T * T);

            System.Numerics.Complex[] roots = expr.Roots();
            if (roots.Any(x => x.Imaginary == 0 && x.Real >= CurrentTime))
            {
                return roots.Where(x => x.Imaginary == 0 && x.Real >= CurrentTime).Min(x => x.Real);
            }

            return null;
        }

        /*public override bool Equals(object obj)
        {
            return obj is SCoverCertificate c && c.K == K && ((c.GetU() == GetU() && c.GetV() == GetV()) || (c.GetU() == GetV() && c.GetV() == GetU())) && Label == c.Label;
        }

        public override int GetHashCode()
        {
            return (Label.GetHashCode() + 2) * (K.GetHashCode() + 1) * (GetU().GetHashCode() * 40 + GetV().GetHashCode() * 60) * 3;
        }*/

        public override string ToString()
        {
            return this.GetCertificateString();
        }
    }
}
