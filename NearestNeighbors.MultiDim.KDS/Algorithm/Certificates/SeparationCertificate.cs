using KDS;
using KDS.Certificates;
using NearestNeighbors.MultiDim.KDS.Algorithm.Data;
using System;
using System.Linq;

namespace NearestNeighbors.MultiDim.KDS.Algorithm.Certificates
{
    public class SeparationCertificate : BaseCertificate<Node>
    {
        public int K { get; }

        public SeparationCertificate(SimulationPoint<Node> u, SimulationPoint<Node> v, double t, int k) : base(u, v, t)
        {
            K = k;
            if (k <= 0)
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
            return D >= Constants.Gamma * Math.Pow(Constants.b, K);
        }

        public override double? GetFailureTime(double CurrentTime)
        {
            MathNet.Numerics.Polynomial D = GetU().SquareDistance(GetV());
            double T = Constants.Gamma * Math.Pow(Constants.b, K);

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
            return obj is SeparationCertificate c && c.K == K && ((c.GetU() == GetU() && c.GetV() == GetV()) || (c.GetU() == GetV() && c.GetV() == GetU()));
        }

        public override int GetHashCode()
        {
            return (K.GetHashCode() + 1) * (GetU().GetHashCode() * 40 + GetV().GetHashCode() * 60) * 4;
        }*/

        public override string ToString()
        {
            return this.GetCertificateString();
        }
    }
}
