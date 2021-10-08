using KDS;
using KDS.Certificates;
using NearestNeighbors.MultiDim.KDS.Algorithm.Data;
using System;
using System.Linq;

namespace NearestNeighbors.MultiDim.KDS.Algorithm.Certificates
{
    public class LongEdgeCertificate : BaseCertificate<Node>
    {
        public int K { get; }

        public LongEdgeCertificate(SimulationPoint<Node> u, SimulationPoint<Node> v, double t, int k) : base(u, v, t)
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
            return D >= 2 * Math.Pow(Constants.b, K);
        }

        public override double? GetFailureTime(double CurrentTime)
        {
            MathNet.Numerics.Polynomial D = GetU().SquareDistance(GetV());
            double T = 2 * Math.Pow(Constants.b, K);

            MathNet.Numerics.Polynomial expr = D - (T * T);

            System.Numerics.Complex[] roots = expr.Roots();
            if (roots.Any(x => x.Imaginary == 0 && x.Real >= CurrentTime))
            {
                return roots.Where(x => x.Imaginary == 0 && x.Real >= CurrentTime).Min(x => x.Real);
            }

            return null;
        }

        public override string ToString()
        {
            return this.GetCertificateString();
        }
    }
}
