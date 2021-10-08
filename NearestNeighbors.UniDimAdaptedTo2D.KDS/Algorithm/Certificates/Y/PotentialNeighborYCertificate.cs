using KDS;
using KDS.Certificates;
using NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm.Data;
using System;
using System.Linq;

namespace NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm.Certificates
{
    public class PotentialNeighborYCertificate : BaseCertificate<Node>
    {
        public PotentialNeighborYCertificate(SimulationPoint<Node> u, SimulationPoint<Node> v, double t) : base(u, v, t) { }

        public override double? GetFailureTime(double CurrentTime)
        {
            MathNet.Numerics.Polynomial expr = ((GetV().Y.Pol - GetU().Y.Pol) * (GetV().Y.Pol - GetU().Y.Pol)) - (Constants.R * Constants.R);
            System.Numerics.Complex[] roots = expr.Roots();
            if (roots.Any(x => x.Imaginary == 0 && x.Real >= CurrentTime))
            {
                return roots.Where(x => x.Imaginary == 0 && x.Real >= CurrentTime).Min(x => x.Real);
            }

            return null;
        }

        public override bool EvaluateValidity(double CurrentTime)
        {
            if (GetFailureTimeAtCreation() != null)
            {
                return CurrentTime < GetFailureTimeAtCreation();
            }

            return Math.Abs(GetU().Y.Position - GetV().Y.Position) > Constants.R;
        }
    }
}
