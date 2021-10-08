using KDS;
using KDS.Certificates;
using NearestNeighbors.UniDim.KDS.Algorithm.Data;
using System.Linq;

namespace NearestNeighbors.UniDim.KDS.Algorithm.Certificates
{
    public class LegitimateLookoutPlusCertificate : BaseCertificate<Node>
    {
        public LegitimateLookoutPlusCertificate(SimulationPoint<Node> u, SimulationPoint<Node> v, double t) : base(u, v, t) { }

        public static double? LessThanCertificate(SimulationPoint<Node> a, SimulationPoint<Node> b, double R, double CurrentTime)
        {
            MathNet.Numerics.Polynomial expr = R - (b.X.Pol - a.X.Pol);
            System.Numerics.Complex[] roots = expr.Roots();
            if (roots.Any(x => x.Imaginary == 0 && x.Real >= CurrentTime))
            {
                return roots.Where(x => x.Imaginary == 0 && x.Real >= CurrentTime).Min(x => x.Real);
            }

            return null;
        }

        public static double? DistancedByLessCertificate(SimulationPoint<Node> U, SimulationPoint<Node> V, double R, double CurrentTime)
        {
            MathNet.Numerics.Polynomial expr = (R * R) - ((V.X.Pol - U.X.Pol) * (V.X.Pol - U.X.Pol));
            System.Numerics.Complex[] roots = expr.Roots();
            if (roots.Any(x => x.Imaginary == 0 && x.Real >= CurrentTime))
            {
                return roots.Where(x => x.Imaginary == 0 && x.Real >= CurrentTime).Min(x => x.Real);
            }

            return null;
        }

        public static double? Between1Certificate(SimulationPoint<Node> a, SimulationPoint<Node> middle, double R, double CurrentTime)
        {
            MathNet.Numerics.Polynomial expr = middle.X.Pol - (a.X.Pol + R);
            System.Numerics.Complex[] roots = expr.Roots();
            if (roots.Any(x => x.Imaginary == 0 && x.Real >= CurrentTime))
            {
                return roots.Where(x => x.Imaginary == 0 && x.Real >= CurrentTime).Min(x => x.Real);
            }

            return null;
        }

        public static double? Between2Certificate(SimulationPoint<Node> middle, SimulationPoint<Node> b, double R, double CurrentTime)
        {
            MathNet.Numerics.Polynomial expr = b.X.Pol - R - middle.X.Pol;
            System.Numerics.Complex[] roots = expr.Roots();
            if (roots.Any(x => x.Imaginary == 0 && x.Real >= CurrentTime))
            {
                return roots.Where(x => x.Imaginary == 0 && x.Real >= CurrentTime).Min(x => x.Real);
            }

            return null;
        }

        public override double? GetFailureTime(double CurrentTime)
        {
            if (GetU().Node.LookoutPointPlus == null)
            {
                return LessThanCertificate(GetU(), GetV(), Constants.R, CurrentTime);
            }

            double upos = GetU().X.Position;
            double lupos = GetU().Node.LookoutPointPlus.X.Position;

            double? s = DistancedByLessCertificate(GetU(), GetU().Node.LookoutPointPlus, 2 * Constants.R, CurrentTime);
            double? s1 = Between1Certificate(GetU(), GetV(), Constants.R, CurrentTime);
            double? s2 = Between2Certificate(GetV(), GetU().Node.LookoutPointPlus, Constants.R, CurrentTime);

            // It is possible that the range is empty til a certain time, if that is the
            // case, compute when it will start being valid and add the certificate
            if (upos + Constants.R > lupos - Constants.R)
            {
                if (s != null)
                {
                    s1 = Between1Certificate(GetU(), GetV(), Constants.R, s.Value);
                    s2 = Between2Certificate(GetV(), GetU().Node.LookoutPointPlus, Constants.R, s.Value);
                }
                else
                {
                    return null;
                }
            }

            if (s1 != null && s2 != null)
            {
                if (s1 < s2)
                {
                    return s1;
                }
                else
                {
                    return s2;
                }
            }
            else if (s1 != null)
            {
                return s1;
            }
            else if (s2 != null)
            {
                return s2;
            }

            return null;
        }

        public override bool EvaluateValidity(double CurrentTime)
        {
            if (GetFailureTimeAtCreation() != null)
            {
                return CurrentTime < GetFailureTimeAtCreation();
            }

            if (GetU().Node.LookoutPointPlus == null)
            {
                return GetV().X.Position <= GetU().X.Position + Constants.R;
            }
            else
            {
                double a = GetU().X.Position + Constants.R;
                double b = GetU().Node.LookoutPointPlus.X.Position - Constants.R;
                if (a < b)
                {
                    return GetV().X.Position < a || GetV().X.Position > b;
                }
                else
                {
                    return true;
                }
            }
        }
    }
}
