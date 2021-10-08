using System.Collections.Generic;

namespace NearestNeighbors.MultiDim.KDS.Algorithm
{
    public enum NodeLinkLabel
    {
        Gamma = 0,
        Beta = 1,
        Alpha = 2,
        Undefined = 3
    }

    public static class Constants
    {
        public const uint R = 100;

        // Algorithm constants
        public const int b = 6;
        public const double Alpha = (b - 2d) / b;
        public const double Beta = (b - 3d) / b;
        public const double Gamma = (b - 4d) / b;

        public static Dictionary<NodeLinkLabel, double> LabelValueDict { get; } = new()
        {
            { NodeLinkLabel.Gamma, Gamma },
            { NodeLinkLabel.Beta, Beta },
            { NodeLinkLabel.Alpha, Alpha },
            { NodeLinkLabel.Undefined, -1 }
        };
    }
}
