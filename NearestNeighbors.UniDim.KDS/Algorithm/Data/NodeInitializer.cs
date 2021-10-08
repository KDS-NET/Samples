using KDS;
using KDS.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.UniDim.KDS.Algorithm.Data
{
    public class NodeInitializer : INodeInitializer<Node>
    {
        public void ComputeNodeValues(IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            CompareWithDumbAlgorithm(PointStructureList);
        }

        #region Initialize structure values
        private static void InitializePointValuesPhase1(SimulationPoint<Node> point)
        {
            // Clear any potential values
            point.Node.Neighbors.Clear();                  // Nu
            point.Node.LookoutPointPlus = null;            // lu+
            point.Node.LookoutPointMinus = null;           // lu-
            point.Node.SupervisingPlus.Clear();            // Su+
            point.Node.SupervisingMinus.Clear();           // Su-
            point.Node.LookoutPointPlusNeighbors.Clear();  // Lu+
            point.Node.LookoutPointMinusNeighbors.Clear(); // Lu-
        }

        private static void InitializePointValuesPhase2(SimulationPoint<Node> point, IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            foreach (SimulationPoint<Node> b in PointStructureList)
            {
                if (Math.Abs(b.X.Static - point.X.Static) <= Constants.R)
                {
                    point.Node.Neighbors.Add(b);
                }
            }

            SimulationPoint<Node>? w = null;

            foreach (SimulationPoint<Node> v in PointStructureList)
            {
                if (point == v)
                {
                    continue;
                }

                if (v.X.Static > point.X.Static + Constants.R)
                {
                    if (w == null || w.X.Static < v.X.Static)
                    {
                        w = v;
                    }
                }
            }
            point.Node.LookoutPointPlus = w;

            if (point.Node.LookoutPointPlus != null)
            {
                point.Node.LookoutPointPlus.Node.SupervisingPlus.Add(point);
            }

            w = null;

            foreach (SimulationPoint<Node> v in PointStructureList)
            {
                if (point == v)
                {
                    continue;
                }

                if (v.X.Static < point.X.Static - Constants.R)
                {
                    if (w == null || w.X.Static > v.X.Static)
                    {
                        w = v;
                    }
                }
            }
            point.Node.LookoutPointMinus = w;

            if (point.Node.LookoutPointMinus != null)
            {
                point.Node.LookoutPointMinus.Node.SupervisingMinus.Add(point);
            }
        }

        private static void InitializePointValuesPhase3(SimulationPoint<Node> point)
        {
            // Finalize
            if (point.Node.LookoutPointPlus != null)
            {
                foreach (SimulationPoint<Node> p in point.Node.LookoutPointPlus.Node.Neighbors)
                {
                    point.Node.LookoutPointPlusNeighbors.Add(p); // Lu+
                }
            }

            if (point.Node.LookoutPointMinus != null)
            {
                foreach (SimulationPoint<Node> p in point.Node.LookoutPointMinus.Node.Neighbors)
                {
                    point.Node.LookoutPointMinusNeighbors.Add(p); // Lu-
                }
            }
        }

        private static void CompareWithDumbAlgorithm(IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase1(it);
            }

            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase2(it, PointStructureList);
            }

            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase3(it);
            }
        }
        #endregion
    }
}
