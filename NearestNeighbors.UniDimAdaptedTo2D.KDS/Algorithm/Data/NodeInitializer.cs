using KDS;
using KDS.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm.Data
{
    public class NodeInitializer : INodeInitializer<Node>
    {
        public void ComputeNodeValues(IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            CompareWithDumbAlgorithmX(PointStructureList);
            CompareWithDumbAlgorithmY(PointStructureList);

            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                it.Node.Neighbors = it.Node.NeighborsX.Intersect(it.Node.NeighborsY).ToList();
            }
        }

        #region Initialize structure values
        private static void InitializePointValuesPhase1X(SimulationPoint<Node> point)
        {
            // Clear any potential values
            point.Node.NeighborsX.Clear();                  // Nu
            point.Node.LookoutPointPlusX = null;            // lu+
            point.Node.LookoutPointMinusX = null;           // lu-
            point.Node.SupervisingPlusX.Clear();            // Su+
            point.Node.SupervisingMinusX.Clear();           // Su-
            point.Node.LookoutPointPlusNeighborsX.Clear();  // Lu+
            point.Node.LookoutPointMinusNeighborsX.Clear(); // Lu-
        }

        private static void InitializePointValuesPhase2X(SimulationPoint<Node> point, IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            foreach (SimulationPoint<Node> b in PointStructureList)
            {
                if (Math.Abs(b.X.Static - point.X.Static) <= Constants.R)
                {
                    point.Node.NeighborsX.Add(b);
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
            point.Node.LookoutPointPlusX = w;

            if (point.Node.LookoutPointPlusX != null)
            {
                point.Node.LookoutPointPlusX.Node.SupervisingPlusX.Add(point);
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
            point.Node.LookoutPointMinusX = w;

            if (point.Node.LookoutPointMinusX != null)
            {
                point.Node.LookoutPointMinusX.Node.SupervisingMinusX.Add(point);
            }
        }

        private static void InitializePointValuesPhase3X(SimulationPoint<Node> point)
        {
            // Finalize
            if (point.Node.LookoutPointPlusX != null)
            {
                foreach (SimulationPoint<Node> p in point.Node.LookoutPointPlusX.Node.NeighborsX)
                {
                    point.Node.LookoutPointPlusNeighborsX.Add(p); // Lu+
                }
            }

            if (point.Node.LookoutPointMinusX != null)
            {
                foreach (SimulationPoint<Node> p in point.Node.LookoutPointMinusX.Node.NeighborsX)
                {
                    point.Node.LookoutPointMinusNeighborsX.Add(p); // Lu-
                }
            }
        }

        private static void CompareWithDumbAlgorithmX(IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase1X(it);
            }

            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase2X(it, PointStructureList);
            }

            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase3X(it);
            }
        }

        private static void InitializePointValuesPhase1Y(SimulationPoint<Node> point)
        {
            // Clear any potential values
            point.Node.NeighborsY.Clear();                  // Nu
            point.Node.LookoutPointPlusY = null;            // lu+
            point.Node.LookoutPointMinusY = null;           // lu-
            point.Node.SupervisingPlusY.Clear();            // Su+
            point.Node.SupervisingMinusY.Clear();           // Su-
            point.Node.LookoutPointPlusNeighborsY.Clear();  // Lu+
            point.Node.LookoutPointMinusNeighborsY.Clear(); // Lu-
        }

        private static void InitializePointValuesPhase2Y(SimulationPoint<Node> point, IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            foreach (SimulationPoint<Node> b in PointStructureList)
            {
                if (Math.Abs(b.Y.Static - point.Y.Static) <= Constants.R)
                {
                    point.Node.NeighborsY.Add(b);
                }
            }

            SimulationPoint<Node>? w = null;

            foreach (SimulationPoint<Node> v in PointStructureList)
            {
                if (point == v)
                {
                    continue;
                }

                if (v.Y.Static > point.Y.Static + Constants.R)
                {
                    if (w == null || w.Y.Static < v.Y.Static)
                    {
                        w = v;
                    }
                }
            }
            point.Node.LookoutPointPlusY = w;

            if (point.Node.LookoutPointPlusY != null)
            {
                point.Node.LookoutPointPlusY.Node.SupervisingPlusY.Add(point);
            }

            w = null;

            foreach (SimulationPoint<Node> v in PointStructureList)
            {
                if (point == v)
                {
                    continue;
                }

                if (v.Y.Static < point.Y.Static - Constants.R)
                {
                    if (w == null || w.Y.Static > v.Y.Static)
                    {
                        w = v;
                    }
                }
            }
            point.Node.LookoutPointMinusY = w;

            if (point.Node.LookoutPointMinusY != null)
            {
                point.Node.LookoutPointMinusY.Node.SupervisingMinusY.Add(point);
            }
        }

        private static void InitializePointValuesPhase3Y(SimulationPoint<Node> point)
        {
            // Finalize
            if (point.Node.LookoutPointPlusY != null)
            {
                foreach (SimulationPoint<Node> p in point.Node.LookoutPointPlusY.Node.NeighborsY)
                {
                    point.Node.LookoutPointPlusNeighborsY.Add(p); // Lu+
                }
            }

            if (point.Node.LookoutPointMinusY != null)
            {
                foreach (SimulationPoint<Node> p in point.Node.LookoutPointMinusY.Node.NeighborsY)
                {
                    point.Node.LookoutPointMinusNeighborsY.Add(p); // Lu-
                }
            }
        }

        private static void CompareWithDumbAlgorithmY(IEnumerable<SimulationPoint<Node>> PointStructureList)
        {
            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase1Y(it);
            }

            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase2Y(it, PointStructureList);
            }

            foreach (SimulationPoint<Node> it in PointStructureList)
            {
                InitializePointValuesPhase3Y(it);
            }
        }
        #endregion
    }
}
