using KDS;
using KDS.Interfaces;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.UniDimAdaptedTo2D.KDS.Algorithm.Data
{
    public class Cost
    {
        public double Certificate { get; set; } = 0;
        public double Structure { get; set; } = 0;
        public double Queries { get; set; } = 0;
    }

    public class Node : INode
    {
        public Cost Cost { get; } = new();

        // Temporary variables for the first running algorithm
        public SimulationPoint<Node>? newLookoutPlusX = null;
        public SimulationPoint<Node>? newLookoutMinusX = null;
        public SimulationPoint<Node>? newLookoutPlusY = null;
        public SimulationPoint<Node>? newLookoutMinusY = null;

        public SimulationPoint<Node>? AttachedSimulationPoint;

        public Node()
        {
        }

        public Node PreviousData { get; set; }

        /// <summary>
        /// The list of neighbors
        /// </summary>
        public List<SimulationPoint<Node>> Neighbors { get; set; } = new();

        /// <summary>
        /// The lookout point on the right
        /// </summary>
        public SimulationPoint<Node>? LookoutPointPlus { get; set; }

        /// <summary>
        /// The lookout point on the left
        /// </summary>
        public SimulationPoint<Node>? LookoutPointMinus { get; set; }

        /// <summary>
        /// The lookout point on the right neighbors
        /// </summary>
        public List<SimulationPoint<Node>> LookoutPointPlusNeighbors { get; set; } = new();

        /// <summary>
        /// The lookout point on the left neighbors
        /// </summary>
        public List<SimulationPoint<Node>> LookoutPointMinusNeighbors { get; set; } = new();

        /// <summary>
        /// The points where this point is the right lookout
        /// </summary>
        public List<SimulationPoint<Node>> SupervisingPlus { get; set; } = new();

        /// <summary>
        /// The points where this point is the left lookout
        /// </summary>
        public List<SimulationPoint<Node>> SupervisingMinus { get; set; } = new();

        /// <summary>
        /// The list of neighbors
        /// </summary>
        public List<SimulationPoint<Node>> NeighborsX { get; set; } = new();

        /// <summary>
        /// The lookout point on the right
        /// </summary>
        public SimulationPoint<Node>? LookoutPointPlusX { get; set; }

        /// <summary>
        /// The lookout point on the left
        /// </summary>
        public SimulationPoint<Node>? LookoutPointMinusX { get; set; }

        /// <summary>
        /// The lookout point on the right neighbors
        /// </summary>
        public List<SimulationPoint<Node>> LookoutPointPlusNeighborsX { get; set; } = new();

        /// <summary>
        /// The lookout point on the left neighbors
        /// </summary>
        public List<SimulationPoint<Node>> LookoutPointMinusNeighborsX { get; set; } = new();

        /// <summary>
        /// The points where this point is the right lookout
        /// </summary>
        public List<SimulationPoint<Node>> SupervisingPlusX { get; set; } = new();

        /// <summary>
        /// The points where this point is the left lookout
        /// </summary>
        public List<SimulationPoint<Node>> SupervisingMinusX { get; set; } = new();

        /// <summary>
        /// The list of neighbors
        /// </summary>
        public List<SimulationPoint<Node>> NeighborsY { get; set; } = new();

        /// <summary>
        /// The lookout point on the right
        /// </summary>
        public SimulationPoint<Node>? LookoutPointPlusY { get; set; }

        /// <summary>
        /// The lookout point on the left
        /// </summary>
        public SimulationPoint<Node>? LookoutPointMinusY { get; set; }

        /// <summary>
        /// The lookout point on the right neighbors
        /// </summary>
        public List<SimulationPoint<Node>> LookoutPointPlusNeighborsY { get; set; } = new();

        /// <summary>
        /// The lookout point on the left neighbors
        /// </summary>
        public List<SimulationPoint<Node>> LookoutPointMinusNeighborsY { get; set; } = new();

        /// <summary>
        /// The points where this point is the right lookout
        /// </summary>
        public List<SimulationPoint<Node>> SupervisingPlusY { get; set; } = new();

        /// <summary>
        /// The points where this point is the left lookout
        /// </summary>
        public List<SimulationPoint<Node>> SupervisingMinusY { get; set; } = new();

        public void CopyTo(Node node)
        {
            node.LookoutPointMinus = LookoutPointMinus;
            node.LookoutPointPlus = LookoutPointPlus;
            node.Neighbors = Neighbors.ToList();
            node.LookoutPointMinusNeighbors = LookoutPointMinusNeighbors.ToList();
            node.LookoutPointPlusNeighbors = LookoutPointPlusNeighbors.ToList();
            node.SupervisingMinus = SupervisingMinus.ToList();
            node.SupervisingPlus = SupervisingPlus.ToList();

            node.LookoutPointMinusX = LookoutPointMinusX;
            node.LookoutPointPlusX = LookoutPointPlusX;
            node.NeighborsX = NeighborsX.ToList();
            node.LookoutPointMinusNeighborsX = LookoutPointMinusNeighborsX.ToList();
            node.LookoutPointPlusNeighborsX = LookoutPointPlusNeighborsX.ToList();
            node.SupervisingMinusX = SupervisingMinusX.ToList();
            node.SupervisingPlusX = SupervisingPlusX.ToList();

            node.LookoutPointMinusY = LookoutPointMinusY;
            node.LookoutPointPlusY = LookoutPointPlusY;
            node.NeighborsY = NeighborsY.ToList();
            node.LookoutPointMinusNeighborsY = LookoutPointMinusNeighborsY.ToList();
            node.LookoutPointPlusNeighborsY = LookoutPointPlusNeighborsY.ToList();
            node.SupervisingMinusY = SupervisingMinusY.ToList();
            node.SupervisingPlusY = SupervisingPlusY.ToList();
        }

        public void CopyTo(INode source)
        {
            CopyTo((Node)source);
        }

        private IEnumerable<string>? Old;

        public int GetNumberOfChanges()
        {
            var collection = Neighbors.OrderBy(x => x.Identifier).Select(x => x.Identifier.ToString()).ToList();

            int changes = collection.Count();
            if (Old != null)
            {
                changes = 0;

                (HashSet<string> added, HashSet<string> removed, HashSet<string> _) = Utils.CompareLists(Old, collection);
                changes += added.Count;
                changes += removed.Count;
            }
            Old = collection;
            return changes;
        }

        public void SetAttachedSimulationPoint(object point)
        {
            AttachedSimulationPoint = (SimulationPoint<Node>)point;
        }
    }
}
