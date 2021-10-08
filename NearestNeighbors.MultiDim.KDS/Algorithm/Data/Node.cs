using KDS;
using KDS.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;

#nullable enable

namespace NearestNeighbors.MultiDim.KDS.Algorithm.Data
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
        public Node? InitialData;
        private SimulationPoint<Node>? u;
        public HashSet<(SimulationPoint<Node> v, int k)> Neighbors { get; set; } = new();
        public HashSet<(SimulationPoint<Node> v, int k)> PotentialNeighbors { get; set; } = new();
        public HashSet<SimulationPoint<Node>> Children { get; set; } = new();
        private IEnumerable<string>? Old;
        private int maximumLevel;
        public int MaximumLevel
        {
            get => maximumLevel; set
            {
                maximumLevel = value;
                if (maximumLevel < 0)
                {
                    throw new Exception("Lowering a point already at level 0");
                }
            }
        }
        private SimulationPoint<Node>? parent;
        public SimulationPoint<Node> Parent
        {
            get => parent ?? u; set
            {
                if (u != null && value != null)
                {
                    //Console.WriteLine($"[PARENT] Changing Parent of {u?.Identifier} from {parent?.Identifier} to {value?.Identifier} - {ToString()} {parent?.Node} {value?.Node}");
                }
                if (parent != value)
                {
                    parent = value;
                    Cost.Structure++;
                }
                if (u != null && value != null)
                {
                    u.EnsureNoCircularRelationship();
                }
            }
        }

        public int GetNumberOfChanges()
        {
            var collection = u.GetCloseNodesUsingStructure(Constants.R, false)
                .Select(x => x.Identifier)
                .OrderBy(x => x)
                .Select(x => x.ToString()).ToList();

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
            u = (SimulationPoint<Node>)point;
        }

        public void CopyTo(Node node)
        {
            node.MaximumLevel = MaximumLevel;
            node.parent = parent;
            node.Neighbors = Neighbors.ToHashSet();
            node.PotentialNeighbors = PotentialNeighbors.ToHashSet();
            node.Children = Children.ToHashSet();
        }

        public override string ToString()
        {
            return $"([{u?.Identifier}] (X=${u?.Axis[0].Static},Y=${u?.Axis[1].Static}) Lu={MaximumLevel}, Parent={Parent.Identifier}, Neighbors=[{string.Join(",", Neighbors.Select(x => $"({x.v.Identifier},{x.k})"))}], Children=[{string.Join(",", Children.Select(x => x.Identifier))}])";
        }
    }
}
