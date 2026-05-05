using System.Collections.Generic;
using Mapf.Authoring;
using Mapf.Core.Graph;
using UnityEngine;

namespace TaskPlanning
{
    public sealed class RoadmapDistanceService
    {
        private readonly Dictionary<MapfNode, int> _nodeIds;
        private readonly RoadmapGraph _graph;

        public RoadmapDistanceService(MapfSceneGraph sceneGraph)
        {
            _graph = sceneGraph.BuildSnapshot(out _nodeIds);
        }

        public MapfNode NearestNode(Vector3 position)
        {
            MapfNode best = null;
            var bestDistance = float.PositiveInfinity;
            foreach (var node in _nodeIds.Keys)
            {
                var distance = Vector2.Distance(position, node.transform.position);
                if (distance >= bestDistance)
                    continue;

                best = node;
                bestDistance = distance;
            }

            return best;
        }

        public double Distance(MapfNode from, MapfNode to)
        {
            if (from == null || to == null)
                return double.PositiveInfinity;

            if (!_nodeIds.TryGetValue(from, out var source) || !_nodeIds.TryGetValue(to, out var target))
                return double.PositiveInfinity;

            if (source == target)
                return 0;

            return ShortestPathDistance(source, target);
        }

        private double ShortestPathDistance(int source, int target)
        {
            var distances = new double[_graph.Count];
            var visited = new bool[_graph.Count];
            for (var i = 0; i < distances.Length; i++)
                distances[i] = double.PositiveInfinity;

            distances[source] = 0;
            for (var step = 0; step < _graph.Count; step++)
            {
                var current = -1;
                var best = double.PositiveInfinity;
                for (var i = 0; i < distances.Length; i++)
                {
                    if (visited[i] || distances[i] >= best)
                        continue;

                    current = i;
                    best = distances[i];
                }

                if (current < 0)
                    break;

                if (current == target)
                    return distances[current];

                visited[current] = true;
                foreach (var edge in _graph.GetNeighbors(current))
                {
                    var candidate = distances[current] + edge.Length;
                    if (candidate < distances[edge.To])
                        distances[edge.To] = candidate;
                }
            }

            return double.PositiveInfinity;
        }
    }
}
