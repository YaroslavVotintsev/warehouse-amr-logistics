using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Undirected key for an edge resource.
/// The scheduler treats an edge as an exclusive shared resource regardless of traversal direction.
/// </summary>
public struct PlanningEdgeKey : IEquatable<PlanningEdgeKey>
{
    public int vertexMinId;
    public int vertexMaxId;

    public PlanningEdgeKey(int firstVertexId, int secondVertexId)
    {
        if (firstVertexId <= secondVertexId)
        {
            vertexMinId = firstVertexId;
            vertexMaxId = secondVertexId;
        }
        else
        {
            vertexMinId = secondVertexId;
            vertexMaxId = firstVertexId;
        }
    }

    public bool ContainsVertex(int vertexId)
    {
        return vertexMinId == vertexId || vertexMaxId == vertexId;
    }

    public int GetOtherVertex(int vertexId)
    {
        if (vertexMinId == vertexId)
        {
            return vertexMaxId;
        }

        if (vertexMaxId == vertexId)
        {
            return vertexMinId;
        }

        return -1;
    }

    public bool Equals(PlanningEdgeKey other)
    {
        return vertexMinId == other.vertexMinId && vertexMaxId == other.vertexMaxId;
    }

    public override bool Equals(object obj)
    {
        return obj is PlanningEdgeKey other && Equals(other);
    }

    public override int GetHashCode()
    {
        unchecked
        {
            return (vertexMinId * 397) ^ vertexMaxId;
        }
    }

    public override string ToString()
    {
        return "[" + vertexMinId + " <-> " + vertexMaxId + "]";
    }
}

/// <summary>
/// Lightweight immutable view of a graph vertex for planning code.
/// </summary>
public class PlanningVertex
{
    public int id;
    public GraphVertex source;
    public Vector3 worldPosition;
}

/// <summary>
/// Lightweight immutable view of a graph edge for planning code.
/// </summary>
public class PlanningEdge
{
    public int id;
    public GraphEdge source;
    public int vertexAId;
    public int vertexBId;
    public float length;
    public PlanningEdgeKey key;

    public bool Connects(int firstVertexId, int secondVertexId)
    {
        return key.Equals(new PlanningEdgeKey(firstVertexId, secondVertexId));
    }

    public int GetOtherVertex(int vertexId)
    {
        return key.GetOtherVertex(vertexId);
    }

    public float GetTravelTime(float speed)
    {
        if (speed <= 0f)
        {
            return float.PositiveInfinity;
        }

        return length / speed;
    }
}

/// <summary>
/// Read-only planning view over the scene graph.
/// It exposes the graph in a way that is convenient for continuous-time scheduling:
/// - vertices by id
/// - undirected exclusive edge resources
/// - edge lengths and travel times
/// </summary>
public class ContinuousPlanningGraph
{
    private readonly Dictionary<int, PlanningVertex> verticesById = new Dictionary<int, PlanningVertex>();
    private readonly Dictionary<PlanningEdgeKey, PlanningEdge> edgesByKey = new Dictionary<PlanningEdgeKey, PlanningEdge>();
    private readonly Dictionary<int, List<PlanningEdge>> incidentEdgesByVertexId = new Dictionary<int, List<PlanningEdge>>();

    public IReadOnlyDictionary<int, PlanningVertex> VerticesById
    {
        get
        {
            return verticesById;
        }
    }

    public IReadOnlyDictionary<PlanningEdgeKey, PlanningEdge> EdgesByKey
    {
        get
        {
            return edgesByKey;
        }
    }

    public static ContinuousPlanningGraph Build(GraphManager graphManager)
    {
        if (graphManager == null)
        {
            Debug.LogError("Cannot build planning graph from a null GraphManager.");
            return null;
        }

        var graph = new ContinuousPlanningGraph();
        string error;
        if (!graph.BuildVertices(graphManager.vertices, out error) ||
            !graph.BuildEdges(graphManager.edges, out error))
        {
            Debug.LogError(error);
            return null;
        }

        return graph;
    }

    public bool ContainsVertex(int vertexId)
    {
        return verticesById.ContainsKey(vertexId);
    }

    public PlanningVertex GetVertex(int vertexId)
    {
        PlanningVertex vertex;
        verticesById.TryGetValue(vertexId, out vertex);
        return vertex;
    }

    public IEnumerable<int> GetNeighborVertexIds(int vertexId)
    {
        List<PlanningEdge> incidentEdges;
        if (!incidentEdgesByVertexId.TryGetValue(vertexId, out incidentEdges))
        {
            yield break;
        }

        for (int i = 0; i < incidentEdges.Count; i++)
        {
            int neighborId = incidentEdges[i].GetOtherVertex(vertexId);
            if (neighborId >= 0)
            {
                yield return neighborId;
            }
        }
    }

    public IEnumerable<PlanningEdge> GetIncidentEdges(int vertexId)
    {
        List<PlanningEdge> incidentEdges;
        if (!incidentEdgesByVertexId.TryGetValue(vertexId, out incidentEdges))
        {
            yield break;
        }

        for (int i = 0; i < incidentEdges.Count; i++)
        {
            yield return incidentEdges[i];
        }
    }

    public int GetVertexDegree(int vertexId)
    {
        List<PlanningEdge> incidentEdges;
        if (!incidentEdgesByVertexId.TryGetValue(vertexId, out incidentEdges))
        {
            return 0;
        }

        return incidentEdges.Count;
    }

    public bool TryGetEdge(int firstVertexId, int secondVertexId, out PlanningEdge edge)
    {
        return edgesByKey.TryGetValue(new PlanningEdgeKey(firstVertexId, secondVertexId), out edge);
    }

    public float GetEdgeLength(int firstVertexId, int secondVertexId)
    {
        PlanningEdge edge;
        if (!TryGetEdge(firstVertexId, secondVertexId, out edge))
        {
            return -1f;
        }

        return edge.length;
    }

    public float GetTravelTime(int firstVertexId, int secondVertexId, float speed)
    {
        PlanningEdge edge;
        if (!TryGetEdge(firstVertexId, secondVertexId, out edge))
        {
            return float.PositiveInfinity;
        }

        return edge.GetTravelTime(speed);
    }

    public Vector3 GetVertexPosition(int vertexId)
    {
        PlanningVertex vertex = GetVertex(vertexId);
        return vertex != null ? vertex.worldPosition : Vector3.zero;
    }

    private bool BuildVertices(List<GraphVertex> sourceVertices, out string error)
    {
        verticesById.Clear();
        incidentEdgesByVertexId.Clear();
        error = null;

        if (sourceVertices == null)
        {
            error = "Planning graph build failed because GraphManager.vertices is null.";
            return false;
        }

        for (int i = 0; i < sourceVertices.Count; i++)
        {
            GraphVertex sourceVertex = sourceVertices[i];
            if (sourceVertex == null)
            {
                error = "Planning graph build failed because GraphManager.vertices contains a null entry.";
                return false;
            }

            if (verticesById.ContainsKey(sourceVertex.id))
            {
                PlanningVertex existingVertex = verticesById[sourceVertex.id];
                string existingName =
                    existingVertex != null && existingVertex.source != null
                        ? existingVertex.source.gameObject.name
                        : "<unknown>";
                error =
                    "Planning graph build failed because vertex id " + sourceVertex.id +
                    " is duplicated by scene objects '" + existingName +
                    "' and '" + sourceVertex.gameObject.name + "'.";
                return false;
            }

            verticesById.Add(sourceVertex.id, new PlanningVertex
            {
                id = sourceVertex.id,
                source = sourceVertex,
                worldPosition = sourceVertex.transform.position
            });

            incidentEdgesByVertexId.Add(sourceVertex.id, new List<PlanningEdge>());
        }

        return true;
    }

    private bool BuildEdges(List<GraphEdge> sourceEdges, out string error)
    {
        edgesByKey.Clear();
        error = null;

        if (sourceEdges == null)
        {
            error = "Planning graph build failed because GraphManager.edges is null.";
            return false;
        }

        for (int i = 0; i < sourceEdges.Count; i++)
        {
            GraphEdge sourceEdge = sourceEdges[i];
            if (sourceEdge == null)
            {
                error = "Planning graph build failed because GraphManager.edges contains a null entry.";
                return false;
            }

            if (sourceEdge.vertexA == null || sourceEdge.vertexB == null)
            {
                error =
                    "Planning graph build failed because edge '" + sourceEdge.gameObject.name +
                    "' does not reference both endpoint transforms.";
                return false;
            }

            GraphVertex vertexA = sourceEdge.vertexA.GetComponent<GraphVertex>();
            GraphVertex vertexB = sourceEdge.vertexB.GetComponent<GraphVertex>();
            if (vertexA == null || vertexB == null)
            {
                error =
                    "Planning graph build failed because edge '" + sourceEdge.gameObject.name +
                    "' references transforms without GraphVertex components.";
                return false;
            }

            if (vertexA.id == vertexB.id)
            {
                error =
                    "Planning graph build failed because edge '" + sourceEdge.gameObject.name +
                    "' is a self-loop on vertex id " + vertexA.id + ".";
                return false;
            }

            if (!ContainsVertex(vertexA.id) || !ContainsVertex(vertexB.id))
            {
                error =
                    "Planning graph build failed because edge '" + sourceEdge.gameObject.name +
                    "' references vertex ids that are not present in the planning graph.";
                return false;
            }

            PlanningEdgeKey edgeKey = new PlanningEdgeKey(vertexA.id, vertexB.id);
            if (edgesByKey.ContainsKey(edgeKey))
            {
                PlanningEdge existingEdge = edgesByKey[edgeKey];
                string existingName =
                    existingEdge != null && existingEdge.source != null
                        ? existingEdge.source.gameObject.name
                        : "<unknown>";
                error =
                    "Planning graph build failed because undirected edge " + edgeKey +
                    " is duplicated by scene objects '" + existingName +
                    "' and '" + sourceEdge.gameObject.name + "'.";
                return false;
            }

            float edgeLength = sourceEdge.length > 0d
                ? (float)sourceEdge.length
                : Vector3.Distance(vertexA.transform.position, vertexB.transform.position);

            if (edgeLength <= 0f)
            {
                error =
                    "Planning graph build failed because edge '" + sourceEdge.gameObject.name +
                    "' has non-positive length.";
                return false;
            }

            PlanningEdge edge = new PlanningEdge
            {
                id = sourceEdge.id,
                source = sourceEdge,
                vertexAId = vertexA.id,
                vertexBId = vertexB.id,
                length = edgeLength,
                key = edgeKey
            };

            edgesByKey.Add(edgeKey, edge);
            incidentEdgesByVertexId[vertexA.id].Add(edge);
            incidentEdgesByVertexId[vertexB.id].Add(edge);
        }

        return true;
    }
}
