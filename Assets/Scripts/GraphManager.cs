using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class GraphManager : MonoBehaviour
{
    public List<GraphVertex> vertices = new List<GraphVertex>();
    public List<GraphEdge> edges = new List<GraphEdge>();

    private void Awake()
    {
        FindVE();
        BuildGraphStructure();
    }

    void Start()
    {
        
    }
    void Update()
    {
        
    }

    private void FindVE()
    {
        vertices.Clear();
        edges.Clear();

        // find by objects by tags "Edge" and "Vertex"
        var temp = GameObject.FindGameObjectsWithTag("Vertex");
        for (int i = 0; i < temp.Length; i++)
        {
            var vertex = temp[i].GetComponent<GraphVertex>();
            if (vertex != null)
            {
                vertices.Add(vertex);
            }
        }
        temp = GameObject.FindGameObjectsWithTag("Edge");
        for (int i = 0; i < temp.Length; i++)
        {
            var edge = temp[i].GetComponent<GraphEdge>();
            if (edge != null)
            {
                edges.Add(edge);
            }
        }

        // Keep discovery deterministic so planning behavior does not depend on
        // the arbitrary scene enumeration order returned by tag lookups.
        vertices.Sort((first, second) =>
        {
            if (first == null && second == null)
            {
                return 0;
            }

            if (first == null)
            {
                return 1;
            }

            if (second == null)
            {
                return -1;
            }

            int idComparison = first.id.CompareTo(second.id);
            return idComparison != 0
                ? idComparison
                : string.CompareOrdinal(first.gameObject.name, second.gameObject.name);
        });

        edges.Sort((first, second) =>
        {
            if (first == null && second == null)
            {
                return 0;
            }

            if (first == null)
            {
                return 1;
            }

            if (second == null)
            {
                return -1;
            }

            int idComparison = first.id.CompareTo(second.id);
            return idComparison != 0
                ? idComparison
                : string.CompareOrdinal(first.gameObject.name, second.gameObject.name);
        });
    }
    void BuildGraphStructure()
    {
        for (int i = 0; i < vertices.Count; i++)
        {
            if (vertices[i] != null)
            {
                vertices[i].ClearNeighbors();
            }
        }

        foreach (var edge in edges)
        {
            if (edge == null || edge.vertexA == null || edge.vertexB == null)
            {
                continue;
            }

            var v1 = edge.vertexA.GetComponent<GraphVertex>();
            var v2 = edge.vertexB.GetComponent<GraphVertex>();

            if (v1 != null && v2 != null)
            {
                v1.AddNeighbor(v2, edge);
                v2.AddNeighbor(v1, edge);
            }
        }
    }

    public GraphVertex GetVertexById(int vertexId)
    {
        return vertices.Find(vertex => vertex != null && vertex.id == vertexId);
    }

    public bool ContainsVertex(int vertexId)
    {
        return GetVertexById(vertexId) != null;
    }

    public List<GraphVertex> GetNeighborVertices(int vertexId)
    {
        GraphVertex vertex = GetVertexById(vertexId);
        if (vertex == null)
        {
            return new List<GraphVertex>();
        }

        return vertex.neighborVertices.Values.ToList();
    }

    public List<int> GetNeighborVertexIds(int vertexId)
    {
        GraphVertex vertex = GetVertexById(vertexId);
        if (vertex == null)
        {
            return new List<int>();
        }

        return vertex.neighborVertices.Keys.ToList();
    }

    public bool AreAdjacent(int fromVertexId, int toVertexId)
    {
        GraphVertex fromVertex = GetVertexById(fromVertexId);
        return fromVertex != null && fromVertex.neighborVertices.ContainsKey(toVertexId);
    }

    public GraphEdge GetEdgeBetween(int fromVertexId, int toVertexId)
    {
        GraphVertex fromVertex = GetVertexById(fromVertexId);
        GraphVertex toVertex = GetVertexById(toVertexId);

        if (fromVertex == null || toVertex == null)
        {
            return null;
        }

        for (int i = 0; i < fromVertex.connectedEdges.Count; i++)
        {
            GraphEdge edge = fromVertex.connectedEdges[i];
            if (edge == null || edge.vertexA == null || edge.vertexB == null)
            {
                continue;
            }

            GraphVertex edgeVertexA = edge.vertexA.GetComponent<GraphVertex>();
            GraphVertex edgeVertexB = edge.vertexB.GetComponent<GraphVertex>();

            if (edgeVertexA == null || edgeVertexB == null)
            {
                continue;
            }

            bool matchesForward = edgeVertexA.id == fromVertexId && edgeVertexB.id == toVertexId;
            bool matchesReverse = edgeVertexA.id == toVertexId && edgeVertexB.id == fromVertexId;
            if (matchesForward || matchesReverse)
            {
                return edge;
            }
        }

        return null;
    }
}
