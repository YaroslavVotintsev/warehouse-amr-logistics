using System.Collections.Generic;
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
        // find by objects by tags "Edge" and "Vertex"
        var temp = GameObject.FindGameObjectsWithTag("Vertex");
        for (int i = 0; i < temp.Length; i++)
        {
            var vertex = temp[i].GetComponent<GraphVertex>();
            vertex.id = i;
            vertices.Add(vertex.GetComponent<GraphVertex>());
        }
        temp = GameObject.FindGameObjectsWithTag("Edge");
        for (int i = 0; i < temp.Length; i++)
        {
            var edge = temp[i].GetComponent<GraphEdge>();
            edge.id = i;
            edges.Add(edge.GetComponent<GraphEdge>());
        }
    }
    void BuildGraphStructure()
    {
        foreach (var edge in edges)
        {
            var v1 = edge.vertexA.GetComponent<GraphVertex>();
            var v2 = edge.vertexB.GetComponent<GraphVertex>();

            if (v1 != null && v2 != null)
            {
                v1.AddNeighbor(v2, edge);
                v2.AddNeighbor(v1, edge);
            }
        }
    }
}
