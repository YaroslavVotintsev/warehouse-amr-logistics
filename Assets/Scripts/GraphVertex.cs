using System.Collections.Generic;
using UnityEngine;

public class GraphVertex : MonoBehaviour
{
    public int id;
    public List<GraphEdge> connectedEdges = new List<GraphEdge>();
    public Dictionary<int, GraphVertex> neighborVertices = new Dictionary<int, GraphVertex>();


    void Start()
    {
        
    }

    
    void Update()
    {
        
    }

    public void AddNeighbor(GraphVertex neighbor, GraphEdge edge)
    {
        if (!neighborVertices.ContainsKey(neighbor.id))
        {
            neighborVertices.Add(neighbor.id, neighbor);
            connectedEdges.Add(edge);
        }
    }
}
