using UnityEngine;

public class GraphEdge : MonoBehaviour
{
    public int id;

    public Transform vertexA;
    public Transform vertexB;

    public double length = 0;

    private LineRenderer lineRenderer;

    void Start()
    {
        if (vertexA == null || vertexB == null)
        {
            Debug.LogError("GraphEdge '" + gameObject.name + "' requires both endpoint transforms.");
            return;
        }

        // check for LineRenderer and add if does not exist
        lineRenderer = GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
        }

        // if length is zero assign distance between vertices
        if (length == 0)
        {
            length = Vector3.Distance(vertexA.position, vertexB.position);
        }

        lineRenderer.SetPosition(0, vertexA.position);
        lineRenderer.SetPosition(1, vertexB.position);
    }

    void Update()
    {
    }

    // Optional: Visualize in Scene view even when not playing
    void OnDrawGizmos()
    {
        lineRenderer = GetComponent<LineRenderer>();
        if (vertexA != null && vertexB != null && lineRenderer != null)
        {
            Gizmos.color = lineRenderer.startColor;
            Gizmos.DrawLine(vertexA.position, vertexB.position);
        }
    }
}
