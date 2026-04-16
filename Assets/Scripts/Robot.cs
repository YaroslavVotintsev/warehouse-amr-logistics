using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
    public List<GraphVertex> path = new List<GraphVertex>();
    public float speed = 2f; // units per second

    private int currentIndex = 0;
    private int nextIndex = 1;

    private float t = 0f; // interpolation parameter

    void Start()
    {
        if (path == null || path.Count < 2)
        {
            Debug.LogError("Path must contain at least 2 vertices.");
            enabled = false;
            return;
        }

        // Start at first vertex
        transform.position = path[0].transform.position;
    }

    void Update()
    {
        MoveAlongPath();
    }

    void MoveAlongPath()
    {
        GraphVertex currentVertex = path[currentIndex];
        GraphVertex nextVertex = path[nextIndex];

        Vector3 startPos = currentVertex.transform.position;
        Vector3 endPos = nextVertex.transform.position;

        float distance = Vector3.Distance(startPos, endPos);

        // Avoid division by zero
        if (distance == 0f) return;

        // Move with constant speed
        t += (speed / distance) * Time.deltaTime;

        transform.position = Vector3.Lerp(startPos, endPos, t);

        // When reached next vertex
        if (t >= 1f)
        {
            t = 0f;
            currentIndex = nextIndex;
            nextIndex = nextIndex + 1 < path.Count ? nextIndex + 1 : nextIndex;
        }
    }
}