using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
    private const float InitialStateFallbackWindow = 0.05f;

    [Header("Identity")]
    public int id = 0;

    [Header("Planning")]
    public GraphVertex goalVertex;
    public GraphVertex initialVertex;
    public RobotLocationKind initialLocationKind = RobotLocationKind.Vertex;
    public GraphVertex initialEdgeFromVertex;
    public GraphVertex initialEdgeToVertex;
    public float initialDistanceFromEdgeStart = 0f;
    public bool useSchedulerMovement = true;
    public float vertexSnapTolerance = 0.05f;

    [Header("Legacy Path Movement")]
    public List<GraphVertex> path = new List<GraphVertex>();
    public float speed = 2f; // units per second

    private int currentIndex = 0;
    private int nextIndex = 1;

    private float t = 0f; // interpolation parameter

    private RobotSchedule activeSchedule;
    private ContinuousPlanningGraph activeGraph;
    private float scheduleStartTime = 0f;
    private Vector3 firstSegmentStartPosition;
    private int activeScheduleGoalVertexId = -1;

    void Start()
    {
        if (initialLocationKind == RobotLocationKind.Edge &&
            initialEdgeFromVertex != null &&
            initialEdgeToVertex != null)
        {
            PlaceOnInitialEdge();
        }
        else if (initialVertex != null)
        {
            transform.position = initialVertex.transform.position;
        }
        else if (path != null && path.Count >= 1)
        {
            transform.position = path[0].transform.position;
        }
    }

    void Update()
    {
        if (useSchedulerMovement && activeSchedule != null)
        {
            FollowSchedule();
            return;
        }

        MoveAlongPath();
    }

    void MoveAlongPath()
    {
        if (path == null || path.Count < 2)
        {
            //Debug.LogError("Path must contain at least 2 vertices.");
            //enabled = false;
            return;
        }

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

    public bool HasGoal()
    {
        return goalVertex != null;
    }

    public int GetGoalVertexId()
    {
        return goalVertex != null ? goalVertex.id : -1;
    }

    public void SetSchedule(RobotSchedule schedule, ContinuousPlanningGraph graph)
    {
        activeSchedule = schedule;
        activeGraph = graph;
        scheduleStartTime = Time.time;
        firstSegmentStartPosition = transform.position;
        activeScheduleGoalVertexId = goalVertex != null ? goalVertex.id : -1;
    }

    public bool NeedsReplan()
    {
        if (!useSchedulerMovement || !HasGoal())
        {
            return false;
        }

        if (activeSchedule == null || activeGraph == null)
        {
            return true;
        }

        if (activeScheduleGoalVertexId != goalVertex.id)
        {
            return true;
        }

        ScheduleSegment activeSegment;
        int segmentIndex;
        return !TryGetScheduleSegmentAtTime(GetScheduleTime(), out activeSegment, out segmentIndex);
    }

    public bool TryBuildPlanningState(ContinuousPlanningGraph graph, out RobotPlanningState state)
    {
        state = null;

        if (graph == null || !HasGoal())
        {
            return false;
        }

        ScheduleSegment activeSegment;
        if (TryGetActiveScheduleSegment(out activeSegment))
        {
            if (activeSegment.type == ScheduleSegmentType.WaitAtVertex)
            {
                state = new RobotPlanningState
                {
                    robotId = id,
                    speed = speed,
                    goalVertexId = goalVertex.id,
                    locationKind = RobotLocationKind.Vertex,
                    currentVertexId = activeSegment.startVertexId
                };
                return true;
            }

            float progress = GetTraversalProgress(activeSegment);
            float edgeLength = graph.GetEdgeLength(activeSegment.startVertexId, activeSegment.endVertexId);
            if (edgeLength <= 0f)
            {
                return false;
            }

            state = new RobotPlanningState
            {
                robotId = id,
                speed = speed,
                goalVertexId = goalVertex.id,
                locationKind = RobotLocationKind.Edge,
                edgeFromVertexId = activeSegment.startVertexId,
                edgeToVertexId = activeSegment.endVertexId,
                distanceFromEdgeStart = edgeLength * progress
            };
            return true;
        }

        bool preferConfiguredInitialState =
            activeSchedule == null && Time.time <= InitialStateFallbackWindow;

        // During the very first planning pass the scheduler may run before this
        // Robot has finished applying its configured initial transform in Start.
        // In that narrow startup window the inspector configuration is the most
        // trustworthy source of truth.
        if (preferConfiguredInitialState && TryBuildConfiguredInitialState(graph, out state))
        {
            return true;
        }

        int snappedVertexId;
        if (TrySnapToVertex(graph, out snappedVertexId))
        {
            state = new RobotPlanningState
            {
                robotId = id,
                speed = speed,
                goalVertexId = goalVertex.id,
                locationKind = RobotLocationKind.Vertex,
                currentVertexId = snappedVertexId
            };
            return true;
        }

        if (TryBuildConfiguredInitialState(graph, out state))
        {
            return true;
        }

        return false;
    }

    private bool TryBuildConfiguredInitialState(ContinuousPlanningGraph graph, out RobotPlanningState state)
    {
        if (TryBuildInitialVertexState(graph, out state))
        {
            return true;
        }

        return TryBuildInitialEdgeState(graph, out state);
    }

    private bool TryBuildInitialVertexState(ContinuousPlanningGraph graph, out RobotPlanningState state)
    {
        state = null;

        if (initialLocationKind != RobotLocationKind.Vertex || initialVertex == null)
        {
            return false;
        }

        if (!graph.ContainsVertex(initialVertex.id))
        {
            return false;
        }

        state = new RobotPlanningState
        {
            robotId = id,
            speed = speed,
            goalVertexId = goalVertex.id,
            locationKind = RobotLocationKind.Vertex,
            currentVertexId = initialVertex.id
        };
        return true;
    }

    private bool TryBuildInitialEdgeState(ContinuousPlanningGraph graph, out RobotPlanningState state)
    {
        state = null;

        if (initialLocationKind != RobotLocationKind.Edge ||
            initialEdgeFromVertex == null ||
            initialEdgeToVertex == null)
        {
            return false;
        }

        float edgeLength = graph.GetEdgeLength(initialEdgeFromVertex.id, initialEdgeToVertex.id);
        if (edgeLength <= 0f)
        {
            return false;
        }

        float clampedDistance = Mathf.Clamp(initialDistanceFromEdgeStart, 0f, edgeLength);
        state = new RobotPlanningState
        {
            robotId = id,
            speed = speed,
            goalVertexId = goalVertex.id,
            locationKind = RobotLocationKind.Edge,
            edgeFromVertexId = initialEdgeFromVertex.id,
            edgeToVertexId = initialEdgeToVertex.id,
            distanceFromEdgeStart = clampedDistance
        };
        return true;
    }

    private void FollowSchedule()
    {
        if (activeSchedule == null || activeGraph == null || activeSchedule.segments == null || activeSchedule.segments.Count == 0)
        {
            return;
        }

        ScheduleSegment activeSegment;
        int segmentIndex;
        if (!TryGetScheduleSegmentAtTime(GetScheduleTime(), out activeSegment, out segmentIndex))
        {
            return;
        }

        if (activeSegment.type == ScheduleSegmentType.WaitAtVertex)
        {
            transform.position = activeGraph.GetVertexPosition(activeSegment.startVertexId);
            return;
        }

        Vector3 startPosition = segmentIndex == 0
            ? firstSegmentStartPosition
            : activeGraph.GetVertexPosition(activeSegment.startVertexId);

        Vector3 endPosition = activeGraph.GetVertexPosition(activeSegment.endVertexId);
        float progress = GetTraversalProgress(activeSegment);
        transform.position = Vector3.Lerp(startPosition, endPosition, progress);
    }

    private float GetScheduleTime()
    {
        return Time.time - scheduleStartTime;
    }

    private bool TryGetActiveScheduleSegment(out ScheduleSegment activeSegment)
    {
        int segmentIndex;
        return TryGetScheduleSegmentAtTime(GetScheduleTime(), out activeSegment, out segmentIndex);
    }

    private bool TryGetScheduleSegmentAtTime(float scheduleTime, out ScheduleSegment activeSegment, out int segmentIndex)
    {
        activeSegment = null;
        segmentIndex = -1;

        if (activeSchedule == null || activeSchedule.segments == null)
        {
            return false;
        }

        for (int i = 0; i < activeSchedule.segments.Count; i++)
        {
            ScheduleSegment segment = activeSchedule.segments[i];
            if (segment == null)
            {
                continue;
            }

            bool containsTime = scheduleTime >= segment.interval.startTime &&
                                (float.IsPositiveInfinity(segment.interval.endTime) ||
                                 scheduleTime < segment.interval.endTime);

            if (containsTime)
            {
                activeSegment = segment;
                segmentIndex = i;
                return true;
            }
        }

        return false;
    }

    private float GetTraversalProgress(ScheduleSegment segment)
    {
        float duration = segment.interval.endTime - segment.interval.startTime;
        if (duration <= 0f)
        {
            return 1f;
        }

        float elapsed = GetScheduleTime() - segment.interval.startTime;
        return Mathf.Clamp01(elapsed / duration);
    }

    private bool TrySnapToVertex(ContinuousPlanningGraph graph, out int vertexId)
    {
        vertexId = -1;

        foreach (KeyValuePair<int, PlanningVertex> pair in graph.VerticesById)
        {
            float distance = Vector3.Distance(transform.position, pair.Value.worldPosition);
            if (distance <= vertexSnapTolerance)
            {
                vertexId = pair.Key;
                return true;
            }
        }

        return false;
    }

    private void PlaceOnInitialEdge()
    {
        Vector3 startPosition = initialEdgeFromVertex.transform.position;
        Vector3 endPosition = initialEdgeToVertex.transform.position;
        float edgeLength = Vector3.Distance(startPosition, endPosition);

        if (edgeLength <= 0f)
        {
            transform.position = startPosition;
            return;
        }

        float clampedDistance = Mathf.Clamp(initialDistanceFromEdgeStart, 0f, edgeLength);
        float progress = clampedDistance / edgeLength;
        transform.position = Vector3.Lerp(startPosition, endPosition, progress);
    }
}
