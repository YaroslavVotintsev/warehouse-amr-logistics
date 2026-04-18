using System;
using System.Collections.Generic;

/// <summary>
/// Closed-open interval [start, end) used for continuous-time occupancy checks.
/// This is a convenient default for scheduling because it makes back-to-back usage explicit.
/// </summary>
public struct TimeInterval : IEquatable<TimeInterval>
{
    private const float HashQuantization = 0.0001f;

    public float startTime;
    public float endTime;

    public TimeInterval(float startTime, float endTime)
    {
        this.startTime = startTime;
        this.endTime = endTime;
    }

    public float Duration
    {
        get
        {
            return endTime - startTime;
        }
    }

    public bool IsValid
    {
        get
        {
            return endTime >= startTime;
        }
    }

    public bool Overlaps(TimeInterval other)
    {
        return startTime < other.endTime && other.startTime < endTime;
    }

    public bool Contains(float time)
    {
        return time >= startTime && time < endTime;
    }

    public bool Equals(TimeInterval other)
    {
        return Math.Abs(startTime - other.startTime) < 0.0001f &&
               ((float.IsPositiveInfinity(endTime) && float.IsPositiveInfinity(other.endTime)) ||
                Math.Abs(endTime - other.endTime) < 0.0001f);
    }

    public override bool Equals(object obj)
    {
        return obj is TimeInterval other && Equals(other);
    }

    public override int GetHashCode()
    {
        unchecked
        {
            long startKey = QuantizeForHash(startTime);
            long endKey = QuantizeForHash(endTime);
            return (startKey.GetHashCode() * 397) ^ endKey.GetHashCode();
        }
    }

    public override string ToString()
    {
        return "[" + startTime + ", " + endTime + ")";
    }

    private static long QuantizeForHash(float time)
    {
        if (float.IsPositiveInfinity(time))
        {
            return long.MaxValue;
        }

        return (long)Math.Round(time / HashQuantization);
    }
}

/// <summary>
/// A collision-free interval during which a robot may occupy a vertex.
/// The low-level planner searches over these intervals instead of raw timestamps.
/// </summary>
public struct SafeInterval : IEquatable<SafeInterval>
{
    private const float HashQuantization = 0.0001f;

    public float startTime;
    public float endTime;

    public SafeInterval(float startTime, float endTime)
    {
        this.startTime = startTime;
        this.endTime = endTime;
    }

    public bool Contains(float time)
    {
        return time >= startTime && time < endTime;
    }

    public bool IsUnbounded
    {
        get
        {
            return float.IsPositiveInfinity(endTime);
        }
    }

    public bool Equals(SafeInterval other)
    {
        return Math.Abs(startTime - other.startTime) < 0.0001f &&
               ((float.IsPositiveInfinity(endTime) && float.IsPositiveInfinity(other.endTime)) ||
                Math.Abs(endTime - other.endTime) < 0.0001f);
    }

    public override bool Equals(object obj)
    {
        return obj is SafeInterval other && Equals(other);
    }

    public override int GetHashCode()
    {
        unchecked
        {
            long startKey = QuantizeForHash(startTime);
            long endKey = QuantizeForHash(endTime);
            return (startKey.GetHashCode() * 397) ^ endKey.GetHashCode();
        }
    }

    public override string ToString()
    {
        return "[" + startTime + ", " + endTime + ")";
    }

    private static long QuantizeForHash(float time)
    {
        if (float.IsPositiveInfinity(time))
        {
            return long.MaxValue;
        }

        return (long)Math.Round(time / HashQuantization);
    }
}

public enum RobotLocationKind
{
    Vertex,
    Edge
}

/// <summary>
/// Continuous-time snapshot of a robot at replanning time.
/// The robot may be waiting at a vertex or already moving along an edge.
/// </summary>
public class RobotPlanningState
{
    public int robotId;
    public float speed;
    public int goalVertexId;
    public RobotLocationKind locationKind;

    // Used when the robot is exactly at a vertex.
    public int currentVertexId = -1;

    // Used when the robot is currently somewhere on an edge.
    public int edgeFromVertexId = -1;
    public int edgeToVertexId = -1;
    public float distanceFromEdgeStart = 0f;

    public bool IsAtVertex
    {
        get
        {
            return locationKind == RobotLocationKind.Vertex;
        }
    }

    public bool IsOnEdge
    {
        get
        {
            return locationKind == RobotLocationKind.Edge;
        }
    }

    public bool IsValid(ContinuousPlanningGraph graph)
    {
        if (graph == null || speed <= 0f || !graph.ContainsVertex(goalVertexId))
        {
            return false;
        }

        if (IsAtVertex)
        {
            return graph.ContainsVertex(currentVertexId);
        }

        PlanningEdge edge;
        if (!graph.TryGetEdge(edgeFromVertexId, edgeToVertexId, out edge))
        {
            return false;
        }

        return distanceFromEdgeStart >= 0f && distanceFromEdgeStart <= edge.length;
    }

    public float GetRemainingTravelTimeOnCurrentEdge(ContinuousPlanningGraph graph)
    {
        if (!IsOnEdge)
        {
            return 0f;
        }

        PlanningEdge edge;
        if (!graph.TryGetEdge(edgeFromVertexId, edgeToVertexId, out edge))
        {
            return float.PositiveInfinity;
        }

        float remainingDistance = edge.length - distanceFromEdgeStart;
        return remainingDistance / speed;
    }
}

/// <summary>
/// Reservation of a vertex resource for a time interval.
/// Only one robot may occupy a vertex at a time in the chosen model.
/// </summary>
public class VertexReservation
{
    public int robotId;
    public int vertexId;
    public TimeInterval interval;
    // The tiny reservation interval is used for pairwise collision checks, while
    // movementInterval preserves the whole underlying schedule action that caused
    // the reservation. CBS can then branch on the full local occupancy instead
    // of only on an epsilon-sized overlap.
    public TimeInterval movementInterval;
    public int fromVertexId = -1;
    public int toVertexId = -1;

    public bool ConflictsWith(int otherRobotId, int otherVertexId, TimeInterval otherInterval)
    {
        return robotId != otherRobotId &&
               vertexId == otherVertexId &&
               interval.Overlaps(otherInterval);
    }

    public bool IsTraversalEvent
    {
        get
        {
            return fromVertexId >= 0 && toVertexId >= 0 && fromVertexId != toVertexId;
        }
    }
}

/// <summary>
/// Reservation of an edge resource for a time interval.
/// Edges are exclusive resources, so direction is stored for diagnostics only.
/// </summary>
public class EdgeReservation
{
    public int robotId;
    public PlanningEdgeKey edgeKey;
    public int fromVertexId;
    public int toVertexId;
    public TimeInterval interval;

    public bool ConflictsWith(int otherRobotId, PlanningEdgeKey otherEdgeKey, TimeInterval otherInterval)
    {
        return robotId != otherRobotId &&
               edgeKey.Equals(otherEdgeKey) &&
               interval.Overlaps(otherInterval);
    }
}

public enum ScheduleSegmentType
{
    WaitAtVertex,
    TraverseEdge
}

/// <summary>
/// One action in a continuous-time schedule.
/// Waiting is only allowed at vertices in the simplified motion model.
/// </summary>
public class ScheduleSegment
{
    public ScheduleSegmentType type;
    public TimeInterval interval;

    public int startVertexId = -1;
    public int endVertexId = -1;
    public PlanningEdgeKey edgeKey;

    public static ScheduleSegment CreateWait(int vertexId, float startTime, float endTime)
    {
        return new ScheduleSegment
        {
            type = ScheduleSegmentType.WaitAtVertex,
            interval = new TimeInterval(startTime, endTime),
            startVertexId = vertexId,
            endVertexId = vertexId
        };
    }

    public static ScheduleSegment CreateTraversal(int fromVertexId, int toVertexId, float startTime, float endTime)
    {
        return new ScheduleSegment
        {
            type = ScheduleSegmentType.TraverseEdge,
            interval = new TimeInterval(startTime, endTime),
            startVertexId = fromVertexId,
            endVertexId = toVertexId,
            edgeKey = new PlanningEdgeKey(fromVertexId, toVertexId)
        };
    }
}

/// <summary>
/// Continuous-time route assigned to a robot.
/// Later CBS layers will build and compare these schedules.
/// </summary>
public class RobotSchedule
{
    public int robotId;
    public List<ScheduleSegment> segments = new List<ScheduleSegment>();

    public float StartTime
    {
        get
        {
            return segments.Count > 0 ? segments[0].interval.startTime : 0f;
        }
    }

    public float EndTime
    {
        get
        {
            return segments.Count > 0 ? segments[segments.Count - 1].interval.endTime : 0f;
        }
    }

    public float CompletionTime
    {
        get
        {
            if (segments.Count == 0)
            {
                return 0f;
            }

            ScheduleSegment lastSegment = segments[segments.Count - 1];
            if (lastSegment.type == ScheduleSegmentType.WaitAtVertex &&
                float.IsPositiveInfinity(lastSegment.interval.endTime))
            {
                return lastSegment.interval.startTime;
            }

            return lastSegment.interval.endTime;
        }
    }

    public int GetFinalVertexId()
    {
        return segments.Count > 0 ? segments[segments.Count - 1].endVertexId : -1;
    }
}
