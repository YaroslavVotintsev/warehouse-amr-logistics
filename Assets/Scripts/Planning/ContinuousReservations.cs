using System.Collections.Generic;

public enum ReservationConflictType
{
    Vertex,
    Edge
}

/// <summary>
/// Describes one detected scheduling conflict between two robots.
/// This is the object the future CBS layer will branch on.
/// </summary>
public class ReservationConflict
{
    public ReservationConflictType type;
    public int robotAId;
    public int robotBId;
    public TimeInterval conflictInterval;
    public TimeInterval robotAInterval;
    public TimeInterval robotBInterval;

    public int vertexId = -1;
    public PlanningEdgeKey edgeKey;
    public int robotAFromVertexId = -1;
    public int robotAToVertexId = -1;
    public int robotBFromVertexId = -1;
    public int robotBToVertexId = -1;
}

/// <summary>
/// Stores continuous-time reservations for exclusive graph resources.
/// The low-level planner will query this structure to check whether a wait or traversal is safe.
/// </summary>
public class ReservationTable
{
    // This epsilon is only for floating-point comparisons. It must stay
    // much smaller than any real reservation window, otherwise tiny event
    // reservations such as traversal vertex-touch events get rounded away.
    private const float Epsilon = 0.000001f;
    // Traversing through a vertex should block that vertex for a meaningful
    // amount of simulation time, otherwise two schedules that differ only by a
    // tiny timestamp can still look like simultaneous occupancy in Unity.
    // The padding is applied symmetrically around the touch time so that
    // "one arrives just before another departs" is still treated as a
    // bottleneck conflict instead of a visually-overlapping near miss.
    private const float VertexEventPadding = 0.1f;

    private readonly Dictionary<int, List<VertexReservation>> vertexReservationsByVertexId =
        new Dictionary<int, List<VertexReservation>>();
    private readonly Dictionary<PlanningEdgeKey, List<EdgeReservation>> edgeReservationsByEdgeKey =
        new Dictionary<PlanningEdgeKey, List<EdgeReservation>>();

    public void Clear()
    {
        vertexReservationsByVertexId.Clear();
        edgeReservationsByEdgeKey.Clear();
    }

    public void AddSchedule(RobotSchedule schedule)
    {
        if (schedule == null || schedule.segments == null)
        {
            return;
        }

        for (int i = 0; i < schedule.segments.Count; i++)
        {
            AddSegment(schedule.robotId, schedule.segments[i]);
        }
    }

    public void AddSchedules(IEnumerable<RobotSchedule> schedules)
    {
        if (schedules == null)
        {
            return;
        }

        foreach (RobotSchedule schedule in schedules)
        {
            AddSchedule(schedule);
        }
    }

    public void AddVertexReservation(VertexReservation reservation)
    {
        if (reservation == null || !reservation.interval.IsValid)
        {
            return;
        }

        List<VertexReservation> reservationsAtVertex;
        if (!vertexReservationsByVertexId.TryGetValue(reservation.vertexId, out reservationsAtVertex))
        {
            reservationsAtVertex = new List<VertexReservation>();
            vertexReservationsByVertexId.Add(reservation.vertexId, reservationsAtVertex);
        }

        reservationsAtVertex.Add(reservation);
    }

    public void AddEdgeReservation(EdgeReservation reservation)
    {
        if (reservation == null || !reservation.interval.IsValid)
        {
            return;
        }

        List<EdgeReservation> reservationsAtEdge;
        if (!edgeReservationsByEdgeKey.TryGetValue(reservation.edgeKey, out reservationsAtEdge))
        {
            reservationsAtEdge = new List<EdgeReservation>();
            edgeReservationsByEdgeKey.Add(reservation.edgeKey, reservationsAtEdge);
        }

        reservationsAtEdge.Add(reservation);
    }

    public IEnumerable<VertexReservation> GetVertexReservations(int vertexId)
    {
        List<VertexReservation> reservationsAtVertex;
        if (!vertexReservationsByVertexId.TryGetValue(vertexId, out reservationsAtVertex))
        {
            yield break;
        }

        for (int i = 0; i < reservationsAtVertex.Count; i++)
        {
            yield return reservationsAtVertex[i];
        }
    }

    public IEnumerable<EdgeReservation> GetEdgeReservations(PlanningEdgeKey edgeKey)
    {
        List<EdgeReservation> reservationsAtEdge;
        if (!edgeReservationsByEdgeKey.TryGetValue(edgeKey, out reservationsAtEdge))
        {
            yield break;
        }

        for (int i = 0; i < reservationsAtEdge.Count; i++)
        {
            yield return reservationsAtEdge[i];
        }
    }

    public bool TryGetVertexConflict(
        int robotId,
        int vertexId,
        TimeInterval interval,
        out VertexReservation conflictingReservation)
    {
        List<VertexReservation> reservationsAtVertex;
        if (vertexReservationsByVertexId.TryGetValue(vertexId, out reservationsAtVertex))
        {
            for (int i = 0; i < reservationsAtVertex.Count; i++)
            {
                VertexReservation reservation = reservationsAtVertex[i];
                if (reservation.ConflictsWith(robotId, vertexId, interval))
                {
                    conflictingReservation = reservation;
                    return true;
                }
            }
        }

        conflictingReservation = null;
        return false;
    }

    public bool TryGetEdgeConflict(
        int robotId,
        PlanningEdgeKey edgeKey,
        TimeInterval interval,
        out EdgeReservation conflictingReservation)
    {
        List<EdgeReservation> reservationsAtEdge;
        if (edgeReservationsByEdgeKey.TryGetValue(edgeKey, out reservationsAtEdge))
        {
            for (int i = 0; i < reservationsAtEdge.Count; i++)
            {
                EdgeReservation reservation = reservationsAtEdge[i];
                if (reservation.ConflictsWith(robotId, edgeKey, interval))
                {
                    conflictingReservation = reservation;
                    return true;
                }
            }
        }

        conflictingReservation = null;
        return false;
    }

    public bool IsVertexReserved(int robotId, int vertexId, TimeInterval interval)
    {
        VertexReservation conflictingReservation;
        return TryGetVertexConflict(robotId, vertexId, interval, out conflictingReservation);
    }

    public bool IsEdgeReserved(int robotId, PlanningEdgeKey edgeKey, TimeInterval interval)
    {
        EdgeReservation conflictingReservation;
        return TryGetEdgeConflict(robotId, edgeKey, interval, out conflictingReservation);
    }

    public List<TimeInterval> GetMergedVertexReservations(int vertexId, int ignoredRobotId = -1)
    {
        List<VertexReservation> reservationsAtVertex;
        if (!vertexReservationsByVertexId.TryGetValue(vertexId, out reservationsAtVertex))
        {
            return new List<TimeInterval>();
        }

        var intervals = new List<TimeInterval>();
        for (int i = 0; i < reservationsAtVertex.Count; i++)
        {
            VertexReservation reservation = reservationsAtVertex[i];
            if (reservation.robotId == ignoredRobotId)
            {
                continue;
            }

            intervals.Add(reservation.interval);
        }

        return MergeIntervals(intervals);
    }

    public List<TimeInterval> GetMergedEdgeReservations(PlanningEdgeKey edgeKey, int ignoredRobotId = -1)
    {
        List<EdgeReservation> reservationsAtEdge;
        if (!edgeReservationsByEdgeKey.TryGetValue(edgeKey, out reservationsAtEdge))
        {
            return new List<TimeInterval>();
        }

        var intervals = new List<TimeInterval>();
        for (int i = 0; i < reservationsAtEdge.Count; i++)
        {
            EdgeReservation reservation = reservationsAtEdge[i];
            if (reservation.robotId == ignoredRobotId)
            {
                continue;
            }

            intervals.Add(reservation.interval);
        }

        return MergeIntervals(intervals);
    }

    public List<SafeInterval> GetSafeVertexIntervals(int vertexId, int ignoredRobotId = -1, float startTime = 0f)
    {
        List<TimeInterval> occupiedIntervals = GetMergedVertexReservations(vertexId, ignoredRobotId);
        var safeIntervals = new List<SafeInterval>();

        float cursor = startTime;
        for (int i = 0; i < occupiedIntervals.Count; i++)
        {
            TimeInterval occupiedInterval = occupiedIntervals[i];
            if (occupiedInterval.endTime <= cursor + Epsilon)
            {
                continue;
            }

            if (occupiedInterval.startTime > cursor + Epsilon)
            {
                safeIntervals.Add(new SafeInterval(cursor, occupiedInterval.startTime));
            }

            if (occupiedInterval.endTime > cursor)
            {
                cursor = occupiedInterval.endTime;
            }
        }

        safeIntervals.Add(new SafeInterval(cursor, float.PositiveInfinity));
        return safeIntervals;
    }

    public bool TryGetSafeIntervalContaining(
        int vertexId,
        float time,
        out SafeInterval safeInterval,
        int ignoredRobotId = -1)
    {
        List<SafeInterval> safeIntervals = GetSafeVertexIntervals(vertexId, ignoredRobotId, 0f);

        for (int i = 0; i < safeIntervals.Count; i++)
        {
            SafeInterval candidate = safeIntervals[i];
            bool containsTime = time + Epsilon >= candidate.startTime &&
                                (candidate.IsUnbounded || time < candidate.endTime - Epsilon);

            if (containsTime)
            {
                safeInterval = candidate;
                return true;
            }
        }

        safeInterval = default(SafeInterval);
        return false;
    }

    public float FindEarliestEdgeAvailability(
        PlanningEdgeKey edgeKey,
        float notBeforeTime,
        float traversalDuration,
        int ignoredRobotId = -1)
    {
        List<TimeInterval> occupiedIntervals = GetMergedEdgeReservations(edgeKey, ignoredRobotId);
        float candidateStartTime = notBeforeTime;

        for (int i = 0; i < occupiedIntervals.Count; i++)
        {
            TimeInterval occupiedInterval = occupiedIntervals[i];
            if (candidateStartTime + traversalDuration <= occupiedInterval.startTime + Epsilon)
            {
                break;
            }

            if (candidateStartTime >= occupiedInterval.endTime - Epsilon)
            {
                continue;
            }

            candidateStartTime = occupiedInterval.endTime;
        }

        return candidateStartTime;
    }

    public ReservationConflict FindFirstConflict(RobotSchedule schedule)
    {
        if (schedule == null || schedule.segments == null)
        {
            return null;
        }

        ReservationConflict bestConflict = null;

        for (int i = 0; i < schedule.segments.Count; i++)
        {
            ScheduleSegment segment = schedule.segments[i];
            if (segment == null || !segment.interval.IsValid)
            {
                continue;
            }

            if (segment.type == ScheduleSegmentType.WaitAtVertex)
            {
                VertexReservation vertexConflict;
                if (TryGetVertexConflict(schedule.robotId, segment.startVertexId, segment.interval, out vertexConflict))
                {
                    ReservationConflict candidate = CreateVertexConflict(
                        schedule.robotId,
                        vertexConflict.robotId,
                        segment.startVertexId,
                        GetOverlap(segment.interval, vertexConflict.interval),
                        segment.interval,
                        GetPreferredVertexConstraintInterval(vertexConflict),
                        segment.startVertexId,
                        segment.startVertexId,
                        vertexConflict.fromVertexId,
                        vertexConflict.toVertexId);
                    bestConflict = ChooseEarlierConflict(bestConflict, candidate);
                }

                continue;
            }

            // Traversal segments also touch their endpoint vertices.
            // These event reservations are what prevent two robots from
            // "passing through" the same bottleneck vertex at effectively
            // the same instant while using different edges.
            VertexReservation startVertexConflict;
            TimeInterval traversalStartEvent = CreateVertexTouchInterval(segment.interval.startTime);
            if (TryGetVertexConflict(
                schedule.robotId,
                segment.startVertexId,
                traversalStartEvent,
                out startVertexConflict))
            {
                ReservationConflict candidate = CreateVertexConflict(
                    schedule.robotId,
                    startVertexConflict.robotId,
                    segment.startVertexId,
                    GetOverlap(traversalStartEvent, startVertexConflict.interval),
                    traversalStartEvent,
                    GetPreferredVertexConstraintInterval(startVertexConflict),
                    segment.startVertexId,
                    segment.endVertexId,
                    startVertexConflict.fromVertexId,
                    startVertexConflict.toVertexId);
                bestConflict = ChooseEarlierConflict(bestConflict, candidate);
            }

            VertexReservation endVertexConflict;
            TimeInterval traversalEndEvent = CreateVertexTouchInterval(segment.interval.endTime);
            if (TryGetVertexConflict(
                schedule.robotId,
                segment.endVertexId,
                traversalEndEvent,
                out endVertexConflict))
            {
                ReservationConflict candidate = CreateVertexConflict(
                    schedule.robotId,
                    endVertexConflict.robotId,
                    segment.endVertexId,
                    GetOverlap(traversalEndEvent, endVertexConflict.interval),
                    segment.interval,
                    GetPreferredVertexConstraintInterval(endVertexConflict),
                    segment.startVertexId,
                    segment.endVertexId,
                    endVertexConflict.fromVertexId,
                    endVertexConflict.toVertexId);
                bestConflict = ChooseEarlierConflict(bestConflict, candidate);
            }

            EdgeReservation edgeConflict;
            if (TryGetEdgeConflict(schedule.robotId, segment.edgeKey, segment.interval, out edgeConflict))
            {
                ReservationConflict candidate = new ReservationConflict
                {
                    type = ReservationConflictType.Edge,
                    robotAId = schedule.robotId,
                    robotBId = edgeConflict.robotId,
                    conflictInterval = GetOverlap(segment.interval, edgeConflict.interval),
                    robotAInterval = segment.interval,
                    robotBInterval = edgeConflict.interval,
                    edgeKey = segment.edgeKey,
                    robotAFromVertexId = segment.startVertexId,
                    robotAToVertexId = segment.endVertexId,
                    robotBFromVertexId = edgeConflict.fromVertexId,
                    robotBToVertexId = edgeConflict.toVertexId
                };
                bestConflict = ChooseEarlierConflict(bestConflict, candidate);
            }
        }

        return bestConflict;
    }

    private void AddSegment(int robotId, ScheduleSegment segment)
    {
        if (segment == null || !segment.interval.IsValid)
        {
            return;
        }

        if (segment.type == ScheduleSegmentType.WaitAtVertex)
        {
            AddVertexReservation(new VertexReservation
            {
                robotId = robotId,
                vertexId = segment.startVertexId,
                interval = segment.interval,
                movementInterval = segment.interval,
                fromVertexId = segment.startVertexId,
                toVertexId = segment.endVertexId
            });
            return;
        }

        AddEdgeReservation(new EdgeReservation
        {
            robotId = robotId,
            edgeKey = segment.edgeKey,
            fromVertexId = segment.startVertexId,
            toVertexId = segment.endVertexId,
            interval = segment.interval
        });

        // Traversal touches both endpoint vertices.
        // A tiny reservation window prevents two robots from "teleporting through"
        // the same vertex at the exact same timestamp when one arrives as another departs.
        AddVertexReservation(new VertexReservation
        {
            robotId = robotId,
            vertexId = segment.startVertexId,
            interval = CreateVertexTouchInterval(segment.interval.startTime),
            movementInterval = segment.interval,
            fromVertexId = segment.startVertexId,
            toVertexId = segment.endVertexId
        });

        AddVertexReservation(new VertexReservation
        {
            robotId = robotId,
            vertexId = segment.endVertexId,
            interval = CreateVertexTouchInterval(segment.interval.endTime),
            movementInterval = segment.interval,
            fromVertexId = segment.startVertexId,
            toVertexId = segment.endVertexId
        });
    }

    private static TimeInterval CreateVertexTouchInterval(float touchTime)
    {
        float startTime = touchTime - VertexEventPadding;
        if (startTime < 0f)
        {
            startTime = 0f;
        }

        return new TimeInterval(startTime, touchTime + VertexEventPadding);
    }

    private static ReservationConflict CreateVertexConflict(
        int robotAId,
        int robotBId,
        int vertexId,
        TimeInterval conflictInterval,
        TimeInterval robotAInterval,
        TimeInterval robotBInterval,
        int robotAFromVertexId,
        int robotAToVertexId,
        int robotBFromVertexId,
        int robotBToVertexId)
    {
        return new ReservationConflict
        {
            type = ReservationConflictType.Vertex,
            robotAId = robotAId,
            robotBId = robotBId,
            conflictInterval = conflictInterval,
            robotAInterval = robotAInterval,
            robotBInterval = robotBInterval,
            vertexId = vertexId,
            robotAFromVertexId = robotAFromVertexId,
            robotAToVertexId = robotAToVertexId,
            robotBFromVertexId = robotBFromVertexId,
            robotBToVertexId = robotBToVertexId
        };
    }

    internal static ReservationConflict ChooseEarlierConflict(
        ReservationConflict bestConflict,
        ReservationConflict candidateConflict)
    {
        if (candidateConflict == null)
        {
            return bestConflict;
        }

        if (bestConflict == null)
        {
            return candidateConflict;
        }

        if (candidateConflict.conflictInterval.startTime < bestConflict.conflictInterval.startTime - Epsilon)
        {
            return candidateConflict;
        }

        if (candidateConflict.conflictInterval.startTime > bestConflict.conflictInterval.startTime + Epsilon)
        {
            return bestConflict;
        }

        if (candidateConflict.type == ReservationConflictType.Edge &&
            bestConflict.type == ReservationConflictType.Vertex)
        {
            return candidateConflict;
        }

        if (candidateConflict.type == ReservationConflictType.Vertex &&
            bestConflict.type == ReservationConflictType.Vertex &&
            GetVertexConflictMovementScore(candidateConflict) > GetVertexConflictMovementScore(bestConflict))
        {
            return candidateConflict;
        }

        return bestConflict;
    }

    private static int GetVertexConflictMovementScore(ReservationConflict conflict)
    {
        if (conflict == null || conflict.type != ReservationConflictType.Vertex)
        {
            return 0;
        }

        int score = 0;
        if (conflict.robotAFromVertexId >= 0 && conflict.robotAToVertexId >= 0 &&
            conflict.robotAFromVertexId != conflict.robotAToVertexId)
        {
            score++;
        }

        if (conflict.robotBFromVertexId >= 0 && conflict.robotBToVertexId >= 0 &&
            conflict.robotBFromVertexId != conflict.robotBToVertexId)
        {
            score++;
        }

        return score;
    }

    private static TimeInterval GetOverlap(TimeInterval first, TimeInterval second)
    {
        float startTime = first.startTime > second.startTime ? first.startTime : second.startTime;
        float endTime = first.endTime < second.endTime ? first.endTime : second.endTime;
        return new TimeInterval(startTime, endTime);
    }

    private static TimeInterval GetPreferredVertexConstraintInterval(VertexReservation reservation)
    {
        if (reservation == null)
        {
            return default(TimeInterval);
        }

        if (!reservation.IsTraversalEvent)
        {
            return reservation.movementInterval.IsValid ? reservation.movementInterval : reservation.interval;
        }

        bool isArrivalOrPassThroughEvent = reservation.vertexId == reservation.toVertexId;
        if (isArrivalOrPassThroughEvent && reservation.movementInterval.IsValid)
        {
            return reservation.movementInterval;
        }

        return reservation.interval;
    }

    private static List<TimeInterval> MergeIntervals(List<TimeInterval> intervals)
    {
        if (intervals == null || intervals.Count == 0)
        {
            return new List<TimeInterval>();
        }

        intervals.Sort((first, second) => first.startTime.CompareTo(second.startTime));

        var merged = new List<TimeInterval>();
        TimeInterval current = intervals[0];

        for (int i = 1; i < intervals.Count; i++)
        {
            TimeInterval next = intervals[i];
            if (next.startTime <= current.endTime + Epsilon)
            {
                float mergedEndTime = next.endTime > current.endTime ? next.endTime : current.endTime;
                current = new TimeInterval(current.startTime, mergedEndTime);
                continue;
            }

            merged.Add(current);
            current = next;
        }

        merged.Add(current);
        return merged;
    }
}

/// <summary>
/// Pairwise conflict detection for robot schedules.
/// This keeps CBS-specific logic out of the time model while still making conflicts easy to inspect.
/// </summary>
public static class ContinuousConflictDetector
{
    public static ReservationConflict FindFirstConflict(IEnumerable<RobotSchedule> schedules)
    {
        if (schedules == null)
        {
            return null;
        }

        var reservationTable = new ReservationTable();
        ReservationConflict bestConflict = null;
        var orderedSchedules = new List<RobotSchedule>();

        foreach (RobotSchedule schedule in schedules)
        {
            if (schedule != null)
            {
                orderedSchedules.Add(schedule);
            }
        }

        orderedSchedules.Sort((first, second) => first.robotId.CompareTo(second.robotId));

        for (int i = 0; i < orderedSchedules.Count; i++)
        {
            RobotSchedule schedule = orderedSchedules[i];
            ReservationConflict conflict = reservationTable.FindFirstConflict(schedule);
            bestConflict = ReservationTable.ChooseEarlierConflict(bestConflict, conflict);

            reservationTable.AddSchedule(schedule);
        }

        return bestConflict;
    }
}
