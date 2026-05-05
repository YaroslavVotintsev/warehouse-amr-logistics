using System.Collections.Generic;
using System.Text;
using UnityEngine;

/// <summary>
/// Shared formatting helpers for planning diagnostics.
/// Keeping these summaries in one place makes failure logs easier to compare
/// across scheduler, low-level planning, and CBS search.
/// </summary>
public static class PlanningDebugUtility
{
    public static string FormatGraph(ContinuousPlanningGraph graph)
    {
        if (graph == null)
        {
            return "graph=<null>";
        }

        return "graph(vertices=" + graph.VerticesById.Count +
               ", edges=" + graph.EdgesByKey.Count + ")";
    }

    public static string FormatRobotState(
        RobotPlanningState robotState,
        ContinuousPlanningGraph graph = null)
    {
        if (robotState == null)
        {
            return "robotState=<null>";
        }

        if (robotState.IsAtVertex)
        {
            return "robot " + robotState.robotId +
                   " at V" + robotState.currentVertexId +
                   " goal=V" + robotState.goalVertexId +
                   " speed=" + robotState.speed.ToString("0.###");
        }

        float edgeLength = graph != null
            ? graph.GetEdgeLength(robotState.edgeFromVertexId, robotState.edgeToVertexId)
            : -1f;

        string edgeLengthText = edgeLength > 0f
            ? "/" + edgeLength.ToString("0.###")
            : string.Empty;

        return "robot " + robotState.robotId +
               " on E" + robotState.edgeFromVertexId + "-" + robotState.edgeToVertexId +
               " d=" + robotState.distanceFromEdgeStart.ToString("0.###") + edgeLengthText +
               " goal=V" + robotState.goalVertexId +
               " speed=" + robotState.speed.ToString("0.###");
    }

    public static string FormatRobotStates(
        IList<RobotPlanningState> robotStates,
        ContinuousPlanningGraph graph = null,
        int maxCount = 8)
    {
        if (robotStates == null)
        {
            return "robotStates=<null>";
        }

        var builder = new StringBuilder();
        builder.Append("robotStates(count=");
        builder.Append(robotStates.Count);
        builder.Append(')');

        int count = 0;
        for (int i = 0; i < robotStates.Count; i++)
        {
            if (robotStates[i] == null)
            {
                continue;
            }

            if (count == 0)
            {
                builder.Append(':');
            }

            builder.Append("\n- ");
            builder.Append(FormatRobotState(robotStates[i], graph));
            count++;

            if (count >= maxCount)
            {
                if (robotStates.Count > count)
                {
                    builder.Append("\n- ...");
                }

                break;
            }
        }

        return builder.ToString();
    }

    public static string FormatConflict(ReservationConflict conflict)
    {
        if (conflict == null)
        {
            return "conflict=<null>";
        }

        if (conflict.type == ReservationConflictType.Vertex)
        {
            return "Vertex V" + conflict.vertexId +
                   " @" + FormatInterval(conflict.conflictInterval) +
                   " r" + conflict.robotAId + "[" + conflict.robotAFromVertexId + "->" + conflict.robotAToVertexId + "]" +
                   " vs r" + conflict.robotBId + "[" + conflict.robotBFromVertexId + "->" + conflict.robotBToVertexId + "]";
        }

        return "Edge E" + conflict.edgeKey.vertexMinId + "-" + conflict.edgeKey.vertexMaxId +
               " @" + FormatInterval(conflict.conflictInterval) +
               " r" + conflict.robotAId + "[" + conflict.robotAFromVertexId + "->" + conflict.robotAToVertexId + "]" +
               " vs r" + conflict.robotBId + "[" + conflict.robotBFromVertexId + "->" + conflict.robotBToVertexId + "]";
    }

    public static string FormatConstraint(ContinuousCbsConstraint constraint)
    {
        if (constraint == null)
        {
            return "<null constraint>";
        }

        if (constraint.type == ReservationConflictType.Vertex)
        {
            return "r" + constraint.robotId +
                   " forbid V" + constraint.vertexId +
                   " @" + FormatInterval(constraint.interval);
        }

        return "r" + constraint.robotId +
               " forbid E" + constraint.edgeKey.vertexMinId + "-" + constraint.edgeKey.vertexMaxId +
               " @" + FormatInterval(constraint.interval);
    }

    public static string FormatConstraints(
        IList<ContinuousCbsConstraint> constraints,
        int maxCount = 10)
    {
        if (constraints == null)
        {
            return "constraints=<null>";
        }

        var builder = new StringBuilder();
        builder.Append("constraints(count=");
        builder.Append(constraints.Count);
        builder.Append(')');

        int written = 0;
        for (int i = 0; i < constraints.Count; i++)
        {
            ContinuousCbsConstraint constraint = constraints[i];
            if (constraint == null)
            {
                continue;
            }

            if (written == 0)
            {
                builder.Append(':');
            }

            builder.Append("\n- ");
            builder.Append(FormatConstraint(constraint));
            written++;

            if (written >= maxCount)
            {
                if (constraints.Count > written)
                {
                    builder.Append("\n- ...");
                }

                break;
            }
        }

        return builder.ToString();
    }

    public static string FormatSchedule(RobotSchedule schedule, int maxSegments = 8)
    {
        if (schedule == null)
        {
            return "schedule=<null>";
        }

        var builder = new StringBuilder();
        builder.Append("schedule r");
        builder.Append(schedule.robotId);
        builder.Append(" completion=");
        builder.Append(schedule.CompletionTime.ToString("0.###"));
        builder.Append(" segments=");
        builder.Append(schedule.segments != null ? schedule.segments.Count : 0);

        if (schedule.segments == null || schedule.segments.Count == 0)
        {
            return builder.ToString();
        }

        int count = 0;
        for (int i = 0; i < schedule.segments.Count; i++)
        {
            ScheduleSegment segment = schedule.segments[i];
            if (segment == null)
            {
                continue;
            }

            builder.Append("\n- ");
            builder.Append(FormatSegment(segment));
            count++;

            if (count >= maxSegments)
            {
                if (schedule.segments.Count > count)
                {
                    builder.Append("\n- ...");
                }

                break;
            }
        }

        return builder.ToString();
    }

    public static string FormatSchedules(
        IDictionary<int, RobotSchedule> schedulesByRobotId,
        int maxRobots = 6,
        int maxSegmentsPerRobot = 4)
    {
        if (schedulesByRobotId == null)
        {
            return "schedules=<null>";
        }

        var builder = new StringBuilder();
        builder.Append("schedules(count=");
        builder.Append(schedulesByRobotId.Count);
        builder.Append(')');

        var orderedRobotIds = new List<int>(schedulesByRobotId.Keys);
        orderedRobotIds.Sort();

        int written = 0;
        for (int i = 0; i < orderedRobotIds.Count; i++)
        {
            int robotId = orderedRobotIds[i];
            RobotSchedule schedule;
            if (!schedulesByRobotId.TryGetValue(robotId, out schedule))
            {
                continue;
            }

            builder.Append("\n- ");
            builder.Append(FormatSchedule(schedule, maxSegmentsPerRobot));
            written++;

            if (written >= maxRobots)
            {
                if (schedulesByRobotId.Count > written)
                {
                    builder.Append("\n- ...");
                }

                break;
            }
        }

        return builder.ToString();
    }

    public static string FormatReservationSnapshot(
        ReservationTable reservations,
        RobotPlanningState robotState,
        ContinuousPlanningGraph graph)
    {
        if (reservations == null)
        {
            return "reservations=<null>";
        }

        if (robotState == null)
        {
            return "reservationSnapshot(robot=<null>)";
        }

        var builder = new StringBuilder();
        builder.Append("reservationSnapshot for ");
        builder.Append(FormatRobotState(robotState, graph));

        if (robotState.IsAtVertex)
        {
            builder.Append("\n- start blocked intervals at V");
            builder.Append(robotState.currentVertexId);
            builder.Append(": ");
            builder.Append(FormatTimeIntervals(
                reservations.GetMergedVertexReservations(robotState.currentVertexId, robotState.robotId),
                6));

            builder.Append("\n- start safe intervals at V");
            builder.Append(robotState.currentVertexId);
            builder.Append(": ");
            builder.Append(FormatSafeIntervals(
                reservations.GetSafeVertexIntervals(robotState.currentVertexId, robotState.robotId),
                6));
        }
        else
        {
            builder.Append("\n- current edge E");
            builder.Append(robotState.edgeFromVertexId);
            builder.Append('-');
            builder.Append(robotState.edgeToVertexId);
            builder.Append(" reservations: ");
            builder.Append(FormatTimeIntervals(
                reservations.GetMergedEdgeReservations(
                    new PlanningEdgeKey(robotState.edgeFromVertexId, robotState.edgeToVertexId),
                    robotState.robotId),
                6));

            builder.Append("\n- destination safe intervals at V");
            builder.Append(robotState.edgeToVertexId);
            builder.Append(": ");
            builder.Append(FormatSafeIntervals(
                reservations.GetSafeVertexIntervals(robotState.edgeToVertexId, robotState.robotId),
                6));

            builder.Append("\n- destination blocked intervals at V");
            builder.Append(robotState.edgeToVertexId);
            builder.Append(": ");
            builder.Append(FormatTimeIntervals(
                reservations.GetMergedVertexReservations(robotState.edgeToVertexId, robotState.robotId),
                6));
        }

        builder.Append("\n- goal blocked intervals at V");
        builder.Append(robotState.goalVertexId);
        builder.Append(": ");
        builder.Append(FormatTimeIntervals(
            reservations.GetMergedVertexReservations(robotState.goalVertexId, robotState.robotId),
            6));

        builder.Append("\n- goal safe intervals at V");
        builder.Append(robotState.goalVertexId);
        builder.Append(": ");
        builder.Append(FormatSafeIntervals(
            reservations.GetSafeVertexIntervals(robotState.goalVertexId, robotState.robotId),
            6));

        return builder.ToString();
    }

    public static string FormatRobotComponent(Robot robot)
    {
        if (robot == null)
        {
            return "robotComponent=<null>";
        }

        Vector3 position = robot.transform.position;
        string location =
            robot.initialLocationKind == RobotLocationKind.Vertex
                ? "initialVertex=" + (robot.initialVertex != null ? "V" + robot.initialVertex.id : "<null>")
                : "initialEdge=" +
                  (robot.initialEdgeFromVertex != null ? "V" + robot.initialEdgeFromVertex.id : "<null>") +
                  "->" +
                  (robot.initialEdgeToVertex != null ? "V" + robot.initialEdgeToVertex.id : "<null>") +
                  " d=" + robot.initialDistanceFromEdgeStart.ToString("0.###");

        return "robotObject=" + robot.name +
               " id=" + robot.id +
               " pos=" + position.ToString("F3") +
               " goal=" + (robot.goalVertex != null ? "V" + robot.goalVertex.id : "<null>") +
               " " + location +
               " speed=" + robot.speed.ToString("0.###") +
               " schedulerMovement=" + robot.useSchedulerMovement;
    }

    public static string FormatInterval(TimeInterval interval)
    {
        return interval.startTime.ToString("0.###") +
               ".." +
               (float.IsPositiveInfinity(interval.endTime)
                   ? "inf"
                   : interval.endTime.ToString("0.###"));
    }

    private static string FormatSegment(ScheduleSegment segment)
    {
        if (segment.type == ScheduleSegmentType.WaitAtVertex)
        {
            return "wait V" + segment.startVertexId + " @" + FormatInterval(segment.interval);
        }

        return "move V" + segment.startVertexId + "->V" + segment.endVertexId +
               " @" + FormatInterval(segment.interval);
    }

    private static string FormatSafeIntervals(IList<SafeInterval> intervals, int maxCount)
    {
        if (intervals == null || intervals.Count == 0)
        {
            return "<none>";
        }

        var builder = new StringBuilder();
        for (int i = 0; i < intervals.Count && i < maxCount; i++)
        {
            if (i > 0)
            {
                builder.Append(", ");
            }

            builder.Append(intervals[i].startTime.ToString("0.###"));
            builder.Append("..");
            builder.Append(float.IsPositiveInfinity(intervals[i].endTime)
                ? "inf"
                : intervals[i].endTime.ToString("0.###"));
        }

        if (intervals.Count > maxCount)
        {
            builder.Append(", ...");
        }

        return builder.ToString();
    }

    private static string FormatTimeIntervals(IList<TimeInterval> intervals, int maxCount)
    {
        if (intervals == null || intervals.Count == 0)
        {
            return "<none>";
        }

        var builder = new StringBuilder();
        for (int i = 0; i < intervals.Count && i < maxCount; i++)
        {
            if (i > 0)
            {
                builder.Append(", ");
            }

            builder.Append(FormatInterval(intervals[i]));
        }

        if (intervals.Count > maxCount)
        {
            builder.Append(", ...");
        }

        return builder.ToString();
    }
}
