using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Planner output wrapper.
/// This keeps the public API simple for runtime code: either we get schedules or a concrete failure reason.
/// </summary>
public class ContinuousPlanningResult
{
    public bool success;
    public bool partialSuccess;
    public string failureReason;
    public Dictionary<int, RobotSchedule> schedulesByRobotId = new Dictionary<int, RobotSchedule>();
    public List<int> unscheduledRobotIds = new List<int>();
}

/// <summary>
/// End-to-end entry point for multi-robot continuous-time planning.
/// It validates the current world snapshot, builds the planning graph, and runs the CBS solver.
/// </summary>
public class ContinuousMultiRobotPlanner
{
    private const float Epsilon = 0.0001f;

    public ContinuousPlanningResult Plan(GraphManager graphManager, IList<RobotPlanningState> robotStates)
    {
        var result = new ContinuousPlanningResult();

        ContinuousPlanningGraph graph = ContinuousPlanningGraph.Build(graphManager);
        if (graph == null)
        {
            result.failureReason = "Failed to build planning graph.";
            Debug.LogWarning(
                "ContinuousMultiRobotPlanner could not build a planning graph from GraphManager " +
                (graphManager != null ? graphManager.name : "<null>") + ".");
            return result;
        }

        return Plan(graph, robotStates);
    }

    public ContinuousPlanningResult Plan(ContinuousPlanningGraph graph, IList<RobotPlanningState> robotStates)
    {
        var result = new ContinuousPlanningResult();

        if (graph == null)
        {
            result.failureReason = "Planner requires a valid planning graph snapshot.";
            Debug.LogError(result.failureReason);
            return result;
        }

        string validationError = ValidateRobotStates(graph, robotStates);
        if (!string.IsNullOrEmpty(validationError))
        {
            result.failureReason = validationError;
            Debug.LogWarning(
                "ContinuousMultiRobotPlanner validation failed: " + result.failureReason +
                "\n" +
                PlanningDebugUtility.FormatGraph(graph) +
                "\n" + PlanningDebugUtility.FormatRobotStates(robotStates, graph));
            return result;
        }

        var solver = new ContinuousConflictBasedSearch(graph);
        Dictionary<int, RobotSchedule> schedules = solver.FindSchedules(robotStates);
        if (schedules == null)
        {
            string fullFailureReason = !string.IsNullOrEmpty(solver.LastFailureReason)
                ? solver.LastFailureReason
                : "No conflict-free schedule could be found for the current robot states.";

            BestEffortPlanningAttempt bestEffortAttempt =
                FindBestEffortSchedules(graph, robotStates);
            if (bestEffortAttempt != null && bestEffortAttempt.schedulesByRobotId.Count > 0)
            {
                result.partialSuccess = true;
                result.failureReason =
                    "Full CBS planning failed: " + fullFailureReason +
                    " Best-effort fallback scheduled " +
                    bestEffortAttempt.schedulesByRobotId.Count + " of " + robotStates.Count +
                    " robots; the remaining robots will hold and retry on the next replan.";
                result.schedulesByRobotId = bestEffortAttempt.schedulesByRobotId;
                result.unscheduledRobotIds = bestEffortAttempt.unscheduledRobotIds;

                Debug.LogWarning(
                    "ContinuousMultiRobotPlanner applied best-effort fallback after solver failure.\n" +
                    "fullFailureReason: " + fullFailureReason +
                    "\nselectedOrder: " + bestEffortAttempt.orderDescription +
                    "\n" + PlanningDebugUtility.FormatGraph(graph) +
                    "\n" + PlanningDebugUtility.FormatRobotStates(robotStates, graph) +
                    "\n" + PlanningDebugUtility.FormatSchedules(result.schedulesByRobotId, 6, 3) +
                    "\nunscheduledRobotIds: " + FormatRobotIdList(result.unscheduledRobotIds));
                return result;
            }

            result.failureReason = fullFailureReason;
            Debug.LogWarning(
                "ContinuousMultiRobotPlanner solver failed: " + result.failureReason +
                "\n" + PlanningDebugUtility.FormatGraph(graph) +
                "\n" + PlanningDebugUtility.FormatRobotStates(robotStates, graph));
            return result;
        }

        result.success = true;
        result.schedulesByRobotId = schedules;
        return result;
    }

    private string ValidateRobotStates(ContinuousPlanningGraph graph, IList<RobotPlanningState> robotStates)
    {
        if (robotStates == null || robotStates.Count == 0)
        {
            return "Planner requires at least one robot state.";
        }

        var seenRobotIds = new HashSet<int>();
        var occupiedVertices = new Dictionary<int, int>();
        var occupiedEdges = new Dictionary<PlanningEdgeKey, int>();

        for (int i = 0; i < robotStates.Count; i++)
        {
            RobotPlanningState robotState = robotStates[i];
            if (robotState == null)
            {
                return "Robot state list contains a null entry.";
            }

            if (!seenRobotIds.Add(robotState.robotId))
            {
                return "Duplicate robot id detected: " + robotState.robotId + ".";
            }

            if (!robotState.IsValid(graph))
            {
                return "Robot " + robotState.robotId + " has an invalid planning state.";
            }

            if (robotState.IsAtVertex)
            {
                int occupyingRobotId;
                if (occupiedVertices.TryGetValue(robotState.currentVertexId, out occupyingRobotId))
                {
                    return "Robots " + occupyingRobotId + " and " + robotState.robotId +
                           " both start on vertex " + robotState.currentVertexId + ".";
                }

                occupiedVertices.Add(robotState.currentVertexId, robotState.robotId);
                continue;
            }

            PlanningEdgeKey edgeKey = new PlanningEdgeKey(robotState.edgeFromVertexId, robotState.edgeToVertexId);
            int edgeOccupyingRobotId;
            if (occupiedEdges.TryGetValue(edgeKey, out edgeOccupyingRobotId))
            {
                return "Robots " + edgeOccupyingRobotId + " and " + robotState.robotId +
                       " both start on edge " + edgeKey + ".";
            }

            occupiedEdges.Add(edgeKey, robotState.robotId);

            // Treat robots that are effectively sitting on an endpoint as occupying that vertex.
            if (robotState.distanceFromEdgeStart <= Epsilon)
            {
                int occupyingRobotOnStartVertex;
                if (occupiedVertices.TryGetValue(robotState.edgeFromVertexId, out occupyingRobotOnStartVertex))
                {
                    return "Robot " + robotState.robotId + " starts at the same vertex as robot " +
                           occupyingRobotOnStartVertex + ".";
                }

                occupiedVertices.Add(robotState.edgeFromVertexId, robotState.robotId);
                continue;
            }

            PlanningEdge edge;
            if (graph.TryGetEdge(robotState.edgeFromVertexId, robotState.edgeToVertexId, out edge) &&
                edge.length - robotState.distanceFromEdgeStart <= Epsilon)
            {
                int occupyingRobotOnEndVertex;
                if (occupiedVertices.TryGetValue(robotState.edgeToVertexId, out occupyingRobotOnEndVertex))
                {
                    return "Robot " + robotState.robotId + " starts at the same vertex as robot " +
                           occupyingRobotOnEndVertex + ".";
                }

                occupiedVertices.Add(robotState.edgeToVertexId, robotState.robotId);
            }
        }

        return null;
    }

    private BestEffortPlanningAttempt FindBestEffortSchedules(
        ContinuousPlanningGraph graph,
        IList<RobotPlanningState> robotStates)
    {
        if (graph == null || robotStates == null || robotStates.Count == 0)
        {
            return null;
        }

        Dictionary<int, float> unconstrainedCompletionTimes =
            BuildUnconstrainedCompletionTimes(graph, robotStates);
        List<BestEffortPlanningOrder> candidateOrders =
            BuildBestEffortPlanningOrders(robotStates, unconstrainedCompletionTimes);

        BestEffortPlanningAttempt bestAttempt = null;
        for (int i = 0; i < candidateOrders.Count; i++)
        {
            BestEffortPlanningAttempt attempt =
                BuildBestEffortAttemptForOrder(graph, robotStates, candidateOrders[i]);

            if (IsBetterBestEffortAttempt(attempt, bestAttempt))
            {
                bestAttempt = attempt;
            }
        }

        return bestAttempt;
    }

    private Dictionary<int, float> BuildUnconstrainedCompletionTimes(
        ContinuousPlanningGraph graph,
        IList<RobotPlanningState> robotStates)
    {
        var completionTimes = new Dictionary<int, float>();
        var emptyReservations = new ReservationTable();

        for (int i = 0; i < robotStates.Count; i++)
        {
            RobotPlanningState robotState = robotStates[i];
            if (robotState == null)
            {
                continue;
            }

            var planner = new ContinuousTimePathPlanner(graph, emptyReservations);
            RobotSchedule schedule = planner.FindSchedule(robotState);
            completionTimes[robotState.robotId] =
                schedule != null ? schedule.CompletionTime : float.PositiveInfinity;
        }

        return completionTimes;
    }

    private List<BestEffortPlanningOrder> BuildBestEffortPlanningOrders(
        IList<RobotPlanningState> robotStates,
        Dictionary<int, float> unconstrainedCompletionTimes)
    {
        var orders = new List<BestEffortPlanningOrder>();
        var seenSignatures = new HashSet<string>();

        AddBestEffortOrder(
            orders,
            seenSignatures,
            robotStates,
            "scene order");

        var shortestFirst = new List<RobotPlanningState>(robotStates);
        shortestFirst.Sort((first, second) =>
        {
            int edgePriorityComparison = first.IsOnEdge == second.IsOnEdge
                ? 0
                : (first.IsOnEdge ? -1 : 1);
            if (edgePriorityComparison != 0)
            {
                return edgePriorityComparison;
            }

            float firstTime = GetUnconstrainedCompletionTime(unconstrainedCompletionTimes, first.robotId);
            float secondTime = GetUnconstrainedCompletionTime(unconstrainedCompletionTimes, second.robotId);
            int completionComparison = firstTime.CompareTo(secondTime);
            if (completionComparison != 0)
            {
                return completionComparison;
            }

            return first.robotId.CompareTo(second.robotId);
        });
        AddBestEffortOrder(
            orders,
            seenSignatures,
            shortestFirst,
            "on-edge first, shortest unconstrained path first");

        var longestFirst = new List<RobotPlanningState>(robotStates);
        longestFirst.Sort((first, second) =>
        {
            float firstTime = GetUnconstrainedCompletionTime(unconstrainedCompletionTimes, first.robotId);
            float secondTime = GetUnconstrainedCompletionTime(unconstrainedCompletionTimes, second.robotId);
            int completionComparison = secondTime.CompareTo(firstTime);
            if (completionComparison != 0)
            {
                return completionComparison;
            }

            return first.robotId.CompareTo(second.robotId);
        });
        AddBestEffortOrder(
            orders,
            seenSignatures,
            longestFirst,
            "longest unconstrained path first");

        var idOrder = new List<RobotPlanningState>(robotStates);
        idOrder.Sort((first, second) => first.robotId.CompareTo(second.robotId));
        AddBestEffortOrder(
            orders,
            seenSignatures,
            idOrder,
            "robot id order");

        return orders;
    }

    private void AddBestEffortOrder(
        List<BestEffortPlanningOrder> orders,
        HashSet<string> seenSignatures,
        IList<RobotPlanningState> orderedStates,
        string description)
    {
        if (orders == null || seenSignatures == null || orderedStates == null)
        {
            return;
        }

        string signature = BuildRobotOrderSignature(orderedStates);
        if (!seenSignatures.Add(signature))
        {
            return;
        }

        orders.Add(new BestEffortPlanningOrder
        {
            description = description,
            orderedStates = new List<RobotPlanningState>(orderedStates)
        });
    }

    private BestEffortPlanningAttempt BuildBestEffortAttemptForOrder(
        ContinuousPlanningGraph graph,
        IList<RobotPlanningState> allRobotStates,
        BestEffortPlanningOrder planningOrder)
    {
        var attempt = new BestEffortPlanningAttempt
        {
            orderDescription = planningOrder != null ? planningOrder.description : "<unknown>"
        };

        if (graph == null || allRobotStates == null || planningOrder == null || planningOrder.orderedStates == null)
        {
            return attempt;
        }

        for (int i = 0; i < planningOrder.orderedStates.Count; i++)
        {
            RobotPlanningState robotState = planningOrder.orderedStates[i];
            if (robotState == null)
            {
                continue;
            }

            // Best-effort fallback assumes every not-yet-selected robot simply
            // holds its current state. That keeps the accepted subset safe even
            // when we cannot find a joint schedule for everyone right now.
            ReservationTable reservationTable =
                BuildBestEffortReservationTable(graph, allRobotStates, attempt.schedulesByRobotId, robotState.robotId);
            var planner = new ContinuousTimePathPlanner(graph, reservationTable);
            RobotSchedule schedule = planner.FindSchedule(robotState);
            if (schedule == null)
            {
                attempt.unscheduledRobotIds.Add(robotState.robotId);
                continue;
            }

            attempt.schedulesByRobotId.Add(robotState.robotId, schedule);
            attempt.totalCompletionTime += schedule.CompletionTime;
        }

        return attempt;
    }

    private ReservationTable BuildBestEffortReservationTable(
        ContinuousPlanningGraph graph,
        IList<RobotPlanningState> allRobotStates,
        Dictionary<int, RobotSchedule> acceptedSchedules,
        int robotIdToPlan)
    {
        var reservationTable = new ReservationTable();

        if (allRobotStates == null)
        {
            return reservationTable;
        }

        for (int i = 0; i < allRobotStates.Count; i++)
        {
            RobotPlanningState otherRobotState = allRobotStates[i];
            if (otherRobotState == null || otherRobotState.robotId == robotIdToPlan)
            {
                continue;
            }

            RobotSchedule acceptedSchedule;
            if (acceptedSchedules != null &&
                acceptedSchedules.TryGetValue(otherRobotState.robotId, out acceptedSchedule))
            {
                reservationTable.AddSchedule(acceptedSchedule);
                continue;
            }

            AddHoldingReservations(reservationTable, graph, otherRobotState);
        }

        return reservationTable;
    }

    private void AddHoldingReservations(
        ReservationTable reservationTable,
        ContinuousPlanningGraph graph,
        RobotPlanningState robotState)
    {
        if (reservationTable == null || graph == null || robotState == null)
        {
            return;
        }

        if (robotState.IsAtVertex)
        {
            TimeInterval holdInterval = new TimeInterval(0f, float.PositiveInfinity);
            reservationTable.AddVertexReservation(new VertexReservation
            {
                robotId = robotState.robotId,
                vertexId = robotState.currentVertexId,
                interval = holdInterval,
                movementInterval = holdInterval,
                fromVertexId = robotState.currentVertexId,
                toVertexId = robotState.currentVertexId
            });
            return;
        }

        // If a robot is already committed to an edge, the safest "hold" behavior
        // is to let it finish that edge and then reserve its arrival vertex
        // indefinitely until a later replan assigns something better.
        float remainingTime = robotState.GetRemainingTravelTimeOnCurrentEdge(graph);
        if (float.IsPositiveInfinity(remainingTime))
        {
            return;
        }

        if (remainingTime > Epsilon)
        {
            reservationTable.AddEdgeReservation(new EdgeReservation
            {
                robotId = robotState.robotId,
                edgeKey = new PlanningEdgeKey(robotState.edgeFromVertexId, robotState.edgeToVertexId),
                fromVertexId = robotState.edgeFromVertexId,
                toVertexId = robotState.edgeToVertexId,
                interval = new TimeInterval(0f, remainingTime)
            });
        }

        TimeInterval destinationHoldInterval = new TimeInterval(
            Mathf.Max(0f, remainingTime),
            float.PositiveInfinity);
        reservationTable.AddVertexReservation(new VertexReservation
        {
            robotId = robotState.robotId,
            vertexId = robotState.edgeToVertexId,
            interval = destinationHoldInterval,
            movementInterval = destinationHoldInterval,
            fromVertexId = robotState.edgeToVertexId,
            toVertexId = robotState.edgeToVertexId
        });
    }

    private float GetUnconstrainedCompletionTime(
        Dictionary<int, float> unconstrainedCompletionTimes,
        int robotId)
    {
        if (unconstrainedCompletionTimes == null)
        {
            return float.PositiveInfinity;
        }

        float completionTime;
        return unconstrainedCompletionTimes.TryGetValue(robotId, out completionTime)
            ? completionTime
            : float.PositiveInfinity;
    }

    private bool IsBetterBestEffortAttempt(
        BestEffortPlanningAttempt candidate,
        BestEffortPlanningAttempt currentBest)
    {
        if (candidate == null)
        {
            return false;
        }

        if (currentBest == null)
        {
            return true;
        }

        int candidateCount =
            candidate.schedulesByRobotId != null ? candidate.schedulesByRobotId.Count : 0;
        int bestCount =
            currentBest.schedulesByRobotId != null ? currentBest.schedulesByRobotId.Count : 0;
        if (candidateCount != bestCount)
        {
            return candidateCount > bestCount;
        }

        if (candidate.totalCompletionTime != currentBest.totalCompletionTime)
        {
            return candidate.totalCompletionTime < currentBest.totalCompletionTime;
        }

        int candidateUnscheduledCount =
            candidate.unscheduledRobotIds != null ? candidate.unscheduledRobotIds.Count : 0;
        int bestUnscheduledCount =
            currentBest.unscheduledRobotIds != null ? currentBest.unscheduledRobotIds.Count : 0;
        if (candidateUnscheduledCount != bestUnscheduledCount)
        {
            return candidateUnscheduledCount < bestUnscheduledCount;
        }

        return string.CompareOrdinal(candidate.orderDescription, currentBest.orderDescription) < 0;
    }

    private string BuildRobotOrderSignature(IList<RobotPlanningState> orderedStates)
    {
        if (orderedStates == null || orderedStates.Count == 0)
        {
            return string.Empty;
        }

        var signature = new System.Text.StringBuilder();
        for (int i = 0; i < orderedStates.Count; i++)
        {
            if (i > 0)
            {
                signature.Append(',');
            }

            signature.Append(orderedStates[i] != null ? orderedStates[i].robotId : -1);
        }

        return signature.ToString();
    }

    private string FormatRobotIdList(IList<int> robotIds)
    {
        if (robotIds == null || robotIds.Count == 0)
        {
            return "<none>";
        }

        var builder = new System.Text.StringBuilder();
        for (int i = 0; i < robotIds.Count; i++)
        {
            if (i > 0)
            {
                builder.Append(", ");
            }

            builder.Append(robotIds[i]);
        }

        return builder.ToString();
    }

    private class BestEffortPlanningOrder
    {
        public string description;
        public List<RobotPlanningState> orderedStates = new List<RobotPlanningState>();
    }

    private class BestEffortPlanningAttempt
    {
        public string orderDescription;
        public float totalCompletionTime;
        public Dictionary<int, RobotSchedule> schedulesByRobotId = new Dictionary<int, RobotSchedule>();
        public List<int> unscheduledRobotIds = new List<int>();
    }
}
