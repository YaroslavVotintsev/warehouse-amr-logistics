using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Low-level continuous-time planner for one robot.
/// It uses SIPP-style search over safe intervals:
/// - states are "robot is at vertex v inside safe interval I"
/// - edges are traversed only when the edge resource is free
/// - waiting is allowed only at vertices
/// </summary>
public class ContinuousTimePathPlanner
{
    // Keep boundary comparisons tighter than the reservation/event padding so
    // the planner cannot "step through" tiny but meaningful blocked windows.
    private const float Epsilon = 0.000001f;
    private const int MaxExpandedNodes = 10000;

    private readonly ContinuousPlanningGraph graph;
    private readonly ReservationTable reservations;

    public string LastFailureReason { get; private set; }
    public string LastFailureDiagnostics { get; private set; }

    public ContinuousTimePathPlanner(ContinuousPlanningGraph graph, ReservationTable reservations)
    {
        this.graph = graph;
        this.reservations = reservations;
    }

    public RobotSchedule FindSchedule(RobotPlanningState robotState)
    {
        LastFailureReason = null;
        LastFailureDiagnostics = null;

        if (graph == null)
        {
            LastFailureReason = "ContinuousTimePathPlanner requires a valid planning graph.";
            Debug.LogError(LastFailureReason);
            return null;
        }

        if (reservations == null)
        {
            LastFailureReason = "ContinuousTimePathPlanner requires a reservation table.";
            Debug.LogError(LastFailureReason);
            return null;
        }

        if (robotState == null || !robotState.IsValid(graph))
        {
            LastFailureReason = "ContinuousTimePathPlanner received an invalid robot planning state.";
            Debug.LogError(
                LastFailureReason + "\n" +
                PlanningDebugUtility.FormatRobotState(robotState, graph));
            return null;
        }

        var openNodes = new List<SearchNode>();
        var bestArrivalByState = new Dictionary<SafeStateKey, float>();

        SearchNode startNode = CreateStartNode(robotState);
        if (startNode == null)
        {
            if (string.IsNullOrEmpty(LastFailureReason))
            {
                LastFailureReason = "No safe start state was available for the robot.";
            }
            return null;
        }

        SafeStateKey startKey = new SafeStateKey(startNode.vertexId, startNode.safeInterval);
        bestArrivalByState[startKey] = startNode.arrivalTime;
        openNodes.Add(startNode);
        int expandedNodes = 0;
        SearchNode lastExpandedNode = null;

        while (openNodes.Count > 0)
        {
            expandedNodes++;
            if (expandedNodes > MaxExpandedNodes)
            {
                LastFailureReason =
                    "Low-level planner aborted after reaching the expansion limit (" +
                    MaxExpandedNodes + ").";
                LastFailureDiagnostics = BuildFailureDiagnostics(
                    robotState,
                    lastExpandedNode,
                    expandedNodes,
                    openNodes.Count,
                    bestArrivalByState.Count);
                return null;
            }

            SearchNode currentNode = RemoveBestNode(openNodes);
            lastExpandedNode = currentNode;
            SafeStateKey currentKey = new SafeStateKey(currentNode.vertexId, currentNode.safeInterval);

            float bestKnownArrival;
            if (bestArrivalByState.TryGetValue(currentKey, out bestKnownArrival) &&
                currentNode.arrivalTime > bestKnownArrival + Epsilon)
            {
                continue;
            }

            if (IsGoalNode(robotState.goalVertexId, currentNode))
            {
                return BuildSchedule(robotState, currentNode);
            }

            foreach (int neighborVertexId in graph.GetNeighborVertexIds(currentNode.vertexId))
            {
                ExpandNeighbor(robotState, currentNode, neighborVertexId, openNodes, bestArrivalByState);
            }
        }

        if (string.IsNullOrEmpty(LastFailureReason))
        {
            LastFailureReason = "Low-level planner exhausted the search space without reaching the goal.";
        }

        LastFailureDiagnostics = BuildFailureDiagnostics(
            robotState,
            lastExpandedNode,
            expandedNodes,
            openNodes.Count,
            bestArrivalByState.Count);
        return null;
    }

    private SearchNode CreateStartNode(RobotPlanningState robotState)
    {
        if (robotState.IsAtVertex)
        {
            SafeInterval safeInterval;
            if (!reservations.TryGetSafeIntervalContaining(
                robotState.currentVertexId,
                0f,
                out safeInterval,
                robotState.robotId))
            {
                LastFailureReason =
                    "Robot start vertex " + robotState.currentVertexId +
                    " is not safe at planning time.";
                LastFailureDiagnostics = BuildStartFailureDiagnostics(robotState);
                return null;
            }

            return new SearchNode
            {
                vertexId = robotState.currentVertexId,
                safeInterval = safeInterval,
                arrivalTime = 0f
            };
        }

        float arrivalTime = robotState.GetRemainingTravelTimeOnCurrentEdge(graph);
        SafeInterval destinationInterval;
        if (!reservations.TryGetSafeIntervalContaining(
            robotState.edgeToVertexId,
            arrivalTime,
            out destinationInterval,
            robotState.robotId))
        {
            LastFailureReason =
                "Robot cannot safely finish its current committed edge traversal into vertex " +
                robotState.edgeToVertexId + ".";
            LastFailureDiagnostics = BuildStartFailureDiagnostics(robotState);
            return null;
        }

        return new SearchNode
        {
            vertexId = robotState.edgeToVertexId,
            safeInterval = destinationInterval,
            arrivalTime = arrivalTime
        };
    }

    private bool IsGoalNode(int goalVertexId, SearchNode node)
    {
        return node.vertexId == goalVertexId && node.safeInterval.IsUnbounded;
    }

    private void ExpandNeighbor(
        RobotPlanningState robotState,
        SearchNode currentNode,
        int neighborVertexId,
        List<SearchNode> openNodes,
        Dictionary<SafeStateKey, float> bestArrivalByState)
    {
        PlanningEdge edge;
        if (!graph.TryGetEdge(currentNode.vertexId, neighborVertexId, out edge))
        {
            return;
        }

        float traversalDuration = edge.GetTravelTime(robotState.speed);
        if (float.IsInfinity(traversalDuration))
        {
            return;
        }

        List<SafeInterval> destinationSafeIntervals =
            reservations.GetSafeVertexIntervals(neighborVertexId, robotState.robotId, 0f);

        for (int i = 0; i < destinationSafeIntervals.Count; i++)
        {
            SafeInterval destinationInterval = destinationSafeIntervals[i];

            float earliestDepartureTime = currentNode.arrivalTime;
            float departureNeededForDestination = destinationInterval.startTime - traversalDuration;
            if (departureNeededForDestination > earliestDepartureTime)
            {
                earliestDepartureTime = departureNeededForDestination;
            }

            if (earliestDepartureTime > currentNode.safeInterval.endTime + Epsilon)
            {
                break;
            }

            float feasibleDepartureTime = reservations.FindEarliestEdgeAvailability(
                edge.key,
                earliestDepartureTime,
                traversalDuration,
                robotState.robotId);

            if (feasibleDepartureTime > currentNode.safeInterval.endTime + Epsilon)
            {
                continue;
            }

            float arrivalTime = feasibleDepartureTime + traversalDuration;
            bool fitsDestinationInterval =
                arrivalTime + Epsilon >= destinationInterval.startTime &&
                (destinationInterval.IsUnbounded || arrivalTime < destinationInterval.endTime - Epsilon);

            if (!fitsDestinationInterval)
            {
                continue;
            }

            var nextNode = new SearchNode
            {
                vertexId = neighborVertexId,
                safeInterval = destinationInterval,
                arrivalTime = arrivalTime,
                parent = currentNode,
                departureTimeFromParent = feasibleDepartureTime
            };

            SafeStateKey stateKey = new SafeStateKey(nextNode.vertexId, nextNode.safeInterval);
            float bestKnownArrival;
            if (bestArrivalByState.TryGetValue(stateKey, out bestKnownArrival) &&
                arrivalTime >= bestKnownArrival - Epsilon)
            {
                continue;
            }

            bestArrivalByState[stateKey] = arrivalTime;
            openNodes.Add(nextNode);
        }
    }

    private RobotSchedule BuildSchedule(RobotPlanningState robotState, SearchNode goalNode)
    {
        var traversalNodes = new List<SearchNode>();
        SearchNode currentNode = goalNode;

        while (currentNode.parent != null)
        {
            traversalNodes.Add(currentNode);
            currentNode = currentNode.parent;
        }

        traversalNodes.Reverse();

        var schedule = new RobotSchedule
        {
            robotId = robotState.robotId
        };

        if (robotState.IsOnEdge)
        {
            float remainingTraversalTime = robotState.GetRemainingTravelTimeOnCurrentEdge(graph);
            schedule.segments.Add(ScheduleSegment.CreateTraversal(
                robotState.edgeFromVertexId,
                robotState.edgeToVertexId,
                0f,
                remainingTraversalTime));
        }

        SearchNode scheduleCursor = currentNode;
        for (int i = 0; i < traversalNodes.Count; i++)
        {
            SearchNode nextNode = traversalNodes[i];

            if (nextNode.departureTimeFromParent > scheduleCursor.arrivalTime + Epsilon)
            {
                schedule.segments.Add(ScheduleSegment.CreateWait(
                    scheduleCursor.vertexId,
                    scheduleCursor.arrivalTime,
                    nextNode.departureTimeFromParent));
            }

            schedule.segments.Add(ScheduleSegment.CreateTraversal(
                scheduleCursor.vertexId,
                nextNode.vertexId,
                nextNode.departureTimeFromParent,
                nextNode.arrivalTime));

            scheduleCursor = nextNode;
        }

        // Reaching the goal means the robot occupies that vertex indefinitely until the next replan.
        schedule.segments.Add(ScheduleSegment.CreateWait(
            goalNode.vertexId,
            goalNode.arrivalTime,
            float.PositiveInfinity));

        return schedule;
    }

    private SearchNode RemoveBestNode(List<SearchNode> openNodes)
    {
        int bestIndex = 0;
        SearchNode bestNode = openNodes[0];

        for (int i = 1; i < openNodes.Count; i++)
        {
            SearchNode candidate = openNodes[i];
            if (candidate.arrivalTime < bestNode.arrivalTime - Epsilon)
            {
                bestIndex = i;
                bestNode = candidate;
            }
        }

        openNodes.RemoveAt(bestIndex);
        return bestNode;
    }

    private string BuildStartFailureDiagnostics(RobotPlanningState robotState)
    {
        return "Low-level planner start-state diagnostics:\n" +
               PlanningDebugUtility.FormatGraph(graph) +
               "\n" + PlanningDebugUtility.FormatRobotState(robotState, graph) +
               "\n" + PlanningDebugUtility.FormatReservationSnapshot(reservations, robotState, graph);
    }

    private string BuildFailureDiagnostics(
        RobotPlanningState robotState,
        SearchNode currentNode,
        int expandedNodes,
        int openNodeCount,
        int exploredStateCount)
    {
        return "Low-level planner diagnostics:\n" +
               PlanningDebugUtility.FormatGraph(graph) +
               "\n" + PlanningDebugUtility.FormatRobotState(robotState, graph) +
               "\nexpandedNodes=" + expandedNodes +
               " openNodes=" + openNodeCount +
               " exploredStates=" + exploredStateCount +
               "\nlastExpanded=" + FormatSearchNode(currentNode) +
               "\n" + PlanningDebugUtility.FormatReservationSnapshot(reservations, robotState, graph);
    }

    private string FormatSearchNode(SearchNode node)
    {
        if (node == null)
        {
            return "<none>";
        }

        return "V" + node.vertexId +
               " arrival=" + node.arrivalTime.ToString("0.###") +
               " safe=" + node.safeInterval.startTime.ToString("0.###") +
               ".." +
               (float.IsPositiveInfinity(node.safeInterval.endTime)
                   ? "inf"
                   : node.safeInterval.endTime.ToString("0.###"));
    }

    private struct SafeStateKey : IEquatable<SafeStateKey>
    {
        public int vertexId;
        public SafeInterval safeInterval;

        public SafeStateKey(int vertexId, SafeInterval safeInterval)
        {
            this.vertexId = vertexId;
            this.safeInterval = safeInterval;
        }

        public bool Equals(SafeStateKey other)
        {
            return vertexId == other.vertexId && safeInterval.Equals(other.safeInterval);
        }

        public override bool Equals(object obj)
        {
            return obj is SafeStateKey other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return (vertexId * 397) ^ safeInterval.GetHashCode();
            }
        }
    }

    private class SearchNode
    {
        public int vertexId;
        public SafeInterval safeInterval;
        public float arrivalTime;

        public SearchNode parent;
        public float departureTimeFromParent;
    }
}
