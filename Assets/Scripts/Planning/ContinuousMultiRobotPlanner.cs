using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Planner output wrapper.
/// This keeps the public API simple for runtime code: either we get schedules or a concrete failure reason.
/// </summary>
public class ContinuousPlanningResult
{
    public bool success;
    public string failureReason;
    public Dictionary<int, RobotSchedule> schedulesByRobotId = new Dictionary<int, RobotSchedule>();
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
            result.failureReason = !string.IsNullOrEmpty(solver.LastFailureReason)
                ? solver.LastFailureReason
                : "No conflict-free schedule could be found for the current robot states.";
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
}
