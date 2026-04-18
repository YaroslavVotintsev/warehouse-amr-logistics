using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Scene-facing scheduler component.
/// Attach it once, point it at the graph, assign robots, and it will run the continuous-time CBS planner.
/// </summary>
public class MultiRobotScheduler : MonoBehaviour
{
    [Header("Scene References")]
    public GraphManager graphManager;
    public List<Robot> robots = new List<Robot>();

    [Header("Planning")]
    public bool planOnStart = true;
    public bool autoReplan = false;
    public float replanInterval = 1f;
    public bool autoReplanOnlyAtVertices = true;
    public bool logPlanningResult = true;

    private readonly ContinuousMultiRobotPlanner planner = new ContinuousMultiRobotPlanner();
    private float nextReplanTime = 0f;
    private const float DeferredReplanDelay = 0.1f;

    void Start()
    {
        if (planOnStart)
        {
            PlanNow();
        }
    }

    void Update()
    {
        if (!autoReplan)
        {
            return;
        }

        if (Time.time >= nextReplanTime)
        {
            PlanNow();
        }
    }

    [ContextMenu("Plan Now")]
    public void PlanNow()
    {
        if (graphManager == null)
        {
            Debug.LogError("MultiRobotScheduler requires a GraphManager reference.");
            ScheduleNextAttempt();
            return;
        }

        if (robots == null || robots.Count == 0)
        {
            robots = new List<Robot>(FindObjectsByType<Robot>(FindObjectsSortMode.None));
        }

        if (autoReplan && !AnyRobotNeedsReplan())
        {
            ScheduleNextAttempt();
            return;
        }

        ContinuousPlanningGraph graph = ContinuousPlanningGraph.Build(graphManager);
        if (graph == null)
        {
            Debug.LogError(
                "MultiRobotScheduler failed to build planning graph from GraphManager " +
                graphManager.name + ".");
            ScheduleNextAttempt();
            return;
        }

        List<RobotPlanningState> robotStates = BuildRobotStates(graph);
        if (robotStates == null)
        {
            ScheduleNextAttempt();
            return;
        }

        if (autoReplan && autoReplanOnlyAtVertices && HasRobotOnEdge(robotStates))
        {
            // In this model a robot can only choose to wait or reverse after it
            // reaches a vertex. Replanning in the middle of an edge often turns
            // a previously valid plan into an artificial dead-end because the
            // current traversal is treated as committed.
            if (logPlanningResult)
            {
                Debug.Log(
                    "MultiRobotScheduler deferred replanning because at least one robot is still on an edge.\n" +
                    PlanningDebugUtility.FormatRobotStates(robotStates, graph));
            }

            nextReplanTime = Time.time + DeferredReplanDelay;
            return;
        }

        ContinuousPlanningResult result = planner.Plan(graph, robotStates);
        if (!result.success)
        {
            if (logPlanningResult)
            {
                Debug.LogWarning(
                    "Planning failed: " + result.failureReason +
                    "\n" + PlanningDebugUtility.FormatGraph(graph) +
                    "\n" + PlanningDebugUtility.FormatRobotStates(robotStates, graph));
            }
            ScheduleNextAttempt();
            return;
        }

        for (int i = 0; i < robots.Count; i++)
        {
            Robot robot = robots[i];
            if (robot == null || !robot.gameObject.activeInHierarchy)
            {
                continue;
            }

            RobotSchedule schedule;
            if (result.schedulesByRobotId.TryGetValue(robot.id, out schedule))
            {
                robot.SetSchedule(schedule, graph);
            }
        }

        ScheduleNextAttempt();

        if (logPlanningResult)
        {
            Debug.Log(
                "Planning succeeded for " + result.schedulesByRobotId.Count + " robots.\n" +
                PlanningDebugUtility.FormatSchedules(result.schedulesByRobotId, 6, 3));
        }
    }

    private List<RobotPlanningState> BuildRobotStates(ContinuousPlanningGraph graph)
    {
        var robotStates = new List<RobotPlanningState>();

        for (int i = 0; i < robots.Count; i++)
        {
            Robot robot = robots[i];
            if (robot == null || !robot.gameObject.activeInHierarchy)
            {
                continue;
            }

            RobotPlanningState robotState;
            if (!robot.TryBuildPlanningState(graph, out robotState))
            {
                Debug.LogError(
                    "Failed to build planning state for robot " + robot.id +
                    ". Make sure it has a goal vertex and is either on a vertex or following a schedule.\n" +
                    PlanningDebugUtility.FormatRobotComponent(robot));
                return null;
            }

            robotStates.Add(robotState);
        }

        return robotStates;
    }

    private bool AnyRobotNeedsReplan()
    {
        if (robots == null)
        {
            return false;
        }

        for (int i = 0; i < robots.Count; i++)
        {
            Robot robot = robots[i];
            if (robot == null || !robot.gameObject.activeInHierarchy)
            {
                continue;
            }

            if (robot.NeedsReplan())
            {
                return true;
            }
        }

        return false;
    }

    private bool HasRobotOnEdge(List<RobotPlanningState> robotStates)
    {
        if (robotStates == null)
        {
            return false;
        }

        for (int i = 0; i < robotStates.Count; i++)
        {
            RobotPlanningState robotState = robotStates[i];
            if (robotState != null && robotState.IsOnEdge)
            {
                return true;
            }
        }

        return false;
    }

    private void ScheduleNextAttempt()
    {
        float interval = autoReplan ? Mathf.Max(DeferredReplanDelay, replanInterval) : replanInterval;
        nextReplanTime = Time.time + interval;
    }
}
