using System;
using System.Collections.Generic;
using System.Threading.Tasks;
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
    public bool rollingReplanAtVertices = true;
    public bool runPlanningAsync = true;
    public bool pauseRobotsDuringAsyncPlanning = true;
    public bool logPlanningResult = true;

    private readonly ContinuousMultiRobotPlanner planner = new ContinuousMultiRobotPlanner();
    private float nextReplanTime = 0f;
    private float nextDeferredEdgeLogTime = 0f;
    private PlanningJob activePlanningJob;
    private int nextPlanningJobId = 1;
    private const float DeferredReplanDelay = 0.1f;
    private const float DeferredEdgeLogInterval = 2f;

    void Start()
    {
        if (planOnStart)
        {
            PlanNow();
        }
    }

    void Update()
    {
        CompleteFinishedPlanningJob();

        if (!autoReplan)
        {
            return;
        }

        if (activePlanningJob != null)
        {
            return;
        }

        if (Time.time >= nextReplanTime)
        {
            PlanNow();
        }
    }

    void OnDisable()
    {
        SetRobotsPlanningPaused(false);
    }

    [ContextMenu("Plan Now")]
    public void PlanNow()
    {
        CompleteFinishedPlanningJob();

        if (activePlanningJob != null)
        {
            ScheduleNextAttempt();
            return;
        }

        if (graphManager == null)
        {
            Debug.LogError("MultiRobotScheduler requires a GraphManager reference.");
            ScheduleNextAttempt();
            return;
        }

        if (robots == null || robots.Count == 0)
        {
            robots = new List<Robot>(FindObjectsByType<Robot>(FindObjectsInactive.Exclude));
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

        Dictionary<int, RobotPlanningState> robotStatesById = IndexRobotStatesById(robotStates);
        bool anyRobotNeedsReplan = AnyRobotNeedsReplan();

        if (autoReplan && !anyRobotNeedsReplan)
        {
            ScheduleNextAttempt();
            return;
        }

        if (autoReplan &&
            autoReplanOnlyAtVertices &&
            HasRobotOnEdge(robotStates) &&
            !HasVertexReadyRobotNeedingReplan(robotStatesById))
        {
            // In this model a robot can only choose to wait or reverse after it
            // reaches a vertex. Replanning in the middle of an edge often turns
            // a previously valid plan into an artificial dead-end because the
            // current traversal is treated as committed.
            if (logPlanningResult && Time.time >= nextDeferredEdgeLogTime)
            {
                Debug.Log(
                    "MultiRobotScheduler deferred replanning because at least one robot is still on an edge.\n" +
                    PlanningDebugUtility.FormatRobotStates(robotStates, graph));
                nextDeferredEdgeLogTime = Time.time + DeferredEdgeLogInterval;
            }

            ScheduleNextAttempt();
            return;
        }

        nextDeferredEdgeLogTime = 0f;

        if (runPlanningAsync)
        {
            StartAsyncPlanningJob(graph, robotStates, robotStatesById);
            ScheduleNextAttempt();
            return;
        }

        ContinuousPlanningResult result = planner.Plan(graph, robotStates);
        HandlePlanningResult(result, graph, robotStates, robotStatesById);
    }

    private void StartAsyncPlanningJob(
        ContinuousPlanningGraph graph,
        List<RobotPlanningState> robotStates,
        Dictionary<int, RobotPlanningState> robotStatesById)
    {
        var job = new PlanningJob
        {
            id = nextPlanningJobId++,
            graph = graph,
            robotStates = new List<RobotPlanningState>(robotStates),
            robotStatesById = new Dictionary<int, RobotPlanningState>(robotStatesById)
        };

        activePlanningJob = job;
        SetRobotsPlanningPaused(pauseRobotsDuringAsyncPlanning);

        job.task = Task.Run(() => planner.Plan(job.graph, job.robotStates));
    }

    private void CompleteFinishedPlanningJob()
    {
        if (activePlanningJob == null || activePlanningJob.task == null || !activePlanningJob.task.IsCompleted)
        {
            return;
        }

        PlanningJob job = activePlanningJob;
        activePlanningJob = null;
        SetRobotsPlanningPaused(false);

        if (job.task.IsCanceled)
        {
            if (logPlanningResult)
            {
                Debug.LogWarning("Async planning job " + job.id + " was canceled.");
            }

            ScheduleNextAttempt();
            return;
        }

        if (job.task.IsFaulted)
        {
            if (logPlanningResult)
            {
                Exception exception = job.task.Exception != null
                    ? job.task.Exception.GetBaseException()
                    : null;
                Debug.LogError(
                    "Async planning job " + job.id + " failed with an exception: " +
                    (exception != null ? exception.Message : "<unknown>"));
            }

            ScheduleNextAttempt();
            return;
        }

        ContinuousPlanningResult result = job.task.Result;
        if (!ArePlanningInputsStillCurrent(job.robotStates))
        {
            if (logPlanningResult)
            {
                Debug.Log(
                    "Async planning job " + job.id +
                    " finished, but its snapshot is stale. A new planning attempt will be started.");
            }

            nextReplanTime = Time.time;
            return;
        }

        HandlePlanningResult(result, job.graph, job.robotStates, job.robotStatesById);
    }

    private void HandlePlanningResult(
        ContinuousPlanningResult result,
        ContinuousPlanningGraph graph,
        List<RobotPlanningState> robotStates,
        Dictionary<int, RobotPlanningState> robotStatesById)
    {
        if (result == null)
        {
            if (logPlanningResult)
            {
                Debug.LogWarning("Planning failed: planner returned a null result.");
            }

            ScheduleNextAttempt();
            return;
        }

        if (!result.success && !result.partialSuccess)
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
                robot.stopAtScheduleVerticesForReplanning =
                    autoReplan && autoReplanOnlyAtVertices && rollingReplanAtVertices;
                robot.SetSchedule(schedule, graph);
                continue;
            }

            if (result.partialSuccess)
            {
                RobotPlanningState robotState;
                if (robotStatesById.TryGetValue(robot.id, out robotState))
                {
                    if (robotState.IsAtVertex)
                    {
                        robot.stopAtScheduleVerticesForReplanning = false;
                        robot.ClearSchedule();
                    }
                    else
                    {
                        robot.stopAtScheduleVerticesForReplanning =
                            autoReplan && autoReplanOnlyAtVertices && rollingReplanAtVertices;
                        robot.SetRetryHoldSchedule(BuildHoldSchedule(robotState, graph), graph);
                    }
                }
                else
                {
                    robot.stopAtScheduleVerticesForReplanning = false;
                    robot.ClearSchedule();
                }
            }
        }

        ScheduleNextAttempt();

        if (logPlanningResult)
        {
            if (result.success)
            {
                Debug.Log(
                    "Planning succeeded for " + result.schedulesByRobotId.Count + " robots.\n" +
                    PlanningDebugUtility.FormatSchedules(result.schedulesByRobotId, 6, 3));
            }
            else
            {
                Debug.LogWarning(
                    "Planning partially succeeded.\n" +
                    result.failureReason +
                    "\n" + PlanningDebugUtility.FormatSchedules(result.schedulesByRobotId, 6, 3) +
                    "\nunscheduledRobotIds: " + FormatRobotIdList(result.unscheduledRobotIds));
            }
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

    private bool HasVertexReadyRobotNeedingReplan(Dictionary<int, RobotPlanningState> robotStatesById)
    {
        if (robotStatesById == null || robots == null)
        {
            return false;
        }

        for (int i = 0; i < robots.Count; i++)
        {
            Robot robot = robots[i];
            if (robot == null || !robot.gameObject.activeInHierarchy || !robot.NeedsReplan())
            {
                continue;
            }

            RobotPlanningState robotState;
            if (robotStatesById.TryGetValue(robot.id, out robotState) &&
                robotState != null &&
                robotState.IsAtVertex)
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

    private Dictionary<int, RobotPlanningState> IndexRobotStatesById(List<RobotPlanningState> robotStates)
    {
        var robotStatesById = new Dictionary<int, RobotPlanningState>();

        if (robotStates == null)
        {
            return robotStatesById;
        }

        for (int i = 0; i < robotStates.Count; i++)
        {
            RobotPlanningState robotState = robotStates[i];
            if (robotState == null)
            {
                continue;
            }

            robotStatesById[robotState.robotId] = robotState;
        }

        return robotStatesById;
    }


    private RobotSchedule BuildHoldSchedule(RobotPlanningState robotState, ContinuousPlanningGraph graph)
    {
        var schedule = new RobotSchedule
        {
            robotId = robotState != null ? robotState.robotId : -1
        };

        if (robotState == null || graph == null)
        {
            return schedule;
        }

        if (robotState.IsAtVertex)
        {
            schedule.segments.Add(ScheduleSegment.CreateWait(
                robotState.currentVertexId,
                0f,
                float.PositiveInfinity));
            return schedule;
        }

        float remainingTraversalTime = robotState.GetRemainingTravelTimeOnCurrentEdge(graph);
        if (float.IsPositiveInfinity(remainingTraversalTime))
        {
            return schedule;
        }

        if (remainingTraversalTime > 0f)
        {
            schedule.segments.Add(ScheduleSegment.CreateTraversal(
                robotState.edgeFromVertexId,
                robotState.edgeToVertexId,
                0f,
                remainingTraversalTime));
        }

        schedule.segments.Add(ScheduleSegment.CreateWait(
            robotState.edgeToVertexId,
            Mathf.Max(0f, remainingTraversalTime),
            float.PositiveInfinity));
        return schedule;
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

    private void ScheduleNextAttempt()
    {
        float interval = autoReplan ? Mathf.Max(DeferredReplanDelay, replanInterval) : replanInterval;
        nextReplanTime = Time.time + interval;
    }

    private void SetRobotsPlanningPaused(bool paused)
    {
        if (robots == null)
        {
            return;
        }

        for (int i = 0; i < robots.Count; i++)
        {
            Robot robot = robots[i];
            if (robot == null || !robot.gameObject.activeInHierarchy)
            {
                continue;
            }

            robot.SetSchedulePaused(paused);
        }
    }

    private bool ArePlanningInputsStillCurrent(List<RobotPlanningState> plannedRobotStates)
    {
        if (plannedRobotStates == null)
        {
            return false;
        }

        for (int i = 0; i < plannedRobotStates.Count; i++)
        {
            RobotPlanningState plannedState = plannedRobotStates[i];
            if (plannedState == null)
            {
                continue;
            }

            Robot robot = FindRobotById(plannedState.robotId);
            if (robot == null || !robot.gameObject.activeInHierarchy)
            {
                return false;
            }

            if (robot.GetGoalVertexId() != plannedState.goalVertexId)
            {
                return false;
            }
        }

        return true;
    }

    private Robot FindRobotById(int robotId)
    {
        if (robots == null)
        {
            return null;
        }

        for (int i = 0; i < robots.Count; i++)
        {
            Robot robot = robots[i];
            if (robot != null && robot.id == robotId)
            {
                return robot;
            }
        }

        return null;
    }

    private class PlanningJob
    {
        public int id;
        public ContinuousPlanningGraph graph;
        public List<RobotPlanningState> robotStates;
        public Dictionary<int, RobotPlanningState> robotStatesById;
        public Task<ContinuousPlanningResult> task;
    }
}
