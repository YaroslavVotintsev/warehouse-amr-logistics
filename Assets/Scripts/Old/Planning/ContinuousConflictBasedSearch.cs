using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

/// <summary>
/// High-level continuous-time CBS constraint.
/// A child node forbids one robot from occupying a conflicting resource during a conflicting interval.
/// </summary>
public class ContinuousCbsConstraint
{
    public int robotId;
    public ReservationConflictType type;
    public TimeInterval interval;

    public int vertexId = -1;
    public PlanningEdgeKey edgeKey;
}

/// <summary>
/// One node in the high-level CBS search tree.
/// It stores per-robot constraints and the best-known schedule for each robot under those constraints.
/// </summary>
public class ContinuousCbsNode
{
    public List<ContinuousCbsConstraint> constraints = new List<ContinuousCbsConstraint>();
    public Dictionary<int, RobotSchedule> schedulesByRobotId = new Dictionary<int, RobotSchedule>();

    public float TotalCost
    {
        get
        {
            float totalCost = 0f;

            foreach (KeyValuePair<int, RobotSchedule> pair in schedulesByRobotId)
            {
                if (pair.Value != null)
                {
                    totalCost += pair.Value.CompletionTime;
                }
            }

            return totalCost;
        }
    }
}

/// <summary>
/// High-level Conflict-Based Search for continuous-time schedules.
/// It replans only the robot selected by a child constraint while keeping other robot schedules fixed in that node.
/// </summary>
public class ContinuousConflictBasedSearch
{
    private const int ConstraintReservationRobotId = int.MinValue;
    private const bool UseCorridorBranching = false;
    private const float CorridorEndpointPadding = 0.0001f;
    private const float TimeComparisonEpsilon = 0.0001f;
    private const float SignatureTimeQuantization = 0.0001f;
    private const int MaxHighLevelIterations = 2000;
    private const int MaxOpenNodes = 4000;
    private const int MaxConflictHistory = 8;
    private const int MaxConflictSummaryEntries = 5;
    private const int MaxChildFailureHistory = 8;

    private readonly ContinuousPlanningGraph graph;
    private bool abortSearch;
    private Dictionary<string, int> childFailureCounts = new Dictionary<string, int>();
    private Queue<string> recentChildFailures = new Queue<string>();
    private string lastExpandedNodeSummary = "<none>";
    private string lastLowLevelFailureDiagnostics;

    public string LastFailureReason { get; private set; }

    public ContinuousConflictBasedSearch(ContinuousPlanningGraph graph)
    {
        this.graph = graph;
    }

    public Dictionary<int, RobotSchedule> FindSchedules(IList<RobotPlanningState> robotStates)
    {
        LastFailureReason = null;
        abortSearch = false;
        childFailureCounts = new Dictionary<string, int>();
        recentChildFailures = new Queue<string>();
        lastExpandedNodeSummary = "<none>";
        lastLowLevelFailureDiagnostics = null;
        var conflictCounts = new Dictionary<string, int>();
        var recentConflicts = new Queue<string>();

        if (graph == null)
        {
            LastFailureReason = "ContinuousConflictBasedSearch requires a valid planning graph.";
            Debug.LogError(LastFailureReason);
            return null;
        }

        Dictionary<int, RobotPlanningState> robotStatesById = IndexRobotStates(robotStates);
        if (robotStatesById == null)
        {
            return null;
        }

        ContinuousCbsNode rootNode = BuildRootNode(robotStatesById);
        if (rootNode == null)
        {
            if (string.IsNullOrEmpty(LastFailureReason))
            {
                LastFailureReason = "Failed to build the root CBS node.";
            }

            Debug.LogWarning(
                "CBS failed while building the root node.\n" +
                PlanningDebugUtility.FormatGraph(graph) +
                "\n" + PlanningDebugUtility.FormatRobotStates(robotStates, graph));
            return null;
        }

        var openNodes = new List<ContinuousCbsNode>();
        var knownNodeSignatures = new HashSet<string>();
        if (!TryRegisterNodeSignature(rootNode.constraints, knownNodeSignatures) ||
            !TryEnqueueNode(rootNode, openNodes))
        {
            if (string.IsNullOrEmpty(LastFailureReason))
            {
                LastFailureReason = "Failed to register the root CBS node.";
            }

            Debug.LogWarning(
                "CBS could not register the root node.\n" +
                "root=" + BuildNodeSummary(rootNode));

            return null;
        }

        int highLevelIterations = 0;

        while (openNodes.Count > 0)
        {
            highLevelIterations++;
            if (highLevelIterations > MaxHighLevelIterations)
            {
                LastFailureReason =
                    "CBS aborted after reaching the high-level iteration limit (" +
                    MaxHighLevelIterations + ").";
                Debug.LogWarning(LastFailureReason);
                Debug.LogWarning("Last expanded CBS node: " + lastExpandedNodeSummary);
                LogConflictSummary(conflictCounts, recentConflicts);
                LogChildFailureSummary();
                return null;
            }

            ContinuousCbsNode currentNode = RemoveBestNode(openNodes);
            lastExpandedNodeSummary = BuildNodeSummary(currentNode);
            ReservationConflict conflict =
                ContinuousConflictDetector.FindFirstConflict(currentNode.schedulesByRobotId.Values);

            if (conflict == null)
            {
                return currentNode.schedulesByRobotId;
            }

            RecordConflict(conflict, conflictCounts, recentConflicts);

            bool addedAnyChild = false;
            if (UseCorridorBranching)
            {
                List<CorridorConflict> corridorConflicts =
                    GetCorridorConflicts(conflict, currentNode.schedulesByRobotId);

                for (int i = 0; i < corridorConflicts.Count; i++)
                {
                    CorridorConflict corridorConflict = corridorConflicts[i];
                    addedAnyChild |= AddChildNodeForCorridorConflict(
                        currentNode,
                        corridorConflict,
                        corridorConflict.robotATraversal.robotId,
                        robotStatesById,
                        openNodes,
                        knownNodeSignatures);

                    if (abortSearch)
                    {
                        return null;
                    }

                    addedAnyChild |= AddChildNodeForCorridorConflict(
                        currentNode,
                        corridorConflict,
                        corridorConflict.robotBTraversal.robotId,
                        robotStatesById,
                        openNodes,
                        knownNodeSignatures);

                    if (abortSearch)
                    {
                        return null;
                    }
                }
            }

            // Corridor reasoning is used here as an additional pruning aid,
            // not as a complete replacement for ordinary CBS branching.
            // Keeping the standard conflict children ensures we can still
            // reach a valid solution when the corridor abstraction is too
            // coarse for a particular junction / refuge arrangement.
            addedAnyChild |= AddChildNodeForConflict(
                currentNode,
                conflict,
                conflict.robotAId,
                robotStatesById,
                openNodes,
                knownNodeSignatures);
            if (abortSearch)
            {
                return null;
            }

            addedAnyChild |= AddChildNodeForConflict(
                currentNode,
                conflict,
                conflict.robotBId,
                robotStatesById,
                openNodes,
                knownNodeSignatures);
            if (abortSearch)
            {
                return null;
            }

            if (!addedAnyChild)
            {
                RecordChildFailure(
                    "no child generated",
                    "No child node could be generated for " +
                    PlanningDebugUtility.FormatConflict(conflict) +
                    " from parent " + BuildNodeSummary(currentNode));
                continue;
            }
        }

        if (string.IsNullOrEmpty(LastFailureReason))
        {
            LastFailureReason = "CBS exhausted all high-level nodes without finding a valid schedule.";
        }

        Debug.LogWarning("Last expanded CBS node: " + lastExpandedNodeSummary);
        LogChildFailureSummary();
        return null;
    }

    private Dictionary<int, RobotPlanningState> IndexRobotStates(IList<RobotPlanningState> robotStates)
    {
        if (robotStates == null)
        {
            Debug.LogError("ContinuousConflictBasedSearch received null robot states.");
            return null;
        }

        var robotStatesById = new Dictionary<int, RobotPlanningState>();
        for (int i = 0; i < robotStates.Count; i++)
        {
            RobotPlanningState robotState = robotStates[i];
            if (robotState == null)
            {
                Debug.LogError("ContinuousConflictBasedSearch received a null robot state.");
                return null;
            }

            if (!robotState.IsValid(graph))
            {
                Debug.LogError("ContinuousConflictBasedSearch received an invalid robot state for robot " +
                               robotState.robotId + ".\n" +
                               PlanningDebugUtility.FormatRobotState(robotState, graph));
                return null;
            }

            if (robotStatesById.ContainsKey(robotState.robotId))
            {
                Debug.LogError("ContinuousConflictBasedSearch received duplicate robot id " +
                               robotState.robotId + ".");
                return null;
            }

            robotStatesById.Add(robotState.robotId, robotState);
        }

        return robotStatesById;
    }

    private ContinuousCbsNode BuildRootNode(Dictionary<int, RobotPlanningState> robotStatesById)
    {
        var rootNode = new ContinuousCbsNode();

        foreach (KeyValuePair<int, RobotPlanningState> pair in robotStatesById)
        {
            RobotSchedule schedule = PlanForRobot(pair.Value, rootNode.constraints);
            if (schedule == null)
            {
                Debug.LogWarning(
                    "CBS root-node planning failed for " +
                    PlanningDebugUtility.FormatRobotState(pair.Value, graph) +
                    ". Reason: " + (string.IsNullOrEmpty(LastFailureReason) ? "<unknown>" : LastFailureReason));
                return null;
            }

            rootNode.schedulesByRobotId.Add(pair.Key, schedule);
        }

        return rootNode;
    }

    private bool AddChildNodeForConflict(
        ContinuousCbsNode parentNode,
        ReservationConflict conflict,
        int constrainedRobotId,
        Dictionary<int, RobotPlanningState> robotStatesById,
        List<ContinuousCbsNode> openNodes,
        HashSet<string> knownNodeSignatures)
    {
        RobotPlanningState robotState;
        if (!robotStatesById.TryGetValue(constrainedRobotId, out robotState))
        {
            RecordChildFailure(
                "missing robot state",
                "Missing robot state for constrained robot " + constrainedRobotId +
                " while branching on " + PlanningDebugUtility.FormatConflict(conflict) + ".");
            return false;
        }

        var childNode = new ContinuousCbsNode
        {
            constraints = new List<ContinuousCbsConstraint>(parentNode.constraints),
            schedulesByRobotId = new Dictionary<int, RobotSchedule>(parentNode.schedulesByRobotId)
        };

        List<ContinuousCbsConstraint> childConstraints =
            CreateConstraintsForConflict(
                conflict,
                constrainedRobotId,
                parentNode.schedulesByRobotId);

        bool addedAnyConstraint = false;
        var appliedConstraints = new List<ContinuousCbsConstraint>();
        for (int i = 0; i < childConstraints.Count; i++)
        {
            if (TryAddConstraint(childNode.constraints, childConstraints[i]))
            {
                addedAnyConstraint = true;
                appliedConstraints.Add(childConstraints[i]);
            }
        }

        if (!addedAnyConstraint)
        {
            RecordChildFailure(
                "no new constraint",
                "Branch for robot " + constrainedRobotId +
                " produced no new constraints for " +
                PlanningDebugUtility.FormatConflict(conflict) +
                ". Proposed:\n" + PlanningDebugUtility.FormatConstraints(childConstraints));
            return false;
        }

        if (!TryRegisterNodeSignature(childNode.constraints, knownNodeSignatures))
        {
            RecordChildFailure(
                "duplicate node signature",
                "Duplicate CBS node for robot " + constrainedRobotId +
                " after applying:\n" +
                PlanningDebugUtility.FormatConstraints(appliedConstraints));
            return false;
        }

        RobotSchedule replannedSchedule =
            PlanForRobot(robotState, childNode.constraints);
        if (replannedSchedule == null)
        {
            RobotSchedule currentSchedule;
            childNode.schedulesByRobotId.TryGetValue(constrainedRobotId, out currentSchedule);
            RecordChildFailure(
                "low-level replan failed",
                "Replan failed for " + PlanningDebugUtility.FormatRobotState(robotState, graph) +
                "\nconflict: " + PlanningDebugUtility.FormatConflict(conflict) +
                "\napplied constraints:\n" + PlanningDebugUtility.FormatConstraints(appliedConstraints) +
                "\nexisting schedule:\n" + PlanningDebugUtility.FormatSchedule(currentSchedule, 6) +
                "\nreason: " + (string.IsNullOrEmpty(LastFailureReason) ? "<unknown>" : LastFailureReason) +
                BuildLowLevelDiagnosticsSuffix());
            return false;
        }

        childNode.schedulesByRobotId[constrainedRobotId] = replannedSchedule;
        if (!TryEnqueueNode(childNode, openNodes))
        {
            RecordChildFailure(
                "enqueue failed",
                "Failed to enqueue child node for robot " + constrainedRobotId +
                " after " + PlanningDebugUtility.FormatConflict(conflict) + ".");
            return false;
        }

        return true;
    }

    private bool AddChildNodeForCorridorConflict(
        ContinuousCbsNode parentNode,
        CorridorConflict corridorConflict,
        int constrainedRobotId,
        Dictionary<int, RobotPlanningState> robotStatesById,
        List<ContinuousCbsNode> openNodes,
        HashSet<string> knownNodeSignatures)
    {
        RobotPlanningState robotState;
        if (!robotStatesById.TryGetValue(constrainedRobotId, out robotState))
        {
            RecordChildFailure(
                "missing robot state",
                "Missing robot state for constrained corridor robot " + constrainedRobotId + ".");
            return false;
        }

        var childNode = new ContinuousCbsNode
        {
            constraints = new List<ContinuousCbsConstraint>(parentNode.constraints),
            schedulesByRobotId = new Dictionary<int, RobotSchedule>(parentNode.schedulesByRobotId)
        };

        List<ContinuousCbsConstraint> corridorConstraints =
            CreateCorridorConstraints(corridorConflict, constrainedRobotId);

        bool addedAnyConstraint = false;
        var appliedConstraints = new List<ContinuousCbsConstraint>();
        for (int i = 0; i < corridorConstraints.Count; i++)
        {
            if (TryAddConstraint(childNode.constraints, corridorConstraints[i]))
            {
                addedAnyConstraint = true;
                appliedConstraints.Add(corridorConstraints[i]);
            }
        }

        if (!addedAnyConstraint)
        {
            RecordChildFailure(
                "no new corridor constraint",
                "Corridor branch for robot " + constrainedRobotId +
                " produced no new constraints.");
            return false;
        }

        if (!TryRegisterNodeSignature(childNode.constraints, knownNodeSignatures))
        {
            RecordChildFailure(
                "duplicate corridor node signature",
                "Duplicate corridor CBS node for robot " + constrainedRobotId +
                " after applying:\n" +
                PlanningDebugUtility.FormatConstraints(appliedConstraints));
            return false;
        }

        RobotSchedule replannedSchedule =
            PlanForRobot(robotState, childNode.constraints);
        if (replannedSchedule == null)
        {
            RecordChildFailure(
                "corridor replan failed",
                "Corridor replan failed for " +
                PlanningDebugUtility.FormatRobotState(robotState, graph) +
                "\napplied constraints:\n" + PlanningDebugUtility.FormatConstraints(appliedConstraints) +
                "\nreason: " + (string.IsNullOrEmpty(LastFailureReason) ? "<unknown>" : LastFailureReason) +
                BuildLowLevelDiagnosticsSuffix());
            return false;
        }

        childNode.schedulesByRobotId[constrainedRobotId] = replannedSchedule;
        if (!TryEnqueueNode(childNode, openNodes))
        {
            RecordChildFailure(
                "enqueue failed",
                "Failed to enqueue corridor child node for robot " + constrainedRobotId + ".");
            return false;
        }

        return true;
    }

    private RobotSchedule PlanForRobot(
        RobotPlanningState robotState,
        List<ContinuousCbsConstraint> constraints)
    {
        ReservationTable reservationTable = BuildReservationTable(robotState.robotId, constraints);
        var planner = new ContinuousTimePathPlanner(graph, reservationTable);
        RobotSchedule schedule = planner.FindSchedule(robotState);
        lastLowLevelFailureDiagnostics = planner.LastFailureDiagnostics;
        if (schedule == null && !string.IsNullOrEmpty(planner.LastFailureReason))
        {
            LastFailureReason = planner.LastFailureReason;
        }

        return schedule;
    }

    private ReservationTable BuildReservationTable(
        int robotId,
        List<ContinuousCbsConstraint> constraints)
    {
        var reservationTable = new ReservationTable();

        if (constraints == null)
        {
            return reservationTable;
        }

        for (int i = 0; i < constraints.Count; i++)
        {
            ContinuousCbsConstraint constraint = constraints[i];
            if (constraint == null || constraint.robotId != robotId)
            {
                continue;
            }

            if (constraint.type == ReservationConflictType.Vertex)
            {
                reservationTable.AddVertexReservation(new VertexReservation
                {
                    robotId = ConstraintReservationRobotId,
                    vertexId = constraint.vertexId,
                    interval = constraint.interval
                });
                continue;
            }

            reservationTable.AddEdgeReservation(new EdgeReservation
            {
                robotId = ConstraintReservationRobotId,
                edgeKey = constraint.edgeKey,
                interval = constraint.interval
            });
        }

        return reservationTable;
    }

    private List<ContinuousCbsConstraint> CreateConstraintsForConflict(
        ReservationConflict conflict,
        int constrainedRobotId,
        Dictionary<int, RobotSchedule> schedulesByRobotId)
    {
        var constraints = new List<ContinuousCbsConstraint>();

        if (conflict == null)
        {
            return constraints;
        }

        if (conflict.type == ReservationConflictType.Vertex)
        {
            int constrainedFromVertexId;
            int constrainedToVertexId;
            TimeInterval constrainedInterval;
            int otherFromVertexId;
            int otherToVertexId;
            TimeInterval otherInterval;
            GetConflictParticipantData(
                conflict,
                constrainedRobotId,
                out constrainedFromVertexId,
                out constrainedToVertexId,
                out constrainedInterval,
                out otherFromVertexId,
                out otherToVertexId,
                out otherInterval);

            int otherRobotId =
                constrainedRobotId == conflict.robotAId ? conflict.robotBId : conflict.robotAId;
            RobotSchedule otherRobotSchedule = null;
            if (schedulesByRobotId != null)
            {
                schedulesByRobotId.TryGetValue(otherRobotId, out otherRobotSchedule);
            }

            TimeInterval blockingInterval = GetVertexBlockingInterval(
                conflict.vertexId,
                otherFromVertexId,
                otherToVertexId,
                otherInterval,
                conflict.conflictInterval,
                otherRobotSchedule);

            constraints.Add(new ContinuousCbsConstraint
            {
                robotId = constrainedRobotId,
                type = ReservationConflictType.Vertex,
                interval = blockingInterval,
                vertexId = conflict.vertexId
            });

            // Blocking only the vertex overlap is too weak when the robot is
            // entering or leaving that vertex as part of a traversal. Adding
            // the matching edge constraint forces the replan to wait or reroute
            // earlier instead of recreating nearly the same vertex timing.
            if (IsArrivalAtVertex(conflict.vertexId, constrainedFromVertexId, constrainedToVertexId) ||
                IsDepartureFromVertex(conflict.vertexId, constrainedFromVertexId, constrainedToVertexId))
            {
                constraints.Add(new ContinuousCbsConstraint
                {
                    robotId = constrainedRobotId,
                    type = ReservationConflictType.Edge,
                    interval = blockingInterval,
                    edgeKey = new PlanningEdgeKey(constrainedFromVertexId, constrainedToVertexId)
                });
            }

            return constraints;
        }

        TimeInterval prioritizedInterval =
            constrainedRobotId == conflict.robotAId
                ? GetMeaningfulInterval(conflict.robotBInterval, conflict.conflictInterval)
                : GetMeaningfulInterval(conflict.robotAInterval, conflict.conflictInterval);

        constraints.Add(new ContinuousCbsConstraint
        {
            robotId = constrainedRobotId,
            type = ReservationConflictType.Edge,
            interval = prioritizedInterval,
            edgeKey = conflict.edgeKey
        });

        // A head-on or following conflict on an exclusive edge is effectively a
        // bottleneck conflict around that edge too. Reserving the edge alone is
        // often too weak because the yielding robot can still step back onto an
        // endpoint vertex immediately and recreate the same conflict at the
        // next CBS level.
        constraints.Add(new ContinuousCbsConstraint
        {
            robotId = constrainedRobotId,
            type = ReservationConflictType.Vertex,
            interval = prioritizedInterval,
            vertexId = conflict.edgeKey.vertexMinId
        });

        constraints.Add(new ContinuousCbsConstraint
        {
            robotId = constrainedRobotId,
            type = ReservationConflictType.Vertex,
            interval = prioritizedInterval,
            vertexId = conflict.edgeKey.vertexMaxId
        });

        return constraints;
    }

    private TimeInterval GetMeaningfulInterval(TimeInterval preferredInterval, TimeInterval fallbackInterval)
    {
        return preferredInterval.IsValid &&
               preferredInterval.endTime > preferredInterval.startTime + TimeComparisonEpsilon
            ? preferredInterval
            : fallbackInterval;
    }

    private TimeInterval GetVertexBlockingInterval(
        int vertexId,
        int otherFromVertexId,
        int otherToVertexId,
        TimeInterval otherInterval,
        TimeInterval fallbackInterval,
        RobotSchedule otherRobotSchedule)
    {
        TimeInterval blockingInterval = GetMeaningfulInterval(otherInterval, fallbackInterval);

        if (IsWaitAtVertex(vertexId, otherFromVertexId, otherToVertexId))
        {
            return blockingInterval;
        }

        if (IsArrivalAtVertex(vertexId, otherFromVertexId, otherToVertexId))
        {
            TimeInterval extendedVertexControlInterval;
            if (TryGetExtendedVertexControlInterval(
                otherRobotSchedule,
                vertexId,
                otherFromVertexId,
                otherToVertexId,
                otherInterval,
                out extendedVertexControlInterval))
            {
                return extendedVertexControlInterval;
            }
        }

        return blockingInterval;
    }

    private bool IsWaitAtVertex(int vertexId, int fromVertexId, int toVertexId)
    {
        return fromVertexId == vertexId && toVertexId == vertexId;
    }

    private bool IsArrivalAtVertex(int vertexId, int fromVertexId, int toVertexId)
    {
        return fromVertexId >= 0 &&
               fromVertexId != vertexId &&
               toVertexId == vertexId;
    }

    private bool IsDepartureFromVertex(int vertexId, int fromVertexId, int toVertexId)
    {
        return fromVertexId == vertexId &&
               toVertexId >= 0 &&
               toVertexId != vertexId;
    }

    private bool TryGetExtendedVertexControlInterval(
        RobotSchedule schedule,
        int vertexId,
        int incomingFromVertexId,
        int incomingToVertexId,
        TimeInterval incomingInterval,
        out TimeInterval controlInterval)
    {
        controlInterval = incomingInterval;

        if (schedule == null || schedule.segments == null || !incomingInterval.IsValid)
        {
            return false;
        }

        for (int i = 0; i < schedule.segments.Count; i++)
        {
            ScheduleSegment segment = schedule.segments[i];
            if (segment == null ||
                segment.type != ScheduleSegmentType.TraverseEdge ||
                segment.startVertexId != incomingFromVertexId ||
                segment.endVertexId != incomingToVertexId ||
                !segment.interval.Equals(incomingInterval))
            {
                continue;
            }

            float extendedEndTime = segment.interval.endTime;
            int nextIndex = i + 1;
            while (nextIndex < schedule.segments.Count)
            {
                ScheduleSegment nextSegment = schedule.segments[nextIndex];
                if (nextSegment == null)
                {
                    nextIndex++;
                    continue;
                }

                if (nextSegment.type == ScheduleSegmentType.WaitAtVertex &&
                    nextSegment.startVertexId == vertexId &&
                    AreTimesTouching(extendedEndTime, nextSegment.interval.startTime))
                {
                    extendedEndTime = nextSegment.interval.endTime;
                    nextIndex++;
                    continue;
                }

                if (nextSegment.type == ScheduleSegmentType.TraverseEdge &&
                    nextSegment.startVertexId == vertexId &&
                    AreTimesTouching(extendedEndTime, nextSegment.interval.startTime))
                {
                    // Conservatively treat the outgoing traversal as part of the
                    // local bottleneck control window, so another robot may not
                    // immediately dive back into the same junction.
                    extendedEndTime = nextSegment.interval.endTime;
                }

                break;
            }

            controlInterval = new TimeInterval(segment.interval.startTime, extendedEndTime);
            return true;
        }

        return false;
    }

    private void GetConflictParticipantData(
        ReservationConflict conflict,
        int constrainedRobotId,
        out int constrainedFromVertexId,
        out int constrainedToVertexId,
        out TimeInterval constrainedInterval,
        out int otherFromVertexId,
        out int otherToVertexId,
        out TimeInterval otherInterval)
    {
        if (constrainedRobotId == conflict.robotAId)
        {
            constrainedFromVertexId = conflict.robotAFromVertexId;
            constrainedToVertexId = conflict.robotAToVertexId;
            constrainedInterval = conflict.robotAInterval;
            otherFromVertexId = conflict.robotBFromVertexId;
            otherToVertexId = conflict.robotBToVertexId;
            otherInterval = conflict.robotBInterval;
            return;
        }

        constrainedFromVertexId = conflict.robotBFromVertexId;
        constrainedToVertexId = conflict.robotBToVertexId;
        constrainedInterval = conflict.robotBInterval;
        otherFromVertexId = conflict.robotAFromVertexId;
        otherToVertexId = conflict.robotAToVertexId;
        otherInterval = conflict.robotAInterval;
    }

    private ContinuousCbsNode RemoveBestNode(List<ContinuousCbsNode> openNodes)
    {
        int bestIndex = 0;
        ContinuousCbsNode bestNode = openNodes[0];

        for (int i = 1; i < openNodes.Count; i++)
        {
            ContinuousCbsNode candidate = openNodes[i];
            if (candidate.TotalCost < bestNode.TotalCost)
            {
                bestIndex = i;
                bestNode = candidate;
            }
        }

        openNodes.RemoveAt(bestIndex);
        return bestNode;
    }

    private bool TryRegisterNodeSignature(
        List<ContinuousCbsConstraint> constraints,
        HashSet<string> knownNodeSignatures)
    {
        if (knownNodeSignatures == null)
        {
            return false;
        }

        string nodeSignature = BuildNodeSignature(constraints);
        if (!knownNodeSignatures.Add(nodeSignature))
        {
            return false;
        }

        return true;
    }

    private bool TryEnqueueNode(
        ContinuousCbsNode node,
        List<ContinuousCbsNode> openNodes)
    {
        if (node == null)
        {
            return false;
        }

        if (openNodes.Count >= MaxOpenNodes)
        {
            if (!abortSearch)
            {
                LastFailureReason =
                    "CBS aborted after reaching the open-node limit (" + MaxOpenNodes + ").";
                Debug.LogWarning(LastFailureReason);
                Debug.LogWarning("Last expanded CBS node: " + lastExpandedNodeSummary);
                LogChildFailureSummary();
            }

            abortSearch = true;
            return false;
        }

        openNodes.Add(node);
        return true;
    }

    private bool TryAddConstraint(List<ContinuousCbsConstraint> constraints, ContinuousCbsConstraint newConstraint)
    {
        if (constraints == null || newConstraint == null || !newConstraint.interval.IsValid)
        {
            return false;
        }

        var mergedConstraint = new ContinuousCbsConstraint
        {
            robotId = newConstraint.robotId,
            type = newConstraint.type,
            interval = newConstraint.interval,
            vertexId = newConstraint.vertexId,
            edgeKey = newConstraint.edgeKey
        };

        for (int i = constraints.Count - 1; i >= 0; i--)
        {
            ContinuousCbsConstraint existingConstraint = constraints[i];
            if (!IsSameConstraintResource(existingConstraint, mergedConstraint))
            {
                continue;
            }

            if (ContainsInterval(existingConstraint.interval, mergedConstraint.interval))
            {
                return false;
            }

            if (DoIntervalsOverlapOrTouch(existingConstraint.interval, mergedConstraint.interval))
            {
                mergedConstraint.interval =
                    MergeIntervals(existingConstraint.interval, mergedConstraint.interval);
                constraints.RemoveAt(i);
            }
        }

        constraints.Add(mergedConstraint);
        return true;
    }

    private bool AreSameConstraint(ContinuousCbsConstraint first, ContinuousCbsConstraint second)
    {
        if (first == null || second == null)
        {
            return false;
        }

        return first.robotId == second.robotId &&
               first.type == second.type &&
               first.vertexId == second.vertexId &&
               first.edgeKey.Equals(second.edgeKey) &&
               first.interval.Equals(second.interval);
    }

    private bool IsSameConstraintResource(ContinuousCbsConstraint first, ContinuousCbsConstraint second)
    {
        if (first == null || second == null)
        {
            return false;
        }

        return first.robotId == second.robotId &&
               first.type == second.type &&
               first.vertexId == second.vertexId &&
               first.edgeKey.Equals(second.edgeKey);
    }

    private bool ContainsInterval(TimeInterval container, TimeInterval containee)
    {
        return container.startTime <= containee.startTime + TimeComparisonEpsilon &&
               container.endTime >= containee.endTime - TimeComparisonEpsilon;
    }

    private bool DoIntervalsOverlapOrTouch(TimeInterval first, TimeInterval second)
    {
        return first.startTime <= second.endTime + TimeComparisonEpsilon &&
               second.startTime <= first.endTime + TimeComparisonEpsilon;
    }

    private TimeInterval MergeIntervals(TimeInterval first, TimeInterval second)
    {
        float mergedStart = Mathf.Min(first.startTime, second.startTime);
        float mergedEnd = Mathf.Max(first.endTime, second.endTime);
        return new TimeInterval(mergedStart, mergedEnd);
    }

    private List<CorridorConflict> GetCorridorConflicts(
        ReservationConflict conflict,
        Dictionary<int, RobotSchedule> schedulesByRobotId)
    {
        var corridorConflicts = new List<CorridorConflict>();

        RobotSchedule scheduleA;
        RobotSchedule scheduleB;
        if (!schedulesByRobotId.TryGetValue(conflict.robotAId, out scheduleA) ||
            !schedulesByRobotId.TryGetValue(conflict.robotBId, out scheduleB))
        {
            return corridorConflicts;
        }

        foreach (Corridor corridor in GetCandidateCorridorsAroundConflict(conflict))
        {
            CorridorTraversal traversalA;
            CorridorTraversal traversalB;
            if (!TryExtractCorridorTraversal(scheduleA, corridor, out traversalA) ||
                !TryExtractCorridorTraversal(scheduleB, corridor, out traversalB))
            {
                continue;
            }

            // The corridor rule is useful even when one robot starts inside the
            // corridor and has to retreat to a farther refuge vertex before the
            // other robot can pass. For that reason we only require opposite
            // monotonic directions with an overlapping corridor footprint here,
            // rather than endpoint-to-endpoint traversals in a single shot.
            bool oppositeDirections = traversalA.direction == -traversalB.direction;
            int sharedStartIndex = Mathf.Max(traversalA.minVertexIndex, traversalB.minVertexIndex);
            int sharedEndIndex = Mathf.Min(traversalA.maxVertexIndex, traversalB.maxVertexIndex);

            if (!oppositeDirections || sharedStartIndex > sharedEndIndex)
            {
                continue;
            }

            corridorConflicts.Add(new CorridorConflict
            {
                corridor = corridor,
                robotATraversal = traversalA,
                robotBTraversal = traversalB,
                sharedStartIndex = sharedStartIndex,
                sharedEndIndex = sharedEndIndex,
                occupancyOverlapDuration = GetTraversalOverlapDuration(traversalA, traversalB)
            });
        }

        corridorConflicts.Sort(CompareCorridorConflicts);
        return corridorConflicts;
    }

    private List<ContinuousCbsConstraint> CreateCorridorConstraints(
        CorridorConflict corridorConflict,
        int constrainedRobotId)
    {
        CorridorTraversal prioritizedTraversal =
            corridorConflict.robotATraversal.robotId == constrainedRobotId
                ? corridorConflict.robotBTraversal
                : corridorConflict.robotATraversal;

        var constraints = new List<ContinuousCbsConstraint>();
        TimeInterval corridorInterval =
            new TimeInterval(prioritizedTraversal.entryTime, prioritizedTraversal.exitTime);

        for (int i = 0; i < corridorConflict.corridor.edgeKeys.Count; i++)
        {
            constraints.Add(new ContinuousCbsConstraint
            {
                robotId = constrainedRobotId,
                type = ReservationConflictType.Edge,
                interval = corridorInterval,
                edgeKey = corridorConflict.corridor.edgeKeys[i]
            });
        }

        // The yielding robot must stay out of the whole one-lane passage while
        // the prioritized robot occupies it. Reserving only internal vertices
        // is too weak for short passages such as a single bottleneck edge
        // C-D with a side refuge at C-E, because the yielding robot may need
        // to vacate endpoint C or D before the other robot can pass.
        for (int i = 0; i < corridorConflict.corridor.vertexIds.Count; i++)
        {
            constraints.Add(new ContinuousCbsConstraint
            {
                robotId = constrainedRobotId,
                type = ReservationConflictType.Vertex,
                interval = corridorInterval,
                vertexId = corridorConflict.corridor.vertexIds[i]
            });
        }

        constraints.Add(new ContinuousCbsConstraint
        {
            robotId = constrainedRobotId,
            type = ReservationConflictType.Vertex,
            interval = new TimeInterval(
                Mathf.Max(0f, prioritizedTraversal.exitTime - CorridorEndpointPadding),
                prioritizedTraversal.exitTime + CorridorEndpointPadding),
            vertexId = prioritizedTraversal.exitVertexId
        });

        return constraints;
    }

    private IEnumerable<Corridor> GetCandidateCorridorsAroundConflict(ReservationConflict conflict)
    {
        var yieldedEdgeKeys = new HashSet<PlanningEdgeKey>();
        var yieldedVertexKeys = new HashSet<int>();

        if (conflict == null)
        {
            yield break;
        }

        if (conflict.type == ReservationConflictType.Edge)
        {
            Corridor edgeCorridor;
            if (yieldedEdgeKeys.Add(conflict.edgeKey) &&
                TryBuildCorridorFromEdge(conflict.edgeKey, out edgeCorridor))
            {
                yield return edgeCorridor;
            }
            yield break;
        }

        if (conflict.vertexId < 0)
        {
            yield break;
        }

        Corridor vertexCorridor;
        if (graph.GetVertexDegree(conflict.vertexId) == 2 &&
            yieldedVertexKeys.Add(conflict.vertexId) &&
            TryBuildCorridorFromVertex(conflict.vertexId, out vertexCorridor))
        {
            yield return vertexCorridor;
        }

        foreach (PlanningEdgeKey candidateEdgeKey in GetCandidateCorridorEdgeKeys(conflict))
        {
            Corridor edgeCorridor;
            if (yieldedEdgeKeys.Add(candidateEdgeKey) &&
                TryBuildCorridorFromEdge(candidateEdgeKey, out edgeCorridor))
            {
                yield return edgeCorridor;
            }
        }
    }

    private IEnumerable<PlanningEdgeKey> GetCandidateCorridorEdgeKeys(ReservationConflict conflict)
    {
        var candidateEdgeKeys = new HashSet<PlanningEdgeKey>();
        AddCandidateCorridorEdgeKey(
            conflict.vertexId,
            conflict.robotAFromVertexId,
            conflict.robotAToVertexId,
            candidateEdgeKeys);
        AddCandidateCorridorEdgeKey(
            conflict.vertexId,
            conflict.robotBFromVertexId,
            conflict.robotBToVertexId,
            candidateEdgeKeys);

        foreach (PlanningEdgeKey candidateEdgeKey in candidateEdgeKeys)
        {
            yield return candidateEdgeKey;
        }
    }

    private void AddCandidateCorridorEdgeKey(
        int conflictVertexId,
        int fromVertexId,
        int toVertexId,
        HashSet<PlanningEdgeKey> candidateEdgeKeys)
    {
        if (candidateEdgeKeys == null)
        {
            return;
        }

        if (fromVertexId == conflictVertexId && toVertexId >= 0 && toVertexId != conflictVertexId)
        {
            PlanningEdge edge;
            if (graph.TryGetEdge(conflictVertexId, toVertexId, out edge))
            {
                candidateEdgeKeys.Add(edge.key);
            }
        }

        if (toVertexId == conflictVertexId && fromVertexId >= 0 && fromVertexId != conflictVertexId)
        {
            PlanningEdge edge;
            if (graph.TryGetEdge(conflictVertexId, fromVertexId, out edge))
            {
                candidateEdgeKeys.Add(edge.key);
            }
        }
    }

    private bool TryBuildCorridorFromEdge(PlanningEdgeKey seedEdgeKey, out Corridor corridor)
    {
        corridor = null;

        var visited = new HashSet<int> { seedEdgeKey.vertexMinId, seedEdgeKey.vertexMaxId };
        List<int> leftExtension;
        List<int> rightExtension;
        if (!TryExtendCorridor(seedEdgeKey.vertexMinId, seedEdgeKey.vertexMaxId, visited, out leftExtension) ||
            !TryExtendCorridor(seedEdgeKey.vertexMaxId, seedEdgeKey.vertexMinId, visited, out rightExtension))
        {
            return false;
        }

        var vertexIds = new List<int>();
        for (int i = leftExtension.Count - 1; i >= 0; i--)
        {
            vertexIds.Add(leftExtension[i]);
        }

        vertexIds.Add(seedEdgeKey.vertexMinId);
        vertexIds.Add(seedEdgeKey.vertexMaxId);

        for (int i = 0; i < rightExtension.Count; i++)
        {
            vertexIds.Add(rightExtension[i]);
        }

        return TryCreateCorridor(vertexIds, out corridor);
    }

    private bool TryBuildCorridorFromVertex(int seedVertexId, out Corridor corridor)
    {
        corridor = null;

        List<int> neighbors = new List<int>(graph.GetNeighborVertexIds(seedVertexId));
        if (neighbors.Count != 2)
        {
            return false;
        }

        var visited = new HashSet<int> { seedVertexId, neighbors[0], neighbors[1] };
        List<int> leftExtension;
        List<int> rightExtension;
        if (!TryExtendCorridor(neighbors[0], seedVertexId, visited, out leftExtension) ||
            !TryExtendCorridor(neighbors[1], seedVertexId, visited, out rightExtension))
        {
            return false;
        }

        var vertexIds = new List<int>();
        for (int i = leftExtension.Count - 1; i >= 0; i--)
        {
            vertexIds.Add(leftExtension[i]);
        }

        vertexIds.Add(neighbors[0]);
        vertexIds.Add(seedVertexId);
        vertexIds.Add(neighbors[1]);

        for (int i = 0; i < rightExtension.Count; i++)
        {
            vertexIds.Add(rightExtension[i]);
        }

        return TryCreateCorridor(vertexIds, out corridor);
    }

    private bool TryExtendCorridor(
        int startVertexId,
        int previousVertexId,
        HashSet<int> visited,
        out List<int> extension)
    {
        extension = new List<int>();

        int currentVertexId = startVertexId;
        int previous = previousVertexId;

        while (graph.GetVertexDegree(currentVertexId) == 2)
        {
            int nextVertexId = GetOtherNeighbor(currentVertexId, previous);
            if (nextVertexId < 0)
            {
                return false;
            }

            if (!visited.Add(nextVertexId))
            {
                return false;
            }

            extension.Add(nextVertexId);
            previous = currentVertexId;
            currentVertexId = nextVertexId;
        }

        return true;
    }

    private int GetOtherNeighbor(int vertexId, int excludedNeighborId)
    {
        foreach (int neighborId in graph.GetNeighborVertexIds(vertexId))
        {
            if (neighborId != excludedNeighborId)
            {
                return neighborId;
            }
        }

        return -1;
    }

    private bool TryCreateCorridor(List<int> vertexIds, out Corridor corridor)
    {
        corridor = null;

        // A one-lane passage may be a long chain or a single exclusive edge
        // between two refuge / branching vertices, so two vertices are enough.
        if (vertexIds == null || vertexIds.Count < 2)
        {
            return false;
        }

        var edgeKeys = new List<PlanningEdgeKey>();
        for (int i = 0; i < vertexIds.Count - 1; i++)
        {
            PlanningEdge edge;
            if (!graph.TryGetEdge(vertexIds[i], vertexIds[i + 1], out edge))
            {
                return false;
            }

            edgeKeys.Add(edge.key);
        }

        corridor = new Corridor(vertexIds, edgeKeys);
        return true;
    }

    private bool TryExtractCorridorTraversal(
        RobotSchedule schedule,
        Corridor corridor,
        out CorridorTraversal traversal)
    {
        traversal = null;

        if (schedule == null || schedule.segments == null || schedule.segments.Count == 0)
        {
            return false;
        }

        int firstTraversalIndex = -1;
        int lastTraversalIndex = -1;

        for (int i = 0; i < schedule.segments.Count; i++)
        {
            ScheduleSegment segment = schedule.segments[i];
            if (segment != null &&
                segment.type == ScheduleSegmentType.TraverseEdge &&
                corridor.edgeKeySet.Contains(segment.edgeKey))
            {
                if (firstTraversalIndex < 0)
                {
                    firstTraversalIndex = i;
                }

                lastTraversalIndex = i;
            }
        }

        if (firstTraversalIndex < 0 || lastTraversalIndex < 0)
        {
            return false;
        }

        ScheduleSegment firstTraversal = schedule.segments[firstTraversalIndex];
        ScheduleSegment lastTraversal = schedule.segments[lastTraversalIndex];
        float entryTime = firstTraversal.interval.startTime;
        float exitTime = lastTraversal.interval.endTime;

        int direction = 0;
        int expectedCurrentVertexId = firstTraversal.startVertexId;
        int currentCorridorIndex;
        if (!corridor.vertexIndexById.TryGetValue(expectedCurrentVertexId, out currentCorridorIndex))
        {
            return false;
        }

        for (int i = firstTraversalIndex; i <= lastTraversalIndex; i++)
        {
            ScheduleSegment segment = schedule.segments[i];
            if (segment == null)
            {
                continue;
            }

            if (segment.type == ScheduleSegmentType.WaitAtVertex)
            {
                if (segment.startVertexId != expectedCurrentVertexId)
                {
                    return false;
                }

                continue;
            }

            if (!corridor.edgeKeySet.Contains(segment.edgeKey))
            {
                return false;
            }

            if (segment.startVertexId != expectedCurrentVertexId)
            {
                return false;
            }

            int nextCorridorIndex;
            if (!corridor.vertexIndexById.TryGetValue(segment.endVertexId, out nextCorridorIndex))
            {
                return false;
            }

            int stepDirection = nextCorridorIndex - currentCorridorIndex;
            if (stepDirection != 1 && stepDirection != -1)
            {
                return false;
            }

            if (direction == 0)
            {
                direction = stepDirection;
            }
            else if (direction != stepDirection)
            {
                return false;
            }

            expectedCurrentVertexId = segment.endVertexId;
            currentCorridorIndex = nextCorridorIndex;
        }

        if (direction == 0)
        {
            return false;
        }

        int entryVertexIndex = corridor.vertexIndexById[firstTraversal.startVertexId];
        int exitVertexIndex = corridor.vertexIndexById[lastTraversal.endVertexId];

        // If the robot waits on the corridor endpoint immediately before
        // entering or immediately after exiting, that wait is part of the
        // effective one-lane passage occupancy too.
        for (int i = firstTraversalIndex - 1; i >= 0; i--)
        {
            ScheduleSegment segment = schedule.segments[i];
            if (segment == null ||
                segment.type != ScheduleSegmentType.WaitAtVertex ||
                segment.startVertexId != firstTraversal.startVertexId ||
                !AreTimesTouching(segment.interval.endTime, entryTime))
            {
                break;
            }

            entryTime = segment.interval.startTime;
        }

        for (int i = lastTraversalIndex + 1; i < schedule.segments.Count; i++)
        {
            ScheduleSegment segment = schedule.segments[i];
            if (segment == null ||
                segment.type != ScheduleSegmentType.WaitAtVertex ||
                segment.startVertexId != lastTraversal.endVertexId ||
                !AreTimesTouching(exitTime, segment.interval.startTime))
            {
                break;
            }

            exitTime = segment.interval.endTime;
        }

        traversal = new CorridorTraversal
        {
            robotId = schedule.robotId,
            entryVertexId = firstTraversal.startVertexId,
            exitVertexId = lastTraversal.endVertexId,
            entryTime = entryTime,
            exitTime = exitTime,
            entryVertexIndex = entryVertexIndex,
            exitVertexIndex = exitVertexIndex,
            // The path inside the corridor is monotonic, so the occupied
            // corridor footprint is the closed index range between the first
            // and last corridor vertices touched by the traversal.
            minVertexIndex = Mathf.Min(entryVertexIndex, exitVertexIndex),
            maxVertexIndex = Mathf.Max(entryVertexIndex, exitVertexIndex),
            direction = direction
        };

        return true;
    }

    private bool AreTimesTouching(float firstTime, float secondTime)
    {
        return Mathf.Abs(firstTime - secondTime) <= TimeComparisonEpsilon;
    }

    private float GetTraversalOverlapDuration(CorridorTraversal first, CorridorTraversal second)
    {
        float overlapStart = Mathf.Max(first.entryTime, second.entryTime);
        float overlapEnd = Mathf.Min(first.exitTime, second.exitTime);
        return Mathf.Max(0f, overlapEnd - overlapStart);
    }

    private int CompareCorridorConflicts(CorridorConflict first, CorridorConflict second)
    {
        int sharedVertexCountFirst = first.sharedEndIndex - first.sharedStartIndex + 1;
        int sharedVertexCountSecond = second.sharedEndIndex - second.sharedStartIndex + 1;

        int sharedVertexComparison = sharedVertexCountSecond.CompareTo(sharedVertexCountFirst);
        if (sharedVertexComparison != 0)
        {
            return sharedVertexComparison;
        }

        int overlapComparison =
            second.occupancyOverlapDuration.CompareTo(first.occupancyOverlapDuration);
        if (overlapComparison != 0)
        {
            return overlapComparison;
        }

        return second.corridor.vertexIds.Count.CompareTo(first.corridor.vertexIds.Count);
    }

    private string BuildNodeSignature(List<ContinuousCbsConstraint> constraints)
    {
        if (constraints == null || constraints.Count == 0)
        {
            return "root";
        }

        var orderedConstraints = new List<ContinuousCbsConstraint>(constraints);
        orderedConstraints.Sort(CompareConstraintsForSignature);

        var builder = new StringBuilder();
        for (int i = 0; i < orderedConstraints.Count; i++)
        {
            ContinuousCbsConstraint constraint = orderedConstraints[i];
            builder.Append(constraint.robotId);
            builder.Append('|');
            builder.Append((int)constraint.type);
            builder.Append('|');

            if (constraint.type == ReservationConflictType.Vertex)
            {
                builder.Append('V');
                builder.Append(constraint.vertexId);
            }
            else
            {
                builder.Append('E');
                builder.Append(constraint.edgeKey.vertexMinId);
                builder.Append('-');
                builder.Append(constraint.edgeKey.vertexMaxId);
            }

            builder.Append('|');
            builder.Append(QuantizeTime(constraint.interval.startTime));
            builder.Append('|');
            builder.Append(QuantizeTime(constraint.interval.endTime));
            builder.Append(';');
        }

        return builder.ToString();
    }

    private int CompareConstraintsForSignature(ContinuousCbsConstraint first, ContinuousCbsConstraint second)
    {
        int robotComparison = first.robotId.CompareTo(second.robotId);
        if (robotComparison != 0)
        {
            return robotComparison;
        }

        int typeComparison = ((int)first.type).CompareTo((int)second.type);
        if (typeComparison != 0)
        {
            return typeComparison;
        }

        if (first.type == ReservationConflictType.Vertex)
        {
            int vertexComparison = first.vertexId.CompareTo(second.vertexId);
            if (vertexComparison != 0)
            {
                return vertexComparison;
            }
        }
        else
        {
            int minComparison = first.edgeKey.vertexMinId.CompareTo(second.edgeKey.vertexMinId);
            if (minComparison != 0)
            {
                return minComparison;
            }

            int maxComparison = first.edgeKey.vertexMaxId.CompareTo(second.edgeKey.vertexMaxId);
            if (maxComparison != 0)
            {
                return maxComparison;
            }
        }

        int startComparison =
            QuantizeTime(first.interval.startTime).CompareTo(QuantizeTime(second.interval.startTime));
        if (startComparison != 0)
        {
            return startComparison;
        }

        return QuantizeTime(first.interval.endTime).CompareTo(QuantizeTime(second.interval.endTime));
    }

    private long QuantizeTime(float time)
    {
        if (float.IsPositiveInfinity(time))
        {
            return long.MaxValue;
        }

        return (long)Math.Round(time / SignatureTimeQuantization);
    }

    private void RecordConflict(
        ReservationConflict conflict,
        Dictionary<string, int> conflictCounts,
        Queue<string> recentConflicts)
    {
        if (conflict == null || conflictCounts == null || recentConflicts == null)
        {
            return;
        }

        string signature = BuildConflictSignature(conflict);
        int count;
        conflictCounts.TryGetValue(signature, out count);
        conflictCounts[signature] = count + 1;

        recentConflicts.Enqueue(signature);
        while (recentConflicts.Count > MaxConflictHistory)
        {
            recentConflicts.Dequeue();
        }
    }

    private void LogConflictSummary(
        Dictionary<string, int> conflictCounts,
        Queue<string> recentConflicts)
    {
        if (conflictCounts == null || conflictCounts.Count == 0)
        {
            return;
        }

        var rankedConflicts = new List<KeyValuePair<string, int>>(conflictCounts);
        rankedConflicts.Sort((first, second) => second.Value.CompareTo(first.Value));

        var builder = new StringBuilder();
        builder.Append("CBS conflict summary:");

        int entryCount = Mathf.Min(MaxConflictSummaryEntries, rankedConflicts.Count);
        for (int i = 0; i < entryCount; i++)
        {
            builder.Append("\n");
            builder.Append(i + 1);
            builder.Append(". ");
            builder.Append(rankedConflicts[i].Value);
            builder.Append("x ");
            builder.Append(rankedConflicts[i].Key);
        }

        if (recentConflicts != null && recentConflicts.Count > 0)
        {
            builder.Append("\nRecent conflicts:");
            foreach (string signature in recentConflicts)
            {
                builder.Append("\n- ");
                builder.Append(signature);
            }
        }

        Debug.LogWarning(builder.ToString());
    }

    private void RecordChildFailure(string kind, string detail)
    {
        if (string.IsNullOrEmpty(kind))
        {
            kind = "unknown";
        }

        int count;
        childFailureCounts.TryGetValue(kind, out count);
        childFailureCounts[kind] = count + 1;

        if (string.IsNullOrEmpty(detail))
        {
            return;
        }

        recentChildFailures.Enqueue(kind + ": " + detail);
        while (recentChildFailures.Count > MaxChildFailureHistory)
        {
            recentChildFailures.Dequeue();
        }
    }

    private void LogChildFailureSummary()
    {
        if (childFailureCounts == null || childFailureCounts.Count == 0)
        {
            return;
        }

        var rankedFailures = new List<KeyValuePair<string, int>>(childFailureCounts);
        rankedFailures.Sort((first, second) => second.Value.CompareTo(first.Value));

        var builder = new StringBuilder();
        builder.Append("CBS child failure summary:");

        for (int i = 0; i < rankedFailures.Count; i++)
        {
            builder.Append("\n");
            builder.Append(i + 1);
            builder.Append(". ");
            builder.Append(rankedFailures[i].Value);
            builder.Append("x ");
            builder.Append(rankedFailures[i].Key);
        }

        if (recentChildFailures != null && recentChildFailures.Count > 0)
        {
            builder.Append("\nRecent child failures:");
            foreach (string sample in recentChildFailures)
            {
                builder.Append("\n- ");
                builder.Append(sample);
            }
        }

        Debug.LogWarning(builder.ToString());
    }

    private string BuildNodeSummary(ContinuousCbsNode node)
    {
        if (node == null)
        {
            return "<null node>";
        }

        return "cost=" + node.TotalCost.ToString("0.###") +
               " constraints=" + (node.constraints != null ? node.constraints.Count : 0) +
               " schedules=" + (node.schedulesByRobotId != null ? node.schedulesByRobotId.Count : 0);
    }

    private string BuildLowLevelDiagnosticsSuffix()
    {
        return string.IsNullOrEmpty(lastLowLevelFailureDiagnostics)
            ? string.Empty
            : "\n" + lastLowLevelFailureDiagnostics;
    }

    private string BuildConflictSignature(ReservationConflict conflict)
    {
        var builder = new StringBuilder();
        builder.Append(conflict.type);
        builder.Append(' ');

        if (conflict.type == ReservationConflictType.Vertex)
        {
            builder.Append("V");
            builder.Append(conflict.vertexId);
        }
        else
        {
            builder.Append("E");
            builder.Append(conflict.edgeKey.vertexMinId);
            builder.Append('-');
            builder.Append(conflict.edgeKey.vertexMaxId);
        }

        builder.Append(" @ ");
        builder.Append(QuantizeTime(conflict.conflictInterval.startTime));
        builder.Append("..");
        builder.Append(QuantizeTime(conflict.conflictInterval.endTime));
        builder.Append(" r");
        builder.Append(conflict.robotAId);
        builder.Append('[');
        builder.Append(conflict.robotAFromVertexId);
        builder.Append("->");
        builder.Append(conflict.robotAToVertexId);
        builder.Append("] vs r");
        builder.Append(conflict.robotBId);
        builder.Append('[');
        builder.Append(conflict.robotBFromVertexId);
        builder.Append("->");
        builder.Append(conflict.robotBToVertexId);
        builder.Append(']');
        return builder.ToString();
    }

    private class Corridor
    {
        public readonly List<int> vertexIds;
        public readonly List<PlanningEdgeKey> edgeKeys;
        public readonly HashSet<PlanningEdgeKey> edgeKeySet = new HashSet<PlanningEdgeKey>();
        public readonly Dictionary<int, int> vertexIndexById = new Dictionary<int, int>();

        public int endpointA
        {
            get { return vertexIds[0]; }
        }

        public int endpointB
        {
            get { return vertexIds[vertexIds.Count - 1]; }
        }

        public Corridor(List<int> vertexIds, List<PlanningEdgeKey> edgeKeys)
        {
            this.vertexIds = vertexIds;
            this.edgeKeys = edgeKeys;

            for (int i = 0; i < edgeKeys.Count; i++)
            {
                edgeKeySet.Add(edgeKeys[i]);
            }

            for (int i = 0; i < vertexIds.Count; i++)
            {
                vertexIndexById[vertexIds[i]] = i;
            }
        }

        public bool IsEndpoint(int vertexId)
        {
            return vertexId == endpointA || vertexId == endpointB;
        }
    }

    private class CorridorTraversal
    {
        public int robotId;
        public int entryVertexId;
        public int exitVertexId;
        public float entryTime;
        public float exitTime;
        public int entryVertexIndex;
        public int exitVertexIndex;
        public int minVertexIndex;
        public int maxVertexIndex;
        public int direction;
    }

    private class CorridorConflict
    {
        public Corridor corridor;
        public CorridorTraversal robotATraversal;
        public CorridorTraversal robotBTraversal;
        public int sharedStartIndex;
        public int sharedEndIndex;
        public float occupancyOverlapDuration;
    }
}
