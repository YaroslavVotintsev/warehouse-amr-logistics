using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Mapf.Authoring;
using Mapf.UnityAdapter;
using UnityEngine;

namespace TaskPlanning
{
    public sealed class TaskScheduler : MonoBehaviour
    {
        [SerializeField] private TaskPlanningAlgorithmType algorithm = TaskPlanningAlgorithmType.GreedyNearestFeasible;
        [SerializeField] private MapfCoordinator coordinator;
        [SerializeField] private MapfSceneGraph sceneGraph;
        [SerializeField] private float arrivalDistance = 0.08f;
        [SerializeField] private float pendingRetryIntervalSeconds = 0.5f;
        [SerializeField] private float costAmrSpeed = 1f;
        [SerializeField] private TaskPlanningCostWeights costWeights = new();
        [SerializeField] private bool enableSoftAmrReservations;
        [SerializeField, Min(0f)] private float reassignmentCostImprovementThreshold = 1f;
        [SerializeField] private bool autoDiscoverSceneObjects = true;
        [SerializeField] private List<TaskPlanningAmr> amrs = new();
        [SerializeField] private List<PalletLoadingPoint> loadingPoints = new();

        private readonly List<ITaskPlanningTask> _pendingTasks = new();
        private readonly Dictionary<TaskPlanningAmr, ActiveTaskExecution> _activeAssignments = new();
        private ITaskDispatchAlgorithm _dispatchAlgorithm;
        private RoadmapDistanceService _distances;
        private float _nextPendingRetryTime;
        private int _generatedRemovalTaskNumber;

        private void Awake()
        {
            coordinator ??= FindAnyObjectByType<MapfCoordinator>();
            sceneGraph ??= FindAnyObjectByType<MapfSceneGraph>();
            _dispatchAlgorithm = CreateAlgorithm(algorithm);

            if (autoDiscoverSceneObjects)
                DiscoverSceneObjects();
        }

        private void Start()
        {
            RefreshDistances();
            TryDispatchPendingTasks();
        }

        private void Update()
        {
            if (_pendingTasks.Count == 0 || Time.time < _nextPendingRetryTime)
                return;

            _nextPendingRetryTime = Time.time + Mathf.Max(0.05f, pendingRetryIntervalSeconds);
            TryDispatchPendingTasks();
        }

        public void EnqueueTask(DeliveryTaskRequest request)
        {
            EnqueueTasks(new[] { request });
        }

        public void EnqueueTasks(IEnumerable<DeliveryTaskRequest> requests)
        {
            if (requests == null)
                throw new ArgumentNullException(nameof(requests));

            var enqueuedTime = Time.time;
            var queuedAny = false;
            foreach (var request in requests)
            {
                if (!ValidateRequest(request))
                    continue;

                _pendingTasks.Add(new DeliveryPlanningTask(request.taskId, request.pallet, request.workstation, enqueuedTime));
                Debug.Log($"MES task queued: task={request.taskId} kit={request.pallet.KitId} workstation={request.workstation.WorkstationId}", this);
                queuedAny = true;
            }

            if (queuedAny)
                TryDispatchPendingTasks();
        }

        [ContextMenu("Discover Scene Objects")]
        public void DiscoverSceneObjects()
        {
            amrs = FindObjectsByType<TaskPlanningAmr>(FindObjectsInactive.Exclude).OrderBy(a => a.AmrId).ToList();
            loadingPoints = FindObjectsByType<PalletLoadingPoint>(FindObjectsInactive.Exclude).OrderBy(p => p.LoadingPointId).ToList();
        }

        private void TryDispatchPendingTasks()
        {
            if (_pendingTasks.Count == 0)
                return;

            if (coordinator == null || sceneGraph == null)
            {
                Debug.LogError("TaskScheduler requires MapfCoordinator and MapfSceneGraph.", this);
                return;
            }

            RefreshDistances();
            CompleteAlreadyParkedRemovalTasks();
            if (_pendingTasks.Count == 0)
                return;

            var taskSnapshot = _pendingTasks.ToArray();
            var evaluator = new TaskPlanningCostEvaluator(_distances, costWeights, costAmrSpeed, taskSnapshot, Time.time);
            var candidateAmrs = GetDispatchCandidateAmrs(taskSnapshot, evaluator);
            if (candidateAmrs.Count == 0)
                return;

            var problem = new DispatchProblem(taskSnapshot, candidateAmrs, loadingPoints, _distances, evaluator, Time.time);
            var plan = _dispatchAlgorithm.Solve(problem);
            foreach (var assignment in plan.Assignments)
            {
                if (!TryStartAssignment(assignment))
                    continue;

                _pendingTasks.Remove(assignment.Task);
            }
        }

        private bool TryStartAssignment(DispatchAssignment assignment)
        {
            if (!assignment.IsValid)
                return false;

            if (_activeAssignments.TryGetValue(assignment.Amr, out var activeAssignment))
            {
                if (!CanReplaceActiveAssignment(activeAssignment, assignment))
                    return false;

                CancelActiveAssignment(activeAssignment, true);
            }

            if (!assignment.Amr.TryReserve())
                return false;

            switch (assignment.Task)
            {
                case DeliveryPlanningTask delivery:
                    return TryStartDelivery(delivery, assignment);
                case PalletRemovalPlanningTask removal:
                    return TryStartRemoval(removal, assignment);
                default:
                    assignment.Amr.Release();
                    return false;
            }
        }

        private bool TryStartDelivery(DeliveryPlanningTask task, DispatchAssignment assignment)
        {
            if (!assignment.Pallet.TryReserve())
            {
                assignment.Amr.Release();
                return false;
            }

            if (!task.Workstation.Enqueue(assignment.Pallet))
            {
                assignment.Pallet.ReleaseReservation();
                assignment.Amr.Release();
                return false;
            }

            if (!assignment.LoadingPoint.Enqueue(assignment.Pallet))
            {
                task.Workstation.RemoveQueued(assignment.Pallet);
                assignment.Pallet.ReleaseReservation();
                assignment.Amr.Release();
                return false;
            }

            Debug.Log(
                $"Task dispatched: task={task.TaskId} amr={assignment.Amr.AmrId} pallet={assignment.Pallet.PalletId} " +
                $"loading={assignment.LoadingPoint.LoadingPointId} score={assignment.Score:0.###}",
                this);
            var execution = RegisterActiveAssignment(assignment);
            execution.Coroutine = StartCoroutine(RunDelivery(task, assignment, execution));
            return true;
        }

        private bool TryStartRemoval(PalletRemovalPlanningTask task, DispatchAssignment assignment)
        {
            if (!assignment.Pallet.TryReserveForRemoval())
            {
                assignment.Amr.Release();
                return false;
            }

            Debug.Log(
                $"Pallet removal dispatched: task={task.TaskId} amr={assignment.Amr.AmrId} pallet={assignment.Pallet.PalletId} " +
                $"parking={NodeId(assignment.RemovalTargetNode)} score={assignment.Score:0.###}",
                this);
            var execution = RegisterActiveAssignment(assignment);
            execution.Coroutine = StartCoroutine(RunPalletRemoval(task, assignment, execution));
            return true;
        }

        private IEnumerator RunDelivery(DeliveryPlanningTask request, DispatchAssignment assignment, ActiveTaskExecution execution)
        {
            yield return MoveAmrTo(assignment.Amr, assignment.Pallet.CurrentNode);
            assignment.Pallet.MarkAttaching();
            yield return new WaitForSeconds(assignment.Pallet.AttachDurationSeconds);
            assignment.Pallet.AttachTo(assignment.Amr);
            execution.HasAttachedPallet = true;
            TryDispatchPendingTasks();

            yield return WaitForLoadingAndWorkstationTurn(request, assignment);
            yield return MoveAmrTo(assignment.Amr, assignment.LoadingPoint.Node);
            assignment.Pallet.MarkLoading();
            yield return new WaitForSeconds(assignment.Pallet.LoadDurationSeconds);
            assignment.Pallet.MarkLoaded();

            coordinator.RequestAgentGoal(assignment.Amr.MapfAgent, request.Workstation.Node);
            assignment.LoadingPoint.ReleaseReservation(assignment.Pallet);
            TryDispatchPendingTasks();
            yield return WaitForAmrAt(assignment.Amr, request.Workstation.Node);

            assignment.Pallet.MarkDetaching();
            yield return new WaitForSeconds(assignment.Pallet.DetachDurationSeconds);
            assignment.Pallet.DetachAt(request.Workstation.Node, PalletStatus.Unloading);
            assignment.Amr.Release();
            CompleteActiveAssignment(assignment.Amr);
            TryDispatchPendingTasks();

            yield return new WaitForSeconds(assignment.Pallet.UnloadDurationSeconds);
            request.Workstation.ReleaseReservation(assignment.Pallet);
            if (IsAtParkingNode(assignment.Pallet))
            {
                assignment.Pallet.MarkUnloadedAvailable();
                Debug.Log(
                    $"Pallet already parked after delivery: task={request.TaskId} pallet={assignment.Pallet.PalletId} node={NodeId(assignment.Pallet.CurrentNode)}",
                    this);
            }
            else
            {
                assignment.Pallet.MarkAwaitingRemoval();
                EnqueueRemovalTask(assignment.Pallet, request.Workstation);
            }

            Debug.Log($"MES task completed: task={request.TaskId} amr={assignment.Amr.AmrId} pallet={assignment.Pallet.PalletId}", this);
            TryDispatchPendingTasks();
        }

        private IEnumerator RunPalletRemoval(PalletRemovalPlanningTask request, DispatchAssignment assignment, ActiveTaskExecution execution)
        {
            yield return MoveAmrTo(assignment.Amr, assignment.Pallet.CurrentNode);
            assignment.Pallet.MarkAttaching();
            yield return new WaitForSeconds(assignment.Pallet.AttachDurationSeconds);
            assignment.Pallet.AttachTo(assignment.Amr);
            execution.HasAttachedPallet = true;

            yield return MoveAmrTo(assignment.Amr, assignment.RemovalTargetNode);
            assignment.Pallet.MarkDetaching();
            yield return new WaitForSeconds(assignment.Pallet.DetachDurationSeconds);
            assignment.Pallet.DetachAt(assignment.RemovalTargetNode);
            assignment.Pallet.MarkUnloadedAvailable();
            assignment.Amr.Release();

            Debug.Log($"Pallet removal completed: task={request.TaskId} amr={assignment.Amr.AmrId} pallet={assignment.Pallet.PalletId}", this);
            CompleteActiveAssignment(assignment.Amr);
            TryDispatchPendingTasks();
        }

        private IEnumerator MoveAmrTo(TaskPlanningAmr amr, MapfNode goal)
        {
            if (amr == null || goal == null)
                yield break;

            coordinator.RequestAgentGoal(amr.MapfAgent, goal);
            yield return WaitForAmrAt(amr, goal);
        }

        private IEnumerator WaitForLoadingAndWorkstationTurn(DeliveryPlanningTask request, DispatchAssignment assignment)
        {
            while (true)
            {
                if (!assignment.LoadingPoint.CanReserveNext(assignment.Pallet) ||
                    !request.Workstation.CanReserveNext(assignment.Pallet))
                {
                    yield return null;
                    continue;
                }

                if (assignment.LoadingPoint.TryReserveNext(assignment.Pallet) &&
                    request.Workstation.TryReserveNext(assignment.Pallet))
                    yield break;

                assignment.LoadingPoint.ReleaseReservation(assignment.Pallet);
                request.Workstation.ReleaseReservation(assignment.Pallet);
                yield return null;
            }
        }

        private IEnumerator WaitForAmrAt(TaskPlanningAmr amr, MapfNode goal)
        {
            if (amr == null || goal == null)
                yield break;

            while (Vector2.Distance(amr.transform.position, goal.transform.position) > arrivalDistance)
                yield return null;
        }

        private List<TaskPlanningAmr> GetDispatchCandidateAmrs(
            IReadOnlyList<ITaskPlanningTask> taskSnapshot,
            TaskPlanningCostEvaluator evaluator)
        {
            var candidates = new List<TaskPlanningAmr>();
            foreach (var amr in amrs)
            {
                if (amr == null)
                    continue;

                if (!amr.IsBusy)
                {
                    candidates.Add(amr);
                    continue;
                }

                if (CanOfferForSoftReassignment(amr, taskSnapshot, evaluator))
                    candidates.Add(amr);
            }

            return candidates;
        }

        private bool CanOfferForSoftReassignment(
            TaskPlanningAmr amr,
            IReadOnlyList<ITaskPlanningTask> taskSnapshot,
            TaskPlanningCostEvaluator evaluator)
        {
            if (!enableSoftAmrReservations ||
                taskSnapshot.Count == 0 ||
                !_activeAssignments.TryGetValue(amr, out var activeAssignment) ||
                !activeAssignment.CanReassign)
                return false;

            var currentCost = EvaluateActiveAssignment(activeAssignment, evaluator);
            if (!currentCost.IsFeasible)
                return false;

            foreach (var task in taskSnapshot)
            {
                foreach (var cost in EvaluatePotentialReassignmentCosts(amr, task, evaluator))
                {
                    if (!cost.IsFeasible)
                        continue;

                    if (IsEnoughReassignmentImprovement(currentCost.TotalCost, cost.TotalCost))
                        return true;
                }
            }

            return false;
        }

        private bool CanReplaceActiveAssignment(ActiveTaskExecution activeAssignment, DispatchAssignment replacement)
        {
            if (!enableSoftAmrReservations || !activeAssignment.CanReassign)
                return false;

            var taskSnapshot = _pendingTasks.ToArray();
            var evaluator = new TaskPlanningCostEvaluator(_distances, costWeights, costAmrSpeed, taskSnapshot, Time.time);
            var currentCost = EvaluateActiveAssignment(activeAssignment, evaluator);
            if (!currentCost.IsFeasible)
                return false;

            var improvement = currentCost.TotalCost - replacement.Score;
            if (!IsEnoughReassignmentImprovement(currentCost.TotalCost, replacement.Score))
                return false;

            Debug.Log(
                $"Task reassigned before pallet attach: amr={activeAssignment.Assignment.Amr.AmrId} " +
                $"from={activeAssignment.Assignment.Task.TaskId} to={replacement.Task.TaskId} " +
                $"improvement={improvement:0.###}",
                this);
            return true;
        }

        private CostEvaluation EvaluateActiveAssignment(
            ActiveTaskExecution activeAssignment,
            TaskPlanningCostEvaluator evaluator)
        {
            switch (activeAssignment.Assignment.Task)
            {
                case DeliveryPlanningTask delivery:
                    return evaluator.EvaluateActiveAssignment(
                        activeAssignment.Assignment.Amr,
                        delivery,
                        activeAssignment.Assignment.LoadingPoint);
                case PalletRemovalPlanningTask removal:
                    return evaluator.EvaluateActiveAssignment(activeAssignment.Assignment.Amr, removal);
                default:
                    return CostEvaluation.Infeasible;
            }
        }

        private IEnumerable<CostEvaluation> EvaluatePotentialReassignmentCosts(
            TaskPlanningAmr amr,
            ITaskPlanningTask task,
            TaskPlanningCostEvaluator evaluator)
        {
            switch (task)
            {
                case DeliveryPlanningTask delivery:
                    foreach (var loadingPoint in loadingPoints)
                        yield return evaluator.Evaluate(amr, delivery, loadingPoint);
                    break;
                case PalletRemovalPlanningTask removal:
                    yield return evaluator.Evaluate(amr, removal);
                    break;
            }
        }

        private bool IsEnoughReassignmentImprovement(double currentCost, double replacementCost)
        {
            var threshold = Math.Max(0.0, reassignmentCostImprovementThreshold);
            return currentCost - replacementCost >= threshold;
        }

        private ActiveTaskExecution RegisterActiveAssignment(DispatchAssignment assignment)
        {
            var execution = new ActiveTaskExecution(assignment);
            _activeAssignments[assignment.Amr] = execution;
            return execution;
        }

        private void CompleteActiveAssignment(TaskPlanningAmr amr)
        {
            if (amr != null)
                _activeAssignments.Remove(amr);
        }

        private void CancelActiveAssignment(ActiveTaskExecution execution, bool requeueTask)
        {
            if (execution.Coroutine != null)
                StopCoroutine(execution.Coroutine);

            var assignment = execution.Assignment;
            switch (assignment.Task)
            {
                case DeliveryPlanningTask delivery:
                    delivery.Workstation.RemoveQueued(assignment.Pallet);
                    delivery.Workstation.ReleaseReservation(assignment.Pallet);
                    assignment.LoadingPoint.RemoveQueued(assignment.Pallet);
                    assignment.LoadingPoint.ReleaseReservation(assignment.Pallet);
                    assignment.Pallet.ReleasePendingReservation(PalletStatus.Available);
                    break;
                case PalletRemovalPlanningTask:
                    assignment.Pallet.ReleasePendingReservation(PalletStatus.AwaitingRemoval);
                    break;
            }

            CompleteActiveAssignment(assignment.Amr);
            assignment.Amr.Release();

            if (requeueTask && !_pendingTasks.Contains(assignment.Task))
                _pendingTasks.Add(assignment.Task);
        }

        private void RefreshDistances()
        {
            if (sceneGraph != null)
                _distances = new RoadmapDistanceService(sceneGraph);
        }

        private static ITaskDispatchAlgorithm CreateAlgorithm(TaskPlanningAlgorithmType algorithm)
        {
            switch (algorithm)
            {
                case TaskPlanningAlgorithmType.NearestDispatching:
                default:
                    return new NearestDispatching();
            }
        }

        private void EnqueueRemovalTask(PalletMarker pallet, WorkstationDeliveryPoint workstation)
        {
            if (pallet == null || pallet.ParkingNode == null)
            {
                Debug.LogWarning($"Pallet '{(pallet != null ? pallet.PalletId : "<null>")}' has no parking node; removal task was not created.", this);
                return;
            }

            if (IsAtParkingNode(pallet))
            {
                pallet.MarkUnloadedAvailable();
                Debug.Log($"Pallet removal skipped: pallet={pallet.PalletId} already at parking node {NodeId(pallet.ParkingNode)}", this);
                return;
            }

            if (_pendingTasks.Any(task => task.TaskType == TaskPlanningTaskType.PalletRemoval && task.Pallet == pallet))
                return;

            var taskId = $"REM-{++_generatedRemovalTaskNumber:0000}";
            _pendingTasks.Add(new PalletRemovalPlanningTask(taskId, pallet, workstation, Time.time));
            Debug.Log($"Pallet removal queued: task={taskId} pallet={pallet.PalletId} parking={NodeId(pallet.ParkingNode)}", this);
        }

        private void CompleteAlreadyParkedRemovalTasks()
        {
            for (var i = _pendingTasks.Count - 1; i >= 0; i--)
            {
                if (_pendingTasks[i] is not PalletRemovalPlanningTask removal || !IsAtParkingNode(removal.Pallet))
                    continue;

                removal.Pallet.MarkUnloadedAvailable();
                _pendingTasks.RemoveAt(i);
                Debug.Log($"Pallet removal completed without AMR: task={removal.TaskId} pallet={removal.Pallet.PalletId} already parked.", this);
            }
        }

        private static bool IsAtParkingNode(PalletMarker pallet)
        {
            return pallet != null &&
                pallet.CurrentNode != null &&
                pallet.ParkingNode != null &&
                pallet.CurrentNode == pallet.ParkingNode;
        }

        private static string NodeId(MapfNode node)
        {
            return node != null ? node.StableId : "<none>";
        }

        private bool ValidateRequest(DeliveryTaskRequest request)
        {
            if (request == null)
            {
                Debug.LogWarning("Ignoring null MES task request.", this);
                return false;
            }

            if (request.pallet == null)
            {
                Debug.LogWarning("Ignoring MES task with missing pallet/kit.", this);
                return false;
            }

            if (request.workstation == null || request.workstation.Node == null)
            {
                Debug.LogWarning($"Ignoring MES task '{request.taskId}' with missing workstation or workstation node.", this);
                return false;
            }

            if (!request.workstation.Accepts(request.pallet))
            {
                Debug.LogWarning($"Ignoring MES task '{request.taskId}' because workstation does not accept kit '{request.pallet.KitId}'.", this);
                return false;
            }

            return true;
        }

        private sealed class ActiveTaskExecution
        {
            public ActiveTaskExecution(DispatchAssignment assignment)
            {
                Assignment = assignment;
            }

            public DispatchAssignment Assignment { get; }
            public Coroutine Coroutine { get; set; }
            public bool HasAttachedPallet { get; set; }

            public bool CanReassign => !HasAttachedPallet && Assignment.Amr.AttachedPallet == null;
        }
    }
}
