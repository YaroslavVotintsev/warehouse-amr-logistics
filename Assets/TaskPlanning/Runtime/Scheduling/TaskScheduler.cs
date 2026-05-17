using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Mapf.Authoring;
using Mapf.UnityAdapter;
using UnityEngine;
using UnityEngine.Serialization;

namespace TaskPlanning
{
    public sealed class TaskScheduler : MonoBehaviour
    {
        [SerializeField] private TaskPlanningAlgorithmType algorithm = TaskPlanningAlgorithmType.FifoDispatching;
        [SerializeField] private TaskPlanningFutureHandlingMode futureHandling = TaskPlanningFutureHandlingMode.ImmediateOnly;
        [SerializeField] private RollingHorizonOptions rollingHorizon = new();
        [SerializeField] private MapfCoordinator coordinator;
        [SerializeField] private MapfSceneGraph sceneGraph;
        [SerializeField] private float arrivalDistance = 0.08f;
        [SerializeField] private float pendingRetryIntervalSeconds = 0.5f;
        [SerializeField] private float costAmrSpeed = 1f;
        [SerializeField] private TaskPlanningCostWeights costWeights = new();
        [SerializeField] private bool enableSoftAmrReservations;
        [FormerlySerializedAs("reassignmentCostImprovementThreshold")]
        [SerializeField, Min(0f)] private float reassignmentCostImprovementPercent = 10f;
        [SerializeField, Min(0f)] private float waitForFutureImprovementPercent = 10f;
        [SerializeField] private bool autoDiscoverSceneObjects = true;
        [SerializeField] private List<TaskPlanningAmr> amrs = new();
        [SerializeField] private List<PalletLoadingPoint> loadingPoints = new();

        private readonly List<ITaskPlanningTask> _pendingTasks = new();
        private readonly Dictionary<TaskPlanningAmr, ActiveTaskExecution> _activeAssignments = new();
        private readonly HashSet<ITaskPlanningTask> _reportedInvalidDeliveryTasks = new();
        private ITaskDispatchAlgorithm _dispatchAlgorithm;
        private ITaskPlanningFuturePolicy _futurePolicy;
        private RoadmapDistanceService _distances;
        private float _nextPendingRetryTime;
        private int _generatedRemovalTaskNumber;
        private int _dispatchCycleNumber;

        public event Action<DispatchAssignment> AssignmentStarted;
        public event Action<DispatchAssignment, TaskPlanningAssignmentActionType, float> AssignmentActionTimeRecorded;
        public event Action<DispatchAssignment, float, float> AssignmentQueueWaitRecorded;
        public event Action<DispatchAssignment> AssignmentCompleted;
        public event Action<DeliveryPlanningTask, DispatchAssignment> DeliveryTaskCompleted;
        public event Action<TaskPlanningDispatchCycleTrace> DispatchCycleCompleted;
        public event Action<DispatchAssignment, DispatchAssignment> AssignmentReassigned;
        public event Action<DispatchAssignment, string, string> AssignmentPhaseChanged;

        public TaskPlanningAlgorithmType Algorithm => algorithm;
        public TaskPlanningFutureHandlingMode FutureHandling => futureHandling;
        public RollingHorizonOptions RollingHorizon => rollingHorizon;
        public TaskPlanningCostWeights CostWeights => costWeights;
        public bool EnableSoftAmrReservations => enableSoftAmrReservations;
        public float ReassignmentCostImprovementPercent => Mathf.Max(0f, reassignmentCostImprovementPercent);
        public float WaitForFutureImprovementPercent => Mathf.Max(0f, waitForFutureImprovementPercent);
        public float CostAmrSpeed => Mathf.Max(0.0001f, costAmrSpeed);
        public float PendingRetryIntervalSeconds => Mathf.Max(0.05f, pendingRetryIntervalSeconds);
        public float ArrivalDistance => Mathf.Max(0f, arrivalDistance);
        public bool IsIdle => _pendingTasks.Count == 0 && _activeAssignments.Count == 0;
        public IReadOnlyList<TaskPlanningAmr> ConfiguredAmrs => amrs;
        public IReadOnlyList<PalletLoadingPoint> ConfiguredLoadingPoints => loadingPoints;

        private void Awake()
        {
            coordinator ??= FindAnyObjectByType<MapfCoordinator>();
            sceneGraph ??= FindAnyObjectByType<MapfSceneGraph>();
            _dispatchAlgorithm = CreateAlgorithm(algorithm);
            _futurePolicy = CreateFuturePolicy(futureHandling, waitForFutureImprovementPercent, rollingHorizon);

            if (autoDiscoverSceneObjects)
                DiscoverSceneObjects();
        }

        public void ConfigureScene(
            MapfCoordinator mapfCoordinator,
            MapfSceneGraph graph,
            IEnumerable<TaskPlanningAmr> scenarioAmrs,
            IEnumerable<PalletLoadingPoint> scenarioLoadingPoints)
        {
            coordinator = mapfCoordinator;
            sceneGraph = graph;
            amrs = scenarioAmrs?.Where(amr => amr != null).OrderBy(amr => amr.AmrId).ToList() ?? new List<TaskPlanningAmr>();
            loadingPoints = scenarioLoadingPoints?.Where(point => point != null).OrderBy(point => point.LoadingPointId).ToList() ?? new List<PalletLoadingPoint>();
            autoDiscoverSceneObjects = false;
        }

        public void ConfigurePlanningMode(TaskPlanningAlgorithmType dispatcher, TaskPlanningFutureHandlingMode handlingMode)
        {
            algorithm = dispatcher;
            futureHandling = handlingMode;
            _dispatchAlgorithm = CreateAlgorithm(algorithm);
            _futurePolicy = CreateFuturePolicy(futureHandling, waitForFutureImprovementPercent, rollingHorizon);
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
            ReportInvalidDeliveryTaskConfigurations(taskSnapshot);
            var evaluator = new TaskPlanningCostEvaluator(_distances, costWeights, costAmrSpeed, taskSnapshot, Time.time);
            var candidateAmrs = GetFreeDispatchCandidateAmrs();
            var softReassignmentOptions = BuildSoftReassignmentOptions(
                taskSnapshot,
                evaluator,
                allowSoftReservations: algorithm != TaskPlanningAlgorithmType.FifoDispatching);
            if (candidateAmrs.Count == 0 && softReassignmentOptions.Count == 0)
                return;

            var futureAvailabilities = GetFutureAvailabilities();
            var problem = new DispatchProblem(
                taskSnapshot,
                candidateAmrs,
                loadingPoints,
                _distances,
                evaluator,
                Time.time,
                futureAvailabilities,
                softReassignmentOptions: softReassignmentOptions);
            var plan = _futurePolicy.Solve(problem, _dispatchAlgorithm);
            DispatchCycleCompleted?.Invoke(new TaskPlanningDispatchCycleTrace(
                ++_dispatchCycleNumber,
                Time.time,
                algorithm,
                futureHandling,
                taskSnapshot,
                candidateAmrs,
                _activeAssignments.Values.Where(execution => execution.CanReassign).Select(execution => execution.Assignment.Amr).ToArray(),
                softReassignmentOptions,
                futureAvailabilities,
                problem.Candidates,
                plan.Assignments));

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
                if (!CanStartSoftReassignment(activeAssignment, assignment))
                    return false;

                LogSoftReassignment(activeAssignment, assignment);
                AssignmentReassigned?.Invoke(activeAssignment.Assignment, assignment);
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
            SetPhase(execution, ActiveTaskPhase.MovingToPallet);
            yield return MoveAmrTo(assignment.Amr, assignment.Pallet.CurrentNode);
            SetPhase(execution, ActiveTaskPhase.Attaching);
            assignment.Pallet.MarkAttaching();
            yield return new WaitForSeconds(assignment.Pallet.AttachDurationSeconds);
            AssignmentActionTimeRecorded?.Invoke(assignment, TaskPlanningAssignmentActionType.Attach, assignment.Pallet.AttachDurationSeconds);
            assignment.Pallet.AttachTo(assignment.Amr);
            execution.HasAttachedPallet = true;
            SetPhase(execution, ActiveTaskPhase.WaitingForLoadingAndWorkstationTurn);
            TryDispatchPendingTasks();

            yield return WaitForLoadingAndWorkstationTurn(request, assignment);
            SetPhase(execution, ActiveTaskPhase.MovingToLoading);
            yield return MoveAmrTo(assignment.Amr, assignment.LoadingPoint.Node);
            SetPhase(execution, ActiveTaskPhase.Loading);
            assignment.Pallet.MarkLoading();
            yield return new WaitForSeconds(assignment.Pallet.LoadDurationSeconds);
            AssignmentActionTimeRecorded?.Invoke(assignment, TaskPlanningAssignmentActionType.Load, assignment.Pallet.LoadDurationSeconds);
            assignment.Pallet.MarkLoaded();

            SetPhase(execution, ActiveTaskPhase.MovingToWorkstation);
            coordinator.RequestAgentGoal(assignment.Amr.MapfAgent, request.Workstation.Node);
            assignment.LoadingPoint.ReleaseReservation(assignment.Pallet);
            TryDispatchPendingTasks();
            yield return WaitForAmrAt(assignment.Amr, request.Workstation.Node);

            SetPhase(execution, ActiveTaskPhase.DetachingAtWorkstation);
            assignment.Pallet.MarkDetaching();
            yield return new WaitForSeconds(assignment.Pallet.DetachDurationSeconds);
            AssignmentActionTimeRecorded?.Invoke(assignment, TaskPlanningAssignmentActionType.Detach, assignment.Pallet.DetachDurationSeconds);
            assignment.Pallet.DetachAt(request.Workstation.Node, PalletStatus.Unloading);
            assignment.Amr.Release();
            AssignmentCompleted?.Invoke(assignment);
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
            DeliveryTaskCompleted?.Invoke(request, assignment);
            TryDispatchPendingTasks();
        }

        private IEnumerator RunPalletRemoval(PalletRemovalPlanningTask request, DispatchAssignment assignment, ActiveTaskExecution execution)
        {
            SetPhase(execution, ActiveTaskPhase.MovingToPallet);
            yield return MoveAmrTo(assignment.Amr, assignment.Pallet.CurrentNode);
            SetPhase(execution, ActiveTaskPhase.Attaching);
            assignment.Pallet.MarkAttaching();
            yield return new WaitForSeconds(assignment.Pallet.AttachDurationSeconds);
            AssignmentActionTimeRecorded?.Invoke(assignment, TaskPlanningAssignmentActionType.Attach, assignment.Pallet.AttachDurationSeconds);
            assignment.Pallet.AttachTo(assignment.Amr);
            execution.HasAttachedPallet = true;

            SetPhase(execution, ActiveTaskPhase.MovingToParking);
            yield return MoveAmrTo(assignment.Amr, assignment.RemovalTargetNode);
            SetPhase(execution, ActiveTaskPhase.DetachingAtParking);
            assignment.Pallet.MarkDetaching();
            yield return new WaitForSeconds(assignment.Pallet.DetachDurationSeconds);
            AssignmentActionTimeRecorded?.Invoke(assignment, TaskPlanningAssignmentActionType.Detach, assignment.Pallet.DetachDurationSeconds);
            assignment.Pallet.DetachAt(assignment.RemovalTargetNode);
            assignment.Pallet.MarkUnloadedAvailable();
            assignment.Amr.Release();

            Debug.Log($"Pallet removal completed: task={request.TaskId} amr={assignment.Amr.AmrId} pallet={assignment.Pallet.PalletId}", this);
            AssignmentCompleted?.Invoke(assignment);
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
            var loadingWaitSeconds = 0f;
            var workstationWaitSeconds = 0f;
            var lastSampleTime = Time.time;
            while (true)
            {
                var waitingForLoadingPoint = !assignment.LoadingPoint.CanReserveNext(assignment.Pallet);
                var waitingForWorkstation = !request.Workstation.CanReserveNext(assignment.Pallet);
                if (waitingForLoadingPoint || waitingForWorkstation)
                {
                    yield return null;
                    AccumulateQueueWait(ref lastSampleTime, waitingForLoadingPoint, waitingForWorkstation, ref loadingWaitSeconds, ref workstationWaitSeconds);
                    continue;
                }

                if (assignment.LoadingPoint.TryReserveNext(assignment.Pallet) &&
                    request.Workstation.TryReserveNext(assignment.Pallet))
                {
                    AssignmentQueueWaitRecorded?.Invoke(assignment, loadingWaitSeconds, workstationWaitSeconds);
                    yield break;
                }

                assignment.LoadingPoint.ReleaseReservation(assignment.Pallet);
                request.Workstation.ReleaseReservation(assignment.Pallet);
                yield return null;
                AccumulateQueueWait(ref lastSampleTime, waitingForLoadingPoint: true, waitingForWorkstation: true, ref loadingWaitSeconds, ref workstationWaitSeconds);
            }
        }

        private static void AccumulateQueueWait(
            ref float lastSampleTime,
            bool waitingForLoadingPoint,
            bool waitingForWorkstation,
            ref float loadingWaitSeconds,
            ref float workstationWaitSeconds)
        {
            var now = Time.time;
            var delta = Mathf.Max(0f, now - lastSampleTime);
            lastSampleTime = now;

            if (waitingForLoadingPoint)
                loadingWaitSeconds += delta;
            if (waitingForWorkstation)
                workstationWaitSeconds += delta;
        }

        private IEnumerator WaitForAmrAt(TaskPlanningAmr amr, MapfNode goal)
        {
            if (amr == null || goal == null)
                yield break;

            while (Vector2.Distance(amr.transform.position, goal.transform.position) > arrivalDistance)
                yield return null;
        }

        private IReadOnlyList<AmrFutureAvailability> GetFutureAvailabilities()
        {
            var availabilities = new List<AmrFutureAvailability>();
            foreach (var execution in _activeAssignments.Values)
            {
                if (TryCreateFutureAvailability(execution, out var availability))
                    availabilities.Add(availability);
            }

            return availabilities;
        }

        private bool TryCreateFutureAvailability(ActiveTaskExecution execution, out AmrFutureAvailability availability)
        {
            availability = default;
            var finishNode = PredictedFinishNode(execution);
            if (finishNode == null)
                return false;

            var priorAssignmentEta = EstimateRemainingEta(execution);
            if (double.IsNaN(priorAssignmentEta) || double.IsInfinity(priorAssignmentEta))
                return false;

            availability = new AmrFutureAvailability(
                execution.Assignment.Amr,
                execution.Assignment.Task,
                finishNode,
                priorAssignmentEta,
                execution.CanReassign);
            return availability.IsValid;
        }

        private MapfNode PredictedFinishNode(ActiveTaskExecution execution)
        {
            switch (execution.Assignment.Task)
            {
                case DeliveryPlanningTask delivery:
                    return delivery.Workstation != null ? delivery.Workstation.Node : null;
                case PalletRemovalPlanningTask:
                    return execution.Assignment.RemovalTargetNode;
                default:
                    return null;
            }
        }

        private double EstimateRemainingEta(ActiveTaskExecution execution)
        {
            switch (execution.Assignment.Task)
            {
                case DeliveryPlanningTask delivery:
                    return EstimateDeliveryRemainingEta(execution, delivery);
                case PalletRemovalPlanningTask removal:
                    return EstimateRemovalRemainingEta(execution, removal);
                default:
                    return double.PositiveInfinity;
            }
        }

        private double EstimateDeliveryRemainingEta(ActiveTaskExecution execution, DeliveryPlanningTask delivery)
        {
            var assignment = execution.Assignment;
            var pallet = assignment.Pallet;
            var loadingNode = assignment.LoadingPoint != null ? assignment.LoadingPoint.Node : null;
            var workstationNode = delivery.Workstation != null ? delivery.Workstation.Node : null;
            if (pallet == null || workstationNode == null)
                return double.PositiveInfinity;

            switch (execution.Phase)
            {
                case ActiveTaskPhase.MovingToPallet:
                    return SumEta(
                        TravelEtaFromCurrent(assignment.Amr, pallet.CurrentNode),
                        pallet.AttachDurationSeconds,
                        TravelEta(pallet.CurrentNode, loadingNode),
                        pallet.LoadDurationSeconds,
                        TravelEta(loadingNode, workstationNode),
                        pallet.DetachDurationSeconds);
                case ActiveTaskPhase.Attaching:
                    return SumEta(
                        pallet.AttachDurationSeconds,
                        TravelEta(pallet.CurrentNode, loadingNode),
                        pallet.LoadDurationSeconds,
                        TravelEta(loadingNode, workstationNode),
                        pallet.DetachDurationSeconds);
                case ActiveTaskPhase.WaitingForLoadingAndWorkstationTurn:
                case ActiveTaskPhase.MovingToLoading:
                    return SumEta(
                        TravelEtaFromCurrent(assignment.Amr, loadingNode),
                        pallet.LoadDurationSeconds,
                        TravelEta(loadingNode, workstationNode),
                        pallet.DetachDurationSeconds);
                case ActiveTaskPhase.Loading:
                    return SumEta(
                        pallet.LoadDurationSeconds,
                        TravelEta(loadingNode, workstationNode),
                        pallet.DetachDurationSeconds);
                case ActiveTaskPhase.MovingToWorkstation:
                    return SumEta(
                        TravelEtaFromCurrent(assignment.Amr, workstationNode),
                        pallet.DetachDurationSeconds);
                case ActiveTaskPhase.DetachingAtWorkstation:
                    return pallet.DetachDurationSeconds;
                default:
                    return double.PositiveInfinity;
            }
        }

        private double EstimateRemovalRemainingEta(ActiveTaskExecution execution, PalletRemovalPlanningTask removal)
        {
            var assignment = execution.Assignment;
            var pallet = assignment.Pallet;
            var parkingNode = assignment.RemovalTargetNode;
            if (pallet == null || parkingNode == null)
                return double.PositiveInfinity;

            switch (execution.Phase)
            {
                case ActiveTaskPhase.MovingToPallet:
                    return SumEta(
                        TravelEtaFromCurrent(assignment.Amr, pallet.CurrentNode),
                        pallet.AttachDurationSeconds,
                        TravelEta(pallet.CurrentNode, parkingNode),
                        pallet.DetachDurationSeconds);
                case ActiveTaskPhase.Attaching:
                    return SumEta(
                        pallet.AttachDurationSeconds,
                        TravelEta(pallet.CurrentNode, parkingNode),
                        pallet.DetachDurationSeconds);
                case ActiveTaskPhase.MovingToParking:
                    return SumEta(
                        TravelEtaFromCurrent(assignment.Amr, parkingNode),
                        pallet.DetachDurationSeconds);
                case ActiveTaskPhase.DetachingAtParking:
                    return pallet.DetachDurationSeconds;
                default:
                    return double.PositiveInfinity;
            }
        }

        private double TravelEtaFromCurrent(TaskPlanningAmr amr, MapfNode destination)
        {
            if (amr == null || destination == null)
                return double.PositiveInfinity;

            var currentNode = _distances.NearestNode(amr.transform.position);
            return TravelEta(currentNode, destination);
        }

        private double TravelEta(MapfNode from, MapfNode to)
        {
            if (from == null || to == null)
                return double.PositiveInfinity;

            return _distances.Distance(from, to) / Mathf.Max(0.0001f, costAmrSpeed);
        }

        private static double SumEta(params double[] values)
        {
            var total = 0.0;
            foreach (var value in values)
            {
                if (double.IsNaN(value) || double.IsInfinity(value))
                    return double.PositiveInfinity;

                total += Math.Max(0.0, value);
            }

            return total;
        }

        private List<TaskPlanningAmr> GetFreeDispatchCandidateAmrs()
        {
            var candidates = new List<TaskPlanningAmr>();
            foreach (var amr in amrs)
            {
                if (amr == null)
                    continue;

                if (!amr.IsBusy)
                {
                    candidates.Add(amr);
                }
            }

            return candidates;
        }

        private List<SoftReassignmentOption> BuildSoftReassignmentOptions(
            IReadOnlyList<ITaskPlanningTask> taskSnapshot,
            TaskPlanningCostEvaluator evaluator,
            bool allowSoftReservations)
        {
            var options = new List<SoftReassignmentOption>();
            if (!enableSoftAmrReservations ||
                !allowSoftReservations ||
                taskSnapshot.Count == 0 ||
                _activeAssignments.Count == 0)
                return options;

            foreach (var activeAssignment in _activeAssignments.Values)
            {
                if (!activeAssignment.CanReassign)
                    continue;

                var activeCost = EvaluateActiveAssignment(activeAssignment, evaluator);
                if (!activeCost.IsFeasible)
                    continue;

                foreach (var task in taskSnapshot)
                {
                    foreach (var replacementCost in EvaluatePotentialReassignmentCosts(activeAssignment.Assignment.Amr, task, evaluator))
                    {
                        if (!replacementCost.IsFeasible)
                            continue;

                        var improvementPercent = ReassignmentImprovementPercent(activeCost.TotalCost, replacementCost.TotalCost);
                        if (improvementPercent < Math.Max(0.0, reassignmentCostImprovementPercent))
                            continue;

                        options.Add(new SoftReassignmentOption(
                            activeAssignment.Assignment,
                            task,
                            activeCost,
                            replacementCost,
                            improvementPercent));
                    }
                }
            }

            return options;
        }

        private bool CanStartSoftReassignment(ActiveTaskExecution activeAssignment, DispatchAssignment replacement)
        {
            if (!enableSoftAmrReservations ||
                !activeAssignment.CanReassign ||
                !replacement.ReplacesActiveAssignment)
                return false;

            var option = replacement.SoftReassignment;
            return option.ActiveAmr == activeAssignment.Assignment.Amr &&
                option.ActiveTask == activeAssignment.Assignment.Task &&
                option.ReplacementTask == replacement.Task;
        }

        private void LogSoftReassignment(ActiveTaskExecution activeAssignment, DispatchAssignment replacement)
        {
            var option = replacement.SoftReassignment;
            var improvement = option.ActiveCost.TotalCost - replacement.Score;
            Debug.Log(
                $"Task reassigned before pallet attach: amr={activeAssignment.Assignment.Amr.AmrId} " +
                $"from={activeAssignment.Assignment.Task.TaskId} to={replacement.Task.TaskId} " +
                $"improvement={improvement:0.###} ({option.ImprovementPercent:0.##}%)",
                this);
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
                    var loadingPointResolution = PalletLoadingPoint.ResolveAcceptedLoadingPoint(delivery.Pallet, loadingPoints);
                    if (loadingPointResolution.IsResolved)
                        yield return evaluator.Evaluate(amr, delivery, loadingPointResolution.LoadingPoint);
                    break;
                case PalletRemovalPlanningTask removal:
                    yield return evaluator.Evaluate(amr, removal);
                    break;
            }
        }

        private bool IsEnoughReassignmentImprovement(double currentCost, double replacementCost)
        {
            var improvementPercent = ReassignmentImprovementPercent(currentCost, replacementCost);
            return improvementPercent >= Math.Max(0.0, reassignmentCostImprovementPercent);
        }

        private static double ReassignmentImprovementPercent(double currentCost, double replacementCost)
        {
            var improvement = currentCost - replacementCost;
            if (improvement <= 0)
                return 0;

            var baseline = Math.Max(0.0001, Math.Abs(currentCost));
            return improvement / baseline * 100.0;
        }

        private ActiveTaskExecution RegisterActiveAssignment(DispatchAssignment assignment)
        {
            var execution = new ActiveTaskExecution(assignment);
            _activeAssignments[assignment.Amr] = execution;
            AssignmentStarted?.Invoke(assignment);
            return execution;
        }

        private void SetPhase(ActiveTaskExecution execution, ActiveTaskPhase phase)
        {
            if (execution == null || execution.Phase == phase)
                return;

            var previousPhase = execution.Phase;
            execution.Phase = phase;
            AssignmentPhaseChanged?.Invoke(execution.Assignment, previousPhase.ToString(), phase.ToString());
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
                case TaskPlanningAlgorithmType.FifoDispatching:
                    return new FifoDispatching();
                case TaskPlanningAlgorithmType.RegretDispatching:
                    return new RegretDispatching();
                case TaskPlanningAlgorithmType.HungarianDispatching:
                    return new HungarianDispatching();
                case TaskPlanningAlgorithmType.NearestDispatching:
                default:
                    return new NearestDispatching();
            }
        }

        private static ITaskPlanningFuturePolicy CreateFuturePolicy(
            TaskPlanningFutureHandlingMode futureHandling,
            float waitForFutureImprovementPercent,
            RollingHorizonOptions rollingHorizonOptions)
        {
            switch (futureHandling)
            {
                case TaskPlanningFutureHandlingMode.LookAhead:
                    return new LookAheadFuturePolicy(waitForFutureImprovementPercent);
                case TaskPlanningFutureHandlingMode.RollingHorizon:
                    return new RollingHorizonFuturePolicy(rollingHorizonOptions);
                case TaskPlanningFutureHandlingMode.ImmediateOnly:
                default:
                    return new ImmediateOnlyFuturePolicy();
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

            var loadingPointResolution = PalletLoadingPoint.ResolveAcceptedLoadingPoint(request.pallet, loadingPoints);
            if (!loadingPointResolution.IsResolved)
            {
                Debug.LogWarning($"Ignoring MES task '{request.taskId}': {loadingPointResolution.Message}", this);
                return false;
            }

            return true;
        }

        private void ReportInvalidDeliveryTaskConfigurations(IEnumerable<ITaskPlanningTask> tasks)
        {
            foreach (var delivery in tasks.OfType<DeliveryPlanningTask>())
            {
                var loadingPointResolution = PalletLoadingPoint.ResolveAcceptedLoadingPoint(delivery.Pallet, loadingPoints);
                if (loadingPointResolution.IsResolved)
                {
                    _reportedInvalidDeliveryTasks.Remove(delivery);
                    continue;
                }

                if (_reportedInvalidDeliveryTasks.Add(delivery))
                    Debug.LogWarning($"Delivery task '{delivery.TaskId}' is infeasible: {loadingPointResolution.Message}", this);
            }
        }

        private sealed class ActiveTaskExecution
        {
            public ActiveTaskExecution(DispatchAssignment assignment)
            {
                Assignment = assignment;
                Phase = ActiveTaskPhase.MovingToPallet;
            }

            public DispatchAssignment Assignment { get; }
            public Coroutine Coroutine { get; set; }
            public bool HasAttachedPallet { get; set; }
            public ActiveTaskPhase Phase { get; set; }

            public bool CanReassign => !HasAttachedPallet && Assignment.Amr.AttachedPallet == null;
        }

        private enum ActiveTaskPhase
        {
            MovingToPallet,
            Attaching,
            WaitingForLoadingAndWorkstationTurn,
            MovingToLoading,
            Loading,
            MovingToWorkstation,
            DetachingAtWorkstation,
            MovingToParking,
            DetachingAtParking
        }
    }
}
