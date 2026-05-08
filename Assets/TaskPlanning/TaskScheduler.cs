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
        [SerializeField] private bool autoDiscoverSceneObjects = true;
        [SerializeField] private List<TaskPlanningAmr> amrs = new();
        [SerializeField] private List<PalletLoadingPoint> loadingPoints = new();

        private readonly Queue<DeliveryTaskRequest> _pendingTasks = new();
        private ITaskDispatchAlgorithm _dispatchAlgorithm;
        private RoadmapDistanceService _distances;
        private float _nextPendingRetryTime;

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
            if (!ValidateRequest(request))
                return;

            _pendingTasks.Enqueue(request);
            Debug.Log($"MES task queued: task={request.taskId} kit={request.pallet.KitId} workstation={request.workstation.WorkstationId}", this);
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

            var remaining = _pendingTasks.Count;
            while (remaining-- > 0 && _pendingTasks.Count > 0)
            {
                var request = _pendingTasks.Dequeue();
                var assignment = _dispatchAlgorithm.SelectAssignment(request, amrs, loadingPoints, _distances);
                if (!assignment.IsValid)
                {
                    _pendingTasks.Enqueue(request);
                    continue;
                }

                if (!assignment.Amr.TryReserve())
                {
                    _pendingTasks.Enqueue(request);
                    continue;
                }

                if (!assignment.Pallet.TryReserve())
                {
                    assignment.Amr.Release();
                    _pendingTasks.Enqueue(request);
                    continue;
                }

                if (!request.workstation.Enqueue(assignment.Pallet))
                {
                    assignment.Pallet.ReleaseReservation();
                    assignment.Amr.Release();
                    _pendingTasks.Enqueue(request);
                    continue;
                }

                if (!assignment.LoadingPoint.Enqueue(assignment.Pallet))
                {
                    request.workstation.ReleaseReservation(assignment.Pallet);
                    assignment.Pallet.ReleaseReservation();
                    assignment.Amr.Release();
                    _pendingTasks.Enqueue(request);
                    continue;
                }

                Debug.Log(
                    $"Task dispatched: task={request.taskId} amr={assignment.Amr.AmrId} pallet={assignment.Pallet.PalletId} " +
                    $"loading={assignment.LoadingPoint.LoadingPointId} score={assignment.Score:0.###}",
                    this);
                StartCoroutine(RunDelivery(request, assignment));
            }
        }

        private IEnumerator RunDelivery(DeliveryTaskRequest request, DispatchAssignment assignment)
        {
            yield return MoveAmrTo(assignment.Amr, assignment.Pallet.CurrentNode);
            assignment.Pallet.MarkAttaching();
            yield return new WaitForSeconds(assignment.Pallet.AttachDurationSeconds);
            assignment.Pallet.AttachTo(assignment.Amr);
            TryDispatchPendingTasks();

            yield return WaitForLoadingAndWorkstationTurn(request, assignment);
            yield return MoveAmrTo(assignment.Amr, assignment.LoadingPoint.Node);
            assignment.Pallet.MarkLoading();
            yield return new WaitForSeconds(assignment.Pallet.LoadDurationSeconds);
            assignment.Pallet.MarkLoaded();

            coordinator.RequestAgentGoal(assignment.Amr.MapfAgent, request.workstation.Node);
            assignment.LoadingPoint.ReleaseReservation(assignment.Pallet);
            TryDispatchPendingTasks();
            yield return WaitForAmrAt(assignment.Amr, request.workstation.Node);

            assignment.Pallet.MarkDetaching();
            yield return new WaitForSeconds(assignment.Pallet.DetachDurationSeconds);
            assignment.Pallet.DetachAt(request.workstation.Node, PalletStatus.Unloading);
            assignment.Amr.Release();
            TryDispatchPendingTasks();

            yield return new WaitForSeconds(assignment.Pallet.UnloadDurationSeconds);
            assignment.Pallet.MarkUnloadedAvailable();
            request.workstation.ReleaseReservation(assignment.Pallet);

            Debug.Log($"MES task completed: task={request.taskId} amr={assignment.Amr.AmrId} pallet={assignment.Pallet.PalletId}", this);
            TryDispatchPendingTasks();
        }

        private IEnumerator MoveAmrTo(TaskPlanningAmr amr, MapfNode goal)
        {
            if (amr == null || goal == null)
                yield break;

            coordinator.RequestAgentGoal(amr.MapfAgent, goal);
            yield return WaitForAmrAt(amr, goal);
        }

        private IEnumerator WaitForLoadingAndWorkstationTurn(DeliveryTaskRequest request, DispatchAssignment assignment)
        {
            while (true)
            {
                if (!assignment.LoadingPoint.CanReserveNext(assignment.Pallet) ||
                    !request.workstation.CanReserveNext(assignment.Pallet))
                {
                    yield return null;
                    continue;
                }

                if (assignment.LoadingPoint.TryReserveNext(assignment.Pallet) &&
                    request.workstation.TryReserveNext(assignment.Pallet))
                    yield break;

                assignment.LoadingPoint.ReleaseReservation(assignment.Pallet);
                request.workstation.ReleaseReservation(assignment.Pallet);
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

        private void RefreshDistances()
        {
            if (sceneGraph != null)
                _distances = new RoadmapDistanceService(sceneGraph);
        }

        private static ITaskDispatchAlgorithm CreateAlgorithm(TaskPlanningAlgorithmType algorithm)
        {
            switch (algorithm)
            {
                case TaskPlanningAlgorithmType.GreedyNearestFeasible:
                default:
                    return new GreedyNearestFeasibleDispatchAlgorithm();
            }
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
    }
}
