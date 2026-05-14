using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace TaskPlanning
{
    public sealed class TaskPlanningMes : MonoBehaviour
    {
        [SerializeField] private TaskScheduler scheduler;
        [SerializeField] private List<DeliveryTaskRequest> inspectorTasks = new();
        [SerializeField] private int[] selectedTaskIndexes = Array.Empty<int>();
        [SerializeField] private TaskPlanningMesAutomationMode automationMode = TaskPlanningMesAutomationMode.ManualOnly;
        [SerializeField] private TaskPlanningScenarioAsset scheduledScenario;
        [SerializeField] private bool startScheduledScenarioOnPlay = true;
        [SerializeField] private bool submitSameTimestampAsBatch = true;
        [SerializeField, Min(0f)] private float scheduledScenarioStartDelaySeconds;

        private int _generatedTaskNumber;
        private List<IndexedScheduledMesTask> _scheduledEntries;
        private Dictionary<string, PalletMarker> _palletsById;
        private Dictionary<string, WorkstationDeliveryPoint> _workstationsById;
        private bool _scheduledScenarioRunning;
        private bool _scheduledScenarioSubmissionCompleted;
        private float _scheduledScenarioStartTime;
        private int _nextScheduledEntryIndex;

        public event Action<TaskPlanningScenarioAsset, int> ScheduledScenarioStarted;
        public event Action<TaskPlanningScenarioAsset> ScheduledScenarioSubmissionCompleted;

        private void Awake()
        {
            scheduler ??= FindAnyObjectByType<TaskScheduler>();
        }

        private void Start()
        {
            if (automationMode == TaskPlanningMesAutomationMode.ScheduledScenario && startScheduledScenarioOnPlay)
                StartScheduledScenarioPlayback();
        }

        private void Update()
        {
            if (!_scheduledScenarioRunning)
                return;

            var elapsedSeconds = Time.time - _scheduledScenarioStartTime;
            if (elapsedSeconds < 0f)
                return;

            var batches = CollectDueScheduledScenarioBatches(elapsedSeconds);
            foreach (var batch in batches)
                SubmitTasks(batch.Requests);
        }

        public TaskScheduler Scheduler => scheduler;
        public TaskPlanningScenarioAsset ScheduledScenario => scheduledScenario;
        public bool IsScheduledScenarioRunning => _scheduledScenarioRunning;
        public int ScheduledScenarioTaskCount => scheduledScenario == null ? 0 : scheduledScenario.OrderedTasks().Count;

        public void ConfigureScheduledScenario(
            TaskScheduler taskScheduler,
            TaskPlanningScenarioAsset scenario,
            bool autoStartOnPlay = true,
            bool batchSameTimestamp = true)
        {
            scheduler = taskScheduler;
            scheduledScenario = scenario;
            automationMode = TaskPlanningMesAutomationMode.ScheduledScenario;
            startScheduledScenarioOnPlay = autoStartOnPlay;
            submitSameTimestampAsBatch = batchSameTimestamp;
            _scheduledEntries = null;
            _nextScheduledEntryIndex = 0;
            _scheduledScenarioRunning = false;
            _scheduledScenarioSubmissionCompleted = false;
        }

        [ContextMenu("Submit Selected Tasks")]
        public void SubmitSelectedTasks()
        {
            if (inspectorTasks.Count == 0)
            {
                Debug.LogWarning("MES has no inspector tasks configured.", this);
                return;
            }

            if (selectedTaskIndexes.Length == 0)
            {
                Debug.LogWarning("MES has no selected task indexes configured.", this);
                return;
            }

            var batch = new List<DeliveryTaskRequest>();
            foreach (var index in selectedTaskIndexes)
            {
                if (index < 0 || index >= inspectorTasks.Count)
                {
                    Debug.LogWarning($"MES task index {index} is outside configured task range 0..{inspectorTasks.Count - 1}.", this);
                    continue;
                }

                batch.Add(CreateSubmittedRequest(inspectorTasks[index]));
            }

            if (batch.Count == 0)
            {
                Debug.LogWarning("MES selected task indexes did not resolve to any valid configured tasks.", this);
                return;
            }

            SubmitTasks(batch);
        }

        public void SubmitTask(DeliveryTaskRequest request)
        {
            if (request == null)
                throw new ArgumentNullException(nameof(request));

            SubmitTasks(new[] { request });
        }

        public void SubmitTasks(IEnumerable<DeliveryTaskRequest> requests)
        {
            if (requests == null)
                throw new ArgumentNullException(nameof(requests));

            scheduler ??= FindAnyObjectByType<TaskScheduler>();
            if (scheduler == null)
            {
                Debug.LogError("No TaskScheduler found for MES task submission.", this);
                return;
            }

            var batch = requests
                .Where(request => request != null)
                .Select(CreateSubmittedRequest)
                .ToArray();
            if (batch.Length == 0)
                return;

            scheduler.EnqueueTasks(batch);
        }

        [ContextMenu("Start Scheduled Scenario")]
        public void StartScheduledScenarioPlayback()
        {
            if (!TryInitializeScheduledScenarioPlayback())
                return;

            _scheduledScenarioRunning = true;
            _scheduledScenarioSubmissionCompleted = false;
            _scheduledScenarioStartTime = Time.time + scheduledScenarioStartDelaySeconds;
            Debug.Log($"MES scheduled scenario started: scenario={scheduledScenario.name} tasks={_scheduledEntries.Count}", this);
            ScheduledScenarioStarted?.Invoke(scheduledScenario, _scheduledEntries.Count);
        }

        [ContextMenu("Stop Scheduled Scenario")]
        public void StopScheduledScenarioPlayback()
        {
            _scheduledScenarioRunning = false;
        }

        [ContextMenu("Restart Scheduled Scenario")]
        public void RestartScheduledScenarioPlayback()
        {
            StopScheduledScenarioPlayback();
            StartScheduledScenarioPlayback();
        }

        public IReadOnlyList<ScheduledDeliveryTaskBatch> CollectDueScheduledScenarioBatches(float elapsedSeconds)
        {
            if (!TryEnsureScheduledScenarioInitialized())
                return Array.Empty<ScheduledDeliveryTaskBatch>();

            var dueBatches = new List<ScheduledDeliveryTaskBatch>();
            elapsedSeconds = Mathf.Max(0f, elapsedSeconds);
            while (_nextScheduledEntryIndex < _scheduledEntries.Count)
            {
                var timestamp = _scheduledEntries[_nextScheduledEntryIndex].TimestampSeconds;
                if (timestamp > elapsedSeconds)
                    break;

                var requests = new List<DeliveryTaskRequest>();
                if (submitSameTimestampAsBatch)
                {
                    while (_nextScheduledEntryIndex < _scheduledEntries.Count &&
                        Mathf.Approximately(_scheduledEntries[_nextScheduledEntryIndex].TimestampSeconds, timestamp))
                    {
                        if (TryCreateScheduledRequest(_scheduledEntries[_nextScheduledEntryIndex], out var request))
                            requests.Add(request);

                        _nextScheduledEntryIndex++;
                    }
                }
                else
                {
                    if (TryCreateScheduledRequest(_scheduledEntries[_nextScheduledEntryIndex], out var request))
                        requests.Add(request);

                    _nextScheduledEntryIndex++;
                }

                if (requests.Count > 0)
                    dueBatches.Add(new ScheduledDeliveryTaskBatch(timestamp, requests));
            }

            if (_nextScheduledEntryIndex >= _scheduledEntries.Count)
            {
                _scheduledScenarioRunning = false;
                if (!_scheduledScenarioSubmissionCompleted)
                {
                    _scheduledScenarioSubmissionCompleted = true;
                    ScheduledScenarioSubmissionCompleted?.Invoke(scheduledScenario);
                }
            }

            return dueBatches;
        }

        private DeliveryTaskRequest CreateSubmittedRequest(DeliveryTaskRequest request)
        {
            var taskId = string.IsNullOrWhiteSpace(request.taskId)
                ? $"MES-{++_generatedTaskNumber:0000}"
                : request.taskId.Trim();

            return new DeliveryTaskRequest(taskId, request.pallet, request.workstation);
        }

        private bool TryEnsureScheduledScenarioInitialized()
        {
            return _scheduledEntries != null || TryInitializeScheduledScenarioPlayback();
        }

        private bool TryInitializeScheduledScenarioPlayback()
        {
            if (scheduledScenario == null)
            {
                Debug.LogWarning("MES scheduled scenario is not assigned.", this);
                return false;
            }

            var orderedTasks = scheduledScenario.OrderedTasks();
            if (orderedTasks.Count == 0)
            {
                Debug.LogWarning($"MES scheduled scenario '{scheduledScenario.name}' contains no tasks.", this);
                return false;
            }

            _scheduledEntries = orderedTasks
                .Select((task, index) => new IndexedScheduledMesTask(task, index))
                .ToList();
            _palletsById = BuildPalletIndex();
            _workstationsById = BuildWorkstationIndex();
            _nextScheduledEntryIndex = 0;
            _scheduledScenarioSubmissionCompleted = false;
            return true;
        }

        private bool TryCreateScheduledRequest(IndexedScheduledMesTask entry, out DeliveryTaskRequest request)
        {
            request = null;
            var scheduledTask = entry.Task;
            var palletId = scheduledTask.palletId?.Trim();
            var workstationId = scheduledTask.workstationId?.Trim();
            if (string.IsNullOrWhiteSpace(palletId))
            {
                Debug.LogWarning($"MES scheduled task at {entry.TimestampSeconds:0.###}s has no pallet/kit id.", this);
                return false;
            }

            if (string.IsNullOrWhiteSpace(workstationId))
            {
                Debug.LogWarning($"MES scheduled task at {entry.TimestampSeconds:0.###}s has no workstation id.", this);
                return false;
            }

            if (!_palletsById.TryGetValue(palletId, out var pallet))
            {
                Debug.LogWarning($"MES scheduled task references missing pallet/kit id '{palletId}'.", this);
                return false;
            }

            if (!_workstationsById.TryGetValue(workstationId, out var workstation))
            {
                Debug.LogWarning($"MES scheduled task references missing workstation id '{workstationId}'.", this);
                return false;
            }

            var taskId = string.IsNullOrWhiteSpace(scheduledTask.taskId)
                ? $"MES-SC-{entry.SourceIndex + 1:0000}"
                : scheduledTask.taskId.Trim();
            request = new DeliveryTaskRequest(taskId, pallet, workstation);
            return true;
        }

        private Dictionary<string, PalletMarker> BuildPalletIndex()
        {
            return BuildSceneIndex(
                FindObjectsByType<PalletMarker>(FindObjectsInactive.Exclude),
                pallet => pallet.PalletId,
                "pallet/kit");
        }

        private Dictionary<string, WorkstationDeliveryPoint> BuildWorkstationIndex()
        {
            return BuildSceneIndex(
                FindObjectsByType<WorkstationDeliveryPoint>(FindObjectsInactive.Exclude),
                workstation => workstation.WorkstationId,
                "workstation");
        }

        private Dictionary<string, TComponent> BuildSceneIndex<TComponent>(
            IEnumerable<TComponent> components,
            Func<TComponent, string> idSelector,
            string label)
            where TComponent : Component
        {
            var index = new Dictionary<string, TComponent>(StringComparer.Ordinal);
            foreach (var component in components.OrderBy(idSelector))
            {
                if (component == null)
                    continue;

                var id = idSelector(component)?.Trim();
                if (string.IsNullOrWhiteSpace(id))
                    continue;

                if (index.ContainsKey(id))
                {
                    Debug.LogWarning($"Duplicate MES {label} id '{id}' found. Scheduled scenario resolution will use the first active object.", this);
                    continue;
                }

                index.Add(id, component);
            }

            return index;
        }

        private sealed class IndexedScheduledMesTask
        {
            public IndexedScheduledMesTask(ScheduledMesTask task, int sourceIndex)
            {
                Task = task;
                SourceIndex = sourceIndex;
            }

            public ScheduledMesTask Task { get; }
            public int SourceIndex { get; }
            public float TimestampSeconds => Mathf.Max(0f, Task.timestampSeconds);
        }
    }
}
