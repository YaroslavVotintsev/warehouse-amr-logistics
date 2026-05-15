using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

namespace TaskPlanning
{
    [DisallowMultipleComponent]
    public sealed class TaskPlanningMetricsCollector : MonoBehaviour
    {
        public const string DefaultMetricsFolder = "Assets/TaskPlanning/Scenarios/Metrics";

        [SerializeField] private TaskPlanningMes mes;
        [SerializeField] private TaskScheduler scheduler;
        [SerializeField] private bool autoDiscover = true;
        [SerializeField] private bool writeReportOnScheduledScenarioCompletion = true;
        [SerializeField] private string metricsFolder = DefaultMetricsFolder;

        private readonly Dictionary<TaskPlanningAmr, AmrMetrics> _amrMetrics = new();

        private bool _isRunning;
        private bool _isSubscribed;
        private bool _submissionsComplete;
        private bool _finalized;
        private float _startTime;
        private DateTime _startTimestamp;
        private string _scenarioName;
        private int _totalMesTasks;
        private int _completedMesTasks;
        private float _totalTaskCompletionTime;
        private float _totalLoadingQueueWait;
        private float _totalWorkstationQueueWait;
        private int _queueWaitSamples;

        public string LastReportPath { get; private set; }
        public string LastReportText { get; private set; }
        public bool IsRunning => _isRunning;

        private void Awake()
        {
            if (autoDiscover)
                DiscoverDependencies();
        }

        private void OnEnable()
        {
            Subscribe();
        }

        private void OnDisable()
        {
            Unsubscribe();
        }

        private void Update()
        {
            if (!_isRunning)
                return;

            UpdateTravelDistances();

            if (!writeReportOnScheduledScenarioCompletion || !_submissionsComplete || _completedMesTasks < _totalMesTasks)
                return;

            if (scheduler != null && !scheduler.IsIdle)
                return;

            FinalizeAndWriteReport(Time.time);
        }

        public void Configure(TaskPlanningMes taskPlanningMes, TaskScheduler taskScheduler)
        {
            Unsubscribe();
            mes = taskPlanningMes;
            scheduler = taskScheduler;
            Subscribe();
        }

        public void BeginScenario(TaskPlanningMesScheduledScenarioAsset scenario, int totalTasks, float startTime)
        {
            ResetMetrics();
            _isRunning = true;
            _startTime = startTime;
            _startTimestamp = DateTime.Now;
            _scenarioName = string.IsNullOrWhiteSpace(scenario != null ? scenario.name : null)
                ? "UnnamedScenario"
                : scenario.name.Trim();
            _totalMesTasks = Mathf.Max(0, totalTasks);
            InitializeAmrMetrics();
            foreach (var metrics in _amrMetrics.Values)
            {
                metrics.LastTravelSampleTime = startTime;
                metrics.HasTravelSampleTime = true;
            }
        }

        public void BeginScenario(TaskPlanningMesScheduledScenarioAsset scenario, int totalTasks)
        {
            BeginScenario(scenario, totalTasks, Time.time);
        }

        public string FinalizeAndWriteReport(float endTime)
        {
            if (_finalized && !string.IsNullOrEmpty(LastReportText))
                return LastReportText;

            UpdateTravelDistances();
            LastReportText = BuildReportText(endTime);
            LastReportPath = WriteReport(LastReportText);
            _isRunning = false;
            _finalized = true;
            return LastReportText;
        }

        public void RecordAssignmentStarted(DispatchAssignment assignment, float time)
        {
            if (assignment.Amr == null)
                return;

            var metrics = GetOrCreateAmrMetrics(assignment.Amr);
            metrics.AssignmentStartTime = time;
            metrics.HasActiveAssignment = true;
            metrics.LastPosition = assignment.Amr.transform.position;
            metrics.HasLastPosition = true;
            metrics.LastTravelSampleTime = time;
            metrics.HasTravelSampleTime = true;
        }

        public void RecordAssignmentActionTime(
            DispatchAssignment assignment,
            TaskPlanningAssignmentActionType actionType,
            float durationSeconds)
        {
            if (assignment.Amr == null)
                return;

            GetOrCreateAmrMetrics(assignment.Amr).ActionTime += Mathf.Max(0f, durationSeconds);
        }

        public void RecordAssignmentQueueWait(
            DispatchAssignment assignment,
            float loadingPointWaitSeconds,
            float workstationWaitSeconds)
        {
            if (assignment.Amr == null)
                return;

            var loadingWait = Mathf.Max(0f, loadingPointWaitSeconds);
            var workstationWait = Mathf.Max(0f, workstationWaitSeconds);
            var metrics = GetOrCreateAmrMetrics(assignment.Amr);
            metrics.LoadingPointQueueWait += loadingWait;
            metrics.WorkstationBlockingWait += workstationWait;
            _totalLoadingQueueWait += loadingWait;
            _totalWorkstationQueueWait += workstationWait;
            _queueWaitSamples++;
        }

        public void RecordAssignmentCompleted(DispatchAssignment assignment, float time)
        {
            if (assignment.Amr == null)
                return;

            var metrics = GetOrCreateAmrMetrics(assignment.Amr);
            if (metrics.HasActiveAssignment)
            {
                metrics.BusyTime += Mathf.Max(0f, time - metrics.AssignmentStartTime);
                metrics.HasActiveAssignment = false;
            }

            if (assignment.Task?.TaskType == TaskPlanningTaskType.Delivery)
                metrics.CompletedDeliveryAssignments += 1f;
            else if (assignment.Task?.TaskType == TaskPlanningTaskType.PalletRemoval)
                metrics.CompletedRemovalAssignments += 1f;
        }

        public void RecordDeliveryTaskCompleted(DeliveryPlanningTask task, DispatchAssignment assignment, float time)
        {
            if (task == null)
                return;

            _completedMesTasks++;
            _totalTaskCompletionTime += Mathf.Max(0f, time - task.EnqueuedTime);
        }

        public string BuildReportText(float endTime)
        {
            var makespan = Mathf.Max(0f, endTime - _startTime);
            var builder = new StringBuilder();
            builder.AppendLine("Task Planning Scenario Metrics");
            builder.AppendLine($"Start timestamp: {_startTimestamp:yyyy-MM-dd HH:mm:ss}");
            builder.AppendLine($"Scenario name: {_scenarioName}");
            builder.AppendLine($"Dispatcher algorithm: {SchedulerAlgorithmLabel()}");
            builder.AppendLine($"Future handling policy: {SchedulerFutureHandlingLabel()}");
            builder.AppendLine($"Total MES tasks: {_totalMesTasks}");
            builder.AppendLine("Cost weights:");
            AppendCostWeights(builder);
            builder.AppendLine($"Makespan: {FormatSeconds(makespan)}");
            builder.AppendLine($"Average task completion time: {FormatSeconds(Average(_totalTaskCompletionTime, _completedMesTasks))}");
            builder.AppendLine($"Average loading-point queue wait: {FormatSeconds(Average(_totalLoadingQueueWait, _queueWaitSamples))}");
            builder.AppendLine($"Average workstation queue wait: {FormatSeconds(Average(_totalWorkstationQueueWait, _queueWaitSamples))}");
            builder.AppendLine();
            builder.AppendLine("AMR metrics:");

            var metrics = _amrMetrics.Values
                .OrderBy(item => item.AmrId, StringComparer.Ordinal)
                .ToArray();
            foreach (var amrMetrics in metrics)
            {
                AppendAmrMetrics(builder, $"AMR {amrMetrics.AmrId}", amrMetrics, makespan);
                builder.AppendLine();
            }

            AppendAggregateMetrics(builder, metrics, makespan);
            return builder.ToString();
        }

        private void DiscoverDependencies()
        {
            scheduler ??= FindAnyObjectByType<TaskScheduler>();
            mes ??= FindAnyObjectByType<TaskPlanningMes>();
        }

        private void Subscribe()
        {
            if (_isSubscribed)
                return;

            if (autoDiscover && (scheduler == null || mes == null))
                DiscoverDependencies();

            if (scheduler != null)
            {
                scheduler.AssignmentStarted += OnAssignmentStarted;
                scheduler.AssignmentActionTimeRecorded += OnAssignmentActionTimeRecorded;
                scheduler.AssignmentQueueWaitRecorded += OnAssignmentQueueWaitRecorded;
                scheduler.AssignmentCompleted += OnAssignmentCompleted;
                scheduler.DeliveryTaskCompleted += OnDeliveryTaskCompleted;
            }

            if (mes != null)
            {
                mes.ScheduledScenarioStarted += OnScheduledScenarioStarted;
                mes.ScheduledScenarioSubmissionCompleted += OnScheduledScenarioSubmissionCompleted;
            }

            _isSubscribed = true;
        }

        private void Unsubscribe()
        {
            if (!_isSubscribed)
                return;

            if (scheduler != null)
            {
                scheduler.AssignmentStarted -= OnAssignmentStarted;
                scheduler.AssignmentActionTimeRecorded -= OnAssignmentActionTimeRecorded;
                scheduler.AssignmentQueueWaitRecorded -= OnAssignmentQueueWaitRecorded;
                scheduler.AssignmentCompleted -= OnAssignmentCompleted;
                scheduler.DeliveryTaskCompleted -= OnDeliveryTaskCompleted;
            }

            if (mes != null)
            {
                mes.ScheduledScenarioStarted -= OnScheduledScenarioStarted;
                mes.ScheduledScenarioSubmissionCompleted -= OnScheduledScenarioSubmissionCompleted;
            }

            _isSubscribed = false;
        }

        private void OnScheduledScenarioStarted(TaskPlanningMesScheduledScenarioAsset scenario, int totalTasks)
        {
            BeginScenario(scenario, totalTasks, Time.time);
        }

        private void OnScheduledScenarioSubmissionCompleted(TaskPlanningMesScheduledScenarioAsset scenario)
        {
            _submissionsComplete = true;
        }

        private void OnAssignmentStarted(DispatchAssignment assignment)
        {
            RecordAssignmentStarted(assignment, Time.time);
        }

        private void OnAssignmentActionTimeRecorded(
            DispatchAssignment assignment,
            TaskPlanningAssignmentActionType actionType,
            float durationSeconds)
        {
            RecordAssignmentActionTime(assignment, actionType, durationSeconds);
        }

        private void OnAssignmentQueueWaitRecorded(
            DispatchAssignment assignment,
            float loadingPointWaitSeconds,
            float workstationWaitSeconds)
        {
            RecordAssignmentQueueWait(assignment, loadingPointWaitSeconds, workstationWaitSeconds);
        }

        private void OnAssignmentCompleted(DispatchAssignment assignment)
        {
            RecordAssignmentCompleted(assignment, Time.time);
        }

        private void OnDeliveryTaskCompleted(DeliveryPlanningTask task, DispatchAssignment assignment)
        {
            RecordDeliveryTaskCompleted(task, assignment, Time.time);
        }

        private void ResetMetrics()
        {
            _amrMetrics.Clear();
            _isRunning = false;
            _submissionsComplete = false;
            _finalized = false;
            _startTime = 0f;
            _startTimestamp = DateTime.Now;
            _scenarioName = "UnnamedScenario";
            _totalMesTasks = 0;
            _completedMesTasks = 0;
            _totalTaskCompletionTime = 0f;
            _totalLoadingQueueWait = 0f;
            _totalWorkstationQueueWait = 0f;
            _queueWaitSamples = 0;
            LastReportPath = null;
            LastReportText = null;
        }

        private void InitializeAmrMetrics()
        {
            var configuredAmrs = scheduler != null
                ? scheduler.ConfiguredAmrs
                : FindObjectsByType<TaskPlanningAmr>(FindObjectsInactive.Exclude);

            foreach (var amr in configuredAmrs)
            {
                if (amr != null)
                    GetOrCreateAmrMetrics(amr);
            }
        }

        private void UpdateTravelDistances()
        {
            foreach (var metrics in _amrMetrics.Values)
            {
                if (metrics.Amr == null)
                    continue;

                var position = metrics.Amr.transform.position;
                if (metrics.HasLastPosition)
                {
                    var distance = Vector3.Distance(metrics.LastPosition, position);
                    metrics.TravelDistance += distance;
                    if (distance > 0.0001f && metrics.HasTravelSampleTime)
                        metrics.MovementTime += Mathf.Max(0f, Time.time - metrics.LastTravelSampleTime);
                }

                metrics.LastPosition = position;
                metrics.HasLastPosition = true;
                metrics.LastTravelSampleTime = Time.time;
                metrics.HasTravelSampleTime = true;
            }
        }

        private AmrMetrics GetOrCreateAmrMetrics(TaskPlanningAmr amr)
        {
            if (_amrMetrics.TryGetValue(amr, out var metrics))
                return metrics;

            metrics = new AmrMetrics(amr);
            _amrMetrics.Add(amr, metrics);
            return metrics;
        }

        private string WriteReport(string reportText)
        {
            var folder = string.IsNullOrWhiteSpace(metricsFolder) ? DefaultMetricsFolder : metricsFolder.Trim();
            Directory.CreateDirectory(folder);

            var scenarioFileName = SanitizeFileName(string.IsNullOrWhiteSpace(_scenarioName) ? "Scenario" : _scenarioName);
            var dispatcherFileName = SanitizeFileName(SchedulerAlgorithmLabel());
            var futurePolicyFileName = SanitizeFileName(SchedulerFutureHandlingLabel());
            var fileNamePrefix = $"{scenarioFileName}_{dispatcherFileName}_{futurePolicyFileName}";
            var path = NextSerialReportPath(folder, fileNamePrefix);
            File.WriteAllText(path, reportText, Encoding.UTF8);
            return path;
        }

        private static string NextSerialReportPath(string folder, string fileNamePrefix)
        {
            for (var serialNumber = 1; serialNumber < int.MaxValue; serialNumber++)
            {
                var path = Path.Combine(folder, $"{fileNamePrefix}_{serialNumber:000}.txt");
                if (!File.Exists(path))
                    return path;
            }

            throw new IOException($"Could not create a unique metrics report path for '{fileNamePrefix}'.");
        }

        private string SchedulerAlgorithmLabel()
        {
            return scheduler == null ? "Unknown" : scheduler.Algorithm.ToString();
        }

        private string SchedulerFutureHandlingLabel()
        {
            return scheduler == null ? "Unknown" : scheduler.FutureHandling.ToString();
        }

        private void AppendCostWeights(StringBuilder builder)
        {
            var weights = scheduler != null ? scheduler.CostWeights : null;
            if (weights == null)
            {
                builder.AppendLine("  <not configured>");
                return;
            }

            var delivery = weights.delivery ?? new DeliveryTaskCostWeights();
            var removal = weights.removal ?? new RemovalTaskCostWeights();
            builder.AppendLine(
                "  Delivery: " +
                $"priorAssignmentEta={Format(delivery.priorAssignmentEta)}, " +
                $"amrToPalletEta={Format(delivery.amrToPalletEta)}, " +
                $"attachTime={Format(delivery.attachTime)}, " +
                $"loadingQueueEta={Format(delivery.loadingQueueEta)}, " +
                $"palletToLoadingEta={Format(delivery.palletToLoadingEta)}, " +
                $"loadTime={Format(delivery.loadTime)}, " +
                $"loadingToWorkstationEta={Format(delivery.loadingToWorkstationEta)}, " +
                $"detachTime={Format(delivery.detachTime)}, " +
                $"blockedDeliveryBias={Format(delivery.blockedDeliveryBias)}, " +
                $"agingWeight={Format(delivery.agingWeight)}, " +
                $"maxAgingBonus={Format(delivery.maxAgingBonus)}");
            builder.AppendLine(
                "  Removal: " +
                $"priorAssignmentEta={Format(removal.priorAssignmentEta)}, " +
                $"amrToPalletEta={Format(removal.amrToPalletEta)}, " +
                $"attachTime={Format(removal.attachTime)}, " +
                $"palletToParkingEta={Format(removal.palletToParkingEta)}, " +
                $"detachTime={Format(removal.detachTime)}, " +
                $"blocksPendingDeliveryMultiplier={Format(removal.blocksPendingDeliveryMultiplier)}, " +
                $"agingWeight={Format(removal.agingWeight)}, " +
                $"maxAgingBonus={Format(removal.maxAgingBonus)}");
        }

        private static void AppendAmrMetrics(StringBuilder builder, string title, AmrMetrics metrics, float observationTime)
        {
            var idleTime = Mathf.Max(0f, observationTime - metrics.BusyTime);
            var productiveTime = Mathf.Max(0f, metrics.MovementTime + metrics.ActionTime);
            var utilization = observationTime <= 0f ? 0f : productiveTime / observationTime;

            builder.AppendLine($"{title}:");
            builder.AppendLine($"  Travel distance: {FormatDistance(metrics.TravelDistance)}");
            builder.AppendLine($"  Busy/assigned time: {FormatSeconds(metrics.BusyTime)}");
            builder.AppendLine($"  Idle/unassigned time: {FormatSeconds(idleTime)}");
            builder.AppendLine($"  Action time: {FormatSeconds(metrics.ActionTime)}");
            builder.AppendLine($"  Loading-point queue wait time: {FormatSeconds(metrics.LoadingPointQueueWait)}");
            builder.AppendLine($"  Workstation blocking wait time: {FormatSeconds(metrics.WorkstationBlockingWait)}");
            builder.AppendLine($"  Utilization: {FormatPercent(utilization)}");
            builder.AppendLine($"  Completed delivery assignments: {Format(metrics.CompletedDeliveryAssignments)}");
            builder.AppendLine($"  Completed removal assignments: {Format(metrics.CompletedRemovalAssignments)}");
        }

        private static void AppendAggregateMetrics(StringBuilder builder, IReadOnlyList<AmrMetrics> metrics, float observationTime)
        {
            var total = new AmrMetrics(null);
            foreach (var item in metrics)
            {
                total.TravelDistance += item.TravelDistance;
                total.MovementTime += item.MovementTime;
                total.BusyTime += item.BusyTime;
                total.ActionTime += item.ActionTime;
                total.LoadingPointQueueWait += item.LoadingPointQueueWait;
                total.WorkstationBlockingWait += item.WorkstationBlockingWait;
                total.CompletedDeliveryAssignments += item.CompletedDeliveryAssignments;
                total.CompletedRemovalAssignments += item.CompletedRemovalAssignments;
            }

            builder.AppendLine("All AMRs total:");
            AppendAmrMetrics(builder, "Total", total, observationTime * Mathf.Max(1, metrics.Count));
            builder.AppendLine();

            var average = new AmrMetrics(null);
            if (metrics.Count > 0)
            {
                average.TravelDistance = total.TravelDistance / metrics.Count;
                average.MovementTime = total.MovementTime / metrics.Count;
                average.BusyTime = total.BusyTime / metrics.Count;
                average.ActionTime = total.ActionTime / metrics.Count;
                average.LoadingPointQueueWait = total.LoadingPointQueueWait / metrics.Count;
                average.WorkstationBlockingWait = total.WorkstationBlockingWait / metrics.Count;
                average.CompletedDeliveryAssignments = total.CompletedDeliveryAssignments / metrics.Count;
                average.CompletedRemovalAssignments = total.CompletedRemovalAssignments / metrics.Count;
            }

            builder.AppendLine("All AMRs average:");
            AppendAmrMetrics(builder, "Average", average, observationTime);
        }

        private static float Average(float total, int count)
        {
            return count <= 0 ? 0f : total / count;
        }

        private static string SanitizeFileName(string value)
        {
            var invalid = Path.GetInvalidFileNameChars();
            var sanitized = new string(value.Select(character => invalid.Contains(character) ? '_' : character).ToArray());
            return string.IsNullOrWhiteSpace(sanitized) ? "Scenario" : sanitized;
        }

        private static string FormatSeconds(float value)
        {
            return $"{Format(value)} s";
        }

        private static string FormatDistance(float value)
        {
            return $"{Format(value)} units";
        }

        private static string FormatPercent(float value)
        {
            return (value * 100f).ToString("0.###", CultureInfo.InvariantCulture) + "%";
        }

        private static string Format(float value)
        {
            return value.ToString("0.###", CultureInfo.InvariantCulture);
        }

        private sealed class AmrMetrics
        {
            public AmrMetrics(TaskPlanningAmr amr)
            {
                Amr = amr;
                AmrId = amr == null ? "Aggregate" : amr.AmrId;
                if (amr != null)
                {
                    LastPosition = amr.transform.position;
                    HasLastPosition = true;
                }
            }

            public TaskPlanningAmr Amr { get; }
            public string AmrId { get; }
            public Vector3 LastPosition;
            public bool HasLastPosition;
            public float LastTravelSampleTime;
            public bool HasTravelSampleTime;
            public float TravelDistance;
            public float MovementTime;
            public float BusyTime;
            public float ActionTime;
            public float LoadingPointQueueWait;
            public float WorkstationBlockingWait;
            public float CompletedDeliveryAssignments;
            public float CompletedRemovalAssignments;
            public float AssignmentStartTime;
            public bool HasActiveAssignment;
        }
    }
}
