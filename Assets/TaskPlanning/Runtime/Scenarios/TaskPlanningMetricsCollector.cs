using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Mapf.Authoring;
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
        private readonly Dictionary<ITaskPlanningTask, AssignmentTraceRecord> _activeAssignmentTraceRecords = new();
        private readonly List<AssignmentTraceRecord> _assignmentTraceRecords = new();
        private readonly List<string> _traceEvents = new();

        private bool _isRunning;
        private bool _isSubscribed;
        private bool _submissionsComplete;
        private bool _finalized;
        private float _startTime;
        private DateTime _startTimestamp;
        private TaskPlanningMesScheduledScenarioAsset _scenario;
        private string _scenarioName = "UnnamedScenario";
        private string _runName;
        private int _totalMesTasks;
        private int _completedMesTasks;
        private int _softReassignmentCount;
        private int _assignmentSequence;
        private float _totalTaskCompletionTime;
        private float _totalLoadingQueueWait;
        private float _totalWorkstationQueueWait;
        private int _queueWaitSamples;

        public string LastReportPath { get; private set; }
        public string LastReportText { get; private set; }
        public string LastMetricsReportPath { get; private set; }
        public string LastConfigReportPath { get; private set; }
        public string LastTraceReportPath { get; private set; }
        public string LastConfigReportText { get; private set; }
        public string LastTraceReportText { get; private set; }
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
            _scenario = scenario;
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

            AppendTraceEvent(startTime, "SCENARIO_STARTED", new[]
            {
                $"Scenario: {_scenarioName}",
                $"Total MES tasks: {_totalMesTasks}"
            });
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
            var runPaths = ReserveReportPaths();
            _runName = runPaths.RunName;
            LastReportText = BuildMetricsReportText(endTime);
            LastConfigReportText = BuildConfigReportText(endTime);
            LastTraceReportText = BuildTraceReportText(endTime);
            WriteReports(runPaths);
            _isRunning = false;
            _finalized = true;
            return LastReportText;
        }

        public string BuildReportText(float endTime)
        {
            return BuildMetricsReportText(endTime);
        }

        public void RecordMesTasksSubmitted(IReadOnlyList<DeliveryTaskRequest> requests, float time)
        {
            if (requests == null || requests.Count == 0)
                return;

            AppendTraceEvent(time, "MES_BATCH_RECEIVED", requests
                .Where(request => request != null)
                .Select(request => $"Task: {TaskId(request.taskId)} | pallet={PalletId(request.pallet)} | workstation={WorkstationId(request.workstation)}")
                .ToArray());
        }

        public void RecordDispatchCycle(TaskPlanningDispatchCycleTrace trace)
        {
            if (trace == null)
                return;

            var details = new List<string>
            {
                $"Algorithm: {trace.Algorithm}",
                $"Future policy: {trace.FutureHandling}",
                $"Pending tasks: {Join(trace.PendingTasks.Select(TaskLabel))}",
                $"Free AMRs: {Join(trace.FreeAmrs.Select(AmrId))}",
                $"Active interruptible AMRs: {Join(trace.ActiveInterruptibleAmrs.Select(AmrId))}"
            };

            if (trace.SoftReassignmentOptions.Count == 0)
            {
                details.Add("Soft reassignment candidates: none");
            }
            else
            {
                details.Add("Soft reassignment candidates:");
                foreach (var option in trace.SoftReassignmentOptions)
                {
                    details.Add(
                        $"  {AmrId(option.ActiveAmr)} replace {TaskLabel(option.ActiveTask)} -> {TaskLabel(option.ReplacementTask)} | " +
                        $"oldCost={Format(option.ActiveCost.TotalCost)} | newCost={Format(option.ReplacementCost.TotalCost)} | " +
                        $"improvement={Format(option.ImprovementPercent)}%");
                }
            }

            if (trace.FutureAvailabilities.Count > 0)
            {
                details.Add("Future AMR availability:");
                foreach (var future in trace.FutureAvailabilities.Where(item => item.IsValid))
                {
                    details.Add(
                        $"  {AmrId(future.Amr)} after {TaskLabel(future.ActiveTask)} | finishNode={NodeId(future.FinishNode)} | " +
                        $"eta={Format(future.PriorAssignmentEta)} s | interruptible={future.IsInterruptible}");
                }
            }

            if (trace.Candidates.Count == 0)
            {
                details.Add("Candidate costs: none");
            }
            else
            {
                details.Add("Candidate costs:");
                foreach (var candidate in trace.Candidates.Where(candidate => candidate.IsValid).OrderBy(candidate => candidate.Score))
                {
                    details.Add(
                        $"  {AmrId(candidate.Amr)} -> {TaskLabel(candidate.Task)} | pallet={PalletId(candidate.Pallet)} | " +
                        $"loading={LoadingPointId(candidate.LoadingPoint)} | workstation={WorkstationId(candidate.Workstation)} | " +
                        $"cost={Format(candidate.Score)} | softReassignment={YesNo(candidate.ReplacesActiveAssignment)}");
                }
            }

            if (trace.SelectedAssignments.Count == 0)
            {
                details.Add("Selected: none");
            }
            else
            {
                details.Add("Selected:");
                foreach (var assignment in trace.SelectedAssignments.Where(assignment => assignment.IsValid))
                {
                    details.Add(
                        $"  {AmrId(assignment.Amr)} -> {TaskLabel(assignment.Task)} | cost={Format(assignment.Score)} | " +
                        $"softReassignment={YesNo(assignment.ReplacesActiveAssignment)} | replaces={TaskLabel(assignment.SoftReassignment.ActiveTask)}");
                }
            }

            AppendTraceEvent(trace.Time, $"DISPATCH_CYCLE #{trace.CycleNumber}", details);
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

            var record = new AssignmentTraceRecord(++_assignmentSequence, assignment, time, metrics.TravelDistance);
            _assignmentTraceRecords.Add(record);
            if (assignment.Task != null)
                _activeAssignmentTraceRecords[assignment.Task] = record;

            AppendTraceEvent(time, "ASSIGNMENT_STARTED", new[]
            {
                $"Assignment: A-{record.Sequence:000}",
                $"Type: {assignment.Task?.TaskType.ToString() ?? "Unknown"}",
                $"Task: {TaskLabel(assignment.Task)}",
                $"AMR: {AmrId(assignment.Amr)}",
                $"Pallet: {PalletId(assignment.Pallet)}",
                $"Loading point: {LoadingPointId(assignment.LoadingPoint)}",
                $"Workstation: {WorkstationId(assignment.Workstation)}",
                $"Removal target: {NodeId(assignment.RemovalTargetNode)}",
                $"Cost: {Format(assignment.Score)}",
                $"Soft reassignment: {YesNo(assignment.ReplacesActiveAssignment)}",
                $"Replaced task: {TaskLabel(assignment.SoftReassignment.ActiveTask)}"
            });
        }

        public void RecordAssignmentActionTime(
            DispatchAssignment assignment,
            TaskPlanningAssignmentActionType actionType,
            float durationSeconds)
        {
            if (assignment.Amr == null)
                return;

            var duration = Mathf.Max(0f, durationSeconds);
            GetOrCreateAmrMetrics(assignment.Amr).ActionTime += duration;
            if (TryGetActiveTraceRecord(assignment, out var record))
            {
                record.ActionTime += duration;
                record.PhaseEvents.Add($"Action {actionType}: {FormatSeconds(duration)}");
            }

            AppendTraceEvent(Time.time, "ASSIGNMENT_ACTION", new[]
            {
                $"Task: {TaskLabel(assignment.Task)}",
                $"AMR: {AmrId(assignment.Amr)}",
                $"Action: {actionType}",
                $"Duration: {FormatSeconds(duration)}"
            });
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

            if (TryGetActiveTraceRecord(assignment, out var record))
            {
                record.LoadingPointQueueWait += loadingWait;
                record.WorkstationQueueWait += workstationWait;
                record.PhaseEvents.Add($"Queue wait: loading={FormatSeconds(loadingWait)}, workstation={FormatSeconds(workstationWait)}");
            }

            AppendTraceEvent(Time.time, "ASSIGNMENT_QUEUE_WAIT", new[]
            {
                $"Task: {TaskLabel(assignment.Task)}",
                $"AMR: {AmrId(assignment.Amr)}",
                $"Loading-point wait: {FormatSeconds(loadingWait)}",
                $"Workstation wait: {FormatSeconds(workstationWait)}"
            });
        }

        public void RecordAssignmentCompleted(DispatchAssignment assignment, float time)
        {
            if (assignment.Amr == null)
                return;

            UpdateTravelDistances();
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

            if (TryGetActiveTraceRecord(assignment, out var record))
            {
                record.CompletedTime = time;
                record.TravelDistance = Mathf.Max(0f, metrics.TravelDistance - record.StartTravelDistance);
                record.Outcome = "completed";
                _activeAssignmentTraceRecords.Remove(assignment.Task);
            }

            AppendTraceEvent(time, "ASSIGNMENT_COMPLETED", new[]
            {
                $"Task: {TaskLabel(assignment.Task)}",
                $"AMR: {AmrId(assignment.Amr)}",
                $"Pallet: {PalletId(assignment.Pallet)}"
            });
        }

        public void RecordAssignmentReassigned(DispatchAssignment previousAssignment, DispatchAssignment replacementAssignment, float time)
        {
            _softReassignmentCount++;
            UpdateTravelDistances();

            if (TryGetActiveTraceRecord(previousAssignment, out var record))
            {
                var metrics = previousAssignment.Amr != null ? GetOrCreateAmrMetrics(previousAssignment.Amr) : null;
                record.CompletedTime = time;
                record.TravelDistance = metrics == null ? 0f : Mathf.Max(0f, metrics.TravelDistance - record.StartTravelDistance);
                record.Outcome = "reassigned";
                record.ReplacedByTaskId = TaskLabel(replacementAssignment.Task);
                _activeAssignmentTraceRecords.Remove(previousAssignment.Task);
            }

            AppendTraceEvent(time, "ASSIGNMENT_REASSIGNED", new[]
            {
                $"AMR: {AmrId(replacementAssignment.Amr)}",
                $"From task: {TaskLabel(previousAssignment.Task)}",
                $"To task: {TaskLabel(replacementAssignment.Task)}",
                $"Improvement: {Format(replacementAssignment.SoftReassignment.ImprovementPercent)}%",
                $"Old task returned to pending: yes"
            });
        }

        public void RecordAssignmentPhaseChanged(
            DispatchAssignment assignment,
            string previousPhase,
            string nextPhase,
            float time)
        {
            if (TryGetActiveTraceRecord(assignment, out var record))
                record.PhaseEvents.Add($"{FormatTimestamp(time)}: {previousPhase} -> {nextPhase}");

            AppendTraceEvent(time, "ASSIGNMENT_PHASE", new[]
            {
                $"Task: {TaskLabel(assignment.Task)}",
                $"AMR: {AmrId(assignment.Amr)}",
                $"{previousPhase} -> {nextPhase}"
            });
        }

        public void RecordDeliveryTaskCompleted(DeliveryPlanningTask task, DispatchAssignment assignment, float time)
        {
            if (task == null)
                return;

            _completedMesTasks++;
            _totalTaskCompletionTime += Mathf.Max(0f, time - task.EnqueuedTime);

            AppendTraceEvent(time, "MES_TASK_COMPLETED", new[]
            {
                $"Task: {TaskLabel(task)}",
                $"AMR: {AmrId(assignment.Amr)}",
                $"Pallet: {PalletId(assignment.Pallet)}"
            });
        }

        public string BuildMetricsReportText(float endTime)
        {
            var makespan = Mathf.Max(0f, endTime - _startTime);
            var builder = new StringBuilder();
            builder.AppendLine("TaskPlanning Scenario Metrics");
            builder.AppendLine($"Run: {RunNameLabel()}");
            builder.AppendLine($"Start timestamp: {_startTimestamp:yyyy-MM-dd HH:mm:ss}");
            builder.AppendLine($"Scenario name: {_scenarioName}");
            builder.AppendLine($"Total MES tasks: {_totalMesTasks}");
            builder.AppendLine($"Completed MES tasks: {_completedMesTasks}");
            builder.AppendLine($"Makespan: {FormatSeconds(makespan)}");
            builder.AppendLine($"Average task completion time: {FormatSeconds(Average(_totalTaskCompletionTime, _completedMesTasks))}");
            builder.AppendLine($"Average loading-point queue wait: {FormatSeconds(Average(_totalLoadingQueueWait, _queueWaitSamples))}");
            builder.AppendLine($"Average workstation queue wait: {FormatSeconds(Average(_totalWorkstationQueueWait, _queueWaitSamples))}");
            builder.AppendLine($"Soft reassignments: {_softReassignmentCount}");
            builder.AppendLine();

            var metrics = _amrMetrics.Values
                .OrderBy(item => item.AmrId, StringComparer.Ordinal)
                .ToArray();
            AppendAggregateMetrics(builder, metrics, makespan);
            builder.AppendLine();
            builder.AppendLine("Per AMR");
            foreach (var amrMetrics in metrics)
            {
                AppendAmrMetrics(builder, $"AMR {amrMetrics.AmrId}", amrMetrics, makespan);
                builder.AppendLine();
            }

            return builder.ToString();
        }

        public string BuildConfigReportText(float endTime)
        {
            var builder = new StringBuilder();
            builder.AppendLine("TaskPlanning Scenario Configuration");
            builder.AppendLine($"Run: {RunNameLabel()}");
            builder.AppendLine($"Start timestamp: {_startTimestamp:yyyy-MM-dd HH:mm:ss}");
            builder.AppendLine($"Scenario name: {_scenarioName}");
            builder.AppendLine();
            AppendMesConfiguration(builder);
            builder.AppendLine();
            AppendSchedulerConfiguration(builder);
            builder.AppendLine();
            AppendSceneConfiguration(builder);
            return builder.ToString();
        }

        public string BuildTraceReportText(float endTime)
        {
            var builder = new StringBuilder();
            builder.AppendLine("TaskPlanning Dispatch Trace");
            builder.AppendLine($"Run: {RunNameLabel()}");
            builder.AppendLine($"Scenario name: {_scenarioName}");
            builder.AppendLine();
            builder.AppendLine("Chronological Events");
            if (_traceEvents.Count == 0)
            {
                builder.AppendLine("<none>");
            }
            else
            {
                foreach (var entry in _traceEvents)
                    builder.AppendLine(entry);
            }

            builder.AppendLine();
            builder.AppendLine("Assignment Trace Summary");
            builder.AppendLine("# | type | task | pallet | amr | replaced task | replaced by | soft reassigned | enqueued | assigned | completed | total time | cost | loading wait | workstation wait | action time | travel distance | outcome");
            foreach (var record in _assignmentTraceRecords.OrderBy(record => record.Sequence))
            {
                builder.AppendLine(
                    $"{record.Sequence} | {record.TaskType} | {record.TaskId} | {record.PalletId} | {record.AmrId} | " +
                    $"{record.ReplacedTaskId} | {record.ReplacedByTaskId} | {YesNo(record.SoftReassigned)} | " +
                    $"{Format(record.EnqueuedTime)} | {Format(record.AssignedTime)} | {FormatOptional(record.CompletedTime)} | " +
                    $"{FormatSeconds(record.TotalTime)} | {Format(record.Cost)} | {FormatSeconds(record.LoadingPointQueueWait)} | " +
                    $"{FormatSeconds(record.WorkstationQueueWait)} | {FormatSeconds(record.ActionTime)} | " +
                    $"{FormatDistance(record.TravelDistance)} | {record.Outcome}");
            }

            builder.AppendLine();
            builder.AppendLine("Assignment Phase Details");
            foreach (var record in _assignmentTraceRecords.OrderBy(record => record.Sequence))
            {
                builder.AppendLine($"A-{record.Sequence:000} / {record.TaskId} / {record.AmrId}");
                if (record.PhaseEvents.Count == 0)
                {
                    builder.AppendLine("  <none>");
                }
                else
                {
                    foreach (var phaseEvent in record.PhaseEvents)
                        builder.AppendLine($"  {phaseEvent}");
                }
            }

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
                scheduler.DispatchCycleCompleted += OnDispatchCycleCompleted;
                scheduler.AssignmentReassigned += OnAssignmentReassigned;
                scheduler.AssignmentPhaseChanged += OnAssignmentPhaseChanged;
            }

            if (mes != null)
            {
                mes.ScheduledScenarioStarted += OnScheduledScenarioStarted;
                mes.ScheduledScenarioSubmissionCompleted += OnScheduledScenarioSubmissionCompleted;
                mes.TasksSubmitted += OnMesTasksSubmitted;
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
                scheduler.DispatchCycleCompleted -= OnDispatchCycleCompleted;
                scheduler.AssignmentReassigned -= OnAssignmentReassigned;
                scheduler.AssignmentPhaseChanged -= OnAssignmentPhaseChanged;
            }

            if (mes != null)
            {
                mes.ScheduledScenarioStarted -= OnScheduledScenarioStarted;
                mes.ScheduledScenarioSubmissionCompleted -= OnScheduledScenarioSubmissionCompleted;
                mes.TasksSubmitted -= OnMesTasksSubmitted;
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
            AppendTraceEvent(Time.time, "MES_SUBMISSION_COMPLETED", new[]
            {
                $"Scenario: {(scenario != null ? scenario.name : "<none>")}"
            });
        }

        private void OnMesTasksSubmitted(IReadOnlyList<DeliveryTaskRequest> requests)
        {
            RecordMesTasksSubmitted(requests, Time.time);
        }

        private void OnDispatchCycleCompleted(TaskPlanningDispatchCycleTrace trace)
        {
            RecordDispatchCycle(trace);
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

        private void OnAssignmentReassigned(DispatchAssignment previousAssignment, DispatchAssignment replacementAssignment)
        {
            RecordAssignmentReassigned(previousAssignment, replacementAssignment, Time.time);
        }

        private void OnAssignmentPhaseChanged(DispatchAssignment assignment, string previousPhase, string nextPhase)
        {
            RecordAssignmentPhaseChanged(assignment, previousPhase, nextPhase, Time.time);
        }

        private void OnDeliveryTaskCompleted(DeliveryPlanningTask task, DispatchAssignment assignment)
        {
            RecordDeliveryTaskCompleted(task, assignment, Time.time);
        }

        private void ResetMetrics()
        {
            _amrMetrics.Clear();
            _activeAssignmentTraceRecords.Clear();
            _assignmentTraceRecords.Clear();
            _traceEvents.Clear();
            _isRunning = false;
            _submissionsComplete = false;
            _finalized = false;
            _startTime = 0f;
            _startTimestamp = DateTime.Now;
            _scenario = null;
            _scenarioName = "UnnamedScenario";
            _runName = null;
            _totalMesTasks = 0;
            _completedMesTasks = 0;
            _softReassignmentCount = 0;
            _assignmentSequence = 0;
            _totalTaskCompletionTime = 0f;
            _totalLoadingQueueWait = 0f;
            _totalWorkstationQueueWait = 0f;
            _queueWaitSamples = 0;
            LastReportPath = null;
            LastReportText = null;
            LastMetricsReportPath = null;
            LastConfigReportPath = null;
            LastTraceReportPath = null;
            LastConfigReportText = null;
            LastTraceReportText = null;
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

        private bool TryGetActiveTraceRecord(DispatchAssignment assignment, out AssignmentTraceRecord record)
        {
            record = null;
            return assignment.Task != null && _activeAssignmentTraceRecords.TryGetValue(assignment.Task, out record);
        }

        private ReportPaths ReserveReportPaths()
        {
            var folder = string.IsNullOrWhiteSpace(metricsFolder) ? DefaultMetricsFolder : metricsFolder.Trim();
            Directory.CreateDirectory(folder);

            var scenarioFileName = SanitizeFileName(string.IsNullOrWhiteSpace(_scenarioName) ? "Scenario" : _scenarioName);
            var dispatcherFileName = SanitizeFileName(SchedulerAlgorithmLabel());
            var futurePolicyFileName = SanitizeFileName(SchedulerFutureHandlingLabel());
            var fileNamePrefix = $"{scenarioFileName}_{dispatcherFileName}_{futurePolicyFileName}";
            var serialNumber = NextSerialNumber(folder, fileNamePrefix);
            var runName = $"{fileNamePrefix}_{serialNumber:000}";
            return new ReportPaths(
                runName,
                Path.Combine(folder, $"{runName}_metrics.txt"),
                Path.Combine(folder, $"{runName}_config.txt"),
                Path.Combine(folder, $"{runName}_trace.txt"));
        }

        private void WriteReports(ReportPaths paths)
        {
            File.WriteAllText(paths.MetricsPath, LastReportText, Encoding.UTF8);
            File.WriteAllText(paths.ConfigPath, LastConfigReportText, Encoding.UTF8);
            File.WriteAllText(paths.TracePath, LastTraceReportText, Encoding.UTF8);
            LastMetricsReportPath = paths.MetricsPath;
            LastConfigReportPath = paths.ConfigPath;
            LastTraceReportPath = paths.TracePath;
            LastReportPath = LastMetricsReportPath;
        }

        private static int NextSerialNumber(string folder, string fileNamePrefix)
        {
            for (var serialNumber = 1; serialNumber < int.MaxValue; serialNumber++)
            {
                var runName = $"{fileNamePrefix}_{serialNumber:000}";
                var metricsPath = Path.Combine(folder, $"{runName}_metrics.txt");
                var configPath = Path.Combine(folder, $"{runName}_config.txt");
                var tracePath = Path.Combine(folder, $"{runName}_trace.txt");
                if (!File.Exists(metricsPath) && !File.Exists(configPath) && !File.Exists(tracePath))
                    return serialNumber;
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

        private string RunNameLabel()
        {
            return string.IsNullOrWhiteSpace(_runName)
                ? $"{SanitizeFileName(_scenarioName)}_{SchedulerAlgorithmLabel()}_{SchedulerFutureHandlingLabel()}_pending"
                : _runName;
        }

        private void AppendMesConfiguration(StringBuilder builder)
        {
            builder.AppendLine("MES");
            if (mes == null)
            {
                builder.AppendLine("  <not configured>");
                return;
            }

            builder.AppendLine($"  Automation mode: {mes.AutomationMode}");
            builder.AppendLine($"  Scheduled scenario: {(mes.ScheduledScenario != null ? mes.ScheduledScenario.name : "<none>")}");
            builder.AppendLine($"  Start scheduled scenario on play: {YesNo(mes.StartScheduledScenarioOnPlay)}");
            builder.AppendLine($"  Submit same timestamp as batch: {YesNo(mes.SubmitSameTimestampAsBatch)}");
            builder.AppendLine($"  Scheduled scenario start delay: {FormatSeconds(mes.ScheduledScenarioStartDelaySeconds)}");
            builder.AppendLine("  Scheduled tasks:");
            var scheduledTasks = (mes.ScheduledScenario ?? _scenario)?.OrderedTasks() ?? Array.Empty<ScheduledMesTask>();
            if (scheduledTasks.Count == 0)
            {
                builder.AppendLine("    <none>");
            }
            else
            {
                foreach (var task in scheduledTasks)
                {
                    builder.AppendLine(
                        $"    {Format(task.timestampSeconds)} s | {TaskId(task.taskId)} | pallet={task.palletId ?? "<none>"} | workstation={task.workstationId ?? "<none>"}");
                }
            }
        }

        private void AppendSchedulerConfiguration(StringBuilder builder)
        {
            builder.AppendLine("Scheduler");
            if (scheduler == null)
            {
                builder.AppendLine("  <not configured>");
                return;
            }

            builder.AppendLine($"  Dispatcher algorithm: {scheduler.Algorithm}");
            builder.AppendLine($"  Future handling policy: {scheduler.FutureHandling}");
            builder.AppendLine($"  Soft AMR reservations: {YesNo(scheduler.EnableSoftAmrReservations)}");
            builder.AppendLine($"  Reassignment improvement threshold: {Format(scheduler.ReassignmentCostImprovementPercent)}%");
            builder.AppendLine($"  Wait-for-future improvement threshold: {Format(scheduler.WaitForFutureImprovementPercent)}%");
            builder.AppendLine($"  AMR speed: {Format(scheduler.CostAmrSpeed)}");
            builder.AppendLine($"  Pending retry interval: {FormatSeconds(scheduler.PendingRetryIntervalSeconds)}");
            builder.AppendLine($"  Arrival distance: {FormatDistance(scheduler.ArrivalDistance)}");
            builder.AppendLine("  Rolling Horizon:");
            var rolling = scheduler.RollingHorizon ?? new RollingHorizonOptions();
            builder.AppendLine($"    Horizon seconds: {FormatSeconds(rolling.horizonSeconds)}");
            builder.AppendLine($"    Max waves: {rolling.maxWaves}");
            builder.AppendLine($"    Wait improvement percent: {Format(rolling.waitImprovementPercent)}%");
            builder.AppendLine("  Cost weights:");
            AppendCostWeights(builder, "    ");
        }

        private void AppendSceneConfiguration(StringBuilder builder)
        {
            builder.AppendLine("Scene Objects");
            var amrs = scheduler != null
                ? scheduler.ConfiguredAmrs.Where(amr => amr != null).OrderBy(amr => amr.AmrId).ToArray()
                : FindObjectsByType<TaskPlanningAmr>(FindObjectsInactive.Exclude).OrderBy(amr => amr.AmrId).ToArray();
            builder.AppendLine("  AMRs:");
            AppendListOrNone(builder, amrs, amr => $"    {amr.AmrId} | startPosition={FormatVector(amr.transform.position)} | busy={YesNo(amr.IsBusy)}");

            var pallets = FindObjectsByType<PalletMarker>(FindObjectsInactive.Exclude)
                .OrderBy(pallet => pallet.PalletId)
                .ToArray();
            builder.AppendLine("  Pallets / Kits:");
            AppendListOrNone(
                builder,
                pallets,
                pallet =>
                    $"    {pallet.PalletId} | currentNode={NodeId(pallet.CurrentNode)} | parkingNode={NodeId(pallet.ParkingNode)} | " +
                    $"status={pallet.Status} | loaded={YesNo(pallet.IsLoaded)} | attach={FormatSeconds(pallet.AttachDurationSeconds)} | " +
                    $"detach={FormatSeconds(pallet.DetachDurationSeconds)} | load={FormatSeconds(pallet.LoadDurationSeconds)} | unload={FormatSeconds(pallet.UnloadDurationSeconds)}");

            var loadingPoints = scheduler != null
                ? scheduler.ConfiguredLoadingPoints.Where(point => point != null).OrderBy(point => point.LoadingPointId).ToArray()
                : FindObjectsByType<PalletLoadingPoint>(FindObjectsInactive.Exclude).OrderBy(point => point.LoadingPointId).ToArray();
            builder.AppendLine("  Loading Points:");
            AppendListOrNone(
                builder,
                loadingPoints,
                point =>
                    $"    {point.LoadingPointId} | node={NodeId(point.Node)} | acceptedPallets={Join(point.AcceptedPallets.Select(PalletId))} | " +
                    $"reservedFor={PalletId(point.ReservedFor)} | queueLength={point.QueueLength}");

            var workstations = FindObjectsByType<WorkstationDeliveryPoint>(FindObjectsInactive.Exclude)
                .OrderBy(workstation => workstation.WorkstationId)
                .ToArray();
            builder.AppendLine("  Workstations:");
            AppendListOrNone(
                builder,
                workstations,
                workstation =>
                    $"    {workstation.WorkstationId} | node={NodeId(workstation.Node)} | acceptedPallets={Join(workstation.AcceptedPallets.Select(PalletId))} | " +
                    $"reservedFor={PalletId(workstation.ReservedFor)} | queueLength={workstation.QueueLength}");
        }

        private void AppendCostWeights(StringBuilder builder, string indent)
        {
            var weights = scheduler != null ? scheduler.CostWeights : null;
            if (weights == null)
            {
                builder.AppendLine($"{indent}<not configured>");
                return;
            }

            var delivery = weights.delivery ?? new DeliveryTaskCostWeights();
            var removal = weights.removal ?? new RemovalTaskCostWeights();
            builder.AppendLine($"{indent}Delivery:");
            builder.AppendLine($"{indent}  priorAssignmentEta: {Format(delivery.priorAssignmentEta)}");
            builder.AppendLine($"{indent}  amrToPalletEta: {Format(delivery.amrToPalletEta)}");
            builder.AppendLine($"{indent}  attachTime: {Format(delivery.attachTime)}");
            builder.AppendLine($"{indent}  loadingQueueEta: {Format(delivery.loadingQueueEta)}");
            builder.AppendLine($"{indent}  palletToLoadingEta: {Format(delivery.palletToLoadingEta)}");
            builder.AppendLine($"{indent}  loadTime: {Format(delivery.loadTime)}");
            builder.AppendLine($"{indent}  loadingToWorkstationEta: {Format(delivery.loadingToWorkstationEta)}");
            builder.AppendLine($"{indent}  detachTime: {Format(delivery.detachTime)}");
            builder.AppendLine($"{indent}  blockedDeliveryBias: {Format(delivery.blockedDeliveryBias)}");
            builder.AppendLine($"{indent}  agingWeight: {Format(delivery.agingWeight)}");
            builder.AppendLine($"{indent}  maxAgingBonus: {Format(delivery.maxAgingBonus)}");
            builder.AppendLine($"{indent}Removal:");
            builder.AppendLine($"{indent}  priorAssignmentEta: {Format(removal.priorAssignmentEta)}");
            builder.AppendLine($"{indent}  amrToPalletEta: {Format(removal.amrToPalletEta)}");
            builder.AppendLine($"{indent}  attachTime: {Format(removal.attachTime)}");
            builder.AppendLine($"{indent}  palletToParkingEta: {Format(removal.palletToParkingEta)}");
            builder.AppendLine($"{indent}  detachTime: {Format(removal.detachTime)}");
            builder.AppendLine($"{indent}  blocksPendingDeliveryMultiplier: {Format(removal.blocksPendingDeliveryMultiplier)}");
            builder.AppendLine($"{indent}  agingWeight: {Format(removal.agingWeight)}");
            builder.AppendLine($"{indent}  maxAgingBonus: {Format(removal.maxAgingBonus)}");
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

        private void AppendTraceEvent(float time, string eventName, IEnumerable<string> details)
        {
            var builder = new StringBuilder();
            builder.AppendLine($"[{FormatTimestamp(time)}] {eventName}");
            if (details != null)
            {
                foreach (var detail in details)
                    builder.AppendLine($"  {detail}");
            }

            _traceEvents.Add(builder.ToString().TrimEnd());
        }

        private static void AppendListOrNone<T>(StringBuilder builder, IReadOnlyList<T> items, Func<T, string> formatter)
        {
            if (items == null || items.Count == 0)
            {
                builder.AppendLine("    <none>");
                return;
            }

            foreach (var item in items)
                builder.AppendLine(formatter(item));
        }

        private static float Average(float total, int count)
        {
            return count <= 0 ? 0f : total / count;
        }

        private static string SanitizeFileName(string value)
        {
            var invalid = Path.GetInvalidFileNameChars();
            var sanitized = new string((value ?? string.Empty).Select(character => invalid.Contains(character) ? '_' : character).ToArray());
            return string.IsNullOrWhiteSpace(sanitized) ? "Scenario" : sanitized;
        }

        private static string TaskLabel(ITaskPlanningTask task)
        {
            return task == null ? "-" : TaskId(task.TaskId);
        }

        private static string TaskId(string taskId)
        {
            return string.IsNullOrWhiteSpace(taskId) ? "<auto>" : taskId.Trim();
        }

        private static string PalletId(PalletMarker pallet)
        {
            return pallet == null ? "-" : pallet.PalletId;
        }

        private static string LoadingPointId(PalletLoadingPoint loadingPoint)
        {
            return loadingPoint == null ? "-" : loadingPoint.LoadingPointId;
        }

        private static string WorkstationId(WorkstationDeliveryPoint workstation)
        {
            return workstation == null ? "-" : workstation.WorkstationId;
        }

        private static string AmrId(TaskPlanningAmr amr)
        {
            return amr == null ? "-" : amr.AmrId;
        }

        private static string NodeId(MapfNode node)
        {
            return node == null ? "-" : node.StableId;
        }

        private static string Join(IEnumerable<string> values)
        {
            var joined = string.Join(", ", values.Where(value => !string.IsNullOrWhiteSpace(value)));
            return string.IsNullOrWhiteSpace(joined) ? "none" : joined;
        }

        private static string YesNo(bool value)
        {
            return value ? "yes" : "no";
        }

        private static string FormatTimestamp(float value)
        {
            return Format(Mathf.Max(0f, value)) + "s";
        }

        private static string FormatSeconds(float value)
        {
            return $"{Format(value)} s";
        }

        private static string FormatSeconds(double value)
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

        private static string FormatVector(Vector3 value)
        {
            return $"({Format(value.x)}, {Format(value.y)}, {Format(value.z)})";
        }

        private static string FormatOptional(float? value)
        {
            return value.HasValue ? Format(value.Value) : "-";
        }

        private static string Format(float value)
        {
            return value.ToString("0.###", CultureInfo.InvariantCulture);
        }

        private static string Format(double value)
        {
            return value.ToString("0.###", CultureInfo.InvariantCulture);
        }

        private readonly struct ReportPaths
        {
            public readonly string RunName;
            public readonly string MetricsPath;
            public readonly string ConfigPath;
            public readonly string TracePath;

            public ReportPaths(string runName, string metricsPath, string configPath, string tracePath)
            {
                RunName = runName;
                MetricsPath = metricsPath;
                ConfigPath = configPath;
                TracePath = tracePath;
            }
        }

        private sealed class AssignmentTraceRecord
        {
            public AssignmentTraceRecord(int sequence, DispatchAssignment assignment, float assignedTime, float startTravelDistance)
            {
                Sequence = sequence;
                TaskType = assignment.Task?.TaskType.ToString() ?? "Unknown";
                TaskId = TaskLabel(assignment.Task);
                PalletId = PalletId(assignment.Pallet);
                AmrId = AmrId(assignment.Amr);
                ReplacedTaskId = TaskLabel(assignment.SoftReassignment.ActiveTask);
                ReplacedByTaskId = "-";
                SoftReassigned = assignment.ReplacesActiveAssignment;
                EnqueuedTime = assignment.Task?.EnqueuedTime ?? assignedTime;
                AssignedTime = assignedTime;
                StartTravelDistance = startTravelDistance;
                Cost = assignment.Score;
                Outcome = "active";
            }

            public int Sequence { get; }
            public string TaskType { get; }
            public string TaskId { get; }
            public string PalletId { get; }
            public string AmrId { get; }
            public string ReplacedTaskId { get; }
            public string ReplacedByTaskId { get; set; }
            public bool SoftReassigned { get; }
            public float EnqueuedTime { get; }
            public float AssignedTime { get; }
            public float? CompletedTime { get; set; }
            public float StartTravelDistance { get; }
            public float TravelDistance { get; set; }
            public double Cost { get; }
            public float LoadingPointQueueWait { get; set; }
            public float WorkstationQueueWait { get; set; }
            public float ActionTime { get; set; }
            public string Outcome { get; set; }
            public List<string> PhaseEvents { get; } = new();

            public float TotalTime => CompletedTime.HasValue ? Mathf.Max(0f, CompletedTime.Value - AssignedTime) : 0f;
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
