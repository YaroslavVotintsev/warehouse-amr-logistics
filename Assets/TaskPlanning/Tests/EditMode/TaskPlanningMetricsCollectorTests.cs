using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class TaskPlanningMetricsCollectorTests
    {
        [Test]
        public void MetricsCollectorCanStartResetAndFinalizeWithoutSceneErrors()
        {
            var collector = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMetricsCollector>("Metrics_Reset_Collector");
            var scenario = CreateScenario("MetricsResetScenario");

            try
            {
                collector.BeginScenario(scenario, 2, 0f);
                collector.BeginScenario(scenario, 1, 1f);
                var report = collector.FinalizeAndWriteReport(6f);

                Assert.That(report, Does.Contain("Scenario name: MetricsResetScenario"));
                Assert.That(report, Does.Contain("Total MES tasks: 1"));
                Assert.That(collector.IsRunning, Is.False);
            }
            finally
            {
                DeleteReports(collector);
                TaskPlanningTestHelpers.Destroy(scenario, collector.gameObject);
            }
        }

        [Test]
        public void GeneratedReportContainsScenarioAndSchedulerSummary()
        {
            var collector = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMetricsCollector>("Metrics_Report_Collector");
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("Metrics_Report_Scheduler");
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("Metrics_Report_AMR");
            var scenario = CreateScenario("MetricsReportScenario");

            try
            {
                scheduler.ConfigureScene(null, null, new[] { amr }, new PalletLoadingPoint[0]);
                scheduler.ConfigurePlanningMode(TaskPlanningAlgorithmType.NearestDispatching, TaskPlanningFutureHandlingMode.LookAhead);
                collector.Configure(null, scheduler);
                collector.BeginScenario(scenario, 1, 0f);

                var report = collector.FinalizeAndWriteReport(5f);

                Assert.That(report, Does.Contain("Scenario name: MetricsReportScenario"));
                Assert.That(report, Does.Contain("Total MES tasks: 1"));
                Assert.That(report, Does.Contain("Makespan: 5 s"));
                Assert.That(report, Does.Contain("AMR Metrics_Report_AMR:"));
                Assert.That(report, Does.Contain("All AMRs total:"));
                Assert.That(report, Does.Contain("All AMRs average:"));
                Assert.That(collector.LastConfigReportText, Does.Contain("Dispatcher algorithm: NearestDispatching"));
                Assert.That(collector.LastConfigReportText, Does.Contain("Future handling policy: LookAhead"));
                Assert.That(collector.LastConfigReportText, Does.Contain("Cost weights:"));
            }
            finally
            {
                DeleteReports(collector);
                TaskPlanningTestHelpers.Destroy(scenario, amr.gameObject, scheduler.gameObject, collector.gameObject);
            }
        }

        [Test]
        public void AmrPhaseAccountingSeparatesActionTimeFromQueueWaitTime()
        {
            var collector = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMetricsCollector>("Metrics_Phase_Collector");
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("Metrics_Phase_AMR");
            var pallet = TaskPlanningTestHelpers.CreatePallet("Metrics_Phase_Pallet");
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("Metrics_Phase_Workstation", null, pallet);
            var scenario = CreateScenario("MetricsPhaseScenario");
            var task = new DeliveryPlanningTask("Task-Phase", pallet, workstation, 0f);
            var assignment = CreateAssignment(amr, task, pallet, workstation);

            try
            {
                collector.BeginScenario(scenario, 1, 0f);
                collector.RecordAssignmentStarted(assignment, 0f);
                collector.RecordAssignmentActionTime(assignment, TaskPlanningAssignmentActionType.Attach, 1f);
                collector.RecordAssignmentActionTime(assignment, TaskPlanningAssignmentActionType.Load, 3f);
                collector.RecordAssignmentActionTime(assignment, TaskPlanningAssignmentActionType.Detach, 1f);
                collector.RecordAssignmentQueueWait(assignment, 2f, 3f);
                collector.RecordAssignmentCompleted(assignment, 20f);

                var report = collector.FinalizeAndWriteReport(20f);

                Assert.That(report, Does.Contain("Busy/assigned time: 20 s"));
                Assert.That(report, Does.Contain("Action time: 5 s"));
                Assert.That(report, Does.Contain("Loading-point queue wait time: 2 s"));
                Assert.That(report, Does.Contain("Workstation blocking wait time: 3 s"));
                Assert.That(report, Does.Contain("Utilization: 25%"));
            }
            finally
            {
                DeleteReports(collector);
                TaskPlanningTestHelpers.Destroy(
                    scenario,
                    workstation.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    collector.gameObject);
            }
        }

        [Test]
        public void CompletedDeliveryAndRemovalAssignmentCountsAreRecorded()
        {
            var collector = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMetricsCollector>("Metrics_Counts_Collector");
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("Metrics_Counts_AMR");
            var pallet = TaskPlanningTestHelpers.CreatePallet("Metrics_Counts_Pallet");
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("Metrics_Counts_Workstation", null, pallet);
            var scenario = CreateScenario("MetricsCountsScenario");
            var deliveryTask = new DeliveryPlanningTask("Task-Delivery", pallet, workstation, 0f);
            var removalTask = new PalletRemovalPlanningTask("Task-Removal", pallet, workstation, 0f);

            try
            {
                collector.BeginScenario(scenario, 1, 0f);

                var delivery = CreateAssignment(amr, deliveryTask, pallet, workstation);
                collector.RecordAssignmentStarted(delivery, 0f);
                collector.RecordAssignmentCompleted(delivery, 4f);

                var removal = CreateAssignment(amr, removalTask, pallet, workstation);
                collector.RecordAssignmentStarted(removal, 5f);
                collector.RecordAssignmentCompleted(removal, 9f);

                var report = collector.FinalizeAndWriteReport(10f);

                Assert.That(report, Does.Contain("Completed delivery assignments: 1"));
                Assert.That(report, Does.Contain("Completed removal assignments: 1"));
            }
            finally
            {
                DeleteReports(collector);
                TaskPlanningTestHelpers.Destroy(
                    scenario,
                    workstation.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    collector.gameObject);
            }
        }

        [Test]
        public void ReportWriterCreatesThreeTxtFilesInExpectedTaskPlanningMetricsFolder()
        {
            var collector = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMetricsCollector>("Metrics_File_Collector");
            var scenario = CreateScenario("MetricsFileScenario");

            try
            {
                collector.BeginScenario(scenario, 1, 0f);
                collector.FinalizeAndWriteReport(1f);

                Assert.That(collector.LastReportPath, Is.Not.Null);
                Assert.That(
                    Path.GetFullPath(collector.LastReportPath),
                    Does.StartWith(Path.GetFullPath(TaskPlanningMetricsCollector.DefaultMetricsFolder)));
                Assert.That(
                    Path.GetFullPath(collector.LastConfigReportPath),
                    Does.StartWith(Path.GetFullPath(TaskPlanningMetricsCollector.DefaultMetricsFolder)));
                Assert.That(
                    Path.GetFullPath(collector.LastTraceReportPath),
                    Does.StartWith(Path.GetFullPath(TaskPlanningMetricsCollector.DefaultMetricsFolder)));
                Assert.That(Path.GetExtension(collector.LastReportPath), Is.EqualTo(".txt"));
                Assert.That(Path.GetExtension(collector.LastConfigReportPath), Is.EqualTo(".txt"));
                Assert.That(Path.GetExtension(collector.LastTraceReportPath), Is.EqualTo(".txt"));
                Assert.That(
                    Path.GetFileName(collector.LastReportPath),
                    Does.Match(@"^MetricsFileScenario_Unknown_Unknown_\d{3}_metrics\.txt$"));
                Assert.That(
                    Path.GetFileName(collector.LastConfigReportPath),
                    Does.Match(@"^MetricsFileScenario_Unknown_Unknown_\d{3}_config\.txt$"));
                Assert.That(
                    Path.GetFileName(collector.LastTraceReportPath),
                    Does.Match(@"^MetricsFileScenario_Unknown_Unknown_\d{3}_trace\.txt$"));
                Assert.That(File.Exists(collector.LastReportPath), Is.True);
                Assert.That(File.Exists(collector.LastConfigReportPath), Is.True);
                Assert.That(File.Exists(collector.LastTraceReportPath), Is.True);
            }
            finally
            {
                DeleteReports(collector);
                TaskPlanningTestHelpers.Destroy(scenario, collector.gameObject);
            }
        }

        [Test]
        public void ConfigAndTraceReportsContainScenarioObjectsAndAssignmentEvents()
        {
            var collector = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMetricsCollector>("Metrics_Trace_Collector");
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("Metrics_Trace_Scheduler");
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("Metrics_Trace_AMR");
            var pallet = TaskPlanningTestHelpers.CreatePallet("Metrics_Trace_Pallet");
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("Metrics_Trace_Loading", null, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("Metrics_Trace_Workstation", null, pallet);
            var scenario = CreateScenario("MetricsTraceScenario");
            var task = new DeliveryPlanningTask("Task-Trace", pallet, workstation, 0f);
            var assignment = CreateAssignment(amr, task, pallet, workstation);

            try
            {
                scheduler.ConfigureScene(null, null, new[] { amr }, new[] { loadingPoint });
                collector.Configure(null, scheduler);
                collector.BeginScenario(scenario, 1, 0f);
                collector.RecordMesTasksSubmitted(new[] { new DeliveryTaskRequest("Task-Trace", pallet, workstation) }, 0f);
                collector.RecordAssignmentStarted(assignment, 1f);
                collector.RecordAssignmentPhaseChanged(assignment, "MovingToPallet", "Attaching", 2f);
                collector.RecordAssignmentActionTime(assignment, TaskPlanningAssignmentActionType.Attach, 1f);
                collector.RecordAssignmentCompleted(assignment, 5f);
                collector.FinalizeAndWriteReport(6f);

                Assert.That(collector.LastConfigReportText, Does.Contain("Scene Objects"));
                Assert.That(collector.LastConfigReportText, Does.Contain("Metrics_Trace_Loading"));
                Assert.That(collector.LastConfigReportText, Does.Contain("Metrics_Trace_Workstation"));
                Assert.That(collector.LastTraceReportText, Does.Contain("MES_BATCH_RECEIVED"));
                Assert.That(collector.LastTraceReportText, Does.Contain("ASSIGNMENT_STARTED"));
                Assert.That(collector.LastTraceReportText, Does.Contain("ASSIGNMENT_PHASE"));
                Assert.That(collector.LastTraceReportText, Does.Contain("Assignment Trace Summary"));
                Assert.That(collector.LastTraceReportText, Does.Contain("Task-Trace"));
            }
            finally
            {
                DeleteReports(collector);
                TaskPlanningTestHelpers.Destroy(
                    scenario,
                    workstation.gameObject,
                    loadingPoint.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    scheduler.gameObject,
                    collector.gameObject);
            }
        }

        [Test]
        public void ConfigReportUsesInitialSceneSnapshotAndLogsRoadmapGraph()
        {
            var collector = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMetricsCollector>("Metrics_Snapshot_Collector");
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("Metrics_Snapshot_Scheduler");
            var node1 = TaskPlanningTestHelpers.CreateNode("1", new Vector2(0f, 0f));
            var node2 = TaskPlanningTestHelpers.CreateNode("2", new Vector2(5f, 0f));
            var node3 = TaskPlanningTestHelpers.CreateNode("3", new Vector2(10f, 0f));
            var edge = TaskPlanningTestHelpers.CreateEdge("1-2", node1, node2);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("AMR_A");
            var pallet = TaskPlanningTestHelpers.CreatePallet("Pallet_A", node1);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("Loading_A", node2, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("Workstation_A", node3, pallet);
            var scenario = CreateScenario("MetricsSnapshotScenario");

            try
            {
                TaskPlanningTestHelpers.SetParkingNode(pallet, node3);
                scheduler.ConfigureScene(null, null, new[] { amr }, new[] { loadingPoint });
                collector.Configure(null, scheduler);
                collector.BeginScenario(scenario, 1, 0f);

                amr.TryReserve();
                amr.transform.position = new Vector3(20f, 0f, 0f);
                pallet.DetachAt(node2, PalletStatus.AwaitingRemoval);
                pallet.MarkLoaded();
                loadingPoint.TryReserve(pallet);
                loadingPoint.Enqueue(pallet);
                workstation.TryReserve(pallet);
                workstation.Enqueue(pallet);

                collector.FinalizeAndWriteReport(5f);

                Assert.That(collector.LastConfigReportText, Does.Contain("AMR_A | startPosition=(0, 0, 0)"));
                Assert.That(collector.LastConfigReportText, Does.Contain("Pallet_A | currentNode=1 | parkingNode=3"));
                Assert.That(collector.LastConfigReportText, Does.Not.Contain("currentNode=2"));
                Assert.That(collector.LastConfigReportText, Does.Not.Contain("busy="));
                Assert.That(collector.LastConfigReportText, Does.Not.Contain("status="));
                Assert.That(collector.LastConfigReportText, Does.Not.Contain("loaded="));
                Assert.That(collector.LastConfigReportText, Does.Not.Contain("reservedFor="));
                Assert.That(collector.LastConfigReportText, Does.Not.Contain("queueLength="));
                Assert.That(collector.LastConfigReportText, Does.Contain("Roadmap Graph"));
                Assert.That(collector.LastConfigReportText, Does.Contain("id=1 | position=(0, 0, 0)"));
                Assert.That(collector.LastConfigReportText, Does.Contain("id=2 | position=(5, 0, 0)"));
                Assert.That(collector.LastConfigReportText, Does.Contain("from=1 | to=2"));
            }
            finally
            {
                DeleteReports(collector);
                TaskPlanningTestHelpers.Destroy(
                    scenario,
                    workstation.gameObject,
                    loadingPoint.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    edge.gameObject,
                    node3.gameObject,
                    node2.gameObject,
                    node1.gameObject,
                    scheduler.gameObject,
                    collector.gameObject);
            }
        }

        private static TaskPlanningMesScheduledScenarioAsset CreateScenario(string name)
        {
            var scenario = TaskPlanningMesScheduledScenarioAsset.Create(new[]
            {
                new ScheduledMesTask(0f, "Pallet-A", "Workstation-A", "Task-A")
            });
            scenario.name = name;
            return scenario;
        }

        private static DispatchAssignment CreateAssignment(
            TaskPlanningAmr amr,
            ITaskPlanningTask task,
            PalletMarker pallet,
            WorkstationDeliveryPoint workstation)
        {
            return new DispatchAssignment(
                amr,
                task,
                pallet,
                loadingPoint: null,
                workstation,
                removalTargetNode: null,
                new CostEvaluation(isFeasible: true, totalCost: 0));
        }

        private static void DeleteReports(TaskPlanningMetricsCollector collector)
        {
            if (collector == null)
                return;

            DeleteReport(collector.LastReportPath);
            DeleteReport(collector.LastConfigReportPath);
            DeleteReport(collector.LastTraceReportPath);
        }

        private static void DeleteReport(string path)
        {
            if (!string.IsNullOrWhiteSpace(path) && File.Exists(path))
                File.Delete(path);
        }
    }
}
