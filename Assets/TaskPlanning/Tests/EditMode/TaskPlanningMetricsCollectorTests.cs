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
                DeleteReport(collector.LastReportPath);
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
                Assert.That(report, Does.Contain("Dispatcher algorithm: NearestDispatching"));
                Assert.That(report, Does.Contain("Future handling policy: LookAhead"));
                Assert.That(report, Does.Contain("Total MES tasks: 1"));
                Assert.That(report, Does.Contain("Cost weights:"));
                Assert.That(report, Does.Contain("Makespan: 5 s"));
                Assert.That(report, Does.Contain("AMR Metrics_Report_AMR:"));
                Assert.That(report, Does.Contain("All AMRs total:"));
                Assert.That(report, Does.Contain("All AMRs average:"));
            }
            finally
            {
                DeleteReport(collector.LastReportPath);
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
                DeleteReport(collector.LastReportPath);
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
                DeleteReport(collector.LastReportPath);
                TaskPlanningTestHelpers.Destroy(
                    scenario,
                    workstation.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    collector.gameObject);
            }
        }

        [Test]
        public void ReportWriterCreatesTxtFileInExpectedTaskPlanningMetricsFolder()
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
                Assert.That(Path.GetExtension(collector.LastReportPath), Is.EqualTo(".txt"));
                Assert.That(
                    Path.GetFileName(collector.LastReportPath),
                    Does.Match(@"^MetricsFileScenario_Unknown_Unknown_\d{3}\.txt$"));
                Assert.That(File.Exists(collector.LastReportPath), Is.True);
            }
            finally
            {
                DeleteReport(collector.LastReportPath);
                TaskPlanningTestHelpers.Destroy(scenario, collector.gameObject);
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

        private static void DeleteReport(string path)
        {
            if (!string.IsNullOrWhiteSpace(path) && File.Exists(path))
                File.Delete(path);
        }
    }
}
