using System.Linq;
using NUnit.Framework;

namespace TaskPlanning.Tests
{
    public sealed class TaskPlanningMesScenarioTests
    {
        [Test]
        public void ScenarioAssetCanBeCreatedFromCode()
        {
            var scenario = TaskPlanningScenarioAsset.Create(new[]
            {
                new ScheduledMesTask(5f, "Pallet-B", "Workstation-B", "Task-B"),
                new ScheduledMesTask(1f, "Pallet-A", "Workstation-A", "Task-A")
            });

            try
            {
                Assert.That(scenario.ScheduledTasks, Has.Count.EqualTo(2));

                var ordered = scenario.OrderedTasks();
                Assert.That(ordered[0].taskId, Is.EqualTo("Task-A"));
                Assert.That(ordered[1].taskId, Is.EqualTo("Task-B"));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(scenario);
            }
        }

        [Test]
        public void MesCollectsSameTimestampScenarioTasksAsOneBatch()
        {
            var mes = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMes>("MesScenarioBatch_Mes");
            var palletA = TaskPlanningTestHelpers.CreatePallet("MesScenarioBatch_PalletA");
            var palletB = TaskPlanningTestHelpers.CreatePallet("MesScenarioBatch_PalletB");
            var workstationA = TaskPlanningTestHelpers.CreateWorkstation("MesScenarioBatch_WorkstationA", null, palletA);
            var workstationB = TaskPlanningTestHelpers.CreateWorkstation("MesScenarioBatch_WorkstationB", null, palletB);
            var scenario = TaskPlanningScenarioAsset.Create(new[]
            {
                new ScheduledMesTask(0f, palletA.PalletId, workstationA.WorkstationId, "Task-A"),
                new ScheduledMesTask(0f, palletB.PalletId, workstationB.WorkstationId, "Task-B")
            });

            try
            {
                TaskPlanningTestHelpers.SetField(mes, "scheduledScenario", scenario);
                TaskPlanningTestHelpers.SetField(mes, "submitSameTimestampAsBatch", true);

                mes.StartScheduledScenarioPlayback();
                var batches = mes.CollectDueScheduledScenarioBatches(0f);

                Assert.That(batches, Has.Count.EqualTo(1));
                Assert.That(batches[0].TimestampSeconds, Is.EqualTo(0f));
                Assert.That(batches[0].Requests, Has.Count.EqualTo(2));
                Assert.That(batches[0].Requests.Select(request => request.taskId), Is.EquivalentTo(new[] { "Task-A", "Task-B" }));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    scenario,
                    workstationB.gameObject,
                    workstationA.gameObject,
                    palletB.gameObject,
                    palletA.gameObject,
                    mes.gameObject);
            }
        }

        [Test]
        public void MesSkipsInvalidScenarioTaskAndContinuesWithValidTask()
        {
            var mes = TaskPlanningTestHelpers.CreateComponent<TaskPlanningMes>("MesScenarioInvalid_Mes");
            var pallet = TaskPlanningTestHelpers.CreatePallet("MesScenarioInvalid_Pallet");
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("MesScenarioInvalid_Workstation", null, pallet);
            var scenario = TaskPlanningScenarioAsset.Create(new[]
            {
                new ScheduledMesTask(0f, "MissingPallet", workstation.WorkstationId, "Invalid"),
                new ScheduledMesTask(0f, pallet.PalletId, workstation.WorkstationId, "Valid")
            });

            try
            {
                TaskPlanningTestHelpers.SetField(mes, "scheduledScenario", scenario);

                mes.StartScheduledScenarioPlayback();
                var batches = mes.CollectDueScheduledScenarioBatches(0f);

                Assert.That(batches, Has.Count.EqualTo(1));
                Assert.That(batches[0].Requests, Has.Count.EqualTo(1));
                Assert.That(batches[0].Requests[0].taskId, Is.EqualTo("Valid"));
                Assert.That(batches[0].Requests[0].pallet, Is.SameAs(pallet));
                Assert.That(batches[0].Requests[0].workstation, Is.SameAs(workstation));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    scenario,
                    workstation.gameObject,
                    pallet.gameObject,
                    mes.gameObject);
            }
        }
    }
}
