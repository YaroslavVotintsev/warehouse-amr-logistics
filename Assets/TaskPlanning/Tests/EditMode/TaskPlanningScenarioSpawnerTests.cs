using System.Collections.Generic;
using System.Linq;
using Mapf.Authoring;
using Mapf.UnityAdapter;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class TaskPlanningScenarioSpawnerTests
    {
        [TestCase(TaskPlanningScenarioPreset.SideBayLoadingBottleneck)]
        [TestCase(TaskPlanningScenarioPreset.SideBayLookAheadTrap)]
        public void SpawnerCreatesCompleteScenarioUnderSpawner(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveScenarioAssetInProject", false);
            var scenario = TaskPlanningScenarioLibrary.Get(preset);

            try
            {
                spawner.Spawn();

                Assert.That(root.GetComponent<MapfSceneGraph>(), Is.Not.Null);
                var coordinator = root.GetComponent<MapfCoordinator>();
                Assert.That(coordinator, Is.Not.Null);
                Assert.That(TaskPlanningTestHelpers.GetField<bool>(coordinator, "planOnStart"), Is.False);
                Assert.That(root.GetComponent<TaskScheduler>(), Is.Not.Null);
                Assert.That(root.GetComponent<TaskPlanningMes>(), Is.Not.Null);
                Assert.That(root.GetComponent<MapfDebugGizmos>(), Is.Not.Null);
                Assert.That(root.GetComponent<TaskPlanningDebugGizmos>(), Is.Not.Null);

                Assert.That(root.GetComponentsInChildren<MapfNode>(), Has.Length.EqualTo(scenario.Nodes.Count));
                Assert.That(root.GetComponentsInChildren<MapfEdge>(), Has.Length.EqualTo(scenario.Edges.Count));
                Assert.That(root.GetComponentsInChildren<TaskPlanningAmr>(), Has.Length.EqualTo(scenario.Amrs.Count));
                Assert.That(root.GetComponentsInChildren<PalletMarker>(), Has.Length.EqualTo(scenario.Pallets.Count));
                Assert.That(root.GetComponentsInChildren<PalletLoadingPoint>(), Has.Length.EqualTo(scenario.LoadingPoints.Count));
                Assert.That(root.GetComponentsInChildren<WorkstationDeliveryPoint>(), Has.Length.EqualTo(scenario.Workstations.Count));
            }
            finally
            {
                var mes = root != null ? root.GetComponent<TaskPlanningMes>() : null;
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.SideBayLoadingBottleneck)]
        [TestCase(TaskPlanningScenarioPreset.SideBayLookAheadTrap)]
        public void SpawnerWiresScheduledMesScenario(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerMesTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveScenarioAssetInProject", false);
            var scenario = TaskPlanningScenarioLibrary.Get(preset);

            try
            {
                spawner.Spawn();

                var mes = root.GetComponent<TaskPlanningMes>();
                Assert.That(mes.ScheduledScenario, Is.Not.Null);
                Assert.That(mes.ScheduledScenario.ScheduledTasks, Has.Count.EqualTo(scenario.ScheduledTasks.Count));
                Assert.That(mes.ScheduledScenario.OrderedTasks().Select(task => task.taskId), Is.EqualTo(scenario.ScheduledTasks.OrderBy(task => task.timestampSeconds).Select(task => task.taskId)));
            }
            finally
            {
                var mes = root != null ? root.GetComponent<TaskPlanningMes>() : null;
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.SideBayLoadingBottleneck)]
        [TestCase(TaskPlanningScenarioPreset.SideBayLookAheadTrap)]
        public void SpawnedPalletsResolveExactlyOneLoadingPoint(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerCompatibilityTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var loadingPoints = root.GetComponentsInChildren<PalletLoadingPoint>();
                foreach (var pallet in root.GetComponentsInChildren<PalletMarker>())
                {
                    var resolution = PalletLoadingPoint.ResolveAcceptedLoadingPoint(pallet, loadingPoints);
                    Assert.That(resolution.IsResolved, Is.True, resolution.Message);
                }
            }
            finally
            {
                var mes = root != null ? root.GetComponent<TaskPlanningMes>() : null;
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.SideBayLoadingBottleneck)]
        [TestCase(TaskPlanningScenarioPreset.SideBayLookAheadTrap)]
        public void SpawnerConfiguresSchedulerWithSpawnedObjects(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerSchedulerTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var scheduler = root.GetComponent<TaskScheduler>();
                var configuredAmrs = TaskPlanningTestHelpers.GetField<List<TaskPlanningAmr>>(scheduler, "amrs");
                var configuredLoadingPoints = TaskPlanningTestHelpers.GetField<List<PalletLoadingPoint>>(scheduler, "loadingPoints");
                var autoDiscover = TaskPlanningTestHelpers.GetField<bool>(scheduler, "autoDiscoverSceneObjects");

                Assert.That(autoDiscover, Is.False);
                Assert.That(configuredAmrs, Is.EquivalentTo(root.GetComponentsInChildren<TaskPlanningAmr>()));
                Assert.That(configuredLoadingPoints, Is.EquivalentTo(root.GetComponentsInChildren<PalletLoadingPoint>()));
            }
            finally
            {
                var mes = root != null ? root.GetComponent<TaskPlanningMes>() : null;
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.SideBayLoadingBottleneck)]
        [TestCase(TaskPlanningScenarioPreset.SideBayLookAheadTrap)]
        public void SpawnerNamesNodesAndPointsByTheirNumericIds(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerNamesTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                foreach (var node in root.GetComponentsInChildren<MapfNode>())
                {
                    Assert.That(node.name, Is.EqualTo(node.StableId));
                    Assert.That(int.TryParse(node.StableId, out _), Is.True, $"Node id '{node.StableId}' should be numeric.");
                }

                foreach (var loadingPoint in root.GetComponentsInChildren<PalletLoadingPoint>())
                {
                    Assert.That(loadingPoint.name, Is.EqualTo(loadingPoint.Node.StableId));
                    Assert.That(loadingPoint.LoadingPointId, Is.EqualTo(loadingPoint.Node.StableId));
                }

                foreach (var workstation in root.GetComponentsInChildren<WorkstationDeliveryPoint>())
                {
                    Assert.That(workstation.name, Is.EqualTo(workstation.Node.StableId));
                    Assert.That(workstation.WorkstationId, Is.EqualTo(workstation.Node.StableId));
                }
            }
            finally
            {
                var mes = root != null ? root.GetComponent<TaskPlanningMes>() : null;
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }
    }
}
