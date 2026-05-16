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
        [TestCase(TaskPlanningScenarioPreset.FifoAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureWaitTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureCapacityTrap)]
        [TestCase(TaskPlanningScenarioPreset.RollingHorizonCapacitySaturation)]
        public void SpawnerCreatesCompleteScenarioUnderSpawner(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);
            var scenario = TaskPlanningScenarioLibrary.Get(preset);

            try
            {
                spawner.Spawn();

                Assert.That(root.GetComponent<MapfSceneGraph>(), Is.Null);
                Assert.That(root.GetComponent<MapfCoordinator>(), Is.Null);
                Assert.That(root.GetComponent<TaskScheduler>(), Is.Null);
                Assert.That(root.GetComponent<TaskPlanningMes>(), Is.Null);
                Assert.That(root.GetComponent<MapfDebugGizmos>(), Is.Null);
                Assert.That(root.GetComponent<TaskPlanningDebugGizmos>(), Is.Null);
                Assert.That(root.GetComponent<TaskPlanningMetricsCollector>(), Is.Null);

                var mapfRoot = RequireChild(root, "MAPF");
                var schedulerRoot = RequireChild(root, "Scheduler");
                var mesRoot = RequireChild(root, "MES");

                Assert.That(mapfRoot.GetComponent<MapfSceneGraph>(), Is.Not.Null);
                var coordinator = mapfRoot.GetComponent<MapfCoordinator>();
                Assert.That(coordinator, Is.Not.Null);
                Assert.That(TaskPlanningTestHelpers.GetField<bool>(coordinator, "planOnStart"), Is.False);
                Assert.That(mapfRoot.GetComponent<MapfDebugGizmos>(), Is.Not.Null);
                Assert.That(schedulerRoot.GetComponent<TaskScheduler>(), Is.Not.Null);
                Assert.That(schedulerRoot.GetComponent<TaskPlanningDebugGizmos>(), Is.Not.Null);
                Assert.That(mesRoot.GetComponent<TaskPlanningMes>(), Is.Not.Null);
                Assert.That(mesRoot.GetComponent<TaskPlanningMetricsCollector>(), Is.Not.Null);

                Assert.That(root.GetComponentsInChildren<MapfNode>(), Has.Length.EqualTo(scenario.Nodes.Count));
                Assert.That(root.GetComponentsInChildren<MapfEdge>(), Has.Length.EqualTo(scenario.Edges.Count));
                Assert.That(root.GetComponentsInChildren<TaskPlanningAmr>(), Has.Length.EqualTo(scenario.Amrs.Count));
                Assert.That(root.GetComponentsInChildren<PalletMarker>(), Has.Length.EqualTo(scenario.Pallets.Count));
                Assert.That(root.GetComponentsInChildren<PalletLoadingPoint>(), Has.Length.EqualTo(scenario.LoadingPoints.Count));
                Assert.That(root.GetComponentsInChildren<WorkstationDeliveryPoint>(), Has.Length.EqualTo(scenario.Workstations.Count));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.FifoAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureWaitTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureCapacityTrap)]
        [TestCase(TaskPlanningScenarioPreset.RollingHorizonCapacitySaturation)]
        public void SpawnerWiresScheduledMesScenario(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerMesTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);
            var scenario = TaskPlanningScenarioLibrary.Get(preset);

            try
            {
                spawner.Spawn();

                var mes = FindMes(root);
                Assert.That(mes.ScheduledScenario, Is.Not.Null);
                Assert.That(mes.ScheduledScenario.ScheduledTasks, Has.Count.EqualTo(scenario.ScheduledTasks.Count));
                Assert.That(mes.ScheduledScenario.OrderedTasks().Select(task => task.taskId), Is.EqualTo(scenario.ScheduledTasks.OrderBy(task => task.timestampSeconds).Select(task => task.taskId)));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.FifoAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureWaitTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureCapacityTrap)]
        [TestCase(TaskPlanningScenarioPreset.RollingHorizonCapacitySaturation)]
        public void SpawnedPalletsResolveExactlyOneLoadingPoint(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerCompatibilityTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

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
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.FifoAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureWaitTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureCapacityTrap)]
        [TestCase(TaskPlanningScenarioPreset.RollingHorizonCapacitySaturation)]
        public void SpawnerConfiguresSchedulerWithSpawnedObjects(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerSchedulerTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var scheduler = root.GetComponentInChildren<TaskScheduler>();
                var configuredAmrs = TaskPlanningTestHelpers.GetField<List<TaskPlanningAmr>>(scheduler, "amrs");
                var configuredLoadingPoints = TaskPlanningTestHelpers.GetField<List<PalletLoadingPoint>>(scheduler, "loadingPoints");
                var autoDiscover = TaskPlanningTestHelpers.GetField<bool>(scheduler, "autoDiscoverSceneObjects");

                Assert.That(autoDiscover, Is.False);
                Assert.That(configuredAmrs, Is.EquivalentTo(root.GetComponentsInChildren<TaskPlanningAmr>()));
                Assert.That(configuredLoadingPoints, Is.EquivalentTo(root.GetComponentsInChildren<PalletLoadingPoint>()));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.FifoAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureWaitTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureCapacityTrap)]
        [TestCase(TaskPlanningScenarioPreset.RollingHorizonCapacitySaturation)]
        public void SpawnerNamesNodesAndPointsByTheirNumericIds(TaskPlanningScenarioPreset preset)
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerNamesTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", preset);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

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
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [TestCase(TaskPlanningScenarioPreset.FifoAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureWaitTrap)]
        [TestCase(TaskPlanningScenarioPreset.FutureCapacityTrap)]
        [TestCase(TaskPlanningScenarioPreset.RollingHorizonCapacitySaturation)]
        public void ScenarioPalletsDoNotSpawnOnLoadingOrWorkstationNodes(TaskPlanningScenarioPreset preset)
        {
            var scenario = TaskPlanningScenarioLibrary.Get(preset);
            var loadingPointNodeIds = scenario.LoadingPoints.Select(point => point.NodeId).ToHashSet();
            var workstationNodeIds = scenario.Workstations.Select(point => point.NodeId).ToHashSet();

            foreach (var pallet in scenario.Pallets)
            {
                Assert.That(
                    loadingPointNodeIds.Contains(pallet.CurrentNodeId),
                    Is.False,
                    $"{scenario.Name}: pallet '{pallet.PalletId}' should not spawn on loading point node '{pallet.CurrentNodeId}'.");
                Assert.That(
                    workstationNodeIds.Contains(pallet.CurrentNodeId),
                    Is.False,
                    $"{scenario.Name}: pallet '{pallet.PalletId}' should not spawn on workstation node '{pallet.CurrentNodeId}'.");
            }
        }

        [Test]
        public void RollingHorizonCapacitySaturationMakesDifferentDecisionThanLookAhead()
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerRollingHorizonTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", TaskPlanningScenarioPreset.RollingHorizonCapacitySaturation);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var scenario = TaskPlanningScenarioLibrary.RollingHorizonCapacitySaturation();
                var graph = root.GetComponentInChildren<MapfSceneGraph>();
                var distances = new RoadmapDistanceService(graph);
                var loadingPoints = root.GetComponentsInChildren<PalletLoadingPoint>();
                var pallets = root.GetComponentsInChildren<PalletMarker>().ToDictionary(pallet => pallet.PalletId);
                var workstations = root.GetComponentsInChildren<WorkstationDeliveryPoint>().ToDictionary(workstation => workstation.WorkstationId);
                var amrs = root.GetComponentsInChildren<TaskPlanningAmr>().ToDictionary(amr => amr.AmrId);
                var targetTasks = scenario.ScheduledTasks
                    .Where(task => task.timestampSeconds > 0f)
                    .OrderBy(task => task.taskId)
                    .Select(task => new DeliveryPlanningTask(task.taskId, pallets[task.palletId], workstations[task.workstationId], 1f))
                    .Cast<ITaskPlanningTask>()
                    .ToArray();
                var primerTask = new DeliveryPlanningTask("PrimeFutureCapacityAmr", pallets["Pallet_Primer"], workstations["259"], 0f);
                var futureAvailability = new[]
                {
                    new AmrFutureAvailability(
                        amrs["AMR_Future"],
                        primerTask,
                        workstations["259"].Node,
                        priorAssignmentEta: 12.0,
                        isInterruptible: false)
                };
                var immediateAmrs = new[]
                {
                    amrs["AMR_Remote_A"],
                    amrs["AMR_Remote_B"],
                    amrs["AMR_Remote_C"]
                };
                var evaluator = new TaskPlanningCostEvaluator(
                    distances,
                    CreateNoAgingWeights(),
                    1f,
                    targetTasks,
                    1f);
                var problem = new DispatchProblem(
                    targetTasks,
                    immediateAmrs,
                    loadingPoints,
                    distances,
                    evaluator,
                    1f,
                    futureAvailability);

                var lookAheadPlan = new LookAheadFuturePolicy(10f).Solve(problem, new NearestDispatching());
                var rollingPlan = new RollingHorizonFuturePolicy(new RollingHorizonOptions
                {
                    horizonSeconds = 30f,
                    maxWaves = 4,
                    waitImprovementPercent = 10f
                }).Solve(problem, new NearestDispatching());

                Assert.That(lookAheadPlan.Assignments, Is.Empty);
                Assert.That(rollingPlan.Assignments, Has.Count.EqualTo(3));
                Assert.That(
                    rollingPlan.Assignments.Select(assignment => assignment.Amr),
                    Is.EquivalentTo(immediateAmrs));
                Assert.That(
                    rollingPlan.Assignments.Select(assignment => assignment.Task.TaskId),
                    Does.Not.Contain("CapacityOverflowA"));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        private static GameObject RequireChild(GameObject root, string childName)
        {
            var child = root.transform.Find(childName);
            Assert.That(child, Is.Not.Null, $"Expected spawned scenario root to contain child '{childName}'.");
            return child.gameObject;
        }

        private static TaskPlanningCostWeights CreateNoAgingWeights()
        {
            var weights = new TaskPlanningCostWeights();
            weights.delivery.agingWeight = 0f;
            weights.delivery.maxAgingBonus = 0f;
            weights.removal.agingWeight = 0f;
            weights.removal.maxAgingBonus = 0f;
            return weights;
        }

        private static TaskPlanningMes FindMes(GameObject root)
        {
            return root != null ? root.GetComponentInChildren<TaskPlanningMes>() : null;
        }
    }
}
