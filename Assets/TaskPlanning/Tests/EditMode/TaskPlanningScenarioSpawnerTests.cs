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
        [TestCase(TaskPlanningScenarioPreset.RegretAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.HungarianAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.RegretDecoyTrap)]
        [TestCase(TaskPlanningScenarioPreset.GlobalAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.SoftReassignmentRescue)]
        [TestCase(TaskPlanningScenarioPreset.GridThroughputBenchmark)]
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
        [TestCase(TaskPlanningScenarioPreset.RegretAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.HungarianAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.RegretDecoyTrap)]
        [TestCase(TaskPlanningScenarioPreset.GlobalAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.SoftReassignmentRescue)]
        [TestCase(TaskPlanningScenarioPreset.GridThroughputBenchmark)]
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
        [TestCase(TaskPlanningScenarioPreset.RegretAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.HungarianAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.RegretDecoyTrap)]
        [TestCase(TaskPlanningScenarioPreset.GlobalAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.SoftReassignmentRescue)]
        [TestCase(TaskPlanningScenarioPreset.GridThroughputBenchmark)]
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
        [TestCase(TaskPlanningScenarioPreset.RegretAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.HungarianAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.RegretDecoyTrap)]
        [TestCase(TaskPlanningScenarioPreset.GlobalAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.SoftReassignmentRescue)]
        [TestCase(TaskPlanningScenarioPreset.GridThroughputBenchmark)]
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
        [TestCase(TaskPlanningScenarioPreset.RegretAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.HungarianAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.RegretDecoyTrap)]
        [TestCase(TaskPlanningScenarioPreset.GlobalAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.SoftReassignmentRescue)]
        [TestCase(TaskPlanningScenarioPreset.GridThroughputBenchmark)]
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
        [TestCase(TaskPlanningScenarioPreset.RegretAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.HungarianAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.RegretDecoyTrap)]
        [TestCase(TaskPlanningScenarioPreset.GlobalAssignmentTrap)]
        [TestCase(TaskPlanningScenarioPreset.SoftReassignmentRescue)]
        [TestCase(TaskPlanningScenarioPreset.GridThroughputBenchmark)]
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
        public void GridThroughputBenchmarkHasLargeSideBayGridAndTimedWorkload()
        {
            var scenario = TaskPlanningScenarioLibrary.GridThroughputBenchmark();
            var objectNodeIds = scenario.Amrs.Select(amr => amr.StartNodeId)
                .Concat(scenario.Pallets.Select(pallet => pallet.CurrentNodeId))
                .Concat(scenario.LoadingPoints.Select(point => point.NodeId))
                .Concat(scenario.Workstations.Select(point => point.NodeId));
            var edgeKeys = scenario.Edges
                .Select(edge => $"{edge.ANodeId}|{edge.BNodeId}")
                .ToHashSet();

            Assert.That(scenario.Name, Is.EqualTo("Grid Throughput Benchmark"));
            Assert.That(scenario.Amrs.Count, Is.EqualTo(8));
            Assert.That(scenario.Pallets.Count, Is.EqualTo(16));
            Assert.That(scenario.LoadingPoints.Count, Is.EqualTo(4));
            Assert.That(scenario.Workstations.Count, Is.EqualTo(12));
            Assert.That(scenario.ScheduledTasks.Count, Is.EqualTo(56));
            Assert.That(
                scenario.ScheduledTasks.Select(task => task.timestampSeconds).Distinct().Count(),
                Is.GreaterThanOrEqualTo(10));

            foreach (var nodeId in objectNodeIds)
            {
                Assert.That(int.Parse(nodeId), Is.GreaterThanOrEqualTo(9000));
            }

            Assert.That(edgeKeys.Contains("0|1") || edgeKeys.Contains("1|0"), Is.True);
            Assert.That(edgeKeys.Contains("0|100") || edgeKeys.Contains("100|0"), Is.True);
            Assert.That(edgeKeys.Contains("800|9000") || edgeKeys.Contains("9000|800"), Is.True);
            Assert.That(edgeKeys.Contains("0|9100") || edgeKeys.Contains("9100|0"), Is.True);
        }

        [Test]
        public void SoftReassignmentRescueMakesLateTaskCheaperForMovingAmrThanBackup()
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerSoftRescueTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", TaskPlanningScenarioPreset.SoftReassignmentRescue);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var graph = root.GetComponentInChildren<MapfSceneGraph>();
                var distances = new RoadmapDistanceService(graph);
                var nodes = root.GetComponentsInChildren<MapfNode>().ToDictionary(node => node.StableId);
                var loadingPoints = root.GetComponentsInChildren<PalletLoadingPoint>().ToDictionary(point => point.LoadingPointId);
                var pallets = root.GetComponentsInChildren<PalletMarker>().ToDictionary(pallet => pallet.PalletId);
                var workstations = root.GetComponentsInChildren<WorkstationDeliveryPoint>().ToDictionary(workstation => workstation.WorkstationId);
                var amrs = root.GetComponentsInChildren<TaskPlanningAmr>().ToDictionary(amr => amr.AmrId);
                var longTask = new DeliveryPlanningTask("SoftRescue_Long", pallets["Pallet_Long"], workstations["126"], 0f);
                var shortTask = new DeliveryPlanningTask("SoftRescue_Short", pallets["Pallet_Short"], workstations["107"], 2f);
                var tasks = new ITaskPlanningTask[] { shortTask };
                var evaluator = new TaskPlanningCostEvaluator(distances, CreateNoAgingWeights(), 1f, tasks, 2f);
                var movingAmrNode = nodes["2"];

                var keepLongCost = evaluator.EvaluateFrom(movingAmrNode, longTask, loadingPoints["123"], 0);
                var switchToShortCost = evaluator
                    .EvaluateFrom(movingAmrNode, shortTask, loadingPoints["105"], 0)
                    .WithReassignmentPenalty(2.0);
                var backupShortCost = evaluator.Evaluate(amrs["AMR_Backup"], shortTask, loadingPoints["105"]);

                Assert.That(keepLongCost.IsFeasible, Is.True);
                Assert.That(switchToShortCost.IsFeasible, Is.True);
                Assert.That(backupShortCost.IsFeasible, Is.True);
                Assert.That(switchToShortCost.TotalCost, Is.LessThan(keepLongCost.TotalCost));
                Assert.That(switchToShortCost.TotalCost, Is.LessThan(backupShortCost.TotalCost));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [Test]
        public void GlobalAssignmentTrapHungarianBeatsOtherDispatchers()
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerGlobalTrapTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", TaskPlanningScenarioPreset.GlobalAssignmentTrap);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var scenario = TaskPlanningScenarioLibrary.GlobalAssignmentTrap();
                var graph = root.GetComponentInChildren<MapfSceneGraph>();
                var distances = new RoadmapDistanceService(graph);
                var loadingPoints = root.GetComponentsInChildren<PalletLoadingPoint>();
                var pallets = root.GetComponentsInChildren<PalletMarker>().ToDictionary(pallet => pallet.PalletId);
                var workstations = root.GetComponentsInChildren<WorkstationDeliveryPoint>().ToDictionary(workstation => workstation.WorkstationId);
                var amrs = root.GetComponentsInChildren<TaskPlanningAmr>().ToDictionary(amr => amr.AmrId);
                var tasks = scenario.ScheduledTasks
                    .Select(task => new DeliveryPlanningTask(task.taskId, pallets[task.palletId], workstations[task.workstationId], 0f))
                    .Cast<ITaskPlanningTask>()
                    .ToArray();
                var evaluator = new TaskPlanningCostEvaluator(
                    distances,
                    CreateNoAgingWeights(),
                    0f,
                    tasks,
                    1f);
                var problem = new DispatchProblem(
                    tasks,
                    amrs.Values.ToArray(),
                    loadingPoints,
                    distances,
                    evaluator,
                    0f);

                var fifoPlan = new FifoDispatching().Solve(problem);
                var nearestPlan = new NearestDispatching().Solve(problem);
                var regretPlan = new RegretDispatching().Solve(problem);
                var hungarianPlan = new HungarianDispatching().Solve(problem);
                var hungarianByTask = hungarianPlan.Assignments.ToDictionary(assignment => assignment.Task.TaskId);
                var hungarianCost = hungarianPlan.Assignments.Sum(assignment => assignment.Score);

                Assert.That(fifoPlan.Assignments.Count, Is.EqualTo(6));
                Assert.That(nearestPlan.Assignments.Count, Is.EqualTo(6));
                Assert.That(regretPlan.Assignments.Count, Is.EqualTo(6));
                Assert.That(hungarianPlan.Assignments.Count, Is.EqualTo(6));
                Assert.That(hungarianByTask["GlobalTrap_H_Flexible"].Amr, Is.SameAs(amrs["AMR_H_Backup"]));
                Assert.That(hungarianByTask["GlobalTrap_H_Scarce"].Amr, Is.SameAs(amrs["AMR_H_Key"]));
                Assert.That(hungarianByTask["GlobalTrap_H_Right"].Amr, Is.SameAs(amrs["AMR_H_Right"]));
                Assert.That(hungarianByTask["GlobalTrap_R_Decoy"].Amr, Is.SameAs(amrs["AMR_R_Far"]));
                Assert.That(hungarianByTask["GlobalTrap_R_LocalA"].Amr, Is.SameAs(amrs["AMR_R_Key"]));
                Assert.That(hungarianByTask["GlobalTrap_R_LocalB"].Amr, Is.SameAs(amrs["AMR_R_Shared"]));
                Assert.That(hungarianCost, Is.LessThan(fifoPlan.Assignments.Sum(assignment => assignment.Score)));
                Assert.That(hungarianCost, Is.LessThan(nearestPlan.Assignments.Sum(assignment => assignment.Score)));
                Assert.That(hungarianCost, Is.LessThan(regretPlan.Assignments.Sum(assignment => assignment.Score)));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [Test]
        public void RegretDecoyTrapMakesDifferentDecisionThanHungarian()
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerRegretDecoyTrapTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", TaskPlanningScenarioPreset.RegretDecoyTrap);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var scenario = TaskPlanningScenarioLibrary.RegretDecoyTrap();
                var graph = root.GetComponentInChildren<MapfSceneGraph>();
                var distances = new RoadmapDistanceService(graph);
                var loadingPoints = root.GetComponentsInChildren<PalletLoadingPoint>();
                var pallets = root.GetComponentsInChildren<PalletMarker>().ToDictionary(pallet => pallet.PalletId);
                var workstations = root.GetComponentsInChildren<WorkstationDeliveryPoint>().ToDictionary(workstation => workstation.WorkstationId);
                var amrs = root.GetComponentsInChildren<TaskPlanningAmr>().ToDictionary(amr => amr.AmrId);
                var tasks = scenario.ScheduledTasks
                    .OrderBy(task => task.taskId)
                    .Select(task => new DeliveryPlanningTask(task.taskId, pallets[task.palletId], workstations[task.workstationId], 0f))
                    .Cast<ITaskPlanningTask>()
                    .ToArray();
                var evaluator = new TaskPlanningCostEvaluator(
                    distances,
                    CreateNoAgingWeights(),
                    0f,
                    tasks,
                    1f);
                var problem = new DispatchProblem(
                    tasks,
                    amrs.Values.ToArray(),
                    loadingPoints,
                    distances,
                    evaluator,
                    0f);

                var regretPlan = new RegretDispatching().Solve(problem);
                var hungarianPlan = new HungarianDispatching().Solve(problem);
                var regretByTask = regretPlan.Assignments.ToDictionary(assignment => assignment.Task.TaskId);
                var hungarianByTask = hungarianPlan.Assignments.ToDictionary(assignment => assignment.Task.TaskId);

                Assert.That(regretPlan.Assignments.Count, Is.EqualTo(3));
                Assert.That(hungarianPlan.Assignments.Count, Is.EqualTo(3));
                Assert.That(regretByTask["RegretDecoyTrap_Decoy"].Amr, Is.SameAs(amrs["AMR_Key"]));
                Assert.That(regretByTask["RegretDecoyTrap_LocalA"].Amr, Is.SameAs(amrs["AMR_Far"]));
                Assert.That(regretByTask["RegretDecoyTrap_LocalB"].Amr, Is.SameAs(amrs["AMR_Shared"]));
                Assert.That(hungarianByTask["RegretDecoyTrap_Decoy"].Amr, Is.SameAs(amrs["AMR_Far"]));
                Assert.That(hungarianByTask["RegretDecoyTrap_LocalA"].Amr, Is.SameAs(amrs["AMR_Key"]));
                Assert.That(hungarianByTask["RegretDecoyTrap_LocalB"].Amr, Is.SameAs(amrs["AMR_Shared"]));
                Assert.That(
                    hungarianPlan.Assignments.Sum(assignment => assignment.Score),
                    Is.LessThan(regretPlan.Assignments.Sum(assignment => assignment.Score)));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [Test]
        public void HungarianAssignmentTrapMakesDifferentDecisionThanNearest()
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerHungarianTrapTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", TaskPlanningScenarioPreset.HungarianAssignmentTrap);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var scenario = TaskPlanningScenarioLibrary.HungarianAssignmentTrap();
                var graph = root.GetComponentInChildren<MapfSceneGraph>();
                var distances = new RoadmapDistanceService(graph);
                var loadingPoints = root.GetComponentsInChildren<PalletLoadingPoint>();
                var pallets = root.GetComponentsInChildren<PalletMarker>().ToDictionary(pallet => pallet.PalletId);
                var workstations = root.GetComponentsInChildren<WorkstationDeliveryPoint>().ToDictionary(workstation => workstation.WorkstationId);
                var amrs = root.GetComponentsInChildren<TaskPlanningAmr>().ToDictionary(amr => amr.AmrId);
                var tasks = scenario.ScheduledTasks
                    .OrderBy(task => task.taskId)
                    .Select(task => new DeliveryPlanningTask(task.taskId, pallets[task.palletId], workstations[task.workstationId], 0f))
                    .Cast<ITaskPlanningTask>()
                    .ToArray();
                var evaluator = new TaskPlanningCostEvaluator(
                    distances,
                    CreateNoAgingWeights(),
                    0f,
                    tasks,
                    1f);
                var problem = new DispatchProblem(
                    tasks,
                    amrs.Values.ToArray(),
                    loadingPoints,
                    distances,
                    evaluator,
                    0f);

                var nearestPlan = new NearestDispatching().Solve(problem);
                var hungarianPlan = new HungarianDispatching().Solve(problem);
                var nearestByTask = nearestPlan.Assignments.ToDictionary(assignment => assignment.Task.TaskId);
                var hungarianByTask = hungarianPlan.Assignments.ToDictionary(assignment => assignment.Task.TaskId);

                Assert.That(nearestPlan.Assignments.Count, Is.EqualTo(3));
                Assert.That(hungarianPlan.Assignments.Count, Is.EqualTo(3));
                Assert.That(nearestByTask["HungarianTrap_Flexible"].Amr, Is.SameAs(amrs["AMR_Key"]));
                Assert.That(nearestByTask["HungarianTrap_Scarce"].Amr, Is.SameAs(amrs["AMR_Backup"]));
                Assert.That(nearestByTask["HungarianTrap_Right"].Amr, Is.SameAs(amrs["AMR_Right"]));
                Assert.That(hungarianByTask["HungarianTrap_Flexible"].Amr, Is.SameAs(amrs["AMR_Backup"]));
                Assert.That(hungarianByTask["HungarianTrap_Scarce"].Amr, Is.SameAs(amrs["AMR_Key"]));
                Assert.That(hungarianByTask["HungarianTrap_Right"].Amr, Is.SameAs(amrs["AMR_Right"]));
                Assert.That(
                    hungarianPlan.Assignments.Sum(assignment => assignment.Score),
                    Is.LessThan(nearestPlan.Assignments.Sum(assignment => assignment.Score)));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
            }
        }

        [Test]
        public void RegretAssignmentTrapMakesDifferentDecisionThanNearest()
        {
            var root = new GameObject("TaskPlanningScenarioSpawnerRegretTrapTest");
            var spawner = root.AddComponent<TaskPlanningScenarioSpawner>();
            TaskPlanningTestHelpers.SetField(spawner, "preset", TaskPlanningScenarioPreset.RegretAssignmentTrap);
            TaskPlanningTestHelpers.SetField(spawner, "saveMesScheduledScenarioAssetInProject", false);

            try
            {
                spawner.Spawn();

                var scenario = TaskPlanningScenarioLibrary.RegretAssignmentTrap();
                var graph = root.GetComponentInChildren<MapfSceneGraph>();
                var distances = new RoadmapDistanceService(graph);
                var loadingPoints = root.GetComponentsInChildren<PalletLoadingPoint>();
                var pallets = root.GetComponentsInChildren<PalletMarker>().ToDictionary(pallet => pallet.PalletId);
                var workstations = root.GetComponentsInChildren<WorkstationDeliveryPoint>().ToDictionary(workstation => workstation.WorkstationId);
                var amrs = root.GetComponentsInChildren<TaskPlanningAmr>().ToDictionary(amr => amr.AmrId);
                var tasks = scenario.ScheduledTasks
                    .OrderBy(task => task.taskId)
                    .Select(task => new DeliveryPlanningTask(task.taskId, pallets[task.palletId], workstations[task.workstationId], 0f))
                    .Cast<ITaskPlanningTask>()
                    .ToArray();
                var evaluator = new TaskPlanningCostEvaluator(
                    distances,
                    CreateNoAgingWeights(),
                    0f,
                    tasks,
                    1f);
                var problem = new DispatchProblem(
                    tasks,
                    amrs.Values.ToArray(),
                    loadingPoints,
                    distances,
                    evaluator,
                    0f);

                var nearestPlan = new NearestDispatching().Solve(problem);
                var regretPlan = new RegretDispatching().Solve(problem);
                var nearestByTask = nearestPlan.Assignments.ToDictionary(assignment => assignment.Task.TaskId);
                var regretByTask = regretPlan.Assignments.ToDictionary(assignment => assignment.Task.TaskId);

                Assert.That(nearestPlan.Assignments, Has.Count.EqualTo(2));
                Assert.That(regretPlan.Assignments, Has.Count.EqualTo(2));
                Assert.That(nearestByTask["RegretTrap_Flexible"].Amr, Is.SameAs(amrs["AMR_Key"]));
                Assert.That(nearestByTask["RegretTrap_Scarce"].Amr, Is.SameAs(amrs["AMR_Backup"]));
                Assert.That(regretByTask["RegretTrap_Flexible"].Amr, Is.SameAs(amrs["AMR_Backup"]));
                Assert.That(regretByTask["RegretTrap_Scarce"].Amr, Is.SameAs(amrs["AMR_Key"]));
                Assert.That(
                    regretPlan.Assignments.Sum(assignment => assignment.Score),
                    Is.LessThan(nearestPlan.Assignments.Sum(assignment => assignment.Score)));
            }
            finally
            {
                var mes = FindMes(root);
                var generatedScenario = mes != null ? mes.ScheduledScenario : null;
                TaskPlanningTestHelpers.Destroy(root);
                TaskPlanningTestHelpers.Destroy(generatedScenario);
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
