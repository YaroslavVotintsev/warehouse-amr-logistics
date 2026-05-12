using Mapf.Authoring;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class RollingHorizonFuturePolicyTests
    {
        [Test]
        public void RollingHorizonSuppressesImmediateAssignmentWhenFuturePlanIsBetter()
        {
            var fixture = CreateSingleTaskFixture("RollingSuppress", futureDistanceToPallet: 0f);

            try
            {
                var task = new DeliveryPlanningTask("D-Suppress", fixture.Pallet, fixture.Workstation, 0f);
                var plan = Solve(
                    fixture,
                    new ITaskPlanningTask[] { task },
                    new[] { fixture.FreeAmr },
                    new[] { FutureAvailability(fixture.BusyAmr, fixture.FutureNode, task, 0.0) },
                    new NearestDispatching(),
                    waitImprovementPercent: 20f);

                Assert.That(plan.Assignments, Is.Empty);
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RollingHorizonDoesNotSuppressWhenFutureImprovementIsBelowThreshold()
        {
            var fixture = CreateSingleTaskFixture("RollingBelowThreshold", futureDistanceToPallet: 50f);

            try
            {
                var task = new DeliveryPlanningTask("D-NoSuppress", fixture.Pallet, fixture.Workstation, 0f);
                var plan = Solve(
                    fixture,
                    new ITaskPlanningTask[] { task },
                    new[] { fixture.FreeAmr },
                    new[] { FutureAvailability(fixture.BusyAmr, fixture.FutureNode, task, 0.0) },
                    new NearestDispatching(),
                    waitImprovementPercent: 60f);

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Amr, Is.SameAs(fixture.FreeAmr));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(task));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RollingHorizonReturnsOnlyAssignmentsThatCanStartNow()
        {
            var fixture = CreateSingleTaskFixture("RollingOnlyImmediate", futureDistanceToPallet: 0f);

            try
            {
                var task = new DeliveryPlanningTask("D-FutureOnly", fixture.Pallet, fixture.Workstation, 0f);
                var plan = Solve(
                    fixture,
                    new ITaskPlanningTask[] { task },
                    new TaskPlanningAmr[] { },
                    new[] { FutureAvailability(fixture.BusyAmr, fixture.FutureNode, task, 0.0) },
                    new NearestDispatching(),
                    waitImprovementPercent: 0f);

                Assert.That(plan.Assignments, Is.Empty);
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RollingHorizonFutureCandidatesWorkWithFifoBaseDispatcher()
        {
            var fixture = CreateSingleTaskFixture("RollingFifo", futureDistanceToPallet: 0f);

            try
            {
                var task = new DeliveryPlanningTask("D-FifoSuppress", fixture.Pallet, fixture.Workstation, 0f);
                fixture.BusyAmr.TryReserve();
                var plan = Solve(
                    fixture,
                    new ITaskPlanningTask[] { task },
                    new[] { fixture.FreeAmr },
                    new[] { FutureAvailability(fixture.BusyAmr, fixture.FutureNode, task, 0.0) },
                    new FifoDispatching(),
                    waitImprovementPercent: 20f);

                Assert.That(plan.Assignments, Is.Empty);
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RollingHorizonUsesVirtualLoadingQueueWhenComparingFutureAssignments()
        {
            var fixture = CreateSharedLoadingFixture("RollingLoadingQueue");

            try
            {
                var queueTask = new DeliveryPlanningTask("D-QueueFirst", fixture.QueuePallet, fixture.Workstation, 0f);
                var targetTask = new DeliveryPlanningTask("D-Target", fixture.TargetPallet, fixture.Workstation, 0f);
                var tasks = new ITaskPlanningTask[] { queueTask, targetTask };
                var plan = Solve(
                    fixture.Graph,
                    tasks,
                    new[] { fixture.FreeAmr },
                    new[]
                    {
                        FutureAvailability(fixture.FutureAmrA, fixture.SharedNode, queueTask, 0.0),
                        FutureAvailability(fixture.FutureAmrB, fixture.SharedNode, targetTask, 0.0)
                    },
                    new[] { fixture.LoadingPoint },
                    new NearestDispatching(),
                    CreateWeights(),
                    waitImprovementPercent: 10f);

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(targetTask));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        private static DispatchPlan Solve(
            SingleTaskFixture fixture,
            ITaskPlanningTask[] tasks,
            TaskPlanningAmr[] immediateAmrs,
            AmrFutureAvailability[] futureAvailabilities,
            ITaskDispatchAlgorithm baseDispatcher,
            float waitImprovementPercent)
        {
            return Solve(
                fixture.Graph,
                tasks,
                immediateAmrs,
                futureAvailabilities,
                new[] { fixture.LoadingPoint },
                baseDispatcher,
                CreateWeights(),
                waitImprovementPercent);
        }

        private static DispatchPlan Solve(
            MapfSceneGraph graph,
            ITaskPlanningTask[] tasks,
            TaskPlanningAmr[] immediateAmrs,
            AmrFutureAvailability[] futureAvailabilities,
            PalletLoadingPoint[] loadingPoints,
            ITaskDispatchAlgorithm baseDispatcher,
            TaskPlanningCostWeights weights,
            float waitImprovementPercent)
        {
            var distances = new RoadmapDistanceService(graph);
            var evaluator = new TaskPlanningCostEvaluator(distances, weights, 1f, tasks, 0f);
            var problem = new DispatchProblem(tasks, immediateAmrs, loadingPoints, distances, evaluator, 0f, futureAvailabilities);
            var options = new RollingHorizonOptions
            {
                horizonSeconds = 200f,
                maxWaves = 4,
                waitImprovementPercent = waitImprovementPercent
            };
            return new RollingHorizonFuturePolicy(options).Solve(problem, baseDispatcher);
        }

        private static TaskPlanningCostWeights CreateWeights()
        {
            var weights = new TaskPlanningCostWeights();
            weights.delivery.priorAssignmentEta = 0f;
            weights.delivery.amrToPalletEta = 1f;
            weights.delivery.attachTime = 0f;
            weights.delivery.loadingQueueEta = 1f;
            weights.delivery.palletToLoadingEta = 0f;
            weights.delivery.loadTime = 0f;
            weights.delivery.loadingToWorkstationEta = 0f;
            weights.delivery.detachTime = 0f;
            weights.delivery.blockedDeliveryBias = 0f;
            weights.delivery.agingWeight = 0f;
            weights.delivery.maxAgingBonus = 0f;
            weights.removal.priorAssignmentEta = 0f;
            return weights;
        }

        private static AmrFutureAvailability FutureAvailability(
            TaskPlanningAmr amr,
            MapfNode finishNode,
            ITaskPlanningTask activeTask,
            double priorAssignmentEta)
        {
            return new AmrFutureAvailability(amr, activeTask, finishNode, priorAssignmentEta, isInterruptible: false);
        }

        private static SingleTaskFixture CreateSingleTaskFixture(string prefix, float futureDistanceToPallet)
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph(prefix + "_Graph");
            var palletNode = TaskPlanningTestHelpers.CreateNode(prefix + "_Pallet", new Vector2(0f, 0f));
            var futureNode = TaskPlanningTestHelpers.CreateNode(prefix + "_Future", new Vector2(futureDistanceToPallet, 0f));
            var freeNode = TaskPlanningTestHelpers.CreateNode(prefix + "_Free", new Vector2(100f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeA", palletNode, futureNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeB", futureNode, freeNode);
            var freeAmr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_FreeAmr");
            var busyAmr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_BusyAmr");
            var pallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_PalletObject", palletNode);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint(prefix + "_Loading", palletNode, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_Workstation", palletNode, pallet);
            freeAmr.transform.position = freeNode.transform.position;
            busyAmr.transform.position = futureNode.transform.position;

            return new SingleTaskFixture(
                graph,
                palletNode,
                futureNode,
                freeNode,
                edgeA,
                edgeB,
                freeAmr,
                busyAmr,
                pallet,
                loadingPoint,
                workstation);
        }

        private static SharedLoadingFixture CreateSharedLoadingFixture(string prefix)
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph(prefix + "_Graph");
            var sharedNode = TaskPlanningTestHelpers.CreateNode(prefix + "_Shared", new Vector2(0f, 0f));
            var freeNode = TaskPlanningTestHelpers.CreateNode(prefix + "_Free", new Vector2(15f, 0f));
            var edge = TaskPlanningTestHelpers.CreateEdge(prefix + "_Edge", sharedNode, freeNode);
            var freeAmr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_FreeAmr");
            var futureAmrA = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_FutureAmrA");
            var futureAmrB = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_FutureAmrB");
            var queuePallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_QueuePallet", sharedNode);
            var targetPallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_TargetPallet", sharedNode);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint(prefix + "_Loading", sharedNode, queuePallet, targetPallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_Workstation", sharedNode, queuePallet, targetPallet);
            TaskPlanningTestHelpers.SetPalletDurations(queuePallet, 0f, 0f, 20f, 0f);
            TaskPlanningTestHelpers.SetPalletDurations(targetPallet, 0f, 0f, 0f, 0f);
            freeAmr.transform.position = freeNode.transform.position;
            futureAmrA.transform.position = sharedNode.transform.position;
            futureAmrB.transform.position = sharedNode.transform.position;

            return new SharedLoadingFixture(
                graph,
                sharedNode,
                freeNode,
                edge,
                freeAmr,
                futureAmrA,
                futureAmrB,
                queuePallet,
                targetPallet,
                loadingPoint,
                workstation);
        }

        private sealed class SingleTaskFixture
        {
            public SingleTaskFixture(
                MapfSceneGraph graph,
                MapfNode palletNode,
                MapfNode futureNode,
                MapfNode freeNode,
                MapfEdge edgeA,
                MapfEdge edgeB,
                TaskPlanningAmr freeAmr,
                TaskPlanningAmr busyAmr,
                PalletMarker pallet,
                PalletLoadingPoint loadingPoint,
                WorkstationDeliveryPoint workstation)
            {
                Graph = graph;
                PalletNode = palletNode;
                FutureNode = futureNode;
                FreeNode = freeNode;
                EdgeA = edgeA;
                EdgeB = edgeB;
                FreeAmr = freeAmr;
                BusyAmr = busyAmr;
                Pallet = pallet;
                LoadingPoint = loadingPoint;
                Workstation = workstation;
            }

            public MapfSceneGraph Graph { get; }
            public MapfNode PalletNode { get; }
            public MapfNode FutureNode { get; }
            public MapfNode FreeNode { get; }
            public MapfEdge EdgeA { get; }
            public MapfEdge EdgeB { get; }
            public TaskPlanningAmr FreeAmr { get; }
            public TaskPlanningAmr BusyAmr { get; }
            public PalletMarker Pallet { get; }
            public PalletLoadingPoint LoadingPoint { get; }
            public WorkstationDeliveryPoint Workstation { get; }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(
                    Workstation.gameObject,
                    LoadingPoint.gameObject,
                    Pallet.gameObject,
                    BusyAmr.gameObject,
                    FreeAmr.gameObject,
                    EdgeB.gameObject,
                    EdgeA.gameObject,
                    FreeNode.gameObject,
                    FutureNode.gameObject,
                    PalletNode.gameObject,
                    Graph.gameObject);
            }
        }

        private sealed class SharedLoadingFixture
        {
            public SharedLoadingFixture(
                MapfSceneGraph graph,
                MapfNode sharedNode,
                MapfNode freeNode,
                MapfEdge edge,
                TaskPlanningAmr freeAmr,
                TaskPlanningAmr futureAmrA,
                TaskPlanningAmr futureAmrB,
                PalletMarker queuePallet,
                PalletMarker targetPallet,
                PalletLoadingPoint loadingPoint,
                WorkstationDeliveryPoint workstation)
            {
                Graph = graph;
                SharedNode = sharedNode;
                FreeNode = freeNode;
                Edge = edge;
                FreeAmr = freeAmr;
                FutureAmrA = futureAmrA;
                FutureAmrB = futureAmrB;
                QueuePallet = queuePallet;
                TargetPallet = targetPallet;
                LoadingPoint = loadingPoint;
                Workstation = workstation;
            }

            public MapfSceneGraph Graph { get; }
            public MapfNode SharedNode { get; }
            public MapfNode FreeNode { get; }
            public MapfEdge Edge { get; }
            public TaskPlanningAmr FreeAmr { get; }
            public TaskPlanningAmr FutureAmrA { get; }
            public TaskPlanningAmr FutureAmrB { get; }
            public PalletMarker QueuePallet { get; }
            public PalletMarker TargetPallet { get; }
            public PalletLoadingPoint LoadingPoint { get; }
            public WorkstationDeliveryPoint Workstation { get; }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(
                    Workstation.gameObject,
                    LoadingPoint.gameObject,
                    TargetPallet.gameObject,
                    QueuePallet.gameObject,
                    FutureAmrB.gameObject,
                    FutureAmrA.gameObject,
                    FreeAmr.gameObject,
                    Edge.gameObject,
                    FreeNode.gameObject,
                    SharedNode.gameObject,
                    Graph.gameObject);
            }
        }
    }
}
