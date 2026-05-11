using Mapf.Authoring;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class LookAheadNearestDispatchingTests
    {
        [Test]
        public void LookaheadSuppressesImmediateAssignmentWhenFutureCostIsSufficientlyBetter()
        {
            var fixture = CreateFixture("LookAheadSuppress");

            try
            {
                var task = new DeliveryPlanningTask("D-Suppress", fixture.Pallet, fixture.Workstation, 0f);
                var plan = Solve(fixture, task, waitForFutureImprovementPercent: 20f, now: 0f);

                Assert.That(plan.Assignments, Is.Empty);
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void LookaheadDoesNotSuppressImmediateAssignmentWhenFutureImprovementIsBelowThreshold()
        {
            var fixture = CreateFixture("LookAheadBelowThreshold");

            try
            {
                var task = new DeliveryPlanningTask("D-NoSuppress", fixture.Pallet, fixture.Workstation, 0f);
                var plan = Solve(fixture, task, waitForFutureImprovementPercent: 60f, now: 0f);

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
        public void LookaheadFallsBackToNearestWhenThereAreNoFutureCandidates()
        {
            var fixture = CreateFixture("LookAheadNoFuture");

            try
            {
                var task = new DeliveryPlanningTask("D-NoFuture", fixture.Pallet, fixture.Workstation, 0f);
                var plan = Solve(fixture, task, waitForFutureImprovementPercent: 20f, now: 0f, includeFuture: false);

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Amr, Is.SameAs(fixture.FreeAmr));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void AgingCanMakeOldTaskAssignableDespiteFutureWaitingBehavior()
        {
            var fixture = CreateFixture("LookAheadAging");

            try
            {
                var youngTask = new DeliveryPlanningTask("D-Young", fixture.Pallet, fixture.Workstation, 0f);
                var youngPlan = Solve(fixture, youngTask, waitForFutureImprovementPercent: 20f, now: 0f, agingWeight: 1f, maxAgingBonus: 400f);
                Assert.That(youngPlan.Assignments, Is.Empty);

                var oldTask = new DeliveryPlanningTask("D-Old", fixture.Pallet, fixture.Workstation, -400f);
                var oldPlan = Solve(fixture, oldTask, waitForFutureImprovementPercent: 20f, now: 0f, agingWeight: 1f, maxAgingBonus: 400f);
                Assert.That(oldPlan.Assignments, Has.Count.EqualTo(1));
                Assert.That(oldPlan.Assignments[0].Task, Is.SameAs(oldTask));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        private static DispatchPlan Solve(
            Fixture fixture,
            DeliveryPlanningTask task,
            float waitForFutureImprovementPercent,
            float now,
            bool includeFuture = true,
            float agingWeight = 0f,
            float maxAgingBonus = 0f)
        {
            var weights = new TaskPlanningCostWeights();
            weights.delivery.priorAssignmentEta = 0f;
            weights.delivery.amrToPalletEta = 1f;
            weights.delivery.attachTime = 0f;
            weights.delivery.loadingQueueEta = 0f;
            weights.delivery.palletToLoadingEta = 0f;
            weights.delivery.loadTime = 0f;
            weights.delivery.loadingToWorkstationEta = 0f;
            weights.delivery.detachTime = 0f;
            weights.delivery.blockedDeliveryBias = 0f;
            weights.delivery.agingWeight = agingWeight;
            weights.delivery.maxAgingBonus = maxAgingBonus;

            var tasks = new ITaskPlanningTask[] { task };
            var distances = new RoadmapDistanceService(fixture.Graph);
            var evaluator = new TaskPlanningCostEvaluator(distances, weights, 1f, tasks, now);
            var future = includeFuture
                ? new[]
                {
                    new AmrFutureAvailability(
                        fixture.BusyAmr,
                        fixture.ActiveTask,
                        fixture.FutureFinishNode,
                        priorAssignmentEta: 0.0,
                        isInterruptible: false)
                }
                : System.Array.Empty<AmrFutureAvailability>();
            var problem = new DispatchProblem(
                tasks,
                new[] { fixture.FreeAmr },
                new[] { fixture.LoadingPoint },
                distances,
                evaluator,
                now,
                future);

            return new LookAheadNearestDispatching(waitForFutureImprovementPercent).Solve(problem);
        }

        private static Fixture CreateFixture(string prefix)
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph(prefix + "_Graph");
            var palletNode = TaskPlanningTestHelpers.CreateNode(prefix + "_PalletNode", new Vector2(0f, 0f));
            var futureNode = TaskPlanningTestHelpers.CreateNode(prefix + "_FutureNode", new Vector2(50f, 0f));
            var freeNode = TaskPlanningTestHelpers.CreateNode(prefix + "_FreeNode", new Vector2(100f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeA", palletNode, futureNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeB", futureNode, freeNode);
            var freeAmr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_FreeAmr");
            var busyAmr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_BusyAmr");
            var pallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_Pallet", palletNode);
            var activePallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_ActivePallet", futureNode);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint(prefix + "_LoadingPoint", palletNode, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_Workstation", palletNode, pallet);
            freeAmr.transform.position = freeNode.transform.position;
            busyAmr.transform.position = futureNode.transform.position;
            busyAmr.TryReserve();
            var activeTask = new DeliveryPlanningTask(prefix + "_ActiveTask", activePallet, workstation, 0f);

            return new Fixture(
                graph,
                palletNode,
                futureNode,
                freeNode,
                edgeA,
                edgeB,
                freeAmr,
                busyAmr,
                pallet,
                activePallet,
                loadingPoint,
                workstation,
                activeTask);
        }

        private sealed class Fixture
        {
            public Fixture(
                MapfSceneGraph graph,
                MapfNode palletNode,
                MapfNode futureFinishNode,
                MapfNode freeNode,
                MapfEdge edgeA,
                MapfEdge edgeB,
                TaskPlanningAmr freeAmr,
                TaskPlanningAmr busyAmr,
                PalletMarker pallet,
                PalletMarker activePallet,
                PalletLoadingPoint loadingPoint,
                WorkstationDeliveryPoint workstation,
                ITaskPlanningTask activeTask)
            {
                Graph = graph;
                PalletNode = palletNode;
                FutureFinishNode = futureFinishNode;
                FreeNode = freeNode;
                EdgeA = edgeA;
                EdgeB = edgeB;
                FreeAmr = freeAmr;
                BusyAmr = busyAmr;
                Pallet = pallet;
                ActivePallet = activePallet;
                LoadingPoint = loadingPoint;
                Workstation = workstation;
                ActiveTask = activeTask;
            }

            public MapfSceneGraph Graph { get; }
            public MapfNode PalletNode { get; }
            public MapfNode FutureFinishNode { get; }
            public MapfNode FreeNode { get; }
            public MapfEdge EdgeA { get; }
            public MapfEdge EdgeB { get; }
            public TaskPlanningAmr FreeAmr { get; }
            public TaskPlanningAmr BusyAmr { get; }
            public PalletMarker Pallet { get; }
            public PalletMarker ActivePallet { get; }
            public PalletLoadingPoint LoadingPoint { get; }
            public WorkstationDeliveryPoint Workstation { get; }
            public ITaskPlanningTask ActiveTask { get; }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(
                    Workstation.gameObject,
                    LoadingPoint.gameObject,
                    ActivePallet.gameObject,
                    Pallet.gameObject,
                    BusyAmr.gameObject,
                    FreeAmr.gameObject,
                    EdgeB.gameObject,
                    EdgeA.gameObject,
                    FreeNode.gameObject,
                    FutureFinishNode.gameObject,
                    PalletNode.gameObject,
                    Graph.gameObject);
            }
        }
    }
}
