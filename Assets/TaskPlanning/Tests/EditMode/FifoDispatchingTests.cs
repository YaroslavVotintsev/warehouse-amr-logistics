using System.Linq;
using Mapf.Authoring;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class FifoDispatchingTests
    {
        [Test]
        public void FifoDispatchingAssignsOldestFeasibleTaskBeforeCheaperNewerTask()
        {
            var fixture = CreateTwoDeliveryFixture("FifoOldestFirst", olderPalletDistance: 10f, newerPalletDistance: 1f);

            try
            {
                var older = new DeliveryPlanningTask("D-Older", fixture.PalletA, fixture.WorkstationA, 1f);
                var newer = new DeliveryPlanningTask("D-Newer", fixture.PalletB, fixture.WorkstationB, 2f);
                var plan = Solve(
                    fixture.Graph,
                    new ITaskPlanningTask[] { newer, older },
                    new[] { fixture.AmrA },
                    new[] { fixture.LoadingA, fixture.LoadingB });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(older));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void FifoDispatchingChoosesCheapestFeasibleAmrForCurrentTask()
        {
            var fixture = CreateTwoDeliveryFixture("FifoCheapestAmr", olderPalletDistance: 5f, newerPalletDistance: 10f);

            try
            {
                fixture.AmrA.transform.position = new Vector3(0f, 0f, 0f);
                fixture.AmrB.transform.position = new Vector3(4f, 0f, 0f);
                var task = new DeliveryPlanningTask("D-Only", fixture.PalletA, fixture.WorkstationA, 0f);
                var plan = Solve(
                    fixture.Graph,
                    new ITaskPlanningTask[] { task },
                    new[] { fixture.AmrA, fixture.AmrB },
                    new[] { fixture.LoadingA });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Amr, Is.SameAs(fixture.AmrB));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void FifoDispatchingSkipsInfeasibleOlderTaskAndContinuesToNewerTask()
        {
            var fixture = CreateTwoDeliveryFixture("FifoSkipInfeasible", olderPalletDistance: 1f, newerPalletDistance: 2f);

            try
            {
                var older = new DeliveryPlanningTask("D-OlderInvalid", fixture.PalletA, fixture.WorkstationA, 1f);
                var newer = new DeliveryPlanningTask("D-NewerValid", fixture.PalletB, fixture.WorkstationB, 2f);
                var plan = Solve(
                    fixture.Graph,
                    new ITaskPlanningTask[] { older, newer },
                    new[] { fixture.AmrA },
                    new[] { fixture.LoadingB });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(newer));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void FifoDispatchingUsesResolvedSingleLoadingPointForDeliveryTask()
        {
            var fixture = CreateTwoDeliveryFixture("FifoResolvedLoading", olderPalletDistance: 1f, newerPalletDistance: 2f);

            try
            {
                var task = new DeliveryPlanningTask("D-Resolved", fixture.PalletA, fixture.WorkstationA, 0f);
                var plan = Solve(
                    fixture.Graph,
                    new ITaskPlanningTask[] { task },
                    new[] { fixture.AmrA },
                    new[] { fixture.LoadingB, fixture.LoadingA });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].LoadingPoint, Is.SameAs(fixture.LoadingA));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void FifoDispatchingDoesNotAssignSameAmrMoreThanOnce()
        {
            var fixture = CreateTwoDeliveryFixture("FifoOneAssignmentPerAmr", olderPalletDistance: 1f, newerPalletDistance: 2f);

            try
            {
                var older = new DeliveryPlanningTask("D-Older", fixture.PalletA, fixture.WorkstationA, 1f);
                var newer = new DeliveryPlanningTask("D-Newer", fixture.PalletB, fixture.WorkstationB, 2f);
                var plan = Solve(
                    fixture.Graph,
                    new ITaskPlanningTask[] { older, newer },
                    new[] { fixture.AmrA },
                    new[] { fixture.LoadingA, fixture.LoadingB });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments.Select(assignment => assignment.Amr).Distinct().Count(), Is.EqualTo(plan.Assignments.Count));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void FifoDispatchingDoesNotAssignSamePalletMoreThanOnce()
        {
            var fixture = CreateTwoDeliveryFixture("FifoOneAssignmentPerPallet", olderPalletDistance: 1f, newerPalletDistance: 2f);

            try
            {
                var older = new DeliveryPlanningTask("D-Older", fixture.PalletA, fixture.WorkstationA, 1f);
                var newer = new DeliveryPlanningTask("D-Newer", fixture.PalletA, fixture.WorkstationB, 2f);
                var plan = Solve(
                    fixture.Graph,
                    new ITaskPlanningTask[] { older, newer },
                    new[] { fixture.AmrA, fixture.AmrB },
                    new[] { fixture.LoadingA });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(older));
                Assert.That(plan.Assignments.Select(assignment => assignment.Pallet).Distinct().Count(), Is.EqualTo(plan.Assignments.Count));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void FifoDispatchingTreatsDeliveryAsInfeasibleWhenPalletHasNoValidSingleLoadingPoint()
        {
            var fixture = CreateTwoDeliveryFixture("FifoInvalidLoading", olderPalletDistance: 1f, newerPalletDistance: 2f);
            var duplicateLoading = TaskPlanningTestHelpers.CreateLoadingPoint("FifoInvalidLoading_DuplicateLoading", fixture.LoadNodeB, fixture.PalletA);

            try
            {
                var task = new DeliveryPlanningTask("D-InvalidLoading", fixture.PalletA, fixture.WorkstationA, 0f);
                var plan = Solve(
                    fixture.Graph,
                    new ITaskPlanningTask[] { task },
                    new[] { fixture.AmrA },
                    new[] { fixture.LoadingA, duplicateLoading });

                Assert.That(plan.Assignments, Is.Empty);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(duplicateLoading.gameObject);
                fixture.Destroy();
            }
        }

        private static DispatchPlan Solve(
            MapfSceneGraph graph,
            ITaskPlanningTask[] tasks,
            TaskPlanningAmr[] amrs,
            PalletLoadingPoint[] loadingPoints)
        {
            var distances = new RoadmapDistanceService(graph);
            var evaluator = new TaskPlanningCostEvaluator(distances, new TaskPlanningCostWeights(), 1f, tasks, 0f);
            var problem = new DispatchProblem(tasks, amrs, loadingPoints, distances, evaluator, 0f);
            return new FifoDispatching().Solve(problem);
        }

        private static TwoDeliveryFixture CreateTwoDeliveryFixture(
            string prefix,
            float olderPalletDistance,
            float newerPalletDistance)
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph(prefix + "_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode(prefix + "_Amr", new Vector2(0f, 0f));
            var amrBNode = TaskPlanningTestHelpers.CreateNode(prefix + "_AmrB", new Vector2(4f, 0f));
            var palletNodeA = TaskPlanningTestHelpers.CreateNode(prefix + "_PalletA", new Vector2(olderPalletDistance, 0f));
            var loadNodeA = TaskPlanningTestHelpers.CreateNode(prefix + "_LoadA", new Vector2(olderPalletDistance + 1f, 0f));
            var workNodeA = TaskPlanningTestHelpers.CreateNode(prefix + "_WorkA", new Vector2(olderPalletDistance + 2f, 0f));
            var palletNodeB = TaskPlanningTestHelpers.CreateNode(prefix + "_PalletB", new Vector2(newerPalletDistance, 0f));
            var loadNodeB = TaskPlanningTestHelpers.CreateNode(prefix + "_LoadB", new Vector2(newerPalletDistance + 1f, 0f));
            var workNodeB = TaskPlanningTestHelpers.CreateNode(prefix + "_WorkB", new Vector2(newerPalletDistance + 2f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeA", amrNode, amrBNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeB", amrNode, palletNodeA);
            var edgeC = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeC", palletNodeA, loadNodeA);
            var edgeD = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeD", loadNodeA, workNodeA);
            var edgeE = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeE", amrNode, palletNodeB);
            var edgeF = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeF", palletNodeB, loadNodeB);
            var edgeG = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeG", loadNodeB, workNodeB);
            var edgeH = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeH", amrBNode, palletNodeA);
            var amrA = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_AmrAObject");
            var amrB = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_AmrBObject");
            var palletA = TaskPlanningTestHelpers.CreatePallet(prefix + "_PalletAObject", palletNodeA);
            var palletB = TaskPlanningTestHelpers.CreatePallet(prefix + "_PalletBObject", palletNodeB);
            var loadingA = TaskPlanningTestHelpers.CreateLoadingPoint(prefix + "_LoadingA", loadNodeA, palletA);
            var loadingB = TaskPlanningTestHelpers.CreateLoadingPoint(prefix + "_LoadingB", loadNodeB, palletB);
            var workstationA = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_WorkstationA", workNodeA, palletA, palletB);
            var workstationB = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_WorkstationB", workNodeB, palletA, palletB);
            amrA.transform.position = amrNode.transform.position;
            amrB.transform.position = amrBNode.transform.position;

            return new TwoDeliveryFixture(
                graph,
                amrNode,
                amrBNode,
                palletNodeA,
                loadNodeA,
                workNodeA,
                palletNodeB,
                loadNodeB,
                workNodeB,
                edgeA,
                edgeB,
                edgeC,
                edgeD,
                edgeE,
                edgeF,
                edgeG,
                edgeH,
                amrA,
                amrB,
                palletA,
                palletB,
                loadingA,
                loadingB,
                workstationA,
                workstationB);
        }

        private sealed class TwoDeliveryFixture
        {
            public TwoDeliveryFixture(
                MapfSceneGraph graph,
                MapfNode amrNode,
                MapfNode amrBNode,
                MapfNode palletNodeA,
                MapfNode loadNodeA,
                MapfNode workNodeA,
                MapfNode palletNodeB,
                MapfNode loadNodeB,
                MapfNode workNodeB,
                MapfEdge edgeA,
                MapfEdge edgeB,
                MapfEdge edgeC,
                MapfEdge edgeD,
                MapfEdge edgeE,
                MapfEdge edgeF,
                MapfEdge edgeG,
                MapfEdge edgeH,
                TaskPlanningAmr amrA,
                TaskPlanningAmr amrB,
                PalletMarker palletA,
                PalletMarker palletB,
                PalletLoadingPoint loadingA,
                PalletLoadingPoint loadingB,
                WorkstationDeliveryPoint workstationA,
                WorkstationDeliveryPoint workstationB)
            {
                Graph = graph;
                AmrNode = amrNode;
                AmrBNode = amrBNode;
                PalletNodeA = palletNodeA;
                LoadNodeA = loadNodeA;
                WorkNodeA = workNodeA;
                PalletNodeB = palletNodeB;
                LoadNodeB = loadNodeB;
                WorkNodeB = workNodeB;
                EdgeA = edgeA;
                EdgeB = edgeB;
                EdgeC = edgeC;
                EdgeD = edgeD;
                EdgeE = edgeE;
                EdgeF = edgeF;
                EdgeG = edgeG;
                EdgeH = edgeH;
                AmrA = amrA;
                AmrB = amrB;
                PalletA = palletA;
                PalletB = palletB;
                LoadingA = loadingA;
                LoadingB = loadingB;
                WorkstationA = workstationA;
                WorkstationB = workstationB;
            }

            public MapfSceneGraph Graph { get; }
            public MapfNode AmrNode { get; }
            public MapfNode AmrBNode { get; }
            public MapfNode PalletNodeA { get; }
            public MapfNode LoadNodeA { get; }
            public MapfNode WorkNodeA { get; }
            public MapfNode PalletNodeB { get; }
            public MapfNode LoadNodeB { get; }
            public MapfNode WorkNodeB { get; }
            public MapfEdge EdgeA { get; }
            public MapfEdge EdgeB { get; }
            public MapfEdge EdgeC { get; }
            public MapfEdge EdgeD { get; }
            public MapfEdge EdgeE { get; }
            public MapfEdge EdgeF { get; }
            public MapfEdge EdgeG { get; }
            public MapfEdge EdgeH { get; }
            public TaskPlanningAmr AmrA { get; }
            public TaskPlanningAmr AmrB { get; }
            public PalletMarker PalletA { get; }
            public PalletMarker PalletB { get; }
            public PalletLoadingPoint LoadingA { get; }
            public PalletLoadingPoint LoadingB { get; }
            public WorkstationDeliveryPoint WorkstationA { get; }
            public WorkstationDeliveryPoint WorkstationB { get; }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(
                    WorkstationB.gameObject,
                    WorkstationA.gameObject,
                    LoadingB.gameObject,
                    LoadingA.gameObject,
                    PalletB.gameObject,
                    PalletA.gameObject,
                    AmrB.gameObject,
                    AmrA.gameObject,
                    EdgeH.gameObject,
                    EdgeG.gameObject,
                    EdgeF.gameObject,
                    EdgeE.gameObject,
                    EdgeD.gameObject,
                    EdgeC.gameObject,
                    EdgeB.gameObject,
                    EdgeA.gameObject,
                    WorkNodeB.gameObject,
                    LoadNodeB.gameObject,
                    PalletNodeB.gameObject,
                    WorkNodeA.gameObject,
                    LoadNodeA.gameObject,
                    PalletNodeA.gameObject,
                    AmrBNode.gameObject,
                    AmrNode.gameObject,
                    Graph.gameObject);
            }
        }
    }
}
