using System.Linq;
using Mapf.Authoring;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class NearestDispatchingTests
    {
        [Test]
        public void NearestDispatchingConsidersAllPendingTasksTogether()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("DispatchBatch_Graph");
            var amrNodeA = TaskPlanningTestHelpers.CreateNode("DispatchBatch_AmrA", new Vector2(0f, 0f));
            var palletNodeA = TaskPlanningTestHelpers.CreateNode("DispatchBatch_PalletA", new Vector2(1f, 0f));
            var loadNodeA = TaskPlanningTestHelpers.CreateNode("DispatchBatch_LoadA", new Vector2(2f, 0f));
            var workNodeA = TaskPlanningTestHelpers.CreateNode("DispatchBatch_WorkA", new Vector2(3f, 0f));
            var amrNodeB = TaskPlanningTestHelpers.CreateNode("DispatchBatch_AmrB", new Vector2(10f, 0f));
            var palletNodeB = TaskPlanningTestHelpers.CreateNode("DispatchBatch_PalletB", new Vector2(11f, 0f));
            var loadNodeB = TaskPlanningTestHelpers.CreateNode("DispatchBatch_LoadB", new Vector2(12f, 0f));
            var workNodeB = TaskPlanningTestHelpers.CreateNode("DispatchBatch_WorkB", new Vector2(13f, 0f));
            var edgeA1 = TaskPlanningTestHelpers.CreateEdge("DispatchBatch_EdgeA1", amrNodeA, palletNodeA);
            var edgeA2 = TaskPlanningTestHelpers.CreateEdge("DispatchBatch_EdgeA2", palletNodeA, loadNodeA);
            var edgeA3 = TaskPlanningTestHelpers.CreateEdge("DispatchBatch_EdgeA3", loadNodeA, workNodeA);
            var edgeB1 = TaskPlanningTestHelpers.CreateEdge("DispatchBatch_EdgeB1", amrNodeB, palletNodeB);
            var edgeB2 = TaskPlanningTestHelpers.CreateEdge("DispatchBatch_EdgeB2", palletNodeB, loadNodeB);
            var edgeB3 = TaskPlanningTestHelpers.CreateEdge("DispatchBatch_EdgeB3", loadNodeB, workNodeB);
            var amrA = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("DispatchBatch_AmrAObject");
            var amrB = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("DispatchBatch_AmrBObject");
            var palletA = TaskPlanningTestHelpers.CreatePallet("DispatchBatch_PalletAObject", palletNodeA);
            var palletB = TaskPlanningTestHelpers.CreatePallet("DispatchBatch_PalletBObject", palletNodeB);
            var loadingA = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchBatch_LoadingA", loadNodeA, palletA);
            var loadingB = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchBatch_LoadingB", loadNodeB, palletB);
            var workstationA = TaskPlanningTestHelpers.CreateWorkstation("DispatchBatch_WorkstationA", workNodeA, palletA);
            var workstationB = TaskPlanningTestHelpers.CreateWorkstation("DispatchBatch_WorkstationB", workNodeB, palletB);

            try
            {
                amrA.transform.position = amrNodeA.transform.position;
                amrB.transform.position = amrNodeB.transform.position;
                var tasks = new ITaskPlanningTask[]
                {
                    new DeliveryPlanningTask("D-A", palletA, workstationA, 0f),
                    new DeliveryPlanningTask("D-B", palletB, workstationB, 0f)
                };
                var plan = Solve(graph, tasks, new[] { amrA, amrB }, new[] { loadingA, loadingB });

                Assert.That(plan.Assignments, Has.Count.EqualTo(2));
                Assert.That(plan.Assignments.Select(a => a.Amr).Distinct().Count(), Is.EqualTo(2));
                Assert.That(plan.Assignments.Select(a => a.Task).Distinct().Count(), Is.EqualTo(2));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstationB.gameObject,
                    workstationA.gameObject,
                    loadingB.gameObject,
                    loadingA.gameObject,
                    palletB.gameObject,
                    palletA.gameObject,
                    amrB.gameObject,
                    amrA.gameObject,
                    edgeB3.gameObject,
                    edgeB2.gameObject,
                    edgeB1.gameObject,
                    edgeA3.gameObject,
                    edgeA2.gameObject,
                    edgeA1.gameObject,
                    workNodeB.gameObject,
                    loadNodeB.gameObject,
                    palletNodeB.gameObject,
                    amrNodeB.gameObject,
                    workNodeA.gameObject,
                    loadNodeA.gameObject,
                    palletNodeA.gameObject,
                    amrNodeA.gameObject,
                    graph.gameObject);
            }
        }

        [Test]
        public void NearestDispatchingUsesOnlyResolvedLoadingPointForPallet()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("DispatchResolvedLoading_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("DispatchResolvedLoading_Amr", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode("DispatchResolvedLoading_Pallet", new Vector2(1f, 0f));
            var decoyLoadNode = TaskPlanningTestHelpers.CreateNode("DispatchResolvedLoading_DecoyLoad", new Vector2(2f, 0f));
            var resolvedLoadNode = TaskPlanningTestHelpers.CreateNode("DispatchResolvedLoading_Load", new Vector2(5f, 0f));
            var workNode = TaskPlanningTestHelpers.CreateNode("DispatchResolvedLoading_Work", new Vector2(6f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("DispatchResolvedLoading_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("DispatchResolvedLoading_EdgeB", palletNode, resolvedLoadNode);
            var edgeC = TaskPlanningTestHelpers.CreateEdge("DispatchResolvedLoading_EdgeC", resolvedLoadNode, workNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("DispatchResolvedLoading_AmrObject");
            var pallet = TaskPlanningTestHelpers.CreatePallet("DispatchResolvedLoading_PalletObject", palletNode);
            var decoyPallet = TaskPlanningTestHelpers.CreatePallet("DispatchResolvedLoading_DecoyPallet");
            var decoyLoading = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchResolvedLoading_Decoy", decoyLoadNode, decoyPallet);
            var resolvedLoading = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchResolvedLoading_Resolved", resolvedLoadNode, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("DispatchResolvedLoading_Workstation", workNode, pallet);

            try
            {
                amr.transform.position = amrNode.transform.position;
                var task = new DeliveryPlanningTask("D-Resolved", pallet, workstation, 0f);
                var plan = Solve(graph, new ITaskPlanningTask[] { task }, new[] { amr }, new[] { decoyLoading, resolvedLoading });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].LoadingPoint, Is.SameAs(resolvedLoading));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    resolvedLoading.gameObject,
                    decoyLoading.gameObject,
                    decoyPallet.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    edgeC.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    workNode.gameObject,
                    resolvedLoadNode.gameObject,
                    decoyLoadNode.gameObject,
                    palletNode.gameObject,
                    amrNode.gameObject,
                    graph.gameObject);
            }
        }

        [Test]
        public void NearestDispatchingRejectsDeliveryWhenMultipleLoadingPointsAcceptPallet()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("DispatchAmbiguousLoading_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("DispatchAmbiguousLoading_Amr", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode("DispatchAmbiguousLoading_Pallet", new Vector2(1f, 0f));
            var loadNodeA = TaskPlanningTestHelpers.CreateNode("DispatchAmbiguousLoading_LoadA", new Vector2(2f, 0f));
            var loadNodeB = TaskPlanningTestHelpers.CreateNode("DispatchAmbiguousLoading_LoadB", new Vector2(2f, 1f));
            var workNode = TaskPlanningTestHelpers.CreateNode("DispatchAmbiguousLoading_Work", new Vector2(3f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("DispatchAmbiguousLoading_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("DispatchAmbiguousLoading_EdgeB", palletNode, loadNodeA);
            var edgeC = TaskPlanningTestHelpers.CreateEdge("DispatchAmbiguousLoading_EdgeC", loadNodeA, workNode);
            var edgeD = TaskPlanningTestHelpers.CreateEdge("DispatchAmbiguousLoading_EdgeD", palletNode, loadNodeB);
            var edgeE = TaskPlanningTestHelpers.CreateEdge("DispatchAmbiguousLoading_EdgeE", loadNodeB, workNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("DispatchAmbiguousLoading_AmrObject");
            var pallet = TaskPlanningTestHelpers.CreatePallet("DispatchAmbiguousLoading_PalletObject", palletNode);
            var loadingA = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchAmbiguousLoading_LoadingA", loadNodeA, pallet);
            var loadingB = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchAmbiguousLoading_LoadingB", loadNodeB, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("DispatchAmbiguousLoading_Workstation", workNode, pallet);

            try
            {
                amr.transform.position = amrNode.transform.position;
                var task = new DeliveryPlanningTask("D-Ambiguous", pallet, workstation, 0f);
                var plan = Solve(graph, new ITaskPlanningTask[] { task }, new[] { amr }, new[] { loadingA, loadingB });

                Assert.That(plan.Assignments, Is.Empty);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    loadingB.gameObject,
                    loadingA.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    edgeE.gameObject,
                    edgeD.gameObject,
                    edgeC.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    workNode.gameObject,
                    loadNodeB.gameObject,
                    loadNodeA.gameObject,
                    palletNode.gameObject,
                    amrNode.gameObject,
                    graph.gameObject);
            }
        }

        [Test]
        public void NearestDispatchingRejectsDeliveryWhenNoLoadingPointAcceptsPallet()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("DispatchMissingLoading_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("DispatchMissingLoading_Amr", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode("DispatchMissingLoading_Pallet", new Vector2(1f, 0f));
            var loadNode = TaskPlanningTestHelpers.CreateNode("DispatchMissingLoading_Load", new Vector2(2f, 0f));
            var workNode = TaskPlanningTestHelpers.CreateNode("DispatchMissingLoading_Work", new Vector2(3f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("DispatchMissingLoading_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("DispatchMissingLoading_EdgeB", palletNode, loadNode);
            var edgeC = TaskPlanningTestHelpers.CreateEdge("DispatchMissingLoading_EdgeC", loadNode, workNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("DispatchMissingLoading_AmrObject");
            var pallet = TaskPlanningTestHelpers.CreatePallet("DispatchMissingLoading_PalletObject", palletNode);
            var otherPallet = TaskPlanningTestHelpers.CreatePallet("DispatchMissingLoading_OtherPallet");
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchMissingLoading_Loading", loadNode, otherPallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("DispatchMissingLoading_Workstation", workNode, pallet);

            try
            {
                amr.transform.position = amrNode.transform.position;
                var task = new DeliveryPlanningTask("D-MissingLoading", pallet, workstation, 0f);
                var plan = Solve(graph, new ITaskPlanningTask[] { task }, new[] { amr }, new[] { loadingPoint });

                Assert.That(plan.Assignments, Is.Empty);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    loadingPoint.gameObject,
                    otherPallet.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    edgeC.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    workNode.gameObject,
                    loadNode.gameObject,
                    palletNode.gameObject,
                    amrNode.gameObject,
                    graph.gameObject);
            }
        }

        [Test]
        public void NearestDispatchingRejectsDeliveryWhenResolvedLoadingPointHasNoNode()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("DispatchLoadingNoNode_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("DispatchLoadingNoNode_Amr", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode("DispatchLoadingNoNode_Pallet", new Vector2(1f, 0f));
            var workNode = TaskPlanningTestHelpers.CreateNode("DispatchLoadingNoNode_Work", new Vector2(2f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("DispatchLoadingNoNode_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("DispatchLoadingNoNode_EdgeB", palletNode, workNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("DispatchLoadingNoNode_AmrObject");
            var pallet = TaskPlanningTestHelpers.CreatePallet("DispatchLoadingNoNode_PalletObject", palletNode);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchLoadingNoNode_Loading", null, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("DispatchLoadingNoNode_Workstation", workNode, pallet);

            try
            {
                amr.transform.position = amrNode.transform.position;
                var task = new DeliveryPlanningTask("D-LoadingNoNode", pallet, workstation, 0f);
                var plan = Solve(graph, new ITaskPlanningTask[] { task }, new[] { amr }, new[] { loadingPoint });

                Assert.That(plan.Assignments, Is.Empty);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    loadingPoint.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    workNode.gameObject,
                    palletNode.gameObject,
                    amrNode.gameObject,
                    graph.gameObject);
            }
        }

        [Test]
        public void NearestDispatchingAssignsEachAmrAtMostOnce()
        {
            var fixture = CreateTwoTaskSingleLineFixture("DispatchOneAmr");

            try
            {
                var tasks = new ITaskPlanningTask[]
                {
                    new DeliveryPlanningTask("D-1", fixture.PalletA, fixture.WorkstationA, 0f),
                    new DeliveryPlanningTask("D-2", fixture.PalletB, fixture.WorkstationB, 0f)
                };
                var plan = Solve(fixture.Graph, tasks, new[] { fixture.AmrA }, new[] { fixture.LoadingPoint });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments.Select(a => a.Amr).Distinct().Count(), Is.EqualTo(plan.Assignments.Count));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void NearestDispatchingAssignsEachPalletAtMostOnce()
        {
            var fixture = CreateTwoTaskSingleLineFixture("DispatchOnePallet");

            try
            {
                var tasks = new ITaskPlanningTask[]
                {
                    new DeliveryPlanningTask("D-1", fixture.PalletA, fixture.WorkstationA, 0f),
                    new DeliveryPlanningTask("D-2", fixture.PalletA, fixture.WorkstationB, 0f)
                };
                var plan = Solve(fixture.Graph, tasks, new[] { fixture.AmrA, fixture.AmrB }, new[] { fixture.LoadingPoint });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments.Select(a => a.Pallet).Distinct().Count(), Is.EqualTo(plan.Assignments.Count));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void NearestDispatchingCanChooseRemovalOverDeliveryByCost()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("DispatchRemovalWins_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("DispatchRemovalWins_Amr", new Vector2(0f, 0f));
            var removalParkingNode = TaskPlanningTestHelpers.CreateNode("DispatchRemovalWins_Parking", new Vector2(1f, 0f));
            var deliveryPalletNode = TaskPlanningTestHelpers.CreateNode("DispatchRemovalWins_DeliveryPallet", new Vector2(100f, 0f));
            var deliveryLoadNode = TaskPlanningTestHelpers.CreateNode("DispatchRemovalWins_Load", new Vector2(101f, 0f));
            var deliveryWorkNode = TaskPlanningTestHelpers.CreateNode("DispatchRemovalWins_Work", new Vector2(102f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("DispatchRemovalWins_EdgeA", amrNode, removalParkingNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("DispatchRemovalWins_EdgeB", removalParkingNode, deliveryPalletNode);
            var edgeC = TaskPlanningTestHelpers.CreateEdge("DispatchRemovalWins_EdgeC", deliveryPalletNode, deliveryLoadNode);
            var edgeD = TaskPlanningTestHelpers.CreateEdge("DispatchRemovalWins_EdgeD", deliveryLoadNode, deliveryWorkNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("DispatchRemovalWins_AmrObject");
            var removalPallet = TaskPlanningTestHelpers.CreatePallet("DispatchRemovalWins_RemovalPallet", amrNode);
            var deliveryPallet = TaskPlanningTestHelpers.CreatePallet("DispatchRemovalWins_DeliveryPalletObject", deliveryPalletNode);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("DispatchRemovalWins_LoadingPoint", deliveryLoadNode, deliveryPallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("DispatchRemovalWins_Workstation", deliveryWorkNode, deliveryPallet, removalPallet);

            try
            {
                amr.transform.position = amrNode.transform.position;
                TaskPlanningTestHelpers.SetParkingNode(removalPallet, removalParkingNode);
                removalPallet.MarkAwaitingRemoval();
                var delivery = new DeliveryPlanningTask("D-Far", deliveryPallet, workstation, 0f);
                var removal = new PalletRemovalPlanningTask("R-Near", removalPallet, workstation, 0f);
                var plan = Solve(graph, new ITaskPlanningTask[] { delivery, removal }, new[] { amr }, new[] { loadingPoint });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(removal));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    loadingPoint.gameObject,
                    deliveryPallet.gameObject,
                    removalPallet.gameObject,
                    amr.gameObject,
                    edgeD.gameObject,
                    edgeC.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    deliveryWorkNode.gameObject,
                    deliveryLoadNode.gameObject,
                    deliveryPalletNode.gameObject,
                    removalParkingNode.gameObject,
                    amrNode.gameObject,
                    graph.gameObject);
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
            return new NearestDispatching().Solve(problem);
        }

        private static TwoTaskFixture CreateTwoTaskSingleLineFixture(string prefix)
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph(prefix + "_Graph");
            var amrNodeA = TaskPlanningTestHelpers.CreateNode(prefix + "_AmrA", new Vector2(0f, 0f));
            var amrNodeB = TaskPlanningTestHelpers.CreateNode(prefix + "_AmrB", new Vector2(0f, 1f));
            var palletNodeA = TaskPlanningTestHelpers.CreateNode(prefix + "_PalletA", new Vector2(1f, 0f));
            var palletNodeB = TaskPlanningTestHelpers.CreateNode(prefix + "_PalletB", new Vector2(2f, 0f));
            var loadNode = TaskPlanningTestHelpers.CreateNode(prefix + "_Load", new Vector2(3f, 0f));
            var workNodeA = TaskPlanningTestHelpers.CreateNode(prefix + "_WorkA", new Vector2(4f, 0f));
            var workNodeB = TaskPlanningTestHelpers.CreateNode(prefix + "_WorkB", new Vector2(5f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeA", amrNodeA, palletNodeA);
            var edgeB = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeB", amrNodeA, amrNodeB);
            var edgeC = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeC", palletNodeA, palletNodeB);
            var edgeD = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeD", palletNodeB, loadNode);
            var edgeE = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeE", loadNode, workNodeA);
            var edgeF = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeF", workNodeA, workNodeB);
            var amrA = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_AmrAObject");
            var amrB = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_AmrBObject");
            var palletA = TaskPlanningTestHelpers.CreatePallet(prefix + "_PalletAObject", palletNodeA);
            var palletB = TaskPlanningTestHelpers.CreatePallet(prefix + "_PalletBObject", palletNodeB);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint(prefix + "_LoadingPoint", loadNode, palletA, palletB);
            var workstationA = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_WorkstationA", workNodeA, palletA, palletB);
            var workstationB = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_WorkstationB", workNodeB, palletA, palletB);
            amrA.transform.position = amrNodeA.transform.position;
            amrB.transform.position = amrNodeB.transform.position;
            return new TwoTaskFixture(
                graph,
                amrNodeA,
                amrNodeB,
                palletNodeA,
                palletNodeB,
                loadNode,
                workNodeA,
                workNodeB,
                edgeA,
                edgeB,
                edgeC,
                edgeD,
                edgeE,
                edgeF,
                amrA,
                amrB,
                palletA,
                palletB,
                loadingPoint,
                workstationA,
                workstationB);
        }

        private sealed class TwoTaskFixture
        {
            public TwoTaskFixture(
                MapfSceneGraph graph,
                MapfNode amrNodeA,
                MapfNode amrNodeB,
                MapfNode palletNodeA,
                MapfNode palletNodeB,
                MapfNode loadNode,
                MapfNode workNodeA,
                MapfNode workNodeB,
                MapfEdge edgeA,
                MapfEdge edgeB,
                MapfEdge edgeC,
                MapfEdge edgeD,
                MapfEdge edgeE,
                MapfEdge edgeF,
                TaskPlanningAmr amrA,
                TaskPlanningAmr amrB,
                PalletMarker palletA,
                PalletMarker palletB,
                PalletLoadingPoint loadingPoint,
                WorkstationDeliveryPoint workstationA,
                WorkstationDeliveryPoint workstationB)
            {
                Graph = graph;
                AmrNodeA = amrNodeA;
                AmrNodeB = amrNodeB;
                PalletNodeA = palletNodeA;
                PalletNodeB = palletNodeB;
                LoadNode = loadNode;
                WorkNodeA = workNodeA;
                WorkNodeB = workNodeB;
                EdgeA = edgeA;
                EdgeB = edgeB;
                EdgeC = edgeC;
                EdgeD = edgeD;
                EdgeE = edgeE;
                EdgeF = edgeF;
                AmrA = amrA;
                AmrB = amrB;
                PalletA = palletA;
                PalletB = palletB;
                LoadingPoint = loadingPoint;
                WorkstationA = workstationA;
                WorkstationB = workstationB;
            }

            public MapfSceneGraph Graph { get; }
            public MapfNode AmrNodeA { get; }
            public MapfNode AmrNodeB { get; }
            public MapfNode PalletNodeA { get; }
            public MapfNode PalletNodeB { get; }
            public MapfNode LoadNode { get; }
            public MapfNode WorkNodeA { get; }
            public MapfNode WorkNodeB { get; }
            public MapfEdge EdgeA { get; }
            public MapfEdge EdgeB { get; }
            public MapfEdge EdgeC { get; }
            public MapfEdge EdgeD { get; }
            public MapfEdge EdgeE { get; }
            public MapfEdge EdgeF { get; }
            public TaskPlanningAmr AmrA { get; }
            public TaskPlanningAmr AmrB { get; }
            public PalletMarker PalletA { get; }
            public PalletMarker PalletB { get; }
            public PalletLoadingPoint LoadingPoint { get; }
            public WorkstationDeliveryPoint WorkstationA { get; }
            public WorkstationDeliveryPoint WorkstationB { get; }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(
                    WorkstationB.gameObject,
                    WorkstationA.gameObject,
                    LoadingPoint.gameObject,
                    PalletB.gameObject,
                    PalletA.gameObject,
                    AmrB.gameObject,
                    AmrA.gameObject,
                    EdgeF.gameObject,
                    EdgeE.gameObject,
                    EdgeD.gameObject,
                    EdgeC.gameObject,
                    EdgeB.gameObject,
                    EdgeA.gameObject,
                    WorkNodeB.gameObject,
                    WorkNodeA.gameObject,
                    LoadNode.gameObject,
                    PalletNodeB.gameObject,
                    PalletNodeA.gameObject,
                    AmrNodeB.gameObject,
                    AmrNodeA.gameObject,
                    Graph.gameObject);
            }
        }
    }
}
