using Mapf.Authoring;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class TaskPlanningCostEvaluatorTests
    {
        [Test]
        public void DeliveryCostUsesDeliveryWeightsOnly()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("CostDelivery_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("CostDelivery_Amr", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode("CostDelivery_Pallet", new Vector2(1f, 0f));
            var loadingNode = TaskPlanningTestHelpers.CreateNode("CostDelivery_Load", new Vector2(3f, 0f));
            var workstationNode = TaskPlanningTestHelpers.CreateNode("CostDelivery_Work", new Vector2(7f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("CostDelivery_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("CostDelivery_EdgeB", palletNode, loadingNode);
            var edgeC = TaskPlanningTestHelpers.CreateEdge("CostDelivery_EdgeC", loadingNode, workstationNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("CostDelivery_AmrObject");
            var pallet = TaskPlanningTestHelpers.CreatePallet("CostDelivery_PalletObject", palletNode);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("CostDelivery_LoadingPoint", loadingNode, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("CostDelivery_Workstation", workstationNode, pallet);

            try
            {
                amr.transform.position = amrNode.transform.position;
                TaskPlanningTestHelpers.SetPalletDurations(pallet, 2f, 3f, 5f, 0f);

                var weights = ZeroWeights();
                weights.delivery.amrToPalletEta = 2f;
                weights.delivery.attachTime = 3f;
                weights.delivery.palletToLoadingEta = 5f;
                weights.delivery.loadTime = 7f;
                weights.delivery.loadingToWorkstationEta = 11f;
                weights.delivery.detachTime = 13f;
                weights.removal.amrToPalletEta = 1000f;
                weights.removal.attachTime = 1000f;
                weights.removal.palletToParkingEta = 1000f;
                weights.removal.detachTime = 1000f;

                var evaluator = CreateEvaluator(graph, weights);
                var task = new DeliveryPlanningTask("D-1", pallet, workstation, 0f);
                var cost = evaluator.Evaluate(amr, task, loadingPoint);

                Assert.That(cost.IsFeasible, Is.True);
                Assert.That(cost.TotalCost, Is.EqualTo(136.0).Within(0.0001));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    loadingPoint.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    edgeC.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    workstationNode.gameObject,
                    loadingNode.gameObject,
                    palletNode.gameObject,
                    amrNode.gameObject,
                    graph.gameObject);
            }
        }

        [Test]
        public void RemovalCostUsesRemovalWeightsOnly()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("CostRemoval_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("CostRemoval_Amr", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode("CostRemoval_Pallet", new Vector2(2f, 0f));
            var parkingNode = TaskPlanningTestHelpers.CreateNode("CostRemoval_Parking", new Vector2(8f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("CostRemoval_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("CostRemoval_EdgeB", palletNode, parkingNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("CostRemoval_AmrObject");
            var pallet = TaskPlanningTestHelpers.CreatePallet("CostRemoval_PalletObject", palletNode);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("CostRemoval_Workstation", palletNode, pallet);

            try
            {
                amr.transform.position = amrNode.transform.position;
                TaskPlanningTestHelpers.SetParkingNode(pallet, parkingNode);
                TaskPlanningTestHelpers.SetPalletDurations(pallet, 2f, 3f, 0f, 0f);
                pallet.MarkAwaitingRemoval();

                var weights = ZeroWeights();
                weights.delivery.amrToPalletEta = 1000f;
                weights.delivery.attachTime = 1000f;
                weights.delivery.detachTime = 1000f;
                weights.removal.amrToPalletEta = 2f;
                weights.removal.attachTime = 3f;
                weights.removal.palletToParkingEta = 5f;
                weights.removal.detachTime = 7f;

                var evaluator = CreateEvaluator(graph, weights);
                var task = new PalletRemovalPlanningTask("R-1", pallet, workstation, 0f);
                var cost = evaluator.Evaluate(amr, task);

                Assert.That(cost.IsFeasible, Is.True);
                Assert.That(cost.TotalCost, Is.EqualTo(61.0).Within(0.0001));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    pallet.gameObject,
                    amr.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    parkingNode.gameObject,
                    palletNode.gameObject,
                    amrNode.gameObject,
                    graph.gameObject);
            }
        }

        [Test]
        public void DeliveryAgingUsesDeliveryAgingWeights()
        {
            var fixture = CreateDeliveryFixture("CostDeliveryAging");

            try
            {
                var weights = ZeroWeights();
                weights.delivery.agingWeight = 2f;
                weights.delivery.maxAgingBonus = 5f;
                weights.removal.agingWeight = 100f;
                weights.removal.maxAgingBonus = 100f;

                var evaluator = CreateEvaluator(fixture.Graph, weights, now: 10f);
                var task = new DeliveryPlanningTask("D-Aging", fixture.Pallet, fixture.Workstation, 7f);
                var cost = evaluator.Evaluate(fixture.Amr, task, fixture.LoadingPoint);

                Assert.That(cost.IsFeasible, Is.True);
                Assert.That(cost.TotalCost, Is.EqualTo(-5.0).Within(0.0001));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RemovalAgingUsesRemovalAgingWeights()
        {
            var fixture = CreateRemovalFixture("CostRemovalAging");

            try
            {
                var weights = ZeroWeights();
                weights.delivery.agingWeight = 100f;
                weights.delivery.maxAgingBonus = 100f;
                weights.removal.agingWeight = 2f;
                weights.removal.maxAgingBonus = 5f;

                var evaluator = CreateEvaluator(fixture.Graph, weights, now: 10f);
                var task = new PalletRemovalPlanningTask("R-Aging", fixture.Pallet, fixture.Workstation, 7f);
                var cost = evaluator.Evaluate(fixture.Amr, task);

                Assert.That(cost.IsFeasible, Is.True);
                Assert.That(cost.TotalCost, Is.EqualTo(-5.0).Within(0.0001));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void BlockedWorkstationAddsDeliveryBias()
        {
            var fixture = CreateDeliveryFixture("CostBlockedDelivery");
            var blocker = TaskPlanningTestHelpers.CreatePallet("CostBlockedDelivery_Blocker");

            try
            {
                blocker.transform.position = fixture.Workstation.Node.transform.position;

                var weights = ZeroWeights();
                weights.delivery.blockedDeliveryBias = 42f;

                var evaluator = CreateEvaluator(fixture.Graph, weights);
                var task = new DeliveryPlanningTask("D-Blocked", fixture.Pallet, fixture.Workstation, 0f);
                var cost = evaluator.Evaluate(fixture.Amr, task, fixture.LoadingPoint);

                Assert.That(cost.IsFeasible, Is.True);
                Assert.That(cost.TotalCost, Is.EqualTo(42.0).Within(0.0001));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(blocker.gameObject);
                fixture.Destroy();
            }
        }

        [Test]
        public void RemovalBlockingPendingDeliveryAppliesRemovalMultiplier()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("CostRemovalMultiplier_Graph");
            var workstationNode = TaskPlanningTestHelpers.CreateNode("CostRemovalMultiplier_Work", new Vector2(0f, 0f));
            var parkingNode = TaskPlanningTestHelpers.CreateNode("CostRemovalMultiplier_Parking", new Vector2(10f, 0f));
            var incomingNode = TaskPlanningTestHelpers.CreateNode("CostRemovalMultiplier_Incoming", new Vector2(2f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("CostRemovalMultiplier_EdgeA", workstationNode, parkingNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("CostRemovalMultiplier_EdgeB", workstationNode, incomingNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("CostRemovalMultiplier_Amr");
            var removalPallet = TaskPlanningTestHelpers.CreatePallet("CostRemovalMultiplier_Removal", workstationNode);
            var incomingPallet = TaskPlanningTestHelpers.CreatePallet("CostRemovalMultiplier_IncomingPallet", incomingNode);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("CostRemovalMultiplier_Workstation", workstationNode, incomingPallet, removalPallet);

            try
            {
                amr.transform.position = workstationNode.transform.position;
                TaskPlanningTestHelpers.SetParkingNode(removalPallet, parkingNode);
                TaskPlanningTestHelpers.SetPalletDurations(removalPallet, 0f, 0f, 0f, 0f);
                removalPallet.MarkAwaitingRemoval();

                var weights = ZeroWeights();
                weights.removal.palletToParkingEta = 10f;
                weights.removal.blocksPendingDeliveryMultiplier = 0.25f;
                var pendingDelivery = new DeliveryPlanningTask("D-Pending", incomingPallet, workstation, 0f);
                var evaluator = CreateEvaluator(graph, weights, pendingTasks: new ITaskPlanningTask[] { pendingDelivery });
                var removal = new PalletRemovalPlanningTask("R-Blocking", removalPallet, workstation, 0f);
                var cost = evaluator.Evaluate(amr, removal);

                Assert.That(cost.IsFeasible, Is.True);
                Assert.That(cost.RemovalPriorityMultiplier, Is.EqualTo(0.25).Within(0.0001));
                Assert.That(cost.TotalCost, Is.EqualTo(25.0).Within(0.0001));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    incomingPallet.gameObject,
                    removalPallet.gameObject,
                    amr.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    incomingNode.gameObject,
                    parkingNode.gameObject,
                    workstationNode.gameObject,
                    graph.gameObject);
            }
        }

        private static TaskPlanningCostEvaluator CreateEvaluator(
            MapfSceneGraph graph,
            TaskPlanningCostWeights weights,
            float now = 0f,
            ITaskPlanningTask[] pendingTasks = null)
        {
            return new TaskPlanningCostEvaluator(
                new RoadmapDistanceService(graph),
                weights,
                1f,
                pendingTasks ?? System.Array.Empty<ITaskPlanningTask>(),
                now);
        }

        private static TaskPlanningCostWeights ZeroWeights()
        {
            var weights = new TaskPlanningCostWeights();
            weights.delivery.amrToPalletEta = 0f;
            weights.delivery.attachTime = 0f;
            weights.delivery.loadingQueueEta = 0f;
            weights.delivery.palletToLoadingEta = 0f;
            weights.delivery.loadTime = 0f;
            weights.delivery.loadingToWorkstationEta = 0f;
            weights.delivery.detachTime = 0f;
            weights.delivery.blockedDeliveryBias = 0f;
            weights.delivery.agingWeight = 0f;
            weights.delivery.maxAgingBonus = 0f;

            weights.removal.amrToPalletEta = 0f;
            weights.removal.attachTime = 0f;
            weights.removal.palletToParkingEta = 0f;
            weights.removal.detachTime = 0f;
            weights.removal.blocksPendingDeliveryMultiplier = 1f;
            weights.removal.agingWeight = 0f;
            weights.removal.maxAgingBonus = 0f;
            return weights;
        }

        private static DeliveryFixture CreateDeliveryFixture(string prefix)
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph(prefix + "_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode(prefix + "_AmrNode", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode(prefix + "_PalletNode", new Vector2(1f, 0f));
            var loadingNode = TaskPlanningTestHelpers.CreateNode(prefix + "_LoadingNode", new Vector2(2f, 0f));
            var workstationNode = TaskPlanningTestHelpers.CreateNode(prefix + "_WorkstationNode", new Vector2(3f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeB", palletNode, loadingNode);
            var edgeC = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeC", loadingNode, workstationNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_Amr");
            var pallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_Pallet", palletNode);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint(prefix + "_LoadingPoint", loadingNode, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_Workstation", workstationNode, pallet);
            amr.transform.position = amrNode.transform.position;
            return new DeliveryFixture(graph, amrNode, palletNode, loadingNode, workstationNode, edgeA, edgeB, edgeC, amr, pallet, loadingPoint, workstation);
        }

        private static RemovalFixture CreateRemovalFixture(string prefix)
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph(prefix + "_Graph");
            var palletNode = TaskPlanningTestHelpers.CreateNode(prefix + "_PalletNode", new Vector2(0f, 0f));
            var parkingNode = TaskPlanningTestHelpers.CreateNode(prefix + "_ParkingNode", new Vector2(1f, 0f));
            var edge = TaskPlanningTestHelpers.CreateEdge(prefix + "_Edge", palletNode, parkingNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_Amr");
            var pallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_Pallet", palletNode);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_Workstation", palletNode, pallet);
            amr.transform.position = palletNode.transform.position;
            TaskPlanningTestHelpers.SetParkingNode(pallet, parkingNode);
            pallet.MarkAwaitingRemoval();
            return new RemovalFixture(graph, palletNode, parkingNode, edge, amr, pallet, workstation);
        }

        private sealed class DeliveryFixture
        {
            public DeliveryFixture(
                MapfSceneGraph graph,
                MapfNode amrNode,
                MapfNode palletNode,
                MapfNode loadingNode,
                MapfNode workstationNode,
                MapfEdge edgeA,
                MapfEdge edgeB,
                MapfEdge edgeC,
                TaskPlanningAmr amr,
                PalletMarker pallet,
                PalletLoadingPoint loadingPoint,
                WorkstationDeliveryPoint workstation)
            {
                Graph = graph;
                AmrNode = amrNode;
                PalletNode = palletNode;
                LoadingNode = loadingNode;
                WorkstationNode = workstationNode;
                EdgeA = edgeA;
                EdgeB = edgeB;
                EdgeC = edgeC;
                Amr = amr;
                Pallet = pallet;
                LoadingPoint = loadingPoint;
                Workstation = workstation;
            }

            public MapfSceneGraph Graph { get; }
            public MapfNode AmrNode { get; }
            public MapfNode PalletNode { get; }
            public MapfNode LoadingNode { get; }
            public MapfNode WorkstationNode { get; }
            public MapfEdge EdgeA { get; }
            public MapfEdge EdgeB { get; }
            public MapfEdge EdgeC { get; }
            public TaskPlanningAmr Amr { get; }
            public PalletMarker Pallet { get; }
            public PalletLoadingPoint LoadingPoint { get; }
            public WorkstationDeliveryPoint Workstation { get; }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(
                    Workstation.gameObject,
                    LoadingPoint.gameObject,
                    Pallet.gameObject,
                    Amr.gameObject,
                    EdgeC.gameObject,
                    EdgeB.gameObject,
                    EdgeA.gameObject,
                    WorkstationNode.gameObject,
                    LoadingNode.gameObject,
                    PalletNode.gameObject,
                    AmrNode.gameObject,
                    Graph.gameObject);
            }
        }

        private sealed class RemovalFixture
        {
            public RemovalFixture(
                MapfSceneGraph graph,
                MapfNode palletNode,
                MapfNode parkingNode,
                MapfEdge edge,
                TaskPlanningAmr amr,
                PalletMarker pallet,
                WorkstationDeliveryPoint workstation)
            {
                Graph = graph;
                PalletNode = palletNode;
                ParkingNode = parkingNode;
                Edge = edge;
                Amr = amr;
                Pallet = pallet;
                Workstation = workstation;
            }

            public MapfSceneGraph Graph { get; }
            public MapfNode PalletNode { get; }
            public MapfNode ParkingNode { get; }
            public MapfEdge Edge { get; }
            public TaskPlanningAmr Amr { get; }
            public PalletMarker Pallet { get; }
            public WorkstationDeliveryPoint Workstation { get; }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(
                    Workstation.gameObject,
                    Pallet.gameObject,
                    Amr.gameObject,
                    Edge.gameObject,
                    ParkingNode.gameObject,
                    PalletNode.gameObject,
                    Graph.gameObject);
            }
        }
    }
}
