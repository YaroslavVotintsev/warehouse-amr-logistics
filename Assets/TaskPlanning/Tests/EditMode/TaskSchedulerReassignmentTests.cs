using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class TaskSchedulerReassignmentTests
    {
        [Test]
        public void ReassignmentThresholdUsesPercentageImprovement()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("ReassignmentThreshold_Scheduler");

            try
            {
                TaskPlanningTestHelpers.SetField(scheduler, "reassignmentCostImprovementPercent", 10f);

                Assert.That(EnoughImprovement(scheduler, currentCost: 100.0, replacementCost: 91.0), Is.False);
                Assert.That(EnoughImprovement(scheduler, currentCost: 100.0, replacementCost: 90.0), Is.True);
                Assert.That(EnoughImprovement(scheduler, currentCost: 100.0, replacementCost: 101.0), Is.False);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(scheduler.gameObject);
            }
        }

        [Test]
        public void SoftReassignmentCandidateIsLimitedToItsReplacementTask()
        {
            var fixture = SoftReassignmentFixture.Create("SoftCandidateLimited");

            try
            {
                var option = fixture.CreateSoftOption(fixture.ReplacementTask);
                var problem = fixture.CreateProblem(
                    new ITaskPlanningTask[] { fixture.ReplacementTask, fixture.OtherTask },
                    new[] { option });

                var candidates = DispatchCandidateBuilder.BuildImmediateCandidates(problem);

                Assert.That(candidates, Has.Count.EqualTo(1));
                Assert.That(candidates[0].Task, Is.SameAs(fixture.ReplacementTask));
                Assert.That(candidates[0].Availability.IsSoftReassignment, Is.True);
                Assert.That(candidates[0].ReplacesActiveAssignment, Is.True);
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void SoftReassignmentMetadataSurvivesDispatchSelection()
        {
            var fixture = SoftReassignmentFixture.Create("SoftCandidateMetadata");

            try
            {
                var option = fixture.CreateSoftOption(fixture.ReplacementTask);
                var problem = fixture.CreateProblem(
                    new ITaskPlanningTask[] { fixture.ReplacementTask },
                    new[] { option });

                var plan = new NearestDispatching().Solve(problem);

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].ReplacesActiveAssignment, Is.True);
                Assert.That(plan.Assignments[0].SoftReassignment.ActiveTask, Is.SameAs(fixture.ActiveTask));
                Assert.That(plan.Assignments[0].SoftReassignment.ReplacementTask, Is.SameAs(fixture.ReplacementTask));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        private static bool EnoughImprovement(TaskScheduler scheduler, double currentCost, double replacementCost)
        {
            return (bool)TaskPlanningTestHelpers.InvokePrivate(
                scheduler,
                "IsEnoughReassignmentImprovement",
                currentCost,
                replacementCost);
        }

        private sealed class SoftReassignmentFixture
        {
            private readonly UnityEngine.Object[] _objects;

            private SoftReassignmentFixture(
                UnityEngine.Object[] objects,
                Mapf.Authoring.MapfSceneGraph graph,
                TaskPlanningAmr amr,
                PalletLoadingPoint loadingPoint,
                WorkstationDeliveryPoint workstation,
                DispatchAssignment activeAssignment,
                DeliveryPlanningTask activeTask,
                DeliveryPlanningTask replacementTask,
                DeliveryPlanningTask otherTask)
            {
                _objects = objects;
                Graph = graph;
                Amr = amr;
                LoadingPoint = loadingPoint;
                Workstation = workstation;
                ActiveAssignment = activeAssignment;
                ActiveTask = activeTask;
                ReplacementTask = replacementTask;
                OtherTask = otherTask;
            }

            public Mapf.Authoring.MapfSceneGraph Graph { get; }
            public TaskPlanningAmr Amr { get; }
            public PalletLoadingPoint LoadingPoint { get; }
            public WorkstationDeliveryPoint Workstation { get; }
            public DispatchAssignment ActiveAssignment { get; }
            public DeliveryPlanningTask ActiveTask { get; }
            public DeliveryPlanningTask ReplacementTask { get; }
            public DeliveryPlanningTask OtherTask { get; }

            public static SoftReassignmentFixture Create(string prefix)
            {
                var graph = TaskPlanningTestHelpers.CreateSceneGraph(prefix + "_Graph");
                var amrNode = TaskPlanningTestHelpers.CreateNode(prefix + "_AmrNode", new Vector2(0f, 0f));
                var activePalletNode = TaskPlanningTestHelpers.CreateNode(prefix + "_ActivePalletNode", new Vector2(0f, 1f));
                var replacementPalletNode = TaskPlanningTestHelpers.CreateNode(prefix + "_ReplacementPalletNode", new Vector2(1f, 0f));
                var otherPalletNode = TaskPlanningTestHelpers.CreateNode(prefix + "_OtherPalletNode", new Vector2(2f, 0f));
                var loadingNode = TaskPlanningTestHelpers.CreateNode(prefix + "_LoadingNode", new Vector2(3f, 0f));
                var workstationNode = TaskPlanningTestHelpers.CreateNode(prefix + "_WorkstationNode", new Vector2(4f, 0f));
                var edgeA = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeA", amrNode, replacementPalletNode);
                var edgeB = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeB", replacementPalletNode, otherPalletNode);
                var edgeC = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeC", otherPalletNode, loadingNode);
                var edgeD = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeD", loadingNode, workstationNode);
                var edgeE = TaskPlanningTestHelpers.CreateEdge(prefix + "_EdgeE", amrNode, activePalletNode);
                var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_Amr");
                var activePallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_ActivePallet", activePalletNode);
                var replacementPallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_ReplacementPallet", replacementPalletNode);
                var otherPallet = TaskPlanningTestHelpers.CreatePallet(prefix + "_OtherPallet", otherPalletNode);
                var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint(prefix + "_LoadingPoint", loadingNode, replacementPallet, otherPallet);
                var workstation = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_Workstation", workstationNode, activePallet, replacementPallet, otherPallet);
                var activeTask = new DeliveryPlanningTask(prefix + "_ActiveTask", activePallet, workstation, 0f);
                var replacementTask = new DeliveryPlanningTask(prefix + "_ReplacementTask", replacementPallet, workstation, 0f);
                var otherTask = new DeliveryPlanningTask(prefix + "_OtherTask", otherPallet, workstation, 0f);

                amr.transform.position = amrNode.transform.position;
                amr.TryReserve();
                var activeAssignment = new DispatchAssignment(
                    amr,
                    activeTask,
                    activePallet,
                    loadingPoint,
                    workstation,
                    null,
                    new CostEvaluation(isFeasible: true, totalCost: 100));

                var objects = new UnityEngine.Object[]
                {
                    workstation.gameObject,
                    loadingPoint.gameObject,
                    otherPallet.gameObject,
                    replacementPallet.gameObject,
                    activePallet.gameObject,
                    amr.gameObject,
                    edgeE.gameObject,
                    edgeD.gameObject,
                    edgeC.gameObject,
                    edgeB.gameObject,
                    edgeA.gameObject,
                    workstationNode.gameObject,
                    loadingNode.gameObject,
                    otherPalletNode.gameObject,
                    replacementPalletNode.gameObject,
                    activePalletNode.gameObject,
                    amrNode.gameObject,
                    graph.gameObject
                };

                return new SoftReassignmentFixture(
                    objects,
                    graph,
                    amr,
                    loadingPoint,
                    workstation,
                    activeAssignment,
                    activeTask,
                    replacementTask,
                    otherTask);
            }

            public SoftReassignmentOption CreateSoftOption(ITaskPlanningTask replacementTask)
            {
                return new SoftReassignmentOption(
                    ActiveAssignment,
                    replacementTask,
                    new CostEvaluation(isFeasible: true, totalCost: 100),
                    new CostEvaluation(isFeasible: true, totalCost: 40),
                    improvementPercent: 60);
            }

            public DispatchProblem CreateProblem(
                ITaskPlanningTask[] tasks,
                SoftReassignmentOption[] softReassignmentOptions)
            {
                var distances = new RoadmapDistanceService(Graph);
                var evaluator = new TaskPlanningCostEvaluator(distances, new TaskPlanningCostWeights(), 1f, tasks, 0f);
                return new DispatchProblem(
                    tasks,
                    System.Array.Empty<TaskPlanningAmr>(),
                    new[] { LoadingPoint },
                    distances,
                    evaluator,
                    0f,
                    softReassignmentOptions: softReassignmentOptions);
            }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(_objects.Where(item => item != null).ToArray());
            }
        }
    }
}
