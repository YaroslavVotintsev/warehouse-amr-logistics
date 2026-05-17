using System.Linq;
using Mapf.Authoring;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class HungarianDispatchingTests
    {
        [Test]
        public void HungarianDispatchingMinimizesTotalAssignmentCost()
        {
            var fixture = CreateCandidateFixture("HungarianTotal");

            try
            {
                var flexible = fixture.CreateTask("D-Flexible", fixture.PalletA);
                var scarce = fixture.CreateTask("D-Scarce", fixture.PalletB);
                var plan = Solve(
                    new[] { flexible, scarce },
                    fixture.Amrs,
                    new[]
                    {
                        fixture.Candidate(fixture.AmrA, flexible, fixture.PalletA, 12),
                        fixture.Candidate(fixture.AmrA, scarce, fixture.PalletB, 13),
                        fixture.Candidate(fixture.AmrB, flexible, fixture.PalletA, 16),
                        fixture.Candidate(fixture.AmrB, scarce, fixture.PalletB, 19)
                    });
                var byTask = plan.Assignments.ToDictionary(assignment => assignment.Task.TaskId);

                Assert.That(plan.Assignments.Count, Is.EqualTo(2));
                Assert.That(byTask["D-Scarce"].Amr, Is.SameAs(fixture.AmrA));
                Assert.That(byTask["D-Flexible"].Amr, Is.SameAs(fixture.AmrB));
                Assert.That(plan.Assignments.Sum(assignment => assignment.Score), Is.EqualTo(29));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void HungarianDispatchingHandlesMoreTasksThanAmrs()
        {
            var fixture = CreateCandidateFixture("HungarianMoreTasks");

            try
            {
                var expensive = fixture.CreateTask("D-Expensive", fixture.PalletA);
                var cheap = fixture.CreateTask("D-Cheap", fixture.PalletB);
                var plan = Solve(
                    new[] { expensive, cheap },
                    new[] { fixture.AmrA },
                    new[]
                    {
                        fixture.Candidate(fixture.AmrA, expensive, fixture.PalletA, 50),
                        fixture.Candidate(fixture.AmrA, cheap, fixture.PalletB, 3)
                    });

                Assert.That(plan.Assignments.Count, Is.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(cheap));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void HungarianDispatchingHandlesMoreAmrsThanTasks()
        {
            var fixture = CreateCandidateFixture("HungarianMoreAmrs");

            try
            {
                var task = fixture.CreateTask("D-Only", fixture.PalletA);
                var plan = Solve(
                    new[] { task },
                    fixture.Amrs,
                    new[]
                    {
                        fixture.Candidate(fixture.AmrA, task, fixture.PalletA, 10),
                        fixture.Candidate(fixture.AmrB, task, fixture.PalletA, 2)
                    });

                Assert.That(plan.Assignments.Count, Is.EqualTo(1));
                Assert.That(plan.Assignments[0].Amr, Is.SameAs(fixture.AmrB));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void HungarianDispatchingSkipsTasksWithoutCandidatesAndContinues()
        {
            var fixture = CreateCandidateFixture("HungarianSkip");

            try
            {
                var infeasible = fixture.CreateTask("D-Infeasible", fixture.PalletA);
                var feasible = fixture.CreateTask("D-Feasible", fixture.PalletB);
                var plan = Solve(
                    new[] { infeasible, feasible },
                    fixture.Amrs,
                    new[]
                    {
                        fixture.Candidate(fixture.AmrB, feasible, fixture.PalletB, 2)
                    });

                Assert.That(plan.Assignments.Count, Is.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(feasible));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void HungarianDispatchingDoesNotAssignSamePalletMoreThanOnce()
        {
            var fixture = CreateCandidateFixture("HungarianOnePallet");

            try
            {
                var taskA = fixture.CreateTask("D-A", fixture.PalletA);
                var taskB = fixture.CreateTask("D-B", fixture.PalletA);
                var plan = Solve(
                    new[] { taskA, taskB },
                    fixture.Amrs,
                    new[]
                    {
                        fixture.Candidate(fixture.AmrA, taskA, fixture.PalletA, 1),
                        fixture.Candidate(fixture.AmrB, taskB, fixture.PalletA, 2)
                    });

                Assert.That(plan.Assignments.Count, Is.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(taskA));
                Assert.That(plan.Assignments.Select(assignment => assignment.Pallet).Distinct().Count(), Is.EqualTo(plan.Assignments.Count));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void HungarianDispatchingUsesResolvedSingleLoadingPointForDeliveryTask()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("HungarianResolvedLoading_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("HungarianResolvedLoading_Amr", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode("HungarianResolvedLoading_Pallet", new Vector2(1f, 0f));
            var decoyLoadNode = TaskPlanningTestHelpers.CreateNode("HungarianResolvedLoading_DecoyLoad", new Vector2(2f, 0f));
            var resolvedLoadNode = TaskPlanningTestHelpers.CreateNode("HungarianResolvedLoading_Load", new Vector2(5f, 0f));
            var workNode = TaskPlanningTestHelpers.CreateNode("HungarianResolvedLoading_Work", new Vector2(6f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("HungarianResolvedLoading_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("HungarianResolvedLoading_EdgeB", palletNode, resolvedLoadNode);
            var edgeC = TaskPlanningTestHelpers.CreateEdge("HungarianResolvedLoading_EdgeC", resolvedLoadNode, workNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("HungarianResolvedLoading_AmrObject");
            var pallet = TaskPlanningTestHelpers.CreatePallet("HungarianResolvedLoading_PalletObject", palletNode);
            var decoyPallet = TaskPlanningTestHelpers.CreatePallet("HungarianResolvedLoading_DecoyPallet");
            var decoyLoading = TaskPlanningTestHelpers.CreateLoadingPoint("HungarianResolvedLoading_Decoy", decoyLoadNode, decoyPallet);
            var resolvedLoading = TaskPlanningTestHelpers.CreateLoadingPoint("HungarianResolvedLoading_Resolved", resolvedLoadNode, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("HungarianResolvedLoading_Workstation", workNode, pallet);

            try
            {
                amr.transform.position = amrNode.transform.position;
                var task = new DeliveryPlanningTask("D-Resolved", pallet, workstation, 0f);
                var distances = new RoadmapDistanceService(graph);
                var evaluator = new TaskPlanningCostEvaluator(
                    distances,
                    new TaskPlanningCostWeights(),
                    1f,
                    new ITaskPlanningTask[] { task },
                    0f);
                var problem = new DispatchProblem(
                    new ITaskPlanningTask[] { task },
                    new[] { amr },
                    new[] { decoyLoading, resolvedLoading },
                    distances,
                    evaluator,
                    0f);
                var plan = new HungarianDispatching().Solve(problem);

                Assert.That(plan.Assignments.Count, Is.EqualTo(1));
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
        public void HungarianDispatchingIsSelectableByTaskScheduler()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("HungarianScheduler");

            try
            {
                scheduler.ConfigurePlanningMode(TaskPlanningAlgorithmType.HungarianDispatching, TaskPlanningFutureHandlingMode.ImmediateOnly);

                Assert.That(scheduler.Algorithm, Is.EqualTo(TaskPlanningAlgorithmType.HungarianDispatching));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(scheduler.gameObject);
            }
        }

        private static DispatchPlan Solve(
            ITaskPlanningTask[] tasks,
            TaskPlanningAmr[] amrs,
            DispatchCandidate[] candidates)
        {
            var problem = new DispatchProblem(
                tasks,
                amrs,
                System.Array.Empty<PalletLoadingPoint>(),
                distances: null,
                costEvaluator: null,
                now: 0f,
                candidates: candidates);
            return new HungarianDispatching().Solve(problem);
        }

        private static CandidateFixture CreateCandidateFixture(string prefix)
        {
            var node = TaskPlanningTestHelpers.CreateNode(prefix + "_Node", Vector2.zero);
            var amrA = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_AmrA");
            var amrB = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>(prefix + "_AmrB");
            var palletA = TaskPlanningTestHelpers.CreatePallet(prefix + "_PalletA");
            var palletB = TaskPlanningTestHelpers.CreatePallet(prefix + "_PalletB");
            var workstation = TaskPlanningTestHelpers.CreateWorkstation(prefix + "_Workstation", null, palletA, palletB);
            return new CandidateFixture(node, amrA, amrB, palletA, palletB, workstation);
        }

        private sealed class CandidateFixture
        {
            public CandidateFixture(
                MapfNode node,
                TaskPlanningAmr amrA,
                TaskPlanningAmr amrB,
                PalletMarker palletA,
                PalletMarker palletB,
                WorkstationDeliveryPoint workstation)
            {
                Node = node;
                AmrA = amrA;
                AmrB = amrB;
                PalletA = palletA;
                PalletB = palletB;
                Workstation = workstation;
            }

            public MapfNode Node { get; }
            public TaskPlanningAmr AmrA { get; }
            public TaskPlanningAmr AmrB { get; }
            public PalletMarker PalletA { get; }
            public PalletMarker PalletB { get; }
            public WorkstationDeliveryPoint Workstation { get; }
            public TaskPlanningAmr[] Amrs => new[] { AmrA, AmrB };

            public DeliveryPlanningTask CreateTask(string taskId, PalletMarker pallet)
            {
                return new DeliveryPlanningTask(taskId, pallet, Workstation, 0f);
            }

            public DispatchCandidate Candidate(TaskPlanningAmr amr, ITaskPlanningTask task, PalletMarker pallet, double cost)
            {
                return new DispatchCandidate(
                    new DispatchAvailability(amr, Node, 0, 0, isImmediate: true),
                    task,
                    pallet,
                    loadingPoint: null,
                    Workstation,
                    removalTargetNode: null,
                    new CostEvaluation(isFeasible: true, totalCost: cost));
            }

            public void Destroy()
            {
                TaskPlanningTestHelpers.Destroy(
                    Workstation.gameObject,
                    PalletB.gameObject,
                    PalletA.gameObject,
                    AmrB.gameObject,
                    AmrA.gameObject,
                    Node.gameObject);
            }
        }
    }
}
