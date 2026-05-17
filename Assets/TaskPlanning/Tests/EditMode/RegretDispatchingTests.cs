using System.Linq;
using Mapf.Authoring;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class RegretDispatchingTests
    {
        [Test]
        public void RegretDispatchingChoosesHighRegretTaskBeforeLowerAbsoluteCostTask()
        {
            var fixture = CreateCandidateFixture("RegretHigh");

            try
            {
                var lowCost = fixture.CreateTask("D-LowCost", fixture.PalletA);
                var highRegret = fixture.CreateTask("D-HighRegret", fixture.PalletB);
                var plan = Solve(
                    new[] { lowCost, highRegret },
                    fixture.Amrs,
                    new[]
                    {
                        fixture.Candidate(fixture.AmrA, lowCost, fixture.PalletA, 1),
                        fixture.Candidate(fixture.AmrB, lowCost, fixture.PalletA, 2),
                        fixture.Candidate(fixture.AmrA, highRegret, fixture.PalletB, 3),
                        fixture.Candidate(fixture.AmrB, highRegret, fixture.PalletB, 100)
                    });

                Assert.That(plan.Assignments, Has.Count.EqualTo(2));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(highRegret));
                Assert.That(plan.Assignments[0].Amr, Is.SameAs(fixture.AmrA));
                Assert.That(plan.Assignments[1].Task, Is.SameAs(lowCost));
                Assert.That(plan.Assignments[1].Amr, Is.SameAs(fixture.AmrB));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RegretDispatchingPrioritizesSingleCandidateTask()
        {
            var fixture = CreateCandidateFixture("RegretSingle");

            try
            {
                var singleCandidate = fixture.CreateTask("D-Single", fixture.PalletA);
                var flexible = fixture.CreateTask("D-Flexible", fixture.PalletB);
                var plan = Solve(
                    new[] { flexible, singleCandidate },
                    fixture.Amrs,
                    new[]
                    {
                        fixture.Candidate(fixture.AmrA, singleCandidate, fixture.PalletA, 50),
                        fixture.Candidate(fixture.AmrA, flexible, fixture.PalletB, 1),
                        fixture.Candidate(fixture.AmrB, flexible, fixture.PalletB, 2)
                    });

                Assert.That(plan.Assignments, Has.Count.EqualTo(2));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(singleCandidate));
                Assert.That(plan.Assignments[0].Amr, Is.SameAs(fixture.AmrA));
                Assert.That(plan.Assignments[1].Task, Is.SameAs(flexible));
                Assert.That(plan.Assignments[1].Amr, Is.SameAs(fixture.AmrB));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RegretDispatchingSkipsTasksWithoutCandidatesAndContinues()
        {
            var fixture = CreateCandidateFixture("RegretSkip");

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

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments[0].Task, Is.SameAs(feasible));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RegretDispatchingDoesNotAssignSameAmrMoreThanOnce()
        {
            var fixture = CreateCandidateFixture("RegretOneAmr");

            try
            {
                var taskA = fixture.CreateTask("D-A", fixture.PalletA);
                var taskB = fixture.CreateTask("D-B", fixture.PalletB);
                var plan = Solve(
                    new[] { taskA, taskB },
                    new[] { fixture.AmrA },
                    new[]
                    {
                        fixture.Candidate(fixture.AmrA, taskA, fixture.PalletA, 1),
                        fixture.Candidate(fixture.AmrA, taskB, fixture.PalletB, 2)
                    });

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments.Select(assignment => assignment.Amr).Distinct().Count(), Is.EqualTo(plan.Assignments.Count));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RegretDispatchingDoesNotAssignSamePalletMoreThanOnce()
        {
            var fixture = CreateCandidateFixture("RegretOnePallet");

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

                Assert.That(plan.Assignments, Has.Count.EqualTo(1));
                Assert.That(plan.Assignments.Select(assignment => assignment.Pallet).Distinct().Count(), Is.EqualTo(plan.Assignments.Count));
            }
            finally
            {
                fixture.Destroy();
            }
        }

        [Test]
        public void RegretDispatchingUsesResolvedSingleLoadingPointForDeliveryTask()
        {
            var graph = TaskPlanningTestHelpers.CreateSceneGraph("RegretResolvedLoading_Graph");
            var amrNode = TaskPlanningTestHelpers.CreateNode("RegretResolvedLoading_Amr", new Vector2(0f, 0f));
            var palletNode = TaskPlanningTestHelpers.CreateNode("RegretResolvedLoading_Pallet", new Vector2(1f, 0f));
            var decoyLoadNode = TaskPlanningTestHelpers.CreateNode("RegretResolvedLoading_DecoyLoad", new Vector2(2f, 0f));
            var resolvedLoadNode = TaskPlanningTestHelpers.CreateNode("RegretResolvedLoading_Load", new Vector2(5f, 0f));
            var workNode = TaskPlanningTestHelpers.CreateNode("RegretResolvedLoading_Work", new Vector2(6f, 0f));
            var edgeA = TaskPlanningTestHelpers.CreateEdge("RegretResolvedLoading_EdgeA", amrNode, palletNode);
            var edgeB = TaskPlanningTestHelpers.CreateEdge("RegretResolvedLoading_EdgeB", palletNode, resolvedLoadNode);
            var edgeC = TaskPlanningTestHelpers.CreateEdge("RegretResolvedLoading_EdgeC", resolvedLoadNode, workNode);
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("RegretResolvedLoading_AmrObject");
            var pallet = TaskPlanningTestHelpers.CreatePallet("RegretResolvedLoading_PalletObject", palletNode);
            var decoyPallet = TaskPlanningTestHelpers.CreatePallet("RegretResolvedLoading_DecoyPallet");
            var decoyLoading = TaskPlanningTestHelpers.CreateLoadingPoint("RegretResolvedLoading_Decoy", decoyLoadNode, decoyPallet);
            var resolvedLoading = TaskPlanningTestHelpers.CreateLoadingPoint("RegretResolvedLoading_Resolved", resolvedLoadNode, pallet);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("RegretResolvedLoading_Workstation", workNode, pallet);

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
                var plan = new RegretDispatching().Solve(problem);

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
        public void RegretDispatchingIsSelectableByTaskScheduler()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("RegretScheduler");

            try
            {
                scheduler.ConfigurePlanningMode(TaskPlanningAlgorithmType.RegretDispatching, TaskPlanningFutureHandlingMode.ImmediateOnly);

                Assert.That(scheduler.Algorithm, Is.EqualTo(TaskPlanningAlgorithmType.RegretDispatching));
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
            return new RegretDispatching().Solve(problem);
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
