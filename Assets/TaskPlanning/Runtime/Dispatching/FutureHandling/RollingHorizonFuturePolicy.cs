using System;
using System.Collections.Generic;
using System.Linq;
using Mapf.Authoring;
using UnityEngine;

namespace TaskPlanning
{
    [Serializable]
    public sealed class RollingHorizonOptions
    {
        [Min(0f)] public float horizonSeconds = 30f;
        [Min(1)] public int maxWaves = 4;
        [Min(0f)] public float waitImprovementPercent = 10f;
    }

    public sealed class RollingHorizonFuturePolicy : ITaskPlanningFuturePolicy
    {
        private const double Epsilon = 0.0001;

        private readonly RollingHorizonOptions _options;

        public RollingHorizonFuturePolicy()
            : this(new RollingHorizonOptions())
        {
        }

        public RollingHorizonFuturePolicy(RollingHorizonOptions options)
        {
            _options = options ?? new RollingHorizonOptions();
        }

        public DispatchPlan Solve(DispatchProblem problem, ITaskDispatchAlgorithm baseDispatcher)
        {
            var immediateCandidates = DispatchCandidateBuilder.BuildImmediateCandidates(problem);
            if (immediateCandidates.Count == 0)
                return baseDispatcher.Solve(problem.WithCandidates(immediateCandidates));

            var futureAssignments = SimulateFutureAssignments(problem, baseDispatcher);
            if (futureAssignments.Count == 0)
                return baseDispatcher.Solve(problem.WithCandidates(immediateCandidates));

            var allowedCandidates = SuppressCandidatesWorthWaitingFor(immediateCandidates, futureAssignments);
            return baseDispatcher.Solve(problem.WithCandidates(allowedCandidates));
        }

        private List<ScheduledCandidate> SimulateFutureAssignments(
            DispatchProblem problem,
            ITaskDispatchAlgorithm baseDispatcher)
        {
            var state = new VirtualState(problem);
            var scheduled = new List<ScheduledCandidate>();
            var horizonEnd = problem.Now + Math.Max(0f, _options.horizonSeconds);
            var maxWaves = Math.Max(1, _options.maxWaves);
            var waveTime = NextAvailabilityAtOrAfter(state, problem.Now);

            for (var wave = 0; wave < maxWaves && state.RemainingTasks.Count > 0; wave++)
            {
                if (!Finite(waveTime) || waveTime > horizonEnd + Epsilon)
                    break;

                var availableAmrs = state.AvailableAmrs(waveTime).ToArray();
                if (availableAmrs.Length == 0)
                {
                    waveTime = NextAvailabilityAfter(state, waveTime);
                    wave--;
                    continue;
                }

                var candidates = BuildVirtualCandidates(problem, state, availableAmrs, horizonEnd);
                if (candidates.Count == 0)
                {
                    var nextTime = NextAvailabilityAfter(state, waveTime);
                    if (!Finite(nextTime))
                        break;

                    waveTime = nextTime;
                    wave--;
                    continue;
                }

                var virtualProblem = problem.WithCandidates(candidates);
                var plan = baseDispatcher.Solve(virtualProblem);
                if (plan.Assignments.Count == 0)
                {
                    var nextTime = NextAvailabilityAfter(state, waveTime);
                    if (!Finite(nextTime))
                        break;

                    waveTime = nextTime;
                    wave--;
                    continue;
                }

                foreach (var assignment in plan.Assignments)
                {
                    if (!state.RemainingTasks.Contains(assignment.Task) ||
                        !state.AmrAvailabilities.TryGetValue(assignment.Amr, out var availability))
                        continue;

                    if (!TryBuildVirtualCandidate(
                            problem,
                            state,
                            availability,
                            assignment.Task,
                            horizonEnd,
                            out var candidate,
                            out var estimate))
                        continue;

                    state.Apply(candidate, estimate);
                    scheduled.Add(new ScheduledCandidate(candidate, estimate));
                }

                waveTime = NextAvailabilityAfter(state, waveTime);
            }

            return scheduled;
        }

        private IReadOnlyList<DispatchCandidate> BuildVirtualCandidates(
            DispatchProblem problem,
            VirtualState state,
            IReadOnlyList<VirtualAmrAvailability> availableAmrs,
            double horizonEnd)
        {
            var candidates = new List<DispatchCandidate>();
            foreach (var availability in availableAmrs)
            {
                foreach (var task in state.RemainingTasks)
                {
                    if (TryBuildVirtualCandidate(problem, state, availability, task, horizonEnd, out var candidate, out _))
                        candidates.Add(candidate);
                }
            }

            return candidates;
        }

        private bool TryBuildVirtualCandidate(
            DispatchProblem problem,
            VirtualState state,
            VirtualAmrAvailability availability,
            ITaskPlanningTask task,
            double horizonEnd,
            out DispatchCandidate candidate,
            out ExecutionEstimate estimate)
        {
            candidate = default;
            estimate = default;

            if (availability == null || !availability.IsValid || task == null)
                return false;

            switch (task)
            {
                case DeliveryPlanningTask delivery:
                    return TryBuildDeliveryCandidate(problem, state, availability, delivery, horizonEnd, out candidate, out estimate);
                case PalletRemovalPlanningTask removal:
                    return TryBuildRemovalCandidate(problem, state, availability, removal, horizonEnd, out candidate, out estimate);
                default:
                    return false;
            }
        }

        private bool TryBuildDeliveryCandidate(
            DispatchProblem problem,
            VirtualState state,
            VirtualAmrAvailability availability,
            DeliveryPlanningTask task,
            double horizonEnd,
            out DispatchCandidate candidate,
            out ExecutionEstimate estimate)
        {
            candidate = default;
            estimate = default;

            var loadingPointResolution = problem.ResolveLoadingPoint(task.Pallet);
            if (!loadingPointResolution.IsResolved)
                return false;

            var loadingPoint = loadingPointResolution.LoadingPoint;
            var baseCost = problem.CostEvaluator.EvaluateFrom(
                availability.StartNode,
                task,
                loadingPoint,
                availability.PriorAssignmentEta);
            if (!baseCost.IsFeasible)
                return false;

            var loadingReadyAt = state.LoadingAvailableAt(loadingPoint, problem.Now);
            var workstationReadyAt = state.WorkstationAvailableAt(task.Workstation, problem.Now);
            if (!Finite(workstationReadyAt))
                return false;

            var loadArrivalAt =
                availability.AvailableAt +
                baseCost.AmrToPalletEta +
                baseCost.AttachTime +
                baseCost.PalletToLoadingEta;
            var naturalLoadStartAt = loadArrivalAt + baseCost.LoadingQueueEta;
            var additionalLoadingWait = Math.Max(0.0, loadingReadyAt - naturalLoadStartAt);
            var loadStartAt = naturalLoadStartAt + additionalLoadingWait;
            var loadFinishAt = loadStartAt + baseCost.LoadTime;
            var workstationArrivalAt = loadFinishAt + baseCost.LoadingToWorkstationEta;
            var additionalWorkstationWait = Math.Max(0.0, workstationReadyAt - workstationArrivalAt);
            var detachStartAt = workstationArrivalAt + additionalWorkstationWait;
            var amrAvailableAt = detachStartAt + baseCost.DetachTime;
            var workstationAvailableAt = amrAvailableAt + task.Pallet.UnloadDurationSeconds;
            if (!Finite(amrAvailableAt) || amrAvailableAt > horizonEnd + Epsilon)
                return false;

            var additionalQueueEta = additionalLoadingWait + additionalWorkstationWait;
            var adjustedCost = additionalQueueEta > Epsilon
                ? problem.CostEvaluator.EvaluateFrom(
                    availability.StartNode,
                    task,
                    loadingPoint,
                    availability.PriorAssignmentEta,
                    additionalQueueEta)
                : baseCost;
            if (!adjustedCost.IsFeasible)
                return false;

            var dispatchAvailability = new DispatchAvailability(
                availability.Amr,
                availability.StartNode,
                availability.AvailableAt,
                availability.PriorAssignmentEta,
                isImmediate: false);
            candidate = new DispatchCandidate(
                dispatchAvailability,
                task,
                task.Pallet,
                loadingPoint,
                task.Workstation,
                null,
                adjustedCost);
            estimate = new ExecutionEstimate(
                task.Workstation.Node,
                amrAvailableAt,
                loadingPoint,
                loadFinishAt,
                task.Workstation,
                workstationAvailableAt);
            return candidate.IsValid && estimate.IsValid;
        }

        private bool TryBuildRemovalCandidate(
            DispatchProblem problem,
            VirtualState state,
            VirtualAmrAvailability availability,
            PalletRemovalPlanningTask task,
            double horizonEnd,
            out DispatchCandidate candidate,
            out ExecutionEstimate estimate)
        {
            candidate = default;
            estimate = default;

            var cost = problem.CostEvaluator.EvaluateFrom(
                availability.StartNode,
                task,
                availability.PriorAssignmentEta);
            if (!cost.IsFeasible)
                return false;

            var amrAvailableAt =
                availability.AvailableAt +
                cost.AmrToPalletEta +
                cost.AttachTime +
                cost.PalletToParkingEta +
                cost.DetachTime;
            if (!Finite(amrAvailableAt) || amrAvailableAt > horizonEnd + Epsilon)
                return false;

            var dispatchAvailability = new DispatchAvailability(
                availability.Amr,
                availability.StartNode,
                availability.AvailableAt,
                availability.PriorAssignmentEta,
                isImmediate: false);
            candidate = new DispatchCandidate(
                dispatchAvailability,
                task,
                task.Pallet,
                null,
                task.SourceWorkstation,
                task.Pallet.ParkingNode,
                cost);
            estimate = new ExecutionEstimate(
                task.Pallet.ParkingNode,
                amrAvailableAt,
                null,
                0,
                task.SourceWorkstation,
                amrAvailableAt);
            return candidate.IsValid && estimate.IsValid;
        }

        private List<DispatchCandidate> SuppressCandidatesWorthWaitingFor(
            IReadOnlyList<DispatchCandidate> immediateCandidates,
            IReadOnlyList<ScheduledCandidate> futureAssignments)
        {
            var allowedCandidates = new List<DispatchCandidate>();
            foreach (var taskGroup in immediateCandidates.GroupBy(candidate => candidate.Task))
            {
                var bestImmediate = taskGroup.OrderBy(candidate => candidate.Score).First();
                var bestFuture = futureAssignments
                    .Where(assignment => assignment.Candidate.Task == taskGroup.Key)
                    .OrderBy(assignment => assignment.Candidate.Score)
                    .FirstOrDefault();

                if (bestFuture.IsValid &&
                    IsFutureImprovementEnough(bestImmediate.Score, bestFuture.Candidate.Score))
                {
                    continue;
                }

                allowedCandidates.AddRange(taskGroup);
            }

            return allowedCandidates;
        }

        private bool IsFutureImprovementEnough(double immediateCost, double futureCost)
        {
            var improvement = immediateCost - futureCost;
            if (improvement <= 0)
                return false;

            var baseline = Math.Max(0.0001, Math.Abs(immediateCost));
            var improvementPercent = improvement / baseline * 100.0;
            return improvementPercent >= Math.Max(0f, _options.waitImprovementPercent);
        }

        private static double NextAvailabilityAtOrAfter(VirtualState state, double time)
        {
            return state.AmrAvailabilities.Values
                .Where(availability => availability.AvailableAt >= time - Epsilon)
                .Select(availability => availability.AvailableAt)
                .DefaultIfEmpty(double.PositiveInfinity)
                .Min();
        }

        private static double NextAvailabilityAfter(VirtualState state, double time)
        {
            return state.AmrAvailabilities.Values
                .Where(availability => availability.AvailableAt > time + Epsilon)
                .Select(availability => availability.AvailableAt)
                .DefaultIfEmpty(double.PositiveInfinity)
                .Min();
        }

        private static bool Finite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private sealed class VirtualState
        {
            private readonly Dictionary<PalletLoadingPoint, double> _loadingAvailableAt = new();
            private readonly Dictionary<WorkstationDeliveryPoint, double> _workstationAvailableAt = new();

            public VirtualState(DispatchProblem problem)
            {
                Now = problem.Now;
                RemainingTasks = problem.Tasks.Where(task => task != null).ToList();
                AmrAvailabilities = new Dictionary<TaskPlanningAmr, VirtualAmrAvailability>();
                foreach (var future in problem.FutureAvailabilities)
                {
                    if (!future.IsValid || AmrAvailabilities.ContainsKey(future.Amr))
                        continue;

                    AmrAvailabilities[future.Amr] = new VirtualAmrAvailability(
                        future.Amr,
                        future.FinishNode,
                        problem.Now + Math.Max(0.0, future.PriorAssignmentEta),
                        Math.Max(0.0, future.PriorAssignmentEta));
                }

                InitializeBlockedWorkstations(problem);
            }

            public double Now { get; }
            public List<ITaskPlanningTask> RemainingTasks { get; }
            public Dictionary<TaskPlanningAmr, VirtualAmrAvailability> AmrAvailabilities { get; }

            public IEnumerable<VirtualAmrAvailability> AvailableAmrs(double time)
            {
                return AmrAvailabilities.Values
                    .Where(availability => availability.IsValid && availability.AvailableAt <= time + Epsilon);
            }

            public double LoadingAvailableAt(PalletLoadingPoint loadingPoint, double fallback)
            {
                return loadingPoint != null && _loadingAvailableAt.TryGetValue(loadingPoint, out var availableAt)
                    ? availableAt
                    : fallback;
            }

            public double WorkstationAvailableAt(WorkstationDeliveryPoint workstation, double fallback)
            {
                return workstation != null && _workstationAvailableAt.TryGetValue(workstation, out var availableAt)
                    ? availableAt
                    : fallback;
            }

            public void Apply(DispatchCandidate candidate, ExecutionEstimate estimate)
            {
                RemainingTasks.Remove(candidate.Task);
                AmrAvailabilities[candidate.Amr] = new VirtualAmrAvailability(
                    candidate.Amr,
                    estimate.FinishNode,
                    estimate.AmrAvailableAt,
                    Math.Max(0.0, estimate.AmrAvailableAt - Now));

                if (estimate.LoadingPoint != null)
                    _loadingAvailableAt[estimate.LoadingPoint] = Math.Max(
                        LoadingAvailableAt(estimate.LoadingPoint, estimate.LoadingPointAvailableAt),
                        estimate.LoadingPointAvailableAt);

                if (estimate.Workstation != null)
                {
                    var current = WorkstationAvailableAt(estimate.Workstation, 0);
                    _workstationAvailableAt[estimate.Workstation] = Finite(current)
                        ? Math.Max(current, estimate.WorkstationAvailableAt)
                        : estimate.WorkstationAvailableAt;
                }
            }

            private void InitializeBlockedWorkstations(DispatchProblem problem)
            {
                var workstationRemovalTasks = RemainingTasks
                    .OfType<PalletRemovalPlanningTask>()
                    .Where(task => task.SourceWorkstation != null)
                    .Select(task => task.SourceWorkstation)
                    .ToHashSet();
                if (workstationRemovalTasks.Count == 0)
                    return;

                foreach (var delivery in RemainingTasks.OfType<DeliveryPlanningTask>())
                {
                    if (delivery.Workstation == null ||
                        !workstationRemovalTasks.Contains(delivery.Workstation) ||
                        !delivery.Workstation.HasBlockingPalletFor(delivery.Pallet))
                        continue;

                    _workstationAvailableAt[delivery.Workstation] = double.PositiveInfinity;
                }
            }
        }

        private sealed class VirtualAmrAvailability
        {
            public VirtualAmrAvailability(TaskPlanningAmr amr, MapfNode startNode, double availableAt, double priorAssignmentEta)
            {
                Amr = amr;
                StartNode = startNode;
                AvailableAt = availableAt;
                PriorAssignmentEta = priorAssignmentEta;
            }

            public TaskPlanningAmr Amr { get; }
            public MapfNode StartNode { get; }
            public double AvailableAt { get; }
            public double PriorAssignmentEta { get; }
            public bool IsValid => Amr != null && StartNode != null && Finite(AvailableAt) && Finite(PriorAssignmentEta);
        }

        private readonly struct ExecutionEstimate
        {
            public readonly MapfNode FinishNode;
            public readonly double AmrAvailableAt;
            public readonly PalletLoadingPoint LoadingPoint;
            public readonly double LoadingPointAvailableAt;
            public readonly WorkstationDeliveryPoint Workstation;
            public readonly double WorkstationAvailableAt;

            public ExecutionEstimate(
                MapfNode finishNode,
                double amrAvailableAt,
                PalletLoadingPoint loadingPoint,
                double loadingPointAvailableAt,
                WorkstationDeliveryPoint workstation,
                double workstationAvailableAt)
            {
                FinishNode = finishNode;
                AmrAvailableAt = amrAvailableAt;
                LoadingPoint = loadingPoint;
                LoadingPointAvailableAt = loadingPointAvailableAt;
                Workstation = workstation;
                WorkstationAvailableAt = workstationAvailableAt;
            }

            public bool IsValid => FinishNode != null && Finite(AmrAvailableAt);
        }

        private readonly struct ScheduledCandidate
        {
            public readonly DispatchCandidate Candidate;
            public readonly ExecutionEstimate Estimate;

            public ScheduledCandidate(DispatchCandidate candidate, ExecutionEstimate estimate)
            {
                Candidate = candidate;
                Estimate = estimate;
            }

            public bool IsValid => Candidate.IsValid && Estimate.IsValid;
        }
    }
}
