using System;
using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public sealed class RegretDispatching : ITaskDispatchAlgorithm
    {
        private const double SingleCandidateRegret = double.PositiveInfinity;

        public DispatchPlan Solve(DispatchProblem problem)
        {
            var selected = new List<DispatchAssignment>();
            var usedAmrs = new HashSet<TaskPlanningAmr>();
            var usedTasks = new HashSet<ITaskPlanningTask>();
            var usedPallets = new HashSet<PalletMarker>();

            while (true)
            {
                var choice = ChooseNext(problem.Candidates, usedAmrs, usedTasks, usedPallets);
                if (!choice.IsValid)
                    break;

                usedAmrs.Add(choice.Candidate.Amr);
                usedTasks.Add(choice.Candidate.Task);
                usedPallets.Add(choice.Candidate.Pallet);
                selected.Add(choice.Candidate.ToAssignment());
            }

            return new DispatchPlan(selected);
        }

        private static RegretChoice ChooseNext(
            IReadOnlyList<DispatchCandidate> candidates,
            HashSet<TaskPlanningAmr> usedAmrs,
            HashSet<ITaskPlanningTask> usedTasks,
            HashSet<PalletMarker> usedPallets)
        {
            var bestChoice = default(RegretChoice);
            foreach (var taskGroup in candidates
                         .Where(candidate => IsAvailable(candidate, usedAmrs, usedTasks, usedPallets))
                         .GroupBy(candidate => candidate.Task))
            {
                var ordered = taskGroup
                    .OrderBy(candidate => candidate.Score)
                    .ThenBy(candidate => candidate.ReplacesActiveAssignment)
                    .ThenBy(candidate => candidate.Amr != null ? candidate.Amr.AmrId : string.Empty, StringComparer.Ordinal)
                    .ToArray();
                if (ordered.Length == 0)
                    continue;

                var best = ordered[0];
                var regret = ordered.Length == 1
                    ? SingleCandidateRegret
                    : Math.Max(0.0, ordered[1].Score - ordered[0].Score);
                var choice = new RegretChoice(best, regret);
                if (!bestChoice.IsValid || IsBetter(choice, bestChoice))
                    bestChoice = choice;
            }

            return bestChoice;
        }

        private static bool IsAvailable(
            DispatchCandidate candidate,
            HashSet<TaskPlanningAmr> usedAmrs,
            HashSet<ITaskPlanningTask> usedTasks,
            HashSet<PalletMarker> usedPallets)
        {
            return candidate.IsValid &&
                candidate.Amr != null &&
                candidate.Task != null &&
                candidate.Pallet != null &&
                !usedAmrs.Contains(candidate.Amr) &&
                !usedTasks.Contains(candidate.Task) &&
                !usedPallets.Contains(candidate.Pallet);
        }

        private static bool IsBetter(RegretChoice candidate, RegretChoice current)
        {
            var regretComparison = CompareDescending(candidate.Regret, current.Regret);
            if (regretComparison != 0)
                return regretComparison < 0;

            var costComparison = candidate.Candidate.Score.CompareTo(current.Candidate.Score);
            if (costComparison != 0)
                return costComparison < 0;

            var enqueueComparison = candidate.Candidate.Task.EnqueuedTime.CompareTo(current.Candidate.Task.EnqueuedTime);
            if (enqueueComparison != 0)
                return enqueueComparison < 0;

            var taskComparison = string.Compare(
                candidate.Candidate.Task.TaskId,
                current.Candidate.Task.TaskId,
                StringComparison.Ordinal);
            if (taskComparison != 0)
                return taskComparison < 0;

            if (candidate.Candidate.ReplacesActiveAssignment != current.Candidate.ReplacesActiveAssignment)
                return !candidate.Candidate.ReplacesActiveAssignment;

            return string.Compare(
                candidate.Candidate.Amr != null ? candidate.Candidate.Amr.AmrId : string.Empty,
                current.Candidate.Amr != null ? current.Candidate.Amr.AmrId : string.Empty,
                StringComparison.Ordinal) < 0;
        }

        private static int CompareDescending(double left, double right)
        {
            if (double.IsPositiveInfinity(left) && double.IsPositiveInfinity(right))
                return 0;

            if (double.IsPositiveInfinity(left))
                return -1;

            if (double.IsPositiveInfinity(right))
                return 1;

            return right.CompareTo(left);
        }

        private readonly struct RegretChoice
        {
            public RegretChoice(DispatchCandidate candidate, double regret)
            {
                Candidate = candidate;
                Regret = regret;
            }

            public DispatchCandidate Candidate { get; }
            public double Regret { get; }
            public bool IsValid => Candidate.IsValid;
        }
    }
}
