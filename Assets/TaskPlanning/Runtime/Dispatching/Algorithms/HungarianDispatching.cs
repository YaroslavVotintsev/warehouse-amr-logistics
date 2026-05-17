using System;
using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public sealed class HungarianDispatching : ITaskDispatchAlgorithm
    {
        public DispatchPlan Solve(DispatchProblem problem)
        {
            var candidates = (problem.Candidates ?? Array.Empty<DispatchCandidate>())
                .Where(IsUsableCandidate)
                .ToArray();
            if (candidates.Length == 0)
                return new DispatchPlan(Array.Empty<DispatchAssignment>());

            var amrs = candidates
                .Select(candidate => candidate.Amr)
                .Distinct()
                .OrderBy(amr => amr != null ? amr.AmrId : string.Empty, StringComparer.Ordinal)
                .ToArray();
            var pallets = candidates
                .Select(candidate => candidate.Pallet)
                .Distinct()
                .OrderBy(pallet => pallet != null ? pallet.PalletId : string.Empty, StringComparer.Ordinal)
                .ToArray();

            if (amrs.Length == 0 || pallets.Length == 0)
                return new DispatchPlan(Array.Empty<DispatchAssignment>());

            var bestByPair = BuildBestCandidateLookup(candidates);
            var matrix = BuildCostMatrix(amrs, pallets, bestByPair);
            var assignment = SolveMinimumCostAssignment(matrix.Costs);
            var selected = new List<DispatchAssignment>();
            var usedTasks = new HashSet<ITaskPlanningTask>();
            var usedPallets = new HashSet<PalletMarker>();

            for (var row = 0; row < amrs.Length && row < assignment.Length; row++)
            {
                var column = assignment[row];
                if (column < 0 || column >= pallets.Length)
                    continue;

                var key = new CandidateKey(amrs[row], pallets[column]);
                if (!bestByPair.TryGetValue(key, out var candidate))
                    continue;

                if (candidate.Score >= matrix.InvalidCost ||
                    usedTasks.Contains(candidate.Task) ||
                    usedPallets.Contains(candidate.Pallet))
                    continue;

                usedTasks.Add(candidate.Task);
                usedPallets.Add(candidate.Pallet);
                selected.Add(candidate.ToAssignment());
            }

            return new DispatchPlan(selected
                .OrderBy(assignment => assignment.Task.EnqueuedTime)
                .ThenBy(assignment => assignment.Task.TaskId, StringComparer.Ordinal)
                .ThenBy(assignment => assignment.Amr != null ? assignment.Amr.AmrId : string.Empty, StringComparer.Ordinal)
                .ToArray());
        }

        private static bool IsUsableCandidate(DispatchCandidate candidate)
        {
            return candidate.IsValid &&
                candidate.Amr != null &&
                candidate.Task != null &&
                candidate.Pallet != null &&
                IsFinite(candidate.Score);
        }

        private static Dictionary<CandidateKey, DispatchCandidate> BuildBestCandidateLookup(
            IReadOnlyList<DispatchCandidate> candidates)
        {
            var bestByPair = new Dictionary<CandidateKey, DispatchCandidate>();
            foreach (var candidate in candidates)
            {
                var key = new CandidateKey(candidate.Amr, candidate.Pallet);
                if (!bestByPair.TryGetValue(key, out var current) || IsBetterCandidate(candidate, current))
                    bestByPair[key] = candidate;
            }

            return bestByPair;
        }

        private static bool IsBetterCandidate(DispatchCandidate candidate, DispatchCandidate current)
        {
            var costComparison = candidate.Score.CompareTo(current.Score);
            if (costComparison != 0)
                return costComparison < 0;

            if (candidate.ReplacesActiveAssignment != current.ReplacesActiveAssignment)
                return !candidate.ReplacesActiveAssignment;

            var enqueueComparison = candidate.Task.EnqueuedTime.CompareTo(current.Task.EnqueuedTime);
            if (enqueueComparison != 0)
                return enqueueComparison < 0;

            var taskComparison = string.Compare(candidate.Task.TaskId, current.Task.TaskId, StringComparison.Ordinal);
            if (taskComparison != 0)
                return taskComparison < 0;

            return string.Compare(
                candidate.Amr != null ? candidate.Amr.AmrId : string.Empty,
                current.Amr != null ? current.Amr.AmrId : string.Empty,
                StringComparison.Ordinal) < 0;
        }

        private static MatrixBuildResult BuildCostMatrix(
            IReadOnlyList<TaskPlanningAmr> amrs,
            IReadOnlyList<PalletMarker> pallets,
            IReadOnlyDictionary<CandidateKey, DispatchCandidate> bestByPair)
        {
            var realAssignmentLimit = Math.Min(amrs.Count, pallets.Count);
            var maxAbsCost = bestByPair.Values
                .Select(candidate => Math.Abs(candidate.Score))
                .DefaultIfEmpty(1.0)
                .Max();
            var assignmentReward = Math.Max(1.0, maxAbsCost) * (realAssignmentLimit + 1) + 1.0;
            var invalidCost = assignmentReward * (amrs.Count + pallets.Count + 1) + maxAbsCost + 1.0;
            var size = amrs.Count + pallets.Count;
            var costs = new double[size, size];

            for (var row = 0; row < size; row++)
            {
                for (var column = 0; column < size; column++)
                    costs[row, column] = 0.0;
            }

            for (var row = 0; row < amrs.Count; row++)
            {
                for (var column = 0; column < pallets.Count; column++)
                {
                    var key = new CandidateKey(amrs[row], pallets[column]);
                    costs[row, column] = bestByPair.TryGetValue(key, out var candidate)
                        ? candidate.Score - assignmentReward
                        : invalidCost;
                }
            }

            return new MatrixBuildResult(costs, invalidCost);
        }

        private static int[] SolveMinimumCostAssignment(double[,] costs)
        {
            var size = costs.GetLength(0);
            var u = new double[size + 1];
            var v = new double[size + 1];
            var p = new int[size + 1];
            var way = new int[size + 1];

            for (var i = 1; i <= size; i++)
            {
                p[0] = i;
                var j0 = 0;
                var minv = new double[size + 1];
                var used = new bool[size + 1];
                for (var j = 0; j <= size; j++)
                    minv[j] = double.PositiveInfinity;

                do
                {
                    used[j0] = true;
                    var i0 = p[j0];
                    var delta = double.PositiveInfinity;
                    var j1 = 0;

                    for (var j = 1; j <= size; j++)
                    {
                        if (used[j])
                            continue;

                        var current = costs[i0 - 1, j - 1] - u[i0] - v[j];
                        if (current < minv[j])
                        {
                            minv[j] = current;
                            way[j] = j0;
                        }

                        if (minv[j] < delta)
                        {
                            delta = minv[j];
                            j1 = j;
                        }
                    }

                    for (var j = 0; j <= size; j++)
                    {
                        if (used[j])
                        {
                            u[p[j]] += delta;
                            v[j] -= delta;
                        }
                        else
                        {
                            minv[j] -= delta;
                        }
                    }

                    j0 = j1;
                }
                while (p[j0] != 0);

                do
                {
                    var j1 = way[j0];
                    p[j0] = p[j1];
                    j0 = j1;
                }
                while (j0 != 0);
            }

            var assignment = Enumerable.Repeat(-1, size).ToArray();
            for (var j = 1; j <= size; j++)
            {
                if (p[j] != 0)
                    assignment[p[j] - 1] = j - 1;
            }

            return assignment;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private readonly struct CandidateKey : IEquatable<CandidateKey>
        {
            private readonly TaskPlanningAmr _amr;
            private readonly PalletMarker _pallet;

            public CandidateKey(TaskPlanningAmr amr, PalletMarker pallet)
            {
                _amr = amr;
                _pallet = pallet;
            }

            public bool Equals(CandidateKey other)
            {
                return _amr == other._amr && _pallet == other._pallet;
            }

            public override bool Equals(object obj)
            {
                return obj is CandidateKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    return ((_amr != null ? _amr.GetHashCode() : 0) * 397) ^
                        (_pallet != null ? _pallet.GetHashCode() : 0);
                }
            }
        }

        private readonly struct MatrixBuildResult
        {
            public MatrixBuildResult(double[,] costs, double invalidCost)
            {
                Costs = costs;
                InvalidCost = invalidCost;
            }

            public double[,] Costs { get; }
            public double InvalidCost { get; }
        }
    }
}
