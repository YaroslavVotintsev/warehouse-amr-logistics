using System;
using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public sealed class LookAheadFuturePolicy : ITaskPlanningFuturePolicy
    {
        private readonly float _waitForFutureImprovementPercent;

        public LookAheadFuturePolicy(float waitForFutureImprovementPercent)
        {
            _waitForFutureImprovementPercent = Math.Max(0f, waitForFutureImprovementPercent);
        }

        public DispatchPlan Solve(DispatchProblem problem, ITaskDispatchAlgorithm baseDispatcher)
        {
            var immediateCandidates = DispatchCandidateBuilder.BuildImmediateCandidates(problem);
            var futureCandidates = DispatchCandidateBuilder.BuildFutureCandidates(problem);
            var allowedCandidates = new List<DispatchCandidate>();

            foreach (var taskGroup in immediateCandidates.GroupBy(candidate => candidate.Task))
            {
                var bestImmediate = taskGroup.OrderBy(candidate => candidate.Score).First();
                var bestFuture = futureCandidates
                    .Where(candidate => candidate.Task == taskGroup.Key)
                    .OrderBy(candidate => candidate.Score)
                    .FirstOrDefault();
                if (bestFuture.IsValid &&
                    IsFutureImprovementEnough(bestImmediate.Score, bestFuture.Score))
                {
                    continue;
                }

                allowedCandidates.AddRange(taskGroup);
            }

            var lookAheadProblem = problem.WithCandidates(allowedCandidates);
            return baseDispatcher.Solve(lookAheadProblem);
        }

        private bool IsFutureImprovementEnough(double immediateCost, double futureCost)
        {
            var improvement = immediateCost - futureCost;
            if (improvement <= 0)
                return false;

            var baseline = Math.Max(0.0001, Math.Abs(immediateCost));
            var improvementPercent = improvement / baseline * 100.0;
            return improvementPercent >= _waitForFutureImprovementPercent;
        }
    }
}
