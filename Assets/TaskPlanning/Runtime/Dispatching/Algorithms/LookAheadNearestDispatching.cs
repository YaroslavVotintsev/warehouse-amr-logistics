using System;
using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public sealed class LookAheadNearestDispatching : NearestDispatching
    {
        private readonly float _waitForFutureImprovementPercent;

        public LookAheadNearestDispatching(float waitForFutureImprovementPercent)
        {
            _waitForFutureImprovementPercent = Math.Max(0f, waitForFutureImprovementPercent);
        }

        public override DispatchPlan Solve(DispatchProblem problem)
        {
            var allowedCandidates = new List<DispatchAssignment>();
            foreach (var task in problem.Tasks)
            {
                var immediateCandidates = BuildImmediateCandidates(problem, task);
                if (immediateCandidates.Count == 0)
                    continue;

                var bestImmediate = immediateCandidates.OrderBy(candidate => candidate.Score).First();
                var bestFuture = BestFutureCost(problem, task);
                if (bestFuture.IsFeasible &&
                    IsFutureImprovementEnough(bestImmediate.Score, bestFuture.TotalCost))
                {
                    continue;
                }

                allowedCandidates.AddRange(immediateCandidates);
            }

            return SelectGreedy(allowedCandidates);
        }

        private CostEvaluation BestFutureCost(DispatchProblem problem, ITaskPlanningTask task)
        {
            var best = CostEvaluation.Infeasible;
            foreach (var availability in problem.FutureAvailabilities)
            {
                if (!availability.IsValid)
                    continue;

                foreach (var cost in EvaluateFutureCosts(problem, task, availability))
                {
                    if (!cost.IsFeasible || cost.TotalCost >= best.TotalCost)
                        continue;

                    best = cost;
                }
            }

            return best;
        }

        private static IEnumerable<CostEvaluation> EvaluateFutureCosts(
            DispatchProblem problem,
            ITaskPlanningTask task,
            AmrFutureAvailability availability)
        {
            switch (task)
            {
                case DeliveryPlanningTask delivery:
                    var loadingPointResolution = problem.ResolveLoadingPoint(delivery.Pallet);
                    if (!loadingPointResolution.IsResolved)
                        yield break;

                    yield return problem.CostEvaluator.EvaluateFrom(
                        availability.FinishNode,
                        delivery,
                        loadingPointResolution.LoadingPoint,
                        availability.PriorAssignmentEta);
                    break;
                case PalletRemovalPlanningTask removal:
                    yield return problem.CostEvaluator.EvaluateFrom(
                        availability.FinishNode,
                        removal,
                        availability.PriorAssignmentEta);
                    break;
            }
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
