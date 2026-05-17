namespace TaskPlanning
{
    public readonly struct SoftReassignmentOption
    {
        public readonly TaskPlanningAmr ActiveAmr;
        public readonly ITaskPlanningTask ActiveTask;
        public readonly PalletMarker ActivePallet;
        public readonly ITaskPlanningTask ReplacementTask;
        public readonly CostEvaluation ActiveCost;
        public readonly CostEvaluation ReplacementBaseCost;
        public readonly CostEvaluation ReplacementCost;
        public readonly double ReassignmentTravelTimeSeconds;
        public readonly double ReassignmentPenalty;
        public readonly double ImprovementPercent;

        public SoftReassignmentOption(
            DispatchAssignment activeAssignment,
            ITaskPlanningTask replacementTask,
            CostEvaluation activeCost,
            CostEvaluation replacementCost,
            double improvementPercent)
            : this(
                activeAssignment,
                replacementTask,
                activeCost,
                replacementCost,
                replacementCost,
                0,
                0,
                improvementPercent)
        {
        }

        public SoftReassignmentOption(
            DispatchAssignment activeAssignment,
            ITaskPlanningTask replacementTask,
            CostEvaluation activeCost,
            CostEvaluation replacementBaseCost,
            CostEvaluation replacementCost,
            double reassignmentTravelTimeSeconds,
            double reassignmentPenalty,
            double improvementPercent)
        {
            ActiveAmr = activeAssignment.Amr;
            ActiveTask = activeAssignment.Task;
            ActivePallet = activeAssignment.Pallet;
            ReplacementTask = replacementTask;
            ActiveCost = activeCost;
            ReplacementBaseCost = replacementBaseCost;
            ReplacementCost = replacementCost;
            ReassignmentTravelTimeSeconds = reassignmentTravelTimeSeconds;
            ReassignmentPenalty = reassignmentPenalty;
            ImprovementPercent = improvementPercent;
        }

        public bool IsValid =>
            ActiveAmr != null &&
            ActiveTask != null &&
            ActivePallet != null &&
            ReplacementTask != null &&
            ActiveCost.IsFeasible &&
            ReplacementCost.IsFeasible &&
            ImprovementPercent >= 0;

        public TaskPlanningAmr Amr => ActiveAmr;
    }
}
