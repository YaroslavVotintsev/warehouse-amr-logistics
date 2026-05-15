namespace TaskPlanning
{
    public readonly struct SoftReassignmentOption
    {
        public readonly TaskPlanningAmr ActiveAmr;
        public readonly ITaskPlanningTask ActiveTask;
        public readonly PalletMarker ActivePallet;
        public readonly ITaskPlanningTask ReplacementTask;
        public readonly CostEvaluation ActiveCost;
        public readonly CostEvaluation ReplacementCost;
        public readonly double ImprovementPercent;

        public SoftReassignmentOption(
            DispatchAssignment activeAssignment,
            ITaskPlanningTask replacementTask,
            CostEvaluation activeCost,
            CostEvaluation replacementCost,
            double improvementPercent)
        {
            ActiveAmr = activeAssignment.Amr;
            ActiveTask = activeAssignment.Task;
            ActivePallet = activeAssignment.Pallet;
            ReplacementTask = replacementTask;
            ActiveCost = activeCost;
            ReplacementCost = replacementCost;
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
