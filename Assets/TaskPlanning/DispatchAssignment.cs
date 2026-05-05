namespace TaskPlanning
{
    public readonly struct DispatchAssignment
    {
        public readonly bool IsValid;
        public readonly TaskPlanningAmr Amr;
        public readonly PalletMarker Pallet;
        public readonly PalletLoadingPoint LoadingPoint;
        public readonly double Score;

        public DispatchAssignment(TaskPlanningAmr amr, PalletMarker pallet, PalletLoadingPoint loadingPoint, double score)
        {
            IsValid = amr != null && pallet != null && loadingPoint != null;
            Amr = amr;
            Pallet = pallet;
            LoadingPoint = loadingPoint;
            Score = score;
        }
    }
}
