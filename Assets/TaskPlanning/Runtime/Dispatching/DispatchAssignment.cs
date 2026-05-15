namespace TaskPlanning
{
    public readonly struct DispatchAssignment
    {
        public readonly bool IsValid;
        public readonly TaskPlanningAmr Amr;
        public readonly ITaskPlanningTask Task;
        public readonly PalletMarker Pallet;
        public readonly PalletLoadingPoint LoadingPoint;
        public readonly WorkstationDeliveryPoint Workstation;
        public readonly Mapf.Authoring.MapfNode RemovalTargetNode;
        public readonly CostEvaluation Cost;
        public readonly SoftReassignmentOption SoftReassignment;

        public double Score => Cost.TotalCost;
        public bool ReplacesActiveAssignment => SoftReassignment.IsValid;

        public DispatchAssignment(
            TaskPlanningAmr amr,
            ITaskPlanningTask task,
            PalletMarker pallet,
            PalletLoadingPoint loadingPoint,
            WorkstationDeliveryPoint workstation,
            Mapf.Authoring.MapfNode removalTargetNode,
            CostEvaluation cost,
            SoftReassignmentOption softReassignment = default)
        {
            IsValid = amr != null && task != null && pallet != null;
            Amr = amr;
            Task = task;
            Pallet = pallet;
            LoadingPoint = loadingPoint;
            Workstation = workstation;
            RemovalTargetNode = removalTargetNode;
            Cost = cost;
            SoftReassignment = softReassignment;
        }
    }
}
