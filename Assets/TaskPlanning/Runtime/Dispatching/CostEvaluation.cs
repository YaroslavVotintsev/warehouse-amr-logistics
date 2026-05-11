namespace TaskPlanning
{
    public readonly struct CostEvaluation
    {
        public readonly bool IsFeasible;
        public readonly double TotalCost;
        public readonly double PriorAssignmentEta;
        public readonly double AmrToPalletEta;
        public readonly double AttachTime;
        public readonly double LoadingQueueEta;
        public readonly double PalletToLoadingEta;
        public readonly double LoadTime;
        public readonly double LoadingToWorkstationEta;
        public readonly double DetachTime;
        public readonly double PalletToParkingEta;
        public readonly double BlockedDeliveryBias;
        public readonly double AgingBonus;
        public readonly double RemovalPriorityMultiplier;

        public CostEvaluation(
            bool isFeasible,
            double totalCost,
            double priorAssignmentEta = 0,
            double amrToPalletEta = 0,
            double attachTime = 0,
            double loadingQueueEta = 0,
            double palletToLoadingEta = 0,
            double loadTime = 0,
            double loadingToWorkstationEta = 0,
            double detachTime = 0,
            double palletToParkingEta = 0,
            double blockedDeliveryBias = 0,
            double agingBonus = 0,
            double removalPriorityMultiplier = 1)
        {
            IsFeasible = isFeasible;
            TotalCost = totalCost;
            PriorAssignmentEta = priorAssignmentEta;
            AmrToPalletEta = amrToPalletEta;
            AttachTime = attachTime;
            LoadingQueueEta = loadingQueueEta;
            PalletToLoadingEta = palletToLoadingEta;
            LoadTime = loadTime;
            LoadingToWorkstationEta = loadingToWorkstationEta;
            DetachTime = detachTime;
            PalletToParkingEta = palletToParkingEta;
            BlockedDeliveryBias = blockedDeliveryBias;
            AgingBonus = agingBonus;
            RemovalPriorityMultiplier = removalPriorityMultiplier;
        }

        public static CostEvaluation Infeasible => new(false, double.PositiveInfinity);
    }
}
