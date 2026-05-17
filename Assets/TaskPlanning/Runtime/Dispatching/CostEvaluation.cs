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
        public readonly double ReassignmentPenalty;

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
            double removalPriorityMultiplier = 1,
            double reassignmentPenalty = 0)
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
            ReassignmentPenalty = reassignmentPenalty;
        }

        public CostEvaluation WithReassignmentPenalty(double reassignmentPenalty)
        {
            if (!IsFeasible)
                return this;

            var penalty = System.Math.Max(0.0, reassignmentPenalty);
            return new CostEvaluation(
                true,
                TotalCost + penalty,
                priorAssignmentEta: PriorAssignmentEta,
                amrToPalletEta: AmrToPalletEta,
                attachTime: AttachTime,
                loadingQueueEta: LoadingQueueEta,
                palletToLoadingEta: PalletToLoadingEta,
                loadTime: LoadTime,
                loadingToWorkstationEta: LoadingToWorkstationEta,
                detachTime: DetachTime,
                palletToParkingEta: PalletToParkingEta,
                blockedDeliveryBias: BlockedDeliveryBias,
                agingBonus: AgingBonus,
                removalPriorityMultiplier: RemovalPriorityMultiplier,
                reassignmentPenalty: ReassignmentPenalty + penalty);
        }

        public static CostEvaluation Infeasible => new(false, double.PositiveInfinity);
    }
}
