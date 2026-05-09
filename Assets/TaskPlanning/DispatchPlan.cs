using System.Collections.Generic;

namespace TaskPlanning
{
    public sealed class DispatchPlan
    {
        public IReadOnlyList<DispatchAssignment> Assignments { get; }

        public DispatchPlan(IReadOnlyList<DispatchAssignment> assignments)
        {
            Assignments = assignments;
        }
    }
}
