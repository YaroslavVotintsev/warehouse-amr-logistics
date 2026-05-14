using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public sealed class ScheduledDeliveryTaskBatch
    {
        private readonly List<DeliveryTaskRequest> _requests;

        public ScheduledDeliveryTaskBatch(float timestampSeconds, IEnumerable<DeliveryTaskRequest> requests)
        {
            TimestampSeconds = timestampSeconds;
            _requests = requests != null
                ? requests.Where(request => request != null).ToList()
                : new List<DeliveryTaskRequest>();
        }

        public float TimestampSeconds { get; }
        public IReadOnlyList<DeliveryTaskRequest> Requests => _requests;
    }
}
