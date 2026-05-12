using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public sealed class ScheduledDeliveryTaskBatch
    {
        public ScheduledDeliveryTaskBatch(float timestampSeconds, IEnumerable<DeliveryTaskRequest> requests)
        {
            TimestampSeconds = timestampSeconds;
            Requests = requests != null
                ? requests.Where(request => request != null).ToArray()
                : System.Array.Empty<DeliveryTaskRequest>();
        }

        public float TimestampSeconds { get; }
        public IReadOnlyList<DeliveryTaskRequest> Requests { get; }
    }
}
