using System;
using UnityEngine;

namespace TaskPlanning
{
    [Serializable]
    public sealed class ScheduledMesTask
    {
        [Min(0f)] public float timestampSeconds;
        public string taskId;
        public string palletId;
        public string workstationId;

        public ScheduledMesTask()
        {
        }

        public ScheduledMesTask(float timestampSeconds, string palletId, string workstationId, string taskId = null)
        {
            this.timestampSeconds = Mathf.Max(0f, timestampSeconds);
            this.taskId = taskId;
            this.palletId = palletId;
            this.workstationId = workstationId;
        }

        public ScheduledMesTask Clone()
        {
            return new ScheduledMesTask(timestampSeconds, palletId, workstationId, taskId);
        }
    }
}
