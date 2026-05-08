using System;
using System.Collections.Generic;
using UnityEngine;

namespace TaskPlanning
{
    public sealed class TaskPlanningMes : MonoBehaviour
    {
        [SerializeField] private TaskScheduler scheduler;
        [SerializeField] private List<DeliveryTaskRequest> inspectorTasks = new();
        [SerializeField] private int[] selectedTaskIndexes = Array.Empty<int>();

        private int _generatedTaskNumber;

        private void Awake()
        {
            scheduler ??= FindAnyObjectByType<TaskScheduler>();
        }

        public TaskScheduler Scheduler => scheduler;

        [ContextMenu("Submit Selected Tasks")]
        public void SubmitSelectedTasks()
        {
            if (inspectorTasks.Count == 0)
            {
                Debug.LogWarning("MES has no inspector tasks configured.", this);
                return;
            }

            if (selectedTaskIndexes.Length == 0)
            {
                Debug.LogWarning("MES has no selected task indexes configured.", this);
                return;
            }

            foreach (var index in selectedTaskIndexes)
            {
                if (index < 0 || index >= inspectorTasks.Count)
                {
                    Debug.LogWarning($"MES task index {index} is outside configured task range 0..{inspectorTasks.Count - 1}.", this);
                    continue;
                }

                SubmitTask(inspectorTasks[index]);
            }
        }

        public void SubmitTask(DeliveryTaskRequest request)
        {
            if (request == null)
                throw new ArgumentNullException(nameof(request));

            scheduler ??= FindAnyObjectByType<TaskScheduler>();
            if (scheduler == null)
            {
                Debug.LogError("No TaskScheduler found for MES task submission.", this);
                return;
            }

            var taskId = string.IsNullOrWhiteSpace(request.taskId)
                ? $"MES-{++_generatedTaskNumber:0000}"
                : request.taskId.Trim();

            scheduler.EnqueueTask(new DeliveryTaskRequest(taskId, request.pallet, request.workstation));
        }
    }
}
