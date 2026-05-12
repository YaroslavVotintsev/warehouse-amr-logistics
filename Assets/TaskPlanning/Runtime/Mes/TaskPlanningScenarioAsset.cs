using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace TaskPlanning
{
    [CreateAssetMenu(
        fileName = "TaskPlanningScenario",
        menuName = "Task Planning/MES Scheduled Scenario")]
    public sealed class TaskPlanningScenarioAsset : ScriptableObject
    {
        [SerializeField] private List<ScheduledMesTask> scheduledTasks = new();

        public IReadOnlyList<ScheduledMesTask> ScheduledTasks => scheduledTasks;

        public static TaskPlanningScenarioAsset Create(IEnumerable<ScheduledMesTask> tasks)
        {
            var scenario = CreateInstance<TaskPlanningScenarioAsset>();
            scenario.SetTasks(tasks);
            return scenario;
        }

        public void SetTasks(IEnumerable<ScheduledMesTask> tasks)
        {
            scheduledTasks.Clear();
            if (tasks == null)
                return;

            foreach (var task in tasks)
            {
                if (task == null)
                    continue;

                scheduledTasks.Add(task.Clone());
            }
        }

        public void AddTask(float timestampSeconds, string palletId, string workstationId, string taskId = null)
        {
            scheduledTasks.Add(new ScheduledMesTask(timestampSeconds, palletId, workstationId, taskId));
        }

        public void ClearTasks()
        {
            scheduledTasks.Clear();
        }

        public IReadOnlyList<ScheduledMesTask> OrderedTasks()
        {
            return scheduledTasks
                .Where(task => task != null)
                .Select((task, index) => new { Task = task, Index = index })
                .OrderBy(item => Mathf.Max(0f, item.Task.timestampSeconds))
                .ThenBy(item => item.Index)
                .Select(item => item.Task)
                .ToArray();
        }
    }
}
