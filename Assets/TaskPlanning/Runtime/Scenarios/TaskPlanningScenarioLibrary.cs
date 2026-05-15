using System.Collections.Generic;
using UnityEngine;

namespace TaskPlanning
{
    public static class TaskPlanningScenarioLibrary
    {
        public static IReadOnlyList<TaskPlanningScenario> All()
        {
            return new[]
            {
                FifoAssignmentTrap()
            };
        }

        public static TaskPlanningScenario Get(TaskPlanningScenarioPreset preset)
        {
            return preset switch
            {
                TaskPlanningScenarioPreset.FifoAssignmentTrap => FifoAssignmentTrap(),
                _ => FifoAssignmentTrap()
            };
        }

        public static TaskPlanningScenario FifoAssignmentTrap()
        {
            var graph = LongCorridorWithBays(length: 22);

            return new TaskPlanningScenario(
                "FIFO Assignment Trap",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Left", 0, "200"),
                    Amr("AMR_Right", 1, "219")
                },
                new[]
                {
                    Pallet("Pallet_Old", "109", "211", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_New", "100", "202", attach: 1f, detach: 1f, load: 1f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("110", "110", "Pallet_Old"),
                    LoadingPoint("101", "101", "Pallet_New")
                },
                new[]
                {
                    Workstation("211", "211", "Pallet_Old"),
                    Workstation("202", "202", "Pallet_New")
                },
                new[]
                {
                    Task(0f, "Pallet_Old", "211", "FifoTrap_OldFirst"),
                    Task(0f, "Pallet_New", "202", "FifoTrap_NeedsLeftAmr")
                });
        }

        private static CorridorGraph LongCorridorWithBays(int length)
        {
            var nodes = new List<TaskPlanningScenarioNode>();
            var edges = new List<TaskPlanningScenarioEdge>();

            for (var i = 0; i <= length; i++)
            {
                nodes.Add(Node(i.ToString(), i, 0));
                if (i > 0)
                    edges.Add(Edge((i - 1).ToString(), i.ToString()));
            }

            for (var i = 0; i <= length; i++)
            {
                var north = (100 + i).ToString();
                var south = (200 + i).ToString();
                nodes.Add(Node(north, i, 1));
                nodes.Add(Node(south, i, -1));
                edges.Add(Edge(i.ToString(), north));
                edges.Add(Edge(i.ToString(), south));
            }

            return new CorridorGraph(nodes, edges);
        }

        private static TaskPlanningScenarioNode Node(string id, float x, float y)
        {
            return new TaskPlanningScenarioNode(id, new Vector2(x, y));
        }

        private static TaskPlanningScenarioEdge Edge(string a, string b)
        {
            return new TaskPlanningScenarioEdge(a, b);
        }

        private static TaskPlanningScenarioAmr Amr(string id, int agentId, string startNodeId)
        {
            return new TaskPlanningScenarioAmr(id, agentId, startNodeId);
        }

        private static TaskPlanningScenarioPallet Pallet(
            string id,
            string currentNodeId,
            string parkingNodeId,
            float attach = 1f,
            float detach = 1f,
            float load = 3f,
            float unload = 3f)
        {
            return new TaskPlanningScenarioPallet(id, currentNodeId, parkingNodeId, attach, detach, load, unload);
        }

        private static TaskPlanningScenarioLoadingPoint LoadingPoint(
            string id,
            string nodeId,
            params string[] acceptedPalletIds)
        {
            return new TaskPlanningScenarioLoadingPoint(id, nodeId, acceptedPalletIds);
        }

        private static TaskPlanningScenarioLoadingPoint LoadingPoint(
            string id,
            string nodeId,
            IReadOnlyList<string> acceptedPalletIds)
        {
            return new TaskPlanningScenarioLoadingPoint(id, nodeId, acceptedPalletIds);
        }

        private static TaskPlanningScenarioWorkstation Workstation(
            string id,
            string nodeId,
            params string[] acceptedPalletIds)
        {
            return new TaskPlanningScenarioWorkstation(id, nodeId, acceptedPalletIds);
        }

        private static ScheduledMesTask Task(float timestamp, string palletId, string workstationId, string taskId)
        {
            return new ScheduledMesTask(timestamp, palletId, workstationId, taskId);
        }

        private readonly struct CorridorGraph
        {
            public readonly IReadOnlyList<TaskPlanningScenarioNode> Nodes;
            public readonly IReadOnlyList<TaskPlanningScenarioEdge> Edges;

            public CorridorGraph(
                IReadOnlyList<TaskPlanningScenarioNode> nodes,
                IReadOnlyList<TaskPlanningScenarioEdge> edges)
            {
                Nodes = nodes;
                Edges = edges;
            }
        }
    }
}
