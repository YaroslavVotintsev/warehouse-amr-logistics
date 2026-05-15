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
                FifoAssignmentTrap(),
                FutureWaitTrap(),
                FutureCapacityTrap()
            };
        }

        public static TaskPlanningScenario Get(TaskPlanningScenarioPreset preset)
        {
            return preset switch
            {
                TaskPlanningScenarioPreset.FifoAssignmentTrap => FifoAssignmentTrap(),
                TaskPlanningScenarioPreset.FutureWaitTrap => FutureWaitTrap(),
                TaskPlanningScenarioPreset.FutureCapacityTrap => FutureCapacityTrap(),
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

        public static TaskPlanningScenario FutureWaitTrap()
        {
            var graph = LongCorridorWithBays(length: 34);

            return new TaskPlanningScenario(
                "Future Wait Trap",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Remote", 0, "200"),
                    Amr("AMR_Future", 1, "224")
                },
                new[]
                {
                    Pallet("Pallet_Primer", "124", "226", attach: 1f, detach: 1f, load: 6f, unload: 1f),
                    Pallet("Pallet_Target", "127", "230", attach: 1f, detach: 1f, load: 1f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("125", "125", "Pallet_Primer"),
                    LoadingPoint("128", "128", "Pallet_Target")
                },
                new[]
                {
                    Workstation("226", "226", "Pallet_Primer"),
                    Workstation("230", "230", "Pallet_Target")
                },
                new[]
                {
                    Task(0f, "Pallet_Primer", "226", "PrimeFutureAmr"),
                    Task(1f, "Pallet_Target", "230", "TargetNearFutureFinish")
                });
        }

        public static TaskPlanningScenario FutureCapacityTrap()
        {
            var graph = LongCorridorWithBays(length: 42);

            return new TaskPlanningScenario(
                "Future Capacity Trap",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Remote", 0, "200"),
                    Amr("AMR_Future_A", 1, "226"),
                    Amr("AMR_Future_B", 2, "232")
                },
                new[]
                {
                    Pallet("Pallet_Primer_A", "126", "228", attach: 1f, detach: 1f, load: 8f, unload: 1f),
                    Pallet("Pallet_Primer_B", "132", "234", attach: 1f, detach: 1f, load: 12f, unload: 1f),
                    Pallet("Pallet_Target_A", "123", "225", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Target_B", "129", "229", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Target_C", "130", "232", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Target_D", "134", "236", attach: 1f, detach: 1f, load: 1f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("127", "127", "Pallet_Primer_A"),
                    LoadingPoint("133", "133", "Pallet_Primer_B"),
                    LoadingPoint("124", "124", "Pallet_Target_A"),
                    LoadingPoint("128", "128", "Pallet_Target_B"),
                    LoadingPoint("131", "131", "Pallet_Target_C"),
                    LoadingPoint("135", "135", "Pallet_Target_D")
                },
                new[]
                {
                    Workstation("228", "228", "Pallet_Primer_A"),
                    Workstation("234", "234", "Pallet_Primer_B"),
                    Workstation("225", "225", "Pallet_Target_A"),
                    Workstation("229", "229", "Pallet_Target_B"),
                    Workstation("232", "232", "Pallet_Target_C"),
                    Workstation("236", "236", "Pallet_Target_D")
                },
                new[]
                {
                    Task(0f, "Pallet_Primer_A", "228", "PrimeFutureA"),
                    Task(0f, "Pallet_Primer_B", "234", "PrimeFutureB"),
                    Task(1f, "Pallet_Target_A", "225", "CapacityTargetA"),
                    Task(1f, "Pallet_Target_B", "229", "CapacityTargetB"),
                    Task(1f, "Pallet_Target_C", "232", "CapacityTargetC"),
                    Task(1f, "Pallet_Target_D", "236", "CapacityTargetD")
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
