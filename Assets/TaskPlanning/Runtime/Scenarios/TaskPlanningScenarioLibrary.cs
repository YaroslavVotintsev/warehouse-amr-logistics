using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace TaskPlanning
{
    public static class TaskPlanningScenarioLibrary
    {
        public static IReadOnlyList<TaskPlanningScenario> All()
        {
            return new[]
            {
                SideBayLoadingBottleneck(),
                SideBayLookAheadTrap()
            };
        }

        public static TaskPlanningScenario Get(TaskPlanningScenarioPreset preset)
        {
            return preset switch
            {
                TaskPlanningScenarioPreset.SideBayLoadingBottleneck => SideBayLoadingBottleneck(),
                TaskPlanningScenarioPreset.SideBayLookAheadTrap => SideBayLookAheadTrap(),
                _ => SideBayLoadingBottleneck()
            };
        }

        public static TaskPlanningScenario SideBayLoadingBottleneck()
        {
            var graph = LongSideBayCorridor(length: 18);
            var pallets = new[]
            {
                Pallet("Pallet_A", "101", "212", load: 8f),
                Pallet("Pallet_B", "102", "213", load: 8f),
                Pallet("Pallet_C", "103", "214", load: 8f),
                Pallet("Pallet_D", "104", "215", load: 8f),
                Pallet("Pallet_E", "105", "216", load: 8f)
            };
            var palletIds = pallets.Select(pallet => pallet.PalletId).ToArray();

            return new TaskPlanningScenario(
                "Side Bay Loading Bottleneck",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_0", 0, "200"),
                    Amr("AMR_1", 1, "202"),
                    Amr("AMR_2", 2, "204"),
                    Amr("AMR_3", 3, "206"),
                    Amr("AMR_4", 4, "208")
                },
                pallets,
                new[]
                {
                    LoadingPoint("110", "110", palletIds)
                },
                new[]
                {
                    Workstation("212", "212", "Pallet_A"),
                    Workstation("213", "213", "Pallet_B"),
                    Workstation("214", "214", "Pallet_C"),
                    Workstation("215", "215", "Pallet_D"),
                    Workstation("216", "216", "Pallet_E")
                },
                new[]
                {
                    Task(0f, "Pallet_A", "212", "Bottleneck_A"),
                    Task(0f, "Pallet_B", "213", "Bottleneck_B"),
                    Task(0f, "Pallet_C", "214", "Bottleneck_C"),
                    Task(0f, "Pallet_D", "215", "Bottleneck_D"),
                    Task(0f, "Pallet_E", "216", "Bottleneck_E")
                });
        }

        public static TaskPlanningScenario SideBayLookAheadTrap()
        {
            var graph = LongSideBayCorridor(length: 18);

            return new TaskPlanningScenario(
                "Side Bay LookAhead Trap",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Far", 0, "200"),
                    Amr("AMR_BusyCandidate", 1, "207")
                },
                new[]
                {
                    Pallet("Pallet_Primer", "108", "212", attach: 1f, detach: 1f, load: 6f, unload: 2f),
                    Pallet("Pallet_Target", "112", "216", attach: 1f, detach: 1f, load: 2f, unload: 2f)
                },
                new[]
                {
                    LoadingPoint("109", "109", "Pallet_Primer"),
                    LoadingPoint("113", "113", "Pallet_Target")
                },
                new[]
                {
                    Workstation("212", "212", "Pallet_Primer"),
                    Workstation("216", "216", "Pallet_Target")
                },
                new[]
                {
                    Task(0f, "Pallet_Primer", "212", "PrimeBusyAmr"),
                    Task(1f, "Pallet_Target", "216", "TrapTarget")
                });
        }

        private static CorridorGraph LongSideBayCorridor(int length)
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
