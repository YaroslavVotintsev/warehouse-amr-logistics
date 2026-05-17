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
                FutureCapacityTrap(),
                RollingHorizonCapacitySaturation(),
                RegretAssignmentTrap(),
                HungarianAssignmentTrap(),
                RegretDecoyTrap(),
                GlobalAssignmentTrap()
            };
        }

        public static TaskPlanningScenario Get(TaskPlanningScenarioPreset preset)
        {
            return preset switch
            {
                TaskPlanningScenarioPreset.FifoAssignmentTrap => FifoAssignmentTrap(),
                TaskPlanningScenarioPreset.FutureWaitTrap => FutureWaitTrap(),
                TaskPlanningScenarioPreset.FutureCapacityTrap => FutureCapacityTrap(),
                TaskPlanningScenarioPreset.RollingHorizonCapacitySaturation => RollingHorizonCapacitySaturation(),
                TaskPlanningScenarioPreset.RegretAssignmentTrap => RegretAssignmentTrap(),
                TaskPlanningScenarioPreset.HungarianAssignmentTrap => HungarianAssignmentTrap(),
                TaskPlanningScenarioPreset.RegretDecoyTrap => RegretDecoyTrap(),
                TaskPlanningScenarioPreset.GlobalAssignmentTrap => GlobalAssignmentTrap(),
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

        public static TaskPlanningScenario RegretAssignmentTrap()
        {
            var graph = LongCorridorWithBays(length: 22);

            return new TaskPlanningScenario(
                "Regret Assignment Trap",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Key", 0, "210"),
                    Amr("AMR_Backup", 1, "216")
                },
                new[]
                {
                    Pallet("Pallet_Flexible", "111", "113", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Scarce", "108", "106", attach: 1f, detach: 1f, load: 1f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("112", "112", "Pallet_Flexible"),
                    LoadingPoint("107", "107", "Pallet_Scarce")
                },
                new[]
                {
                    Workstation("113", "113", "Pallet_Flexible"),
                    Workstation("106", "106", "Pallet_Scarce")
                },
                new[]
                {
                    Task(0f, "Pallet_Flexible", "113", "RegretTrap_Flexible"),
                    Task(0f, "Pallet_Scarce", "106", "RegretTrap_Scarce")
                });
        }

        public static TaskPlanningScenario HungarianAssignmentTrap()
        {
            var graph = LongCorridorWithBays(length: 42);

            return new TaskPlanningScenario(
                "Hungarian Assignment Trap",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Key", 0, "211"),
                    Amr("AMR_Backup", 1, "230"),
                    Amr("AMR_Right", 2, "240")
                },
                new[]
                {
                    Pallet("Pallet_Scarce", "100", "102", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Flexible", "120", "122", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Right", "140", "138", attach: 1f, detach: 1f, load: 1f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("101", "101", "Pallet_Scarce"),
                    LoadingPoint("121", "121", "Pallet_Flexible"),
                    LoadingPoint("139", "139", "Pallet_Right")
                },
                new[]
                {
                    Workstation("102", "102", "Pallet_Scarce"),
                    Workstation("122", "122", "Pallet_Flexible"),
                    Workstation("138", "138", "Pallet_Right")
                },
                new[]
                {
                    Task(0f, "Pallet_Scarce", "102", "HungarianTrap_Scarce"),
                    Task(0f, "Pallet_Flexible", "122", "HungarianTrap_Flexible"),
                    Task(0f, "Pallet_Right", "138", "HungarianTrap_Right")
                });
        }

        public static TaskPlanningScenario RegretDecoyTrap()
        {
            var graph = TBranchWithBays(verticalLength: 30, horizontalLength: 4);

            return new TaskPlanningScenario(
                "Regret Decoy Trap",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Key", 0, "200"),
                    Amr("AMR_Shared", 1, "104"),
                    Amr("AMR_Far", 2, "339")
                },
                new[]
                {
                    Pallet("Pallet_Decoy", "319", "321", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Local_A", "102", "103", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Local_B", "203", "202", attach: 1f, detach: 1f, load: 1f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("320", "320", "Pallet_Decoy"),
                    LoadingPoint("101", "101", "Pallet_Local_A"),
                    LoadingPoint("204", "204", "Pallet_Local_B")
                },
                new[]
                {
                    Workstation("321", "321", "Pallet_Decoy"),
                    Workstation("103", "103", "Pallet_Local_A"),
                    Workstation("202", "202", "Pallet_Local_B")
                },
                new[]
                {
                    Task(0f, "Pallet_Decoy", "321", "RegretDecoyTrap_Decoy"),
                    Task(0f, "Pallet_Local_A", "103", "RegretDecoyTrap_LocalA"),
                    Task(0f, "Pallet_Local_B", "202", "RegretDecoyTrap_LocalB")
                });
        }

        public static TaskPlanningScenario GlobalAssignmentTrap()
        {
            var nearestTrapGraph = LongCorridorWithBays(length: 42);
            var regretTrapGraph = OffsetGraph(
                TBranchWithBays(verticalLength: 30, horizontalLength: 4),
                idOffset: 1000,
                xOffset: 120f,
                yOffset: 0f);
            var nodes = new List<TaskPlanningScenarioNode>();
            var edges = new List<TaskPlanningScenarioEdge>();
            nodes.AddRange(nearestTrapGraph.Nodes);
            nodes.AddRange(regretTrapGraph.Nodes);
            edges.AddRange(nearestTrapGraph.Edges);
            edges.AddRange(regretTrapGraph.Edges);
            edges.Add(Edge("42", "1000"));

            return new TaskPlanningScenario(
                "Global Assignment Trap",
                nodes,
                edges,
                new[]
                {
                    Amr("AMR_H_Key", 0, "211"),
                    Amr("AMR_H_Backup", 1, "230"),
                    Amr("AMR_H_Right", 2, "240"),
                    Amr("AMR_R_Key", 3, "1200"),
                    Amr("AMR_R_Shared", 4, "1104"),
                    Amr("AMR_R_Far", 5, "1339")
                },
                new[]
                {
                    Pallet("Pallet_H_Scarce", "100", "102", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_H_Flexible", "120", "122", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_H_Right", "140", "138", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_R_Decoy", "1319", "1321", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_R_Local_A", "1102", "1103", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_R_Local_B", "1203", "1202", attach: 1f, detach: 1f, load: 1f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("101", "101", "Pallet_H_Scarce"),
                    LoadingPoint("121", "121", "Pallet_H_Flexible"),
                    LoadingPoint("139", "139", "Pallet_H_Right"),
                    LoadingPoint("1320", "1320", "Pallet_R_Decoy"),
                    LoadingPoint("1101", "1101", "Pallet_R_Local_A"),
                    LoadingPoint("1204", "1204", "Pallet_R_Local_B")
                },
                new[]
                {
                    Workstation("102", "102", "Pallet_H_Scarce"),
                    Workstation("122", "122", "Pallet_H_Flexible"),
                    Workstation("138", "138", "Pallet_H_Right"),
                    Workstation("1321", "1321", "Pallet_R_Decoy"),
                    Workstation("1103", "1103", "Pallet_R_Local_A"),
                    Workstation("1202", "1202", "Pallet_R_Local_B")
                },
                new[]
                {
                    Task(0f, "Pallet_H_Flexible", "122", "GlobalTrap_H_Flexible"),
                    Task(0f, "Pallet_H_Scarce", "102", "GlobalTrap_H_Scarce"),
                    Task(0f, "Pallet_H_Right", "138", "GlobalTrap_H_Right"),
                    Task(0f, "Pallet_R_Decoy", "1321", "GlobalTrap_R_Decoy"),
                    Task(0f, "Pallet_R_Local_A", "1103", "GlobalTrap_R_LocalA"),
                    Task(0f, "Pallet_R_Local_B", "1202", "GlobalTrap_R_LocalB")
                });
        }

        public static TaskPlanningScenario RollingHorizonCapacitySaturation()
        {
            var graph = LongCorridorWithBays(length: 70);

            return new TaskPlanningScenario(
                "Rolling Horizon Capacity Saturation",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Future", 0, "258"),
                    Amr("AMR_Remote_A", 1, "205"),
                    Amr("AMR_Remote_B", 2, "208"),
                    Amr("AMR_Remote_C", 3, "211")
                },
                new[]
                {
                    Pallet("Pallet_Primer", "158", "259", attach: 1f, detach: 1f, load: 4f, unload: 1f),
                    Pallet("Pallet_Target_A", "160", "261", attach: 1f, detach: 1f, load: 4f, unload: 1f),
                    Pallet("Pallet_Target_B", "162", "263", attach: 1f, detach: 1f, load: 4f, unload: 1f),
                    Pallet("Pallet_Target_C", "164", "265", attach: 1f, detach: 1f, load: 4f, unload: 1f),
                    Pallet("Pallet_Target_D", "166", "267", attach: 1f, detach: 1f, load: 4f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("159", "159", "Pallet_Primer"),
                    LoadingPoint("161", "161", "Pallet_Target_A"),
                    LoadingPoint("163", "163", "Pallet_Target_B"),
                    LoadingPoint("165", "165", "Pallet_Target_C"),
                    LoadingPoint("167", "167", "Pallet_Target_D")
                },
                new[]
                {
                    Workstation("259", "259", "Pallet_Primer"),
                    Workstation("261", "261", "Pallet_Target_A"),
                    Workstation("263", "263", "Pallet_Target_B"),
                    Workstation("265", "265", "Pallet_Target_C"),
                    Workstation("267", "267", "Pallet_Target_D")
                },
                new[]
                {
                    Task(0f, "Pallet_Primer", "259", "PrimeFutureCapacityAmr"),
                    Task(1f, "Pallet_Target_A", "261", "CapacityOverflowA"),
                    Task(1f, "Pallet_Target_B", "263", "CapacityOverflowB"),
                    Task(1f, "Pallet_Target_C", "265", "CapacityOverflowC"),
                    Task(1f, "Pallet_Target_D", "267", "CapacityOverflowD")
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

        private static CorridorGraph OffsetGraph(
            CorridorGraph graph,
            int idOffset,
            float xOffset,
            float yOffset)
        {
            var nodes = new List<TaskPlanningScenarioNode>();
            var edges = new List<TaskPlanningScenarioEdge>();
            foreach (var node in graph.Nodes)
            {
                if (!int.TryParse(node.NodeId, out var numericId))
                    continue;

                nodes.Add(Node(
                    (numericId + idOffset).ToString(),
                    node.Position.x + xOffset,
                    node.Position.y + yOffset));
            }

            foreach (var edge in graph.Edges)
            {
                if (!int.TryParse(edge.ANodeId, out var a) ||
                    !int.TryParse(edge.BNodeId, out var b))
                    continue;

                edges.Add(Edge((a + idOffset).ToString(), (b + idOffset).ToString()));
            }

            return new CorridorGraph(nodes, edges);
        }

        private static CorridorGraph TBranchWithBays(int verticalLength, int horizontalLength)
        {
            var nodes = new List<TaskPlanningScenarioNode>();
            var edges = new List<TaskPlanningScenarioEdge>();

            for (var i = 0; i <= horizontalLength; i++)
            {
                var id = i.ToString();
                nodes.Add(Node(id, i, 0));
                if (i > 0)
                    edges.Add(Edge((i - 1).ToString(), id));

                var north = (100 + i).ToString();
                var south = (200 + i).ToString();
                nodes.Add(Node(north, i, 1));
                nodes.Add(Node(south, i, -1));
                edges.Add(Edge(id, north));
                edges.Add(Edge(id, south));
            }

            for (var y = 1; y <= verticalLength; y++)
            {
                var id = (9 + y).ToString();
                nodes.Add(Node(id, 0, y));
                edges.Add(Edge(y == 1 ? "0" : (8 + y).ToString(), id));

                var side = (300 + 9 + y).ToString();
                nodes.Add(Node(side, -1, y));
                edges.Add(Edge(id, side));
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
