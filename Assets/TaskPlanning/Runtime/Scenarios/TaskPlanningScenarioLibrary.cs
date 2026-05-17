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
                GlobalAssignmentTrap(),
                SoftReassignmentRescue(),
                GridThroughputBenchmark()
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
                TaskPlanningScenarioPreset.SoftReassignmentRescue => SoftReassignmentRescue(),
                TaskPlanningScenarioPreset.GridThroughputBenchmark => GridThroughputBenchmark(),
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

        public static TaskPlanningScenario SoftReassignmentRescue()
        {
            var graph = LongCorridorWithBays(length: 52);

            return new TaskPlanningScenario(
                "Soft Reassignment Rescue",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_Switchable", 0, "200"),
                    Amr("AMR_Backup", 1, "252")
                },
                new[]
                {
                    Pallet("Pallet_Long", "122", "126", attach: 1f, detach: 1f, load: 1f, unload: 1f),
                    Pallet("Pallet_Short", "104", "107", attach: 1f, detach: 1f, load: 1f, unload: 1f)
                },
                new[]
                {
                    LoadingPoint("123", "123", "Pallet_Long"),
                    LoadingPoint("105", "105", "Pallet_Short")
                },
                new[]
                {
                    Workstation("126", "126", "Pallet_Long"),
                    Workstation("107", "107", "Pallet_Short")
                },
                new[]
                {
                    Task(0f, "Pallet_Long", "126", "SoftRescue_Long"),
                    Task(2f, "Pallet_Short", "107", "SoftRescue_Short")
                });
        }

        public static TaskPlanningScenario GridThroughputBenchmark()
        {
            var graph = GridWithSideBays(columns: 13, rows: 9);
            var allPalletIds = new[]
            {
                "Pallet_A1",
                "Pallet_A2",
                "Pallet_A3",
                "Pallet_A4",
                "Pallet_B1",
                "Pallet_B2",
                "Pallet_B3",
                "Pallet_B4",
                "Pallet_C1",
                "Pallet_C2",
                "Pallet_C3",
                "Pallet_C4",
                "Pallet_D1",
                "Pallet_D2",
                "Pallet_D3",
                "Pallet_D4"
            };

            return new TaskPlanningScenario(
                "Grid Throughput Benchmark",
                graph.Nodes,
                graph.Edges,
                new[]
                {
                    Amr("AMR_W1", 0, "9200"),
                    Amr("AMR_W2", 1, "9202"),
                    Amr("AMR_W3", 2, "9204"),
                    Amr("AMR_W4", 3, "9206"),
                    Amr("AMR_E1", 4, "9301"),
                    Amr("AMR_E2", 5, "9303"),
                    Amr("AMR_E3", 6, "9305"),
                    Amr("AMR_E4", 7, "9307")
                },
                new[]
                {
                    Pallet("Pallet_A1", "9000", "9000", attach: 1f, detach: 1f, load: 2f, unload: 2f),
                    Pallet("Pallet_A2", "9002", "9002", attach: 1f, detach: 1f, load: 2f, unload: 2f),
                    Pallet("Pallet_A3", "9003", "9003", attach: 1f, detach: 1f, load: 3f, unload: 2f),
                    Pallet("Pallet_A4", "9004", "9004", attach: 1f, detach: 1f, load: 2f, unload: 3f),
                    Pallet("Pallet_B1", "9008", "9008", attach: 1f, detach: 1f, load: 3f, unload: 2f),
                    Pallet("Pallet_B2", "9009", "9009", attach: 1f, detach: 1f, load: 2f, unload: 2f),
                    Pallet("Pallet_B3", "9010", "9010", attach: 1f, detach: 1f, load: 3f, unload: 3f),
                    Pallet("Pallet_B4", "9012", "9012", attach: 1f, detach: 1f, load: 2f, unload: 2f),
                    Pallet("Pallet_C1", "9100", "9100", attach: 1f, detach: 1f, load: 2f, unload: 2f),
                    Pallet("Pallet_C2", "9102", "9102", attach: 1f, detach: 1f, load: 3f, unload: 2f),
                    Pallet("Pallet_C3", "9103", "9103", attach: 1f, detach: 1f, load: 2f, unload: 3f),
                    Pallet("Pallet_C4", "9104", "9104", attach: 1f, detach: 1f, load: 3f, unload: 2f),
                    Pallet("Pallet_D1", "9108", "9108", attach: 1f, detach: 1f, load: 3f, unload: 3f),
                    Pallet("Pallet_D2", "9109", "9109", attach: 1f, detach: 1f, load: 2f, unload: 2f),
                    Pallet("Pallet_D3", "9110", "9110", attach: 1f, detach: 1f, load: 3f, unload: 2f),
                    Pallet("Pallet_D4", "9112", "9112", attach: 1f, detach: 1f, load: 2f, unload: 3f)
                },
                new[]
                {
                    LoadingPoint("9001", "9001", "Pallet_A1", "Pallet_A2", "Pallet_A3", "Pallet_A4"),
                    LoadingPoint("9011", "9011", "Pallet_B1", "Pallet_B2", "Pallet_B3", "Pallet_B4"),
                    LoadingPoint("9101", "9101", "Pallet_C1", "Pallet_C2", "Pallet_C3", "Pallet_C4"),
                    LoadingPoint("9111", "9111", "Pallet_D1", "Pallet_D2", "Pallet_D3", "Pallet_D4")
                },
                new[]
                {
                    Workstation("9201", "9201", allPalletIds),
                    Workstation("9203", "9203", allPalletIds),
                    Workstation("9205", "9205", allPalletIds),
                    Workstation("9207", "9207", allPalletIds),
                    Workstation("9208", "9208", allPalletIds),
                    Workstation("9300", "9300", allPalletIds),
                    Workstation("9302", "9302", allPalletIds),
                    Workstation("9304", "9304", allPalletIds),
                    Workstation("9306", "9306", allPalletIds),
                    Workstation("9308", "9308", allPalletIds),
                    Workstation("9006", "9006", allPalletIds),
                    Workstation("9106", "9106", allPalletIds)
                },
                new[]
                {
                    Task(0f, "Pallet_A1", "9300", "GridWave00_A1_E0"),
                    Task(0f, "Pallet_B1", "9201", "GridWave00_B1_W1"),
                    Task(0f, "Pallet_C1", "9302", "GridWave00_C1_E2"),
                    Task(0f, "Pallet_D1", "9203", "GridWave00_D1_W3"),
                    Task(0f, "Pallet_A2", "9006", "GridWave00_A2_N6"),
                    Task(0f, "Pallet_D2", "9106", "GridWave00_D2_S6"),
                    Task(15f, "Pallet_B2", "9205", "GridWave15_B2_W5"),
                    Task(15f, "Pallet_C2", "9304", "GridWave15_C2_E4"),
                    Task(15f, "Pallet_A3", "9306", "GridWave15_A3_E6"),
                    Task(15f, "Pallet_D3", "9207", "GridWave15_D3_W7"),
                    Task(28f, "Pallet_A4", "9308", "GridWave28_A4_E8"),
                    Task(28f, "Pallet_B3", "9208", "GridWave28_B3_W8"),
                    Task(28f, "Pallet_C3", "9006", "GridWave28_C3_N6"),
                    Task(28f, "Pallet_D4", "9106", "GridWave28_D4_S6"),
                    Task(28f, "Pallet_B4", "9201", "GridWave28_B4_W1"),
                    Task(45f, "Pallet_A1", "9304", "GridWave45_A1_E4"),
                    Task(45f, "Pallet_C4", "9306", "GridWave45_C4_E6"),
                    Task(45f, "Pallet_B1", "9205", "GridWave45_B1_W5"),
                    Task(45f, "Pallet_D1", "9208", "GridWave45_D1_W8"),
                    Task(45f, "Pallet_C1", "9308", "GridWave45_C1_E8"),
                    Task(45f, "Pallet_A2", "9203", "GridWave45_A2_W3"),
                    Task(65f, "Pallet_B2", "9300", "GridWave65_B2_E0"),
                    Task(65f, "Pallet_D2", "9201", "GridWave65_D2_W1"),
                    Task(65f, "Pallet_A3", "9302", "GridWave65_A3_E2"),
                    Task(65f, "Pallet_C2", "9207", "GridWave65_C2_W7"),
                    Task(82f, "Pallet_C3", "9304", "GridWave82_C3_E4"),
                    Task(82f, "Pallet_A4", "9205", "GridWave82_A4_W5"),
                    Task(82f, "Pallet_B3", "9306", "GridWave82_B3_E6"),
                    Task(82f, "Pallet_D3", "9203", "GridWave82_D3_W3"),
                    Task(82f, "Pallet_C4", "9308", "GridWave82_C4_E8"),
                    Task(82f, "Pallet_B4", "9006", "GridWave82_B4_N6"),
                    Task(105f, "Pallet_A1", "9106", "GridWave105_A1_S6"),
                    Task(105f, "Pallet_B1", "9208", "GridWave105_B1_W8"),
                    Task(105f, "Pallet_D4", "9302", "GridWave105_D4_E2"),
                    Task(105f, "Pallet_C1", "9201", "GridWave105_C1_W1"),
                    Task(105f, "Pallet_A2", "9300", "GridWave105_A2_E0"),
                    Task(128f, "Pallet_D1", "9006", "GridWave128_D1_N6"),
                    Task(128f, "Pallet_C2", "9304", "GridWave128_C2_E4"),
                    Task(128f, "Pallet_B2", "9203", "GridWave128_B2_W3"),
                    Task(128f, "Pallet_A3", "9308", "GridWave128_A3_E8"),
                    Task(128f, "Pallet_D2", "9205", "GridWave128_D2_W5"),
                    Task(128f, "Pallet_C3", "9106", "GridWave128_C3_S6"),
                    Task(150f, "Pallet_B3", "9300", "GridWave150_B3_E0"),
                    Task(150f, "Pallet_A4", "9207", "GridWave150_A4_W7"),
                    Task(150f, "Pallet_C4", "9306", "GridWave150_C4_E6"),
                    Task(150f, "Pallet_D3", "9208", "GridWave150_D3_W8"),
                    Task(172f, "Pallet_A1", "9302", "GridWave172_A1_E2"),
                    Task(172f, "Pallet_B4", "9201", "GridWave172_B4_W1"),
                    Task(172f, "Pallet_C1", "9304", "GridWave172_C1_E4"),
                    Task(172f, "Pallet_D4", "9205", "GridWave172_D4_W5"),
                    Task(172f, "Pallet_B1", "9106", "GridWave172_B1_S6"),
                    Task(200f, "Pallet_A2", "9308", "GridWave200_A2_E8"),
                    Task(200f, "Pallet_C2", "9203", "GridWave200_C2_W3"),
                    Task(200f, "Pallet_D1", "9006", "GridWave200_D1_N6"),
                    Task(200f, "Pallet_A3", "9300", "GridWave200_A3_E0"),
                    Task(200f, "Pallet_B2", "9207", "GridWave200_B2_W7")
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

        private static CorridorGraph GridWithSideBays(int columns, int rows)
        {
            var nodes = new List<TaskPlanningScenarioNode>();
            var edges = new List<TaskPlanningScenarioEdge>();
            const float spacing = 2f;
            const float bayOffset = 1.5f;

            for (var row = 0; row < rows; row++)
            {
                for (var column = 0; column < columns; column++)
                {
                    var id = GridNodeId(row, column);
                    nodes.Add(Node(id, column * spacing, row * spacing));

                    if (column > 0)
                        edges.Add(Edge(GridNodeId(row, column - 1), id));

                    if (row > 0)
                        edges.Add(Edge(GridNodeId(row - 1, column), id));
                }
            }

            for (var column = 0; column < columns; column++)
            {
                var northId = NorthBayId(column);
                var southId = SouthBayId(column);
                nodes.Add(Node(northId, column * spacing, (rows - 1) * spacing + bayOffset));
                nodes.Add(Node(southId, column * spacing, -bayOffset));
                edges.Add(Edge(GridNodeId(rows - 1, column), northId));
                edges.Add(Edge(GridNodeId(0, column), southId));
            }

            for (var row = 0; row < rows; row++)
            {
                var westId = WestBayId(row);
                var eastId = EastBayId(row);
                nodes.Add(Node(westId, -bayOffset, row * spacing));
                nodes.Add(Node(eastId, (columns - 1) * spacing + bayOffset, row * spacing));
                edges.Add(Edge(GridNodeId(row, 0), westId));
                edges.Add(Edge(GridNodeId(row, columns - 1), eastId));
            }

            return new CorridorGraph(nodes, edges);
        }

        private static string GridNodeId(int row, int column)
        {
            return (row * 100 + column).ToString();
        }

        private static string NorthBayId(int column)
        {
            return (9000 + column).ToString();
        }

        private static string SouthBayId(int column)
        {
            return (9100 + column).ToString();
        }

        private static string WestBayId(int row)
        {
            return (9200 + row).ToString();
        }

        private static string EastBayId(int row)
        {
            return (9300 + row).ToString();
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
