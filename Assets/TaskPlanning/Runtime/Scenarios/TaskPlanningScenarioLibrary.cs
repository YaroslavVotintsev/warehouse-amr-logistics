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
                LoadingPointBottleneck(),
                LookAheadTrap()
            };
        }

        public static TaskPlanningScenario Get(TaskPlanningScenarioPreset preset)
        {
            return preset switch
            {
                TaskPlanningScenarioPreset.LoadingPointBottleneck => LoadingPointBottleneck(),
                TaskPlanningScenarioPreset.LookAheadTrap => LookAheadTrap(),
                _ => LoadingPointBottleneck()
            };
        }

        public static TaskPlanningScenario LoadingPointBottleneck()
        {
            var pallets = new[]
            {
                Pallet("Pallet_A", "Pallet_A_Node", "Pallet_A_Node", load: 8f, unload: 4f),
                Pallet("Pallet_B", "Pallet_B_Node", "Pallet_B_Node", load: 8f, unload: 4f),
                Pallet("Pallet_C", "Pallet_C_Node", "Pallet_C_Node", load: 8f, unload: 4f),
                Pallet("Pallet_D", "Pallet_D_Node", "Pallet_D_Node", load: 8f, unload: 4f)
            };
            var palletIds = new[] { "Pallet_A", "Pallet_B", "Pallet_C", "Pallet_D" };

            return new TaskPlanningScenario(
                "Loading Point Bottleneck",
                new[]
                {
                    Node("AMR_A_Start", 0, -1),
                    Node("AMR_B_Start", 0, 0),
                    Node("AMR_C_Start", 0, 1),
                    Node("Pallet_A_Node", 2, -2),
                    Node("Pallet_B_Node", 2, -0.7f),
                    Node("Pallet_C_Node", 2, 0.7f),
                    Node("Pallet_D_Node", 2, 2),
                    Node("Loading_One_Node", 4, 0),
                    Node("Workstation_One_Node", 7, -1.5f),
                    Node("Workstation_Two_Node", 7, 0),
                    Node("Workstation_Three_Node", 7, 1.5f)
                },
                new[]
                {
                    Edge("AMR_A_Start", "AMR_B_Start"),
                    Edge("AMR_B_Start", "AMR_C_Start"),
                    Edge("AMR_B_Start", "Pallet_B_Node"),
                    Edge("Pallet_B_Node", "Pallet_A_Node"),
                    Edge("Pallet_B_Node", "Pallet_C_Node"),
                    Edge("Pallet_C_Node", "Pallet_D_Node"),
                    Edge("Pallet_B_Node", "Loading_One_Node"),
                    Edge("Pallet_C_Node", "Loading_One_Node"),
                    Edge("Loading_One_Node", "Workstation_One_Node"),
                    Edge("Loading_One_Node", "Workstation_Two_Node"),
                    Edge("Loading_One_Node", "Workstation_Three_Node"),
                    Edge("Workstation_One_Node", "Workstation_Two_Node"),
                    Edge("Workstation_Two_Node", "Workstation_Three_Node")
                },
                new[]
                {
                    Amr("AMR_A", 0, "AMR_A_Start"),
                    Amr("AMR_B", 1, "AMR_B_Start"),
                    Amr("AMR_C", 2, "AMR_C_Start")
                },
                pallets,
                new[]
                {
                    LoadingPoint("Loading_One", "Loading_One_Node", palletIds)
                },
                new[]
                {
                    Workstation("Workstation_One", "Workstation_One_Node", "Pallet_A", "Pallet_D"),
                    Workstation("Workstation_Two", "Workstation_Two_Node", "Pallet_B"),
                    Workstation("Workstation_Three", "Workstation_Three_Node", "Pallet_C")
                },
                new[]
                {
                    Task(0f, "Pallet_A", "Workstation_One", "Bottleneck_A"),
                    Task(0f, "Pallet_B", "Workstation_Two", "Bottleneck_B"),
                    Task(0f, "Pallet_C", "Workstation_Three", "Bottleneck_C"),
                    Task(12f, "Pallet_D", "Workstation_One", "Bottleneck_D")
                });
        }

        public static TaskPlanningScenario LookAheadTrap()
        {
            return new TaskPlanningScenario(
                "LookAhead Trap",
                new[]
                {
                    Node("Far_AMR_Start", 0, 0),
                    Node("Corridor_One", 3, 0),
                    Node("Corridor_Two", 6, 0),
                    Node("Busy_AMR_Start", 8, 0),
                    Node("Active_Pallet_Node", 9, 0),
                    Node("Active_Load_Node", 10, 0),
                    Node("Active_Workstation_Node", 12, 0),
                    Node("Target_Pallet_Node", 12, 1),
                    Node("Target_Load_Node", 13, 1),
                    Node("Target_Workstation_Node", 14, 1)
                },
                new[]
                {
                    Edge("Far_AMR_Start", "Corridor_One"),
                    Edge("Corridor_One", "Corridor_Two"),
                    Edge("Corridor_Two", "Busy_AMR_Start"),
                    Edge("Busy_AMR_Start", "Active_Pallet_Node"),
                    Edge("Active_Pallet_Node", "Active_Load_Node"),
                    Edge("Active_Load_Node", "Active_Workstation_Node"),
                    Edge("Active_Workstation_Node", "Target_Pallet_Node"),
                    Edge("Target_Pallet_Node", "Target_Load_Node"),
                    Edge("Target_Load_Node", "Target_Workstation_Node")
                },
                new[]
                {
                    Amr("AMR_Far", 0, "Far_AMR_Start"),
                    Amr("AMR_BusyCandidate", 1, "Busy_AMR_Start")
                },
                new[]
                {
                    Pallet("Pallet_Active", "Active_Pallet_Node", "Active_Pallet_Node", attach: 1f, detach: 1f, load: 5f, unload: 3f),
                    Pallet("Pallet_Target", "Target_Pallet_Node", "Target_Pallet_Node", attach: 1f, detach: 1f, load: 2f, unload: 3f)
                },
                new[]
                {
                    LoadingPoint("Loading_Active", "Active_Load_Node", "Pallet_Active"),
                    LoadingPoint("Loading_Target", "Target_Load_Node", "Pallet_Target")
                },
                new[]
                {
                    Workstation("Workstation_Active", "Active_Workstation_Node", "Pallet_Active"),
                    Workstation("Workstation_Target", "Target_Workstation_Node", "Pallet_Target")
                },
                new[]
                {
                    Task(0f, "Pallet_Active", "Workstation_Active", "PrimeBusyAmr"),
                    Task(1f, "Pallet_Target", "Workstation_Target", "TrapTarget")
                });
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
    }
}
