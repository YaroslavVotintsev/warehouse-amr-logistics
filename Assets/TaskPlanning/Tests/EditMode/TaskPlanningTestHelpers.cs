using System;
using System.Reflection;
using Mapf.Authoring;
using UnityEngine;

namespace TaskPlanning.Tests
{
    internal static class TaskPlanningTestHelpers
    {
        public static T CreateComponent<T>(string name = null)
            where T : Component
        {
            var gameObject = new GameObject(name ?? typeof(T).Name);
            return gameObject.AddComponent<T>();
        }

        public static MapfNode CreateNode(string name, Vector2 position)
        {
            var node = CreateComponent<MapfNode>(name);
            node.Configure(name);
            node.transform.position = new Vector3(position.x, position.y, 0f);
            return node;
        }

        public static PalletMarker CreatePallet(string name, MapfNode currentNode = null)
        {
            var pallet = CreateComponent<PalletMarker>(name);
            SetField(pallet, "palletId", name);
            SetField(pallet, "currentNode", currentNode);
            if (currentNode != null)
                pallet.transform.position = currentNode.transform.position;
            return pallet;
        }

        public static PalletLoadingPoint CreateLoadingPoint(string name, MapfNode node, params PalletMarker[] acceptedPallets)
        {
            var point = CreateComponent<PalletLoadingPoint>(name);
            SetField(point, "loadingPointId", name);
            SetField(point, "node", node);
            SetField(point, "acceptedPallets", acceptedPallets);
            return point;
        }

        public static WorkstationDeliveryPoint CreateWorkstation(string name, MapfNode node, params PalletMarker[] acceptedPallets)
        {
            var point = CreateComponent<WorkstationDeliveryPoint>(name);
            SetField(point, "workstationId", name);
            SetField(point, "node", node);
            SetField(point, "acceptedPallets", acceptedPallets);
            return point;
        }

        public static void SetField<TTarget, TValue>(TTarget target, string fieldName, TValue value)
        {
            var field = typeof(TTarget).GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic);
            if (field == null)
                throw new MissingFieldException(typeof(TTarget).FullName, fieldName);

            field.SetValue(target, value);
        }

        public static void Destroy(UnityEngine.Object unityObject)
        {
            if (unityObject != null)
                UnityEngine.Object.DestroyImmediate(unityObject);
        }
    }
}
