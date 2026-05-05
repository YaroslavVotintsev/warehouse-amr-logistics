using UnityEditor;
using UnityEngine;

public static class GraphEdgeCreator
{
    private static GraphToolsSettings settings;

    private static GraphToolsSettings GetSettings()
    {
        if (settings == null)
        {
            settings = AssetDatabase.LoadAssetAtPath<GraphToolsSettings>("Assets/GraphToolsSettings.asset");

            if (settings == null)
            {
                Debug.LogError("GraphToolsSettings asset not found at Assets/GraphToolsSettings.asset");
            }
        }

        return settings;
    }

    [MenuItem("Tools/Graph/Create Edge From Selected Vertices")]
    public static void CreateEdgeFromSelectedVertices()
    {
        var settings = GetSettings();
        if (settings == null || settings.edgePrefab == null)
        {
            EditorUtility.DisplayDialog(
                "Missing Prefab",
                "Assign an Edge Prefab in GraphToolsSettings.",
                "OK"
            );
            return;
        }

        GameObject[] selectedObjects = Selection.gameObjects;

        if (selectedObjects.Length != 2)
        {
            EditorUtility.DisplayDialog("Error", "Select exactly 2 vertices.", "OK");
            return;
        }

        GraphVertex v1 = selectedObjects[0].GetComponent<GraphVertex>();
        GraphVertex v2 = selectedObjects[1].GetComponent<GraphVertex>();

        if (v1 == null || v2 == null)
        {
            EditorUtility.DisplayDialog("Error", "Both must have GraphVertex.", "OK");
            return;
        }

        GraphVertex a = v1.id <= v2.id ? v1 : v2;
        GraphVertex b = v1.id <= v2.id ? v2 : v1;

        string edgeName = $"{a.id}-{b.id}";

        // Instantiate prefab properly (keeps prefab connection)
        GameObject edgeObject = (GameObject)PrefabUtility.InstantiatePrefab(settings.edgePrefab);
        Undo.RegisterCreatedObjectUndo(edgeObject, "Create Graph Edge");

        edgeObject.name = edgeName;

        // Parent (optional)
        GameObject parent = GameObject.Find("Edges");
        if (parent == null)
        {
            parent = new GameObject("Edges");
            Undo.RegisterCreatedObjectUndo(parent, "Create Edges Parent");
        }

        edgeObject.transform.SetParent(parent.transform);

        // Position midway
        edgeObject.transform.position = (a.transform.position + b.transform.position) / 2f;

        // Assign component
        GraphEdge edge = edgeObject.GetComponent<GraphEdge>();
        if (edge == null)
        {
            Debug.LogError("Edge prefab does not have GraphEdge component!");
            return;
        }

        edge.vertexA = a.transform;
        edge.vertexB = b.transform;

        EditorUtility.SetDirty(edge);

        Selection.activeGameObject = edgeObject;

        Debug.Log($"Created edge from prefab: {edgeName}");
    }

    [MenuItem("Tools/Graph/Create Edge From Selected Vertices", true)]
    private static bool Validate()
    {
        return Selection.gameObjects.Length == 2 &&
               Selection.gameObjects[0].GetComponent<GraphVertex>() != null &&
               Selection.gameObjects[1].GetComponent<GraphVertex>() != null;
    }
}