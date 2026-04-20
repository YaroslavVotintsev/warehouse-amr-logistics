using TMPro;
using UnityEditor;
using UnityEngine;

public static class GraphVertexRenamer
{
    [MenuItem("Tools/Graph/Rename Vertices And Update Labels")]
    public static void RenameVerticesAndUpdateLabels()
    {
        GraphVertex[] vertices = Object.FindObjectsByType<GraphVertex>(FindObjectsSortMode.None);

        int updatedCount = 0;

        foreach (GraphVertex vertex in vertices)
        {
            if (vertex == null)
                continue;

            GameObject go = vertex.gameObject;
            string newName = vertex.id.ToString();

            Undo.RecordObject(go, "Rename Graph Vertex");
            go.name = newName;
            EditorUtility.SetDirty(go);

            // Looks for TMP text in children, including inactive ones
            TMP_Text tmpText = go.GetComponentInChildren<TMP_Text>(true);

            if (tmpText != null)
            {
                Undo.RecordObject(tmpText, "Update Graph Vertex Label");
                tmpText.text = newName;
                EditorUtility.SetDirty(tmpText);
            }

            updatedCount++;
        }

        Debug.Log($"Updated {updatedCount} GraphVertex objects.");
    }
}