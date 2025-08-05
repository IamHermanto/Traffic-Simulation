using UnityEngine;

public class NodeCreator : MonoBehaviour
{
    public Transform tilemap;
    
    [ContextMenu("Create Nodes")]
    void CreateNodes()
    {
        if (tilemap == null)
        {
            Debug.LogError("Tilemap reference is null!");
            return;
        }
        
        int nodesCreated = 0;
        
        foreach (Transform child in tilemap)
        {
            if (child.GetComponent<Node>() == null)
            {
                child.gameObject.AddComponent<Node>();
                nodesCreated++;
            }
        }
        
        Debug.Log($"Created {nodesCreated} nodes total");
    }
}