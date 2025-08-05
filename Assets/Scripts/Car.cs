using System.Collections.Generic;
using UnityEngine;

public class Car : MonoBehaviour
{
    public float speed = 5f;
    public List<Node> path = new List<Node>();
    public int currentNodeIndex = 0;
    
    bool isMoving = false;
    Rigidbody rb;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }
    
    void Update()
    {
        if (isMoving && path.Count > 0)
        {
            MoveAlongPath();
        }
    }
    
    public void SetPath(List<Node> newPath)
    {
        path = newPath;
        currentNodeIndex = 0;
        isMoving = true;
        Debug.Log($"Car received path with {newPath.Count} nodes");
    }
    
    void MoveAlongPath()
    {
        if (currentNodeIndex >= path.Count)
        {
            isMoving = false;
            rb.linearVelocity = Vector3.zero;
            Debug.Log("Car reached destination! Despawning...");
            
            // Despawn the car
            Destroy(gameObject);
            return;
        }
        
        Vector3 targetPos = path[currentNodeIndex].transform.position;
        Vector3 direction = (targetPos - transform.position).normalized;
        
        // Move in X and Z, keep Y position
        Vector3 movement = new Vector3(direction.x, 0, direction.z) * speed;
        rb.linearVelocity = movement;
        
        // Rotate car to face movement direction
        if (movement != Vector3.zero)
        {
            transform.rotation = Quaternion.LookRotation(movement);
        }
        
        if (Vector3.Distance(transform.position, targetPos) < 1f)
        {
            currentNodeIndex++;
            Debug.Log($"Moving to node {currentNodeIndex}");
        }
    }
}