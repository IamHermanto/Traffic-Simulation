using System.Collections.Generic;
using UnityEngine;

public class RealisticCar : MonoBehaviour
{
    [Header("Wheel Colliders")]
    public WheelCollider frontLeftWheel;
    public WheelCollider frontRightWheel; 
    public WheelCollider rearLeftWheel;
    public WheelCollider rearRightWheel;
    
    [Header("Wheel Meshes")]
    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;
    
    [Header("Car Settings")]
    public float maxMotorTorque = 1500f;
    public float maxSteerAngle = 30f;
    public float maxBrakeTorque = 3000f;
    
    [Header("Dynamic Speed Control")]
    public float straightRoadSpeedMultiplier = 2.5f; // Speed boost on straight roads
    public float turnDetectionAngle = 15f; // Angle threshold to detect turns
    public float straightRoadMinDistance = 8f; // Min distance to consider "straight"
    public bool enableSpeedBoost = true;
    public bool debugSpeed = true;
    
    [Header("AI Pathfinding")]
    public List<Node> path = new List<Node>();
    public int currentNodeIndex = 0;
    public float stoppingDistance = 2f;
    public float detectionRange = 5f;
    public float nodeReachDistance = 2f;
    public float despawnDelay = 1f;
    public float maxLookAheadDistance = 4f; // Reduced to prevent shortcuts
    public float pathRecalculateInterval = 2f;
    
    [Header("Lane Changing System")]
    public float laneChangeDetectionRange = 6f; // Distance to check for lane changes
    public float targetLaneCheckRadius = 2.5f; // Radius to check target lane occupation
    public float laneChangeWaitTime = 0.5f; // Minimum wait time before retrying lane change
    public float laneChangeSlowdownFactor = 0.3f; // Speed when waiting for lane change
    public bool debugLaneChanges = true;
    
    [Header("Road Constraint")]
    public bool strictRoadFollowing = true; // Force follow road network
    public float maxSteerAnglePerFrame = 2f; // Prevent sharp steering
    public float roadWidthTolerance = 3f; // How far from nodes before correcting
    public bool debugRoadConstraint = true;
    
    [Header("Intersection Behavior")]
    public float intersectionDetectionRange = 6f;
    public float intersectionSlowdownFactor = 0.4f;
    public float intersectionStopDistance = 1.5f;
    public float otherCarDetectionRange = 4f;
    public bool debugIntersections = true;
    
    [Header("Performance Settings")]
    public float obstacleCheckInterval = 0.1f;
    public float turnSlowdownFactor = 0.3f;
    
    Rigidbody rb;
    bool isMoving = false;
    bool isBlocked = false;
    bool isAtIntersection = false;
    bool shouldYieldAtIntersection = false;
    bool isOffRoad = false;
    
    // Lane changing variables
    bool isWaitingForLaneChange = false;
    float laneChangeWaitTimer = 0f;
    Node targetLaneNode = null;
    bool isLaneChangeBlocked = false;
    
    private float obstacleCheckTimer;
    private float pathRecalculateTimer;
    private Node targetEndNode;
    private bool hasInitialAlignment = false;
    private float alignmentTime = 0f;
    private float lastSteerAngle = 0f;
    
    // Intersection detection cache
    private List<Node> nearbyIntersectionNodes = new List<Node>();
    private float intersectionCheckTimer = 0f;
    private float intersectionCheckInterval = 0.2f;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.5f, 0.5f);
        
        ValidateWheelSetup();
        SetupWheelFriction();
    }
    
    void ValidateWheelSetup()
    {
        WheelCollider[] wheels = { frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel };
        Transform[] meshes = { frontLeftMesh, frontRightMesh, rearLeftMesh, rearRightMesh };
        string[] wheelNames = { "Front Left", "Front Right", "Rear Left", "Rear Right" };
        
        for (int i = 0; i < wheels.Length; i++)
        {
            if (wheels[i] == null)
            {
                Debug.LogError($"Missing {wheelNames[i]} wheel collider on {gameObject.name}");
            }
            if (meshes[i] == null)
            {
                Debug.LogWarning($"Missing {wheelNames[i]} wheel mesh on {gameObject.name}");
            }
        }
    }
    
    void SetupWheelFriction()
    {
        WheelFrictionCurve forwardFriction = new WheelFrictionCurve();
        forwardFriction.extremumSlip = 0.4f;
        forwardFriction.extremumValue = 1f;
        forwardFriction.asymptoteSlip = 0.8f;
        forwardFriction.asymptoteValue = 0.5f;
        forwardFriction.stiffness = 1f;

        WheelFrictionCurve sidewaysFriction = new WheelFrictionCurve();
        sidewaysFriction.extremumSlip = 0.2f;
        sidewaysFriction.extremumValue = 1f;
        sidewaysFriction.asymptoteSlip = 0.5f;
        sidewaysFriction.asymptoteValue = 0.75f;
        sidewaysFriction.stiffness = 1f;

        WheelCollider[] wheels = { frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel };
        
        foreach (WheelCollider wheel in wheels)
        {
            if (wheel == null)
            {
                Debug.LogError($"Missing wheel collider on {gameObject.name}");
                continue;
            }
            wheel.forwardFriction = forwardFriction;
            wheel.sidewaysFriction = sidewaysFriction;
        }
    }
    
    void Update()
    {
        if (hasInitialAlignment)
        {
            alignmentTime += Time.deltaTime;
        }
        
        if (isMoving && path.Count > 0)
        {
            // Optimize checks with timers
            obstacleCheckTimer += Time.deltaTime;
            if (obstacleCheckTimer >= obstacleCheckInterval)
            {
                CheckForObstacles();
                CheckForOtherCars();
                CheckRoadConstraints();
                CheckLaneChangeOpportunity(); // NEW: Lane change system
                obstacleCheckTimer = 0f;
            }
            
            // Intersection detection
            intersectionCheckTimer += Time.deltaTime;
            if (intersectionCheckTimer >= intersectionCheckInterval)
            {
                DetectIntersections();
                intersectionCheckTimer = 0f;
            }
            
            // Path recalculation
            pathRecalculateTimer += Time.deltaTime;
            if (pathRecalculateTimer >= pathRecalculateInterval && targetEndNode != null)
            {
                RecalculatePath();
                pathRecalculateTimer = 0f;
            }
            
            // Lane change wait timer
            if (isWaitingForLaneChange)
            {
                laneChangeWaitTimer += Time.deltaTime;
                if (laneChangeWaitTimer >= laneChangeWaitTime)
                {
                    // Reset wait state and try again
                    isWaitingForLaneChange = false;
                    laneChangeWaitTimer = 0f;
                }
            }
            
            AIController();
        }
        
        UpdateWheelMeshes();
    }
    
    /// <summary>
    /// NEW: Check if car wants to change lanes and if it's safe to do so
    /// </summary>
    void CheckLaneChangeOpportunity()
    {
        // Don't check lane changes if we're already waiting
        if (isWaitingForLaneChange) return;
        
        // Get upcoming lane change node
        Node upcomingNode = GetUpcomingLaneChangeNode();
        if (upcomingNode == null)
        {
            isLaneChangeBlocked = false;
            targetLaneNode = null;
            return;
        }
        
        targetLaneNode = upcomingNode;
        
        // Check if target lane is occupied
        bool isTargetLaneOccupied = IsLaneOccupied(upcomingNode);
        
        if (isTargetLaneOccupied)
        {
            isLaneChangeBlocked = true;
            isWaitingForLaneChange = true;
            laneChangeWaitTimer = 0f;
            
            if (debugLaneChanges)
            {
                Debug.Log($"Car {gameObject.name}: Lane change to {upcomingNode.name} BLOCKED - waiting for clear path");
            }
        }
        else
        {
            isLaneChangeBlocked = false;
            isWaitingForLaneChange = false;
            
            if (debugLaneChanges && targetLaneNode != null)
            {
                Debug.Log($"Car {gameObject.name}: Lane change to {upcomingNode.name} CLEAR - proceeding");
            }
        }
    }
    
    /// <summary>
    /// NEW: Find if there's an upcoming lane change in our path
    /// </summary>
    Node GetUpcomingLaneChangeNode()
    {
        if (currentNodeIndex >= path.Count - 1) return null;
        
        // Look ahead for potential lane changes
        for (int i = currentNodeIndex; i < Mathf.Min(currentNodeIndex + 3, path.Count); i++)
        {
            if (path[i] == null) continue;
            
            float distanceToNode = Vector3.Distance(transform.position, path[i].transform.position);
            
            // If this node is within lane change detection range, check if it's a lane change
            if (distanceToNode <= laneChangeDetectionRange && IsLaneChangeNode(path[i]))
            {
                return path[i];
            }
        }
        
        return null;
    }
    
    /// <summary>
    /// NEW: Determine if a node represents a lane change (simplified logic)
    /// </summary>
    bool IsLaneChangeNode(Node node)
    {
        // A node is considered a lane change if:
        // 1. It has multiple neighbors (more than 2)
        // 2. OR it has "Lane" in its name (for multi-lane intersections)
        // 3. OR the angle between current direction and target direction is significant
        
        if (node.name.ToLower().Contains("lane"))
        {
            return true;
        }
        
        // Check if this node has multiple paths (potential lane choice)
        List<Node> neighbors = node.GetNeighbors();
        if (neighbors.Count > 2)
        {
            return true;
        }
        
        // Check angle change - if we're turning more than a certain angle, consider it a lane change
        if (currentNodeIndex < path.Count - 1)
        {
            Vector3 currentDirection = transform.forward;
            Vector3 targetDirection = (node.transform.position - transform.position).normalized;
            
            float angle = Vector3.Angle(currentDirection, targetDirection);
            if (angle > 25f) // Threshold for considering it a lane change
            {
                return true;
            }
        }
        
        return false;
    }
    
    /// <summary>
    /// NEW: Check if a lane/node is occupied by another car
    /// </summary>
    bool IsLaneOccupied(Node targetNode)
    {
        if (targetNode == null) return true;
        
        Vector3 checkPosition = targetNode.transform.position;
        
        // Check for cars in the target lane
        Collider[] carsInLane = Physics.OverlapSphere(checkPosition, targetLaneCheckRadius);
        
        foreach (Collider col in carsInLane)
        {
            RealisticCar otherCar = col.GetComponent<RealisticCar>();
            if (otherCar != null && otherCar != this)
            {
                // Additional check: make sure the other car is actually moving/active
                if (otherCar.isMoving)
                {
                    if (debugLaneChanges)
                    {
                        Debug.Log($"Lane at {targetNode.name} occupied by {otherCar.name}");
                    }
                    return true;
                }
            }
        }
        
        return false; // Lane is clear
    }
    
    public void SetPath(List<Node> newPath)
    {
        if (newPath == null || newPath.Count == 0)
        {
            Debug.LogWarning($"Invalid path provided to {gameObject.name}");
            return;
        }
        
        path = newPath;
        currentNodeIndex = 0;
        isMoving = true;
        targetEndNode = newPath[newPath.Count - 1];
        
        // Reset lane change state
        isWaitingForLaneChange = false;
        isLaneChangeBlocked = false;
        targetLaneNode = null;
        laneChangeWaitTimer = 0f;
        
        AlignToRoadDirection();
        Debug.Log($"Car received path with {newPath.Count} nodes to {targetEndNode.name}");
    }
    
    public void SetPathWithoutAlignment(List<Node> newPath)
    {
        if (newPath == null || newPath.Count == 0)
        {
            Debug.LogWarning($"Invalid path provided to {gameObject.name}");
            return;
        }
        
        path = newPath;
        currentNodeIndex = 0;
        isMoving = true;
        targetEndNode = newPath[newPath.Count - 1];
        
        // Reset lane change state
        isWaitingForLaneChange = false;
        isLaneChangeBlocked = false;
        targetLaneNode = null;
        laneChangeWaitTimer = 0f;
        
        hasInitialAlignment = true;
        alignmentTime = 0f;
        
        if (debugIntersections)
            Debug.Log($"Car received path: {path[0].name} → {targetEndNode.name} ({newPath.Count} nodes)");
    }
    
    void AlignToRoadDirection()
    {
        if (path == null || path.Count < 1)
        {
            Debug.LogWarning("AlignToRoadDirection: No path available");
            return;
        }
        
        Node startNode = path[0];
        if (startNode == null)
        {
            Debug.LogWarning("AlignToRoadDirection: Start node is null");
            return;
        }
        
        Vector3 roadDirection = startNode.GetSpawnDirection();
        
        if (roadDirection.magnitude > 0.1f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(roadDirection);
            transform.rotation = targetRotation;
            hasInitialAlignment = true;
            alignmentTime = 0f;
            
            if (debugIntersections)
                Debug.Log($"Car aligned to direction: {roadDirection}");
        }
    }
    
    void CheckRoadConstraints()
    {
        if (!strictRoadFollowing) return;
        
        // Check if we're too far from the road network
        float distanceToNearestNode = GetDistanceToNearestPathNode();
        
        if (distanceToNearestNode > roadWidthTolerance)
        {
            isOffRoad = true;
            
            if (debugRoadConstraint)
            {
                Debug.LogWarning($"Car {gameObject.name} is off-road! Distance to path: {distanceToNearestNode:F1}");
            }
        }
        else
        {
            isOffRoad = false;
        }
    }
    
    float GetDistanceToNearestPathNode()
    {
        if (path == null || path.Count == 0) return float.MaxValue;
        
        float nearestDistance = float.MaxValue;
        
        // Check current and next few nodes
        for (int i = currentNodeIndex; i < Mathf.Min(currentNodeIndex + 3, path.Count); i++)
        {
            if (path[i] != null)
            {
                float distance = Vector3.Distance(transform.position, path[i].transform.position);
                nearestDistance = Mathf.Min(nearestDistance, distance);
            }
        }
        
        return nearestDistance;
    }
    
    void DetectIntersections()
    {
        nearbyIntersectionNodes.Clear();
        isAtIntersection = false;
        
        // Check if current or upcoming nodes are intersections
        for (int i = currentNodeIndex; i < Mathf.Min(currentNodeIndex + 3, path.Count); i++)
        {
            if (path[i] == null) continue;
            
            float distanceToNode = Vector3.Distance(transform.position, path[i].transform.position);
            
            if (IsIntersectionNode(path[i]) && distanceToNode < intersectionDetectionRange)
            {
                nearbyIntersectionNodes.Add(path[i]);
                
                if (distanceToNode < intersectionStopDistance)
                {
                    isAtIntersection = true;
                }
            }
        }
        
        // Check for yield conditions at intersections
        shouldYieldAtIntersection = isAtIntersection && ShouldYieldAtIntersection();
        
        if (debugIntersections && nearbyIntersectionNodes.Count > 0)
        {
            Debug.Log($"Car detecting intersection. At intersection: {isAtIntersection}, Should yield: {shouldYieldAtIntersection}");
        }
    }
    
    bool IsIntersectionNode(Node node)
    {
        // A node is considered an intersection if it has more than 2 neighbors
        List<Node> neighbors = node.GetNeighbors();
        return neighbors.Count > 2;
    }
    
    bool ShouldYieldAtIntersection()
    {
        // Simple yield logic - yield if there's another car nearby
        Collider[] nearbyCars = Physics.OverlapSphere(transform.position, otherCarDetectionRange);
        
        foreach (Collider col in nearbyCars)
        {
            RealisticCar otherCar = col.GetComponent<RealisticCar>();
            if (otherCar != null && otherCar != this)
            {
                // Simple right-of-way: yield to cars coming from the right
                Vector3 relativePosition = otherCar.transform.position - transform.position;
                Vector3 localRelativePos = transform.InverseTransformPoint(otherCar.transform.position);
                
                // If other car is to our right and moving, yield
                if (localRelativePos.x > 0 && otherCar.isMoving)
                {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    void CheckForObstacles()
    {
        RaycastHit hit;
        Vector3 rayStart = transform.position + Vector3.up * 0.5f;
        
        if (Physics.Raycast(rayStart, transform.forward, out hit, detectionRange))
        {
            if (hit.collider != null && 
                !hit.collider.CompareTag("Ground") && 
                !hit.collider.CompareTag("Road") &&
                hit.collider.gameObject != gameObject)
            {
                isBlocked = true;
                return;
            }
        }
        
        isBlocked = false;
    }
    
    void CheckForOtherCars()
    {
        // Enhanced car detection for better traffic flow
        RaycastHit hit;
        Vector3 rayStart = transform.position + Vector3.up * 0.5f;
        
        // Check directly ahead
        if (Physics.Raycast(rayStart, transform.forward, out hit, detectionRange))
        {
            RealisticCar otherCar = hit.collider.GetComponent<RealisticCar>();
            if (otherCar != null)
            {
                // Calculate relative speed
                float relativeSpeed = Vector3.Dot(rb.linearVelocity - otherCar.rb.linearVelocity, transform.forward);
                
                // If we're approaching another car too fast, slow down
                if (relativeSpeed > 2f && hit.distance < detectionRange * 0.7f)
                {
                    isBlocked = true;
                }
            }
        }
    }
    
    void AIController()
    {
        if (currentNodeIndex >= path.Count)
        {
            isMoving = false;
            ApplyBraking(maxBrakeTorque);
            
            if (debugIntersections)
                Debug.Log($"Car reached destination: {targetEndNode.name}");
            
            Destroy(gameObject, despawnDelay);
            return;
        }
        
        // Get target position - STRICT ROAD FOLLOWING
        Vector3 targetPos = GetRoadConstrainedTarget();
        Vector3 localTarget = transform.InverseTransformPoint(targetPos);
        
        // Calculate steering with constraints
        float steerInput = 0f;
        if (localTarget.magnitude > 0.1f)
        {
            steerInput = Mathf.Clamp(localTarget.x / localTarget.magnitude, -1f, 1f);
            
            // Apply steering smoothing to prevent sharp turns off-road
            if (strictRoadFollowing)
            {
                float maxSteerChange = maxSteerAnglePerFrame * Time.deltaTime;
                float steerChange = steerInput - lastSteerAngle;
                steerChange = Mathf.Clamp(steerChange, -maxSteerChange, maxSteerChange);
                steerInput = lastSteerAngle + steerChange;
                lastSteerAngle = steerInput;
            }
            
            // Less aggressive steering for road following
            steerInput = Mathf.Sign(steerInput) * Mathf.Pow(Mathf.Abs(steerInput), 0.8f);
        }
        
        float distanceToCurrentNode = Vector3.Distance(transform.position, path[currentNodeIndex].transform.position);
        
        // ENHANCED: Detect if we're on a straight road
        bool isOnStraightRoad = enableSpeedBoost && IsRoadStraight();
        
        // Speed calculation with dynamic speed control
        float speedMultiplier = 1f;
        
        // SPEED BOOST: Accelerate on straight roads
        if (isOnStraightRoad)
        {
            speedMultiplier *= straightRoadSpeedMultiplier;
            
            if (debugSpeed)
                Debug.Log($"SPEED BOOST: Straight road detected! Speed: {speedMultiplier:F1}x");
        }
        
        // NEW: Lane change blocking - slow down if waiting for lane change
        if (isLaneChangeBlocked || isWaitingForLaneChange)
        {
            speedMultiplier *= laneChangeSlowdownFactor;
            
            if (debugLaneChanges)
            {
                Debug.Log($"LANE CHANGE WAIT: Slowing to {speedMultiplier:F1}x speed");
            }
        }
        
        // Slow down if off-road
        if (isOffRoad)
        {
            speedMultiplier *= 0.2f; // Very slow when off-road
            
            if (debugRoadConstraint)
                Debug.Log("Car off-road - slowing down");
        }
        
        // Intersection slowdown
        if (nearbyIntersectionNodes.Count > 0)
        {
            speedMultiplier *= intersectionSlowdownFactor;
            
            if (debugSpeed)
                Debug.Log($"Intersection ahead - slowing to {speedMultiplier:F1}x");
        }
        
        // Stop at intersection if should yield
        if (shouldYieldAtIntersection)
        {
            speedMultiplier = 0f;
        }
        
        // Turn slowdown - more aggressive for sharp turns
        float turnSharpness = Mathf.Abs(steerInput);
        if (turnSharpness > 0.3f)
        {
            float turnPenalty = Mathf.Lerp(1f, turnSlowdownFactor, (turnSharpness - 0.3f) / 0.7f);
            speedMultiplier *= turnPenalty;
            
            if (debugSpeed && turnPenalty < 0.8f)
                Debug.Log($"Sharp turn detected - slowing to {speedMultiplier:F1}x");
        }
        
        // Approach slowdown
        if (distanceToCurrentNode < stoppingDistance * 2f)
        {
            float approachPenalty = Mathf.Clamp01(distanceToCurrentNode / (stoppingDistance * 2f));
            speedMultiplier *= approachPenalty;
        }
        
        // Final motor input calculation
        bool shouldStop = isBlocked || shouldYieldAtIntersection || (isLaneChangeBlocked && !isWaitingForLaneChange);
        float motorInput = shouldStop ? 0f : speedMultiplier;
        
        ApplySteering(steerInput);
        ApplyMotor(motorInput);
        
        // Node progression - only advance if close enough AND not blocked by lane change
        if (distanceToCurrentNode < nodeReachDistance && !isLaneChangeBlocked)
        {
            currentNodeIndex++;
            
            if (debugIntersections)
                Debug.Log($"Reached node {currentNodeIndex-1}, moving to node {currentNodeIndex}");
        }
    }
    
    Vector3 GetRoadConstrainedTarget()
    {
        if (currentNodeIndex >= path.Count) return transform.position;
        
        // STRICT: Only target the current node or next immediate node
        Vector3 currentNodePos = path[currentNodeIndex].transform.position;
        
        // If we're close to current node, look at next node
        float distanceToCurrent = Vector3.Distance(transform.position, currentNodePos);
        
        if (distanceToCurrent < nodeReachDistance * 1.5f && currentNodeIndex + 1 < path.Count)
        {
            return path[currentNodeIndex + 1].transform.position;
        }
        
        return currentNodePos;
    }
    
    bool IsRoadStraight()
    {
        if (path == null || currentNodeIndex >= path.Count - 2) return false;
        
        // Look ahead at next few nodes to determine if road is straight
        int lookAheadNodes = Mathf.Min(4, path.Count - currentNodeIndex - 1);
        
        if (lookAheadNodes < 2) return false;
        
        // Get current direction
        Vector3 currentPos = transform.position;
        Vector3 currentDirection = transform.forward;
        
        float totalStraightDistance = 0f;
        bool isStraight = true;
        
        // Check each upcoming road segment
        for (int i = currentNodeIndex; i < currentNodeIndex + lookAheadNodes - 1; i++)
        {
            if (i + 1 >= path.Count) break;
            
            Vector3 segmentStart = path[i].transform.position;
            Vector3 segmentEnd = path[i + 1].transform.position;
            Vector3 segmentDirection = (segmentEnd - segmentStart).normalized;
            
            // Calculate angle between current direction and this segment
            float angle = Vector3.Angle(currentDirection, segmentDirection);
            
            // If any segment has a sharp turn, this isn't a straight road
            if (angle > turnDetectionAngle)
            {
                isStraight = false;
                break;
            }
            
            // Add to total straight distance
            totalStraightDistance += Vector3.Distance(segmentStart, segmentEnd);
            
            // Update current direction for next iteration
            currentDirection = segmentDirection;
        }
        
        // Must be straight AND have sufficient distance ahead
        bool hasEnoughDistance = totalStraightDistance >= straightRoadMinDistance;
        
        // Extra check: make sure we're not approaching an intersection
        bool noIntersectionAhead = nearbyIntersectionNodes.Count == 0;
        
        if (debugSpeed && isStraight && hasEnoughDistance && noIntersectionAhead)
        {
            Debug.Log($"Straight road detected: {totalStraightDistance:F1}m ahead, max angle: {turnDetectionAngle}°");
        }
        
        return isStraight && hasEnoughDistance && noIntersectionAhead;
    }
    
    void RecalculatePath()
    {
        if (targetEndNode == null) return;
        
        Node nearestNode = FindNearestNode();
        if (nearestNode == null) return;
        
        List<Node> newPath = PathFinder.FindPath(nearestNode, targetEndNode);
        
        if (newPath != null && newPath.Count > 0)
        {
            path = newPath;
            currentNodeIndex = 0;
            
            // Reset lane change state when path changes
            isWaitingForLaneChange = false;
            isLaneChangeBlocked = false;
            targetLaneNode = null;
            
            if (debugIntersections)
                Debug.Log($"Recalculated path: {newPath.Count} nodes");
        }
    }
    
    Node FindNearestNode()
    {
        Node[] allNodes = FindObjectsOfType<Node>();
        Node nearestNode = null;
        float nearestDistance = float.MaxValue;
        
        foreach (Node node in allNodes)
        {
            if (node == null) continue;
            
            float distance = Vector3.Distance(transform.position, node.transform.position);
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearestNode = node;
            }
        }
        
        return nearestNode;
    }
    
    void ApplySteering(float steerInput)
    {
        if (frontLeftWheel == null || frontRightWheel == null) return;
        
        if (!ShouldAllowRotationControl())
        {
            frontLeftWheel.steerAngle = 0;
            frontRightWheel.steerAngle = 0;
            return;
        }
        
        float steerAngle = steerInput * maxSteerAngle;
        frontLeftWheel.steerAngle = steerAngle;
        frontRightWheel.steerAngle = steerAngle;
    }
    
    void ApplyMotor(float motorInput)
    {
        if (rearLeftWheel == null || rearRightWheel == null) return;
        
        float motor = motorInput * maxMotorTorque;
        
        rearLeftWheel.motorTorque = motor;
        rearRightWheel.motorTorque = motor;
        
        if (motorInput == 0 || isBlocked)
        {
            ApplyBraking(maxBrakeTorque * 0.7f);
        }
        else
        {
            ApplyBraking(0);
        }
    }
    
    bool ShouldAllowRotationControl()
    {
        return !hasInitialAlignment || alignmentTime > 0.5f;
    }
    
    void ApplyBraking(float brakeForce)
    {
        WheelCollider[] wheels = { frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel };
        
        foreach (WheelCollider wheel in wheels)
        {
            if (wheel != null)
            {
                wheel.brakeTorque = brakeForce;
            }
        }
    }
    
    void UpdateWheelMeshes()
    {
        UpdateWheelMesh(frontLeftWheel, frontLeftMesh);
        UpdateWheelMesh(frontRightWheel, frontRightMesh);
        UpdateWheelMesh(rearLeftWheel, rearLeftMesh);
        UpdateWheelMesh(rearRightWheel, rearRightMesh);
    }
    
    void UpdateWheelMesh(WheelCollider wheelCollider, Transform wheelMesh)
    {
        if (wheelCollider == null || wheelMesh == null) return;
        
        Vector3 position;
        Quaternion rotation;
        
        wheelCollider.GetWorldPose(out position, out rotation);
        
        wheelMesh.position = position;
        wheelMesh.rotation = rotation;
    }
    
    void OnDrawGizmos()
    {
        // Obstacle detection ray
        Gizmos.color = isBlocked ? Color.red : Color.green;
        Vector3 rayStart = transform.position + Vector3.up * 0.5f;
        Gizmos.DrawRay(rayStart, transform.forward * detectionRange);
        
        // Road constraint indicator
        if (isOffRoad)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, roadWidthTolerance);
        }
        
        // Speed boost indicator
        if (enableSpeedBoost && IsRoadStraight())
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(transform.position + Vector3.up * 2f, Vector3.one * 0.5f);
            
            // Draw speed boost effect
            Gizmos.color = new Color(1f, 1f, 0f, 0.3f);
            Gizmos.DrawSphere(transform.position, 1.5f);
        }
        
        // NEW: Lane change visualization
        if (targetLaneNode != null)
        {
            if (isLaneChangeBlocked || isWaitingForLaneChange)
            {
                // Red if blocked/waiting
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(targetLaneNode.transform.position, targetLaneCheckRadius);
                
                // Draw line to blocked lane
                Gizmos.color = new Color(1f, 0f, 0f, 0.5f);
                Gizmos.DrawLine(transform.position, targetLaneNode.transform.position);
                
                // Draw wait indicator above car
                Gizmos.color = Color.red;
                Gizmos.DrawWireCube(transform.position + Vector3.up * 2.5f, Vector3.one * 0.3f);
            }
            else
            {
                // Green if clear
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(targetLaneNode.transform.position, targetLaneCheckRadius);
                
                // Draw line to clear lane
                Gizmos.color = new Color(0f, 1f, 0f, 0.5f);
                Gizmos.DrawLine(transform.position, targetLaneNode.transform.position);
            }
        }
        
        // Current target (should be very close to current node)
        if (isMoving && currentNodeIndex < path.Count)
        {
            Vector3 target = GetRoadConstrainedTarget();
            
            // Green sphere for current target
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(target, 0.8f);
            
            // Line to target
            Gizmos.color = Color.white;
            Gizmos.DrawLine(transform.position, target);
        }
        
        // Path visualization - only show immediate path
        if (path != null && path.Count > 1 && currentNodeIndex < path.Count)
        {
            Gizmos.color = Color.blue;
            
            // Only draw next few nodes to prevent confusion
            for (int i = currentNodeIndex; i < Mathf.Min(currentNodeIndex + 3, path.Count - 1); i++)
            {
                if (path[i] != null && path[i + 1] != null)
                {
                    Gizmos.DrawLine(path[i].transform.position, path[i + 1].transform.position);
                }
            }
        }
        
        // Intersection detection
        if (nearbyIntersectionNodes.Count > 0)
        {
            Gizmos.color = shouldYieldAtIntersection ? Color.red : new Color(1f, 0.5f, 0f);
            Gizmos.DrawWireSphere(transform.position, intersectionDetectionRange);
        }
        
        // Other car detection range
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, otherCarDetectionRange);
    }
}