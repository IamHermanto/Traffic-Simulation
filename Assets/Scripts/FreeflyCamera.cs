using UnityEngine;

public class FreeFlyCamera : MonoBehaviour
{
    [Header("Movement Settings")]
    [Tooltip("Base movement speed")]
    public float moveSpeed = 10f;
    
    [Tooltip("Speed when holding Shift (sprint)")]
    public float sprintMultiplier = 2f;
    
    [Tooltip("Speed when holding Ctrl (slow)")]
    public float slowMultiplier = 0.3f;
    
    [Tooltip("How fast the camera accelerates to target speed")]
    public float acceleration = 5f;
    
    [Tooltip("How much drag is applied when no input is given")]
    public float drag = 5f;
    
    [Header("Mouse Look Settings")]
    [Tooltip("Mouse sensitivity for looking around")]
    public float mouseSensitivity = 2f;
    
    [Tooltip("Limit how far up/down you can look (in degrees)")]
    public float maxLookAngle = 90f;
    
    [Tooltip("Should the cursor be locked to the center of screen?")]
    public bool lockCursor = true;
    
    [Header("Vertical Movement")]
    [Tooltip("Key for moving up (default: Space)")]
    public KeyCode upKey = KeyCode.Space;
    
    [Tooltip("Key for moving down (default: Left Ctrl)")]
    public KeyCode downKey = KeyCode.LeftControl;
    
    [Tooltip("Alternative down key (C key)")]
    public KeyCode downKeyAlt = KeyCode.C;
    
    [Header("Controls Info")]
    [Tooltip("Show controls in console on start")]
    public bool showControlsOnStart = true;
    
    // Private variables
    private Vector3 velocity = Vector3.zero;
    private float verticalRotation = 0f;
    private bool isControlEnabled = true;
    
    void Start()
    {
        // Lock cursor if enabled
        if (lockCursor)
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
        
        // Show controls
        if (showControlsOnStart)
        {
            ShowControls();
        }
        
        // Get initial rotation
        verticalRotation = transform.eulerAngles.x;
        
        Debug.Log("🎮 Free Fly Camera activated! Press ESC to toggle cursor lock.");
    }
    
    void Update()
    {
        // Toggle cursor lock with Escape
        HandleCursorToggle();
        
        // Only process movement if controls are enabled
        if (isControlEnabled)
        {
            HandleMouseLook();
            HandleMovement();
        }
    }
    
    void HandleCursorToggle()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            lockCursor = !lockCursor;
            
            if (lockCursor)
            {
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
                isControlEnabled = true;
                Debug.Log("🔒 Cursor locked - Camera controls enabled");
            }
            else
            {
                Cursor.lockState = CursorLockMode.None;
                Cursor.visible = true;
                isControlEnabled = false;
                Debug.Log("🔓 Cursor unlocked - Camera controls disabled");
            }
        }
    }
    
    void HandleMouseLook()
    {
        if (!lockCursor) return;
        
        // Get mouse movement
        float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
        float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;
        
        // Rotate horizontally
        transform.Rotate(Vector3.up * mouseX);
        
        // Rotate vertically (with clamping)
        verticalRotation -= mouseY;
        verticalRotation = Mathf.Clamp(verticalRotation, -maxLookAngle, maxLookAngle);
        
        // Apply vertical rotation
        transform.rotation = Quaternion.Euler(verticalRotation, transform.eulerAngles.y, 0);
    }
    
    void HandleMovement()
    {
        // Get input
        Vector3 inputVector = GetMovementInput();
        
        // Calculate target velocity
        Vector3 targetVelocity = CalculateTargetVelocity(inputVector);
        
        // Smooth velocity change
        velocity = Vector3.Lerp(velocity, targetVelocity, acceleration * Time.deltaTime);
        
        // Apply drag when no input
        if (inputVector.magnitude < 0.1f)
        {
            velocity = Vector3.Lerp(velocity, Vector3.zero, drag * Time.deltaTime);
        }
        
        // Move the camera
        transform.position += velocity * Time.deltaTime;
    }
    
    Vector3 GetMovementInput()
    {
        Vector3 input = Vector3.zero;
        
        // WASD movement (relative to camera rotation)
        if (Input.GetKey(KeyCode.W))
            input += transform.forward;
        if (Input.GetKey(KeyCode.S))
            input -= transform.forward;
        if (Input.GetKey(KeyCode.A))
            input -= transform.right;
        if (Input.GetKey(KeyCode.D))
            input += transform.right;
        
        // Vertical movement (world space)
        if (Input.GetKey(upKey))
            input += Vector3.up;
        if (Input.GetKey(downKey) || Input.GetKey(downKeyAlt))
            input += Vector3.down;
        
        return input;
    }
    
    Vector3 CalculateTargetVelocity(Vector3 inputVector)
    {
        if (inputVector.magnitude < 0.1f)
            return Vector3.zero;
        
        // Normalize input to prevent faster diagonal movement
        Vector3 normalizedInput = inputVector.normalized;
        
        // Calculate speed multiplier based on held keys
        float currentSpeed = moveSpeed;
        
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
        {
            currentSpeed *= sprintMultiplier;
        }
        else if (Input.GetKey(KeyCode.LeftControl) && !Input.GetKey(downKey))
        {
            // Only apply slow multiplier if Ctrl isn't being used for down movement
            currentSpeed *= slowMultiplier;
        }
        
        return normalizedInput * currentSpeed;
    }
    
    void ShowControls()
    {
        Debug.Log("=== 🎮 FREE FLY CAMERA CONTROLS ===");
        Debug.Log("🔄 Mouse: Look around");
        Debug.Log("⬆️ W: Move forward");
        Debug.Log("⬇️ S: Move backward");
        Debug.Log("⬅️ A: Move left");
        Debug.Log("➡️ D: Move right");
        Debug.Log($"🔼 {upKey}: Fly up");
        Debug.Log($"🔽 {downKey}/{downKeyAlt}: Fly down");
        Debug.Log("🏃 Hold Shift: Sprint (faster movement)");
        Debug.Log("🐌 Hold Ctrl: Slow movement");
        Debug.Log("🔄 ESC: Toggle cursor lock");
        Debug.Log("=====================================");
    }
    
    [ContextMenu("Reset Position")]
    void ResetPosition()
    {
        transform.position = Vector3.zero;
        transform.rotation = Quaternion.identity;
        velocity = Vector3.zero;
        verticalRotation = 0f;
        Debug.Log("📍 Camera position reset to origin");
    }
    
    [ContextMenu("Show Current Status")]
    void ShowCurrentStatus()
    {
        Debug.Log("=== 📊 CAMERA STATUS ===");
        Debug.Log($"Position: {transform.position}");
        Debug.Log($"Rotation: {transform.eulerAngles}");
        Debug.Log($"Velocity: {velocity} (Speed: {velocity.magnitude:F1})");
        Debug.Log($"Cursor Locked: {lockCursor}");
        Debug.Log($"Controls Enabled: {isControlEnabled}");
        Debug.Log("========================");
    }
    
    // Public methods for external control
    public void SetCameraSpeed(float newSpeed)
    {
        moveSpeed = newSpeed;
        Debug.Log($"🎯 Camera speed set to: {newSpeed}");
    }
    
    public void TeleportTo(Vector3 position)
    {
        transform.position = position;
        velocity = Vector3.zero;
        Debug.Log($"📍 Camera teleported to: {position}");
    }
    
    public void TeleportTo(Transform target)
    {
        TeleportTo(target.position);
    }
    
    public void SetMouseSensitivity(float sensitivity)
    {
        mouseSensitivity = sensitivity;
        Debug.Log($"🖱️ Mouse sensitivity set to: {sensitivity}");
    }
    
    public void ToggleControls()
    {
        isControlEnabled = !isControlEnabled;
        Debug.Log($"🎮 Camera controls: {(isControlEnabled ? "Enabled" : "Disabled")}");
    }
    
    void OnGUI()
    {
        // Show basic controls in top-left corner
        if (!lockCursor)
        {
            GUI.Box(new Rect(10, 10, 200, 60), "Camera Controls Disabled\nPress ESC to enable");
        }
        else
        {
            GUI.Box(new Rect(10, 10, 200, 80), $"🎮 Free Fly Camera\nSpeed: {velocity.magnitude:F1}\nPress ESC for cursor");
        }
    }
}