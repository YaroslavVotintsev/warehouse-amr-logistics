using UnityEngine;

public class CameraController2D : MonoBehaviour
{
    [Header("Target Settings")]
    public Transform target;           // The player transform
    public Vector2 offset = new Vector2(0, 0);
    
    [Header("Pan Settings")]
    public bool enablePan = true;
    public float panSpeed = 20f;
    public float panBorderThickness = 10f;  // Edge panning
    public bool useEdgePanning = false;
    public Vector2 panLimitMin = new Vector2(-100, -100);
    public Vector2 panLimitMax = new Vector2(100, 100);
    
    [Header("Zoom Settings")]
    public bool enableZoom = true;
    public float zoomSpeed = 5f;
    public float minZoom = 3f;
    public float maxZoom = 15f;
    
    [Header("Follow Settings")]
    public bool followPlayer = true;
    public float followSmoothSpeed = 5f;
    
    private Camera cam;
    private Vector3 dragOrigin;
    private bool isDragging = false;
    private float currentZoom;
    
    void Start()
    {
        cam = GetComponent<Camera>();
        if (cam == null) cam = Camera.main;
        
        currentZoom = cam.orthographicSize;
        
        // If no target assigned, try to find player
        if (target == null)
        {
            GameObject player = GameObject.FindGameObjectWithTag("Player");
            if (player != null) target = player.transform;
        }
    }
    
    void LateUpdate()
    {
        HandleZoom();
        HandlePan();
        HandleFollow();
    }
    
    void HandleZoom()
    {
        if (!enableZoom) return;
        
        float scrollInput = Input.GetAxis("Mouse ScrollWheel");
        if (scrollInput != 0)
        {
            currentZoom -= scrollInput * zoomSpeed;
            currentZoom = Mathf.Clamp(currentZoom, minZoom, maxZoom);
            cam.orthographicSize = currentZoom;
        }
    }
    
    void HandlePan()
    {
        if (!enablePan) return;
        
        // Mouse drag panning (middle mouse button or right click)
        if (Input.GetMouseButtonDown(2) || Input.GetMouseButtonDown(1))
        {
            dragOrigin = cam.ScreenToWorldPoint(Input.mousePosition);
            isDragging = true;
        }
        
        if (Input.GetMouseButtonUp(2) || Input.GetMouseButtonUp(1))
        {
            isDragging = false;
        }
        
        if (isDragging)
        {
            Vector3 difference = dragOrigin - cam.ScreenToWorldPoint(Input.mousePosition);
            Vector3 newPosition = transform.position + difference;
            newPosition = ClampCameraPosition(newPosition);
            transform.position = newPosition;
        }
        
        // Edge panning (move mouse to screen edges)
        if (useEdgePanning && !isDragging && !followPlayer)
        {
            Vector3 moveDirection = Vector3.zero;
            
            if (Input.mousePosition.x >= Screen.width - panBorderThickness)
                moveDirection.x += 1;
            if (Input.mousePosition.x <= panBorderThickness)
                moveDirection.x -= 1;
            if (Input.mousePosition.y >= Screen.height - panBorderThickness)
                moveDirection.y += 1;
            if (Input.mousePosition.y <= panBorderThickness)
                moveDirection.y -= 1;
            
            if (moveDirection != Vector3.zero)
            {
                Vector3 newPosition = transform.position + moveDirection * panSpeed * Time.deltaTime;
                newPosition = ClampCameraPosition(newPosition);
                transform.position = newPosition;
            }
        }
        
        // WASD/Arrow keys panning
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");
        
        if ((horizontal != 0 || vertical != 0) && !followPlayer)
        {
            Vector3 move = new Vector3(horizontal, vertical, 0) * panSpeed * Time.deltaTime;
            Vector3 newPosition = transform.position + move;
            newPosition = ClampCameraPosition(newPosition);
            transform.position = newPosition;
        }
    }
    
    void HandleFollow()
    {
        if (!followPlayer || target == null) return;
        
        Vector3 targetPosition = new Vector3(
            target.position.x + offset.x,
            target.position.y + offset.y,
            transform.position.z
        );
        
        // Clamp to limits if needed
        targetPosition = ClampCameraPosition(targetPosition);
        
        transform.position = Vector3.Lerp(
            transform.position, 
            targetPosition, 
            followSmoothSpeed * Time.deltaTime
        );
    }
    
    Vector3 ClampCameraPosition(Vector3 position)
    {
        // Calculate camera bounds based on orthographic size
        float vertExtent = cam.orthographicSize;
        float horzExtent = vertExtent * Screen.width / Screen.height;
        
        float minX = panLimitMin.x + horzExtent;
        float maxX = panLimitMax.x - horzExtent;
        float minY = panLimitMin.y + vertExtent;
        float maxY = panLimitMax.y - vertExtent;
        
        position.x = Mathf.Clamp(position.x, minX, maxX);
        position.y = Mathf.Clamp(position.y, minY, maxY);
        
        return position;
    }
    
    // Public methods for external control
    public void CenterOnPlayer()
    {
        if (target != null)
        {
            Vector3 targetPos = new Vector3(target.position.x + offset.x, target.position.y + offset.y, transform.position.z);
            transform.position = ClampCameraPosition(targetPos);
        }
    }
    
    public void SetFollow(bool follow)
    {
        followPlayer = follow;
    }
    
    public void SetZoom(float zoomLevel)
    {
        currentZoom = Mathf.Clamp(zoomLevel, minZoom, maxZoom);
        cam.orthographicSize = currentZoom;
    }
}