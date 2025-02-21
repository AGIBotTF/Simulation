using UnityEngine;

public class SimpleCameraController : MonoBehaviour
{
    public float movementSpeed = 5.0f;      // Base movement speed
    public float fastMovementSpeed = 15.0f; // Movement speed when Shift is held down
    public float rotationSpeed = 2.0f;      // Mouse rotation sensitivity
    public float upDownSpeed = 5.0f;        // Up/Down movement speed when Space or Ctrl is held down

    private float yaw = 0.0f;  // Horizontal rotation
    private float pitch = 0.0f; // Vertical rotation

    void Start()
    {
        // Lock and hide the cursor
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;

        // Initialize rotation values
        yaw = transform.eulerAngles.y;
        pitch = transform.eulerAngles.x;
    }

    void Update()
    {
        // Exit the camera control if Escape key is pressed
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            // Unlock and show the cursor
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
            return;
        }

        // Rotation
        float mouseMovementX = Input.GetAxis("Mouse X") * rotationSpeed;
        float mouseMovementY = Input.GetAxis("Mouse Y") * rotationSpeed;

        yaw += mouseMovementX;
        pitch -= mouseMovementY;
        pitch = Mathf.Clamp(pitch, -90f, 90f); // Limit the pitch to avoid flipping

        transform.eulerAngles = new Vector3(pitch, yaw, 0.0f);

        // Movement
        float speed = Input.GetKey(KeyCode.LeftShift) ? fastMovementSpeed : movementSpeed;

        // Forward and backward movement
        if (Input.GetKey(KeyCode.W))
        {
            transform.position += transform.forward * speed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.S))
        {
            transform.position -= transform.forward * speed * Time.deltaTime;
        }

        // Left and right movement
        if (Input.GetKey(KeyCode.A))
        {
            transform.position -= transform.right * speed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.D))
        {
            transform.position += transform.right * speed * Time.deltaTime;
        }

        // Up and down movement
        if (Input.GetKey(KeyCode.Space))
        {
            transform.position += Vector3.up * upDownSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.C))
        {
            transform.position -= Vector3.up * upDownSpeed * Time.deltaTime;
        }
    }
}
