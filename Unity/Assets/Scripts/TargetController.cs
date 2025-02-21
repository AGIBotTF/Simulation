using UnityEngine;

public class TargetController : MonoBehaviour
{
    public float movementSpeed = 2.0f;  // Movement speed of the target
    public float verticalSpeed = 2.0f;  // Vertical movement speed of the target
    public float boundaryRadius = 5.0f; // Radius of the movement boundary

    private Vector3 startPosition;

    void Start()
    {
        // Store the initial position of the target
        startPosition = transform.position;
    }

    void Update()
    {
        // Get input from arrow keys for horizontal movement
        float moveHorizontal = 0f;
        float moveVertical = 0f;

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            moveHorizontal = -1f;
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            moveHorizontal = 1f;
        }
        if (Input.GetKey(KeyCode.UpArrow))
        {
            moveVertical = 1f;
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            moveVertical = -1f;
        }

        // Get input from 'E' and 'Q' keys for vertical movement
        float moveUpDown = 0f;

        if (Input.GetKey(KeyCode.E) || Input.GetKey(KeyCode.PageUp))
        {
            moveUpDown = 1f;
        }
        if (Input.GetKey(KeyCode.Q) || Input.GetKey(KeyCode.PageDown))
        {
            moveUpDown = -1f;
        }

        // Create movement vector
        Vector3 movement = new Vector3(moveHorizontal, moveUpDown, moveVertical) * movementSpeed * Time.deltaTime;

        // Update the target's position
        transform.position += movement;

        // Enforce movement boundaries
        Vector3 offsetFromStart = transform.position - startPosition;
        if (offsetFromStart.magnitude > boundaryRadius)
        {
            // Clamp the position to stay within the boundary
            offsetFromStart = offsetFromStart.normalized * boundaryRadius;
            transform.position = startPosition + offsetFromStart;
        }
    }
}
