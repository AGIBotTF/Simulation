using UnityEngine;

public class CustomJoint : MonoBehaviour
{
    [Header("Joint Settings")]
    [Tooltip("The empty GameObject acting as the joint anchor.")]
    public Transform jointAnchor;

    [Tooltip("Rotation speed in degrees per second.")]
    public float rotationSpeed = 90f;

    [Header("Collision Detection")]
    [Tooltip("Radius for collision detection.")]
    public float collisionRadius = 0.1f;

    // Target rotation
    private Quaternion targetRotation;

    // Rotation state
    private bool isRotating = false;

    // Reference to the Rigidbody (calf)
    private Rigidbody calfRigidbody;

    // Initial local position of the calf
    private Vector3 initialLocalPosition;

    void Start()
    {
        if (jointAnchor == null)
        {
            Debug.LogError("Joint Anchor is not assigned.");
            return;
        }

        // Get the Rigidbody component of the calf
        calfRigidbody = GetComponent<Rigidbody>();
        if (calfRigidbody == null)
        {
            Debug.LogError("CustomJoint script must be attached to a GameObject with a Rigidbody.");
            return;
        }

        // Make the Rigidbody kinematic to prevent it from being affected by physics
        calfRigidbody.isKinematic = true;

        // Store the initial local position to maintain consistent positioning
        initialLocalPosition = calfRigidbody.transform.localPosition;

        // Initialize target rotation
        targetRotation = jointAnchor.localRotation;
    }

    void Update()
    {
        if (isRotating)
        {
            RotateTowardsTarget();
        }

		if (Input.GetKeyDown(KeyCode.Space))
		{
			RotateJoint(Quaternion.Euler(0, 0, -180));
		}
    }

    /// <summary>
    /// Initiates rotation towards the specified target rotation.
    /// </summary>
    /// <param name="newTargetRotation">The desired rotation as a Quaternion.</param>
    public void RotateJoint(Quaternion newTargetRotation)
    {
        targetRotation = newTargetRotation;
        isRotating = true;
    }

    /// <summary>
    /// Initiates rotation towards the specified target Euler angles.
    /// </summary>
    /// <param name="newTargetEuler">The desired rotation as Euler angles.</param>
    public void RotateJoint(Vector3 newTargetEuler)
    {
        targetRotation = Quaternion.Euler(newTargetEuler);
        isRotating = true;
    }

    private void RotateTowardsTarget()
    {
        // Smoothly rotate the joint anchor towards the target rotation
        jointAnchor.localRotation = Quaternion.RotateTowards(
            jointAnchor.localRotation,
            targetRotation,
            rotationSpeed * Time.deltaTime
        );

        // Update the calf's local position to maintain the initial offset
        calfRigidbody.transform.localPosition = initialLocalPosition;

        // Optionally, synchronize the calf's rotation with the joint anchor
        // Uncomment the following line if you want the calf to inherit the joint's rotation
        // calfRigidbody.transform.localRotation = jointAnchor.localRotation;

        // Check for collisions at the calf's current position
        if (CheckCollision())
        {
            Debug.LogWarning("Collision detected during joint rotation!");
            isRotating = false; // Stop rotating on collision
            return;
        }

        // Check if the rotation is complete
        if (Quaternion.Angle(jointAnchor.localRotation, targetRotation) < 0.1f)
        {
            isRotating = false;
        }
    }

    private bool CheckCollision()
    {
        // Use OverlapSphere to detect any colliders overlapping with the calf's position
        Collider[] hits = Physics.OverlapSphere(calfRigidbody.transform.position, collisionRadius);

        // Iterate through all colliders found
        foreach (Collider hit in hits)
        {
            // Ignore the calf's own collider to prevent self-collision
            if (hit.attachedRigidbody == calfRigidbody)
                continue;

            // Collision detected
            return true;
        }

        // No collision detected
        return false;
    }

    // Optional: Visualize the collision detection in the editor
    void OnDrawGizmosSelected()
    {
        if (jointAnchor == null) return;

        Gizmos.color = Color.red;
        Vector3 position = calfRigidbody != null ? calfRigidbody.transform.position : jointAnchor.position;
        Gizmos.DrawWireSphere(position, collisionRadius);
    }
}

