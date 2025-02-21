using UnityEngine;

public class LegController : MonoBehaviour
{
    public Transform jointY; // Assign the Y joint in the Inspector
    public Transform jointZ1; // Assign the first Z joint in the Inspector
    public Transform jointZ2; // Assign the second Z joint in the Inspector

    public float rotationSpeed = 1f; // Speed of rotation

    void FixedUpdate()
    {
        // Rotate jointY on its chosen axis (e.g., X-axis)
        if (Input.GetKey(KeyCode.Q))
        {
            jointY.Rotate(Vector3.right * rotationSpeed );
        }
        if (Input.GetKey(KeyCode.E))
        {
            jointY.Rotate(Vector3.right * -rotationSpeed );
        }

        // Rotate jointZ1 on its chosen axis (e.g., Y-axis)
        if (Input.GetKey(KeyCode.A))
        {
            jointZ1.Rotate(Vector3.up * rotationSpeed );
        }
        if (Input.GetKey(KeyCode.D))
        {
            jointZ1.Rotate(Vector3.up * -rotationSpeed );
        }

        // Rotate jointZ2 on its chosen axis (e.g., Z-axis)
        if (Input.GetKey(KeyCode.Z))
        {
            jointZ2.Rotate(Vector3.forward * rotationSpeed );
        }
        if (Input.GetKey(KeyCode.C))
        {
            jointZ2.Rotate(Vector3.forward * -rotationSpeed );
        }
    }
}

