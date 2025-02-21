using UnityEngine;

[RequireComponent(typeof(HingeJoint))]
public class StepperMotorSimulation : MonoBehaviour
{
    [Header("Stepper Motor Settings")]
    [Tooltip("How much the joint angle changes per step in degrees.")]
    public float stepAngle = 5f;
    
    [Tooltip("Spring force used to hold the target angle.")]
    public float springForce = 1000f;
    
    [Tooltip("Damping applied to stabilize the joint at the target angle.")]
    public float damping = 100f;
    
    [Header("Debug/Testing Controls")]
    [Tooltip("Optional: Set an initial target angle at Start.")]
    public float initialTargetAngle = 0f;

    private HingeJoint hinge;
    private JointSpring jointSpring;
    private float currentTargetAngle = 0f;

    void Start()
    {
        hinge = GetComponent<HingeJoint>();

        // Configure the hinge joint to use springs
        hinge.useSpring = true;
        
        // Set initial joint spring settings
        jointSpring = hinge.spring;
        jointSpring.spring = springForce;
        jointSpring.damper = damping;
        hinge.spring = jointSpring;

        // Set the initial angle if desired
        SetTargetAngle(initialTargetAngle);
    }

    void Update()
    {
        // Example input controls (for testing):
        // Press E to step forward, Q to step backward.
        if (Input.GetKey(KeyCode.E))
        {
            StepForward();
			Debug.Log("Current rotation: " + hinge.angle);
        }
        else if (Input.GetKey(KeyCode.Q))
        {
            StepBackward();
			Debug.Log("Current rotation: " + hinge.angle);
        }
    }

    /// <summary>
    /// Moves the hinge joint one "step" forward (increasing the target angle).
    /// </summary>
    public void StepForward()
    {
        SetTargetAngle(currentTargetAngle + stepAngle);
    }

    /// <summary>
    /// Moves the hinge joint one "step" backward (decreasing the target angle).
    /// </summary>
    public void StepBackward()
    {
        SetTargetAngle(currentTargetAngle - stepAngle);
    }

    /// <summary>
    /// Sets the hinge joint to a specific target angle.
    /// </summary>
    /// <param name="angle">The angle in degrees to set the hinge joint to.</param>
    public void SetTargetAngle(float angle)
    {
        currentTargetAngle = angle;

        // Update the joint spring’s target position.
        jointSpring.targetPosition = currentTargetAngle;
        hinge.spring = jointSpring;
    }
}

