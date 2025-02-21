using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;

public class HumanoidAgentStand : Agent
{
    [Header("Body Parts")]
    public Transform torso;

    [Header("Left Leg Joints")]
    public HingeJoint leftHipJoint;
    public HingeJoint leftKneeJoint; 

    [Header("Right Leg Joints")]
	public HingeJoint rightHipJoint;
	public HingeJoint rightKneeJoint;
	
	[Header("Joint's script")]
	public MotorController leftHipScript;
	public MotorController rightHipScript;
	public MotorController leftKneeScript;
	public MotorController rightKneeScript;

    [Header("Feet Colliders")]
    public Collider leftFootCollider;
    public Collider rightFootCollider;

    [Header("Environment")]
    public Collider floorCollider;

    private Rigidbody[] allRigidbodies;
    private Dictionary<Transform, Vector3> initialPositions = new Dictionary<Transform, Vector3>();
    private Dictionary<Transform, Quaternion> initialRotations = new Dictionary<Transform, Quaternion>();

    private bool hasFallen = false;

    public float maxJointForce = 100f;
    public float maxJointSpeed = 100f;

	private Dictionary<HingeJoint, Quaternion> initialJointLocalRotations = new Dictionary<HingeJoint, Quaternion>();

    public override void Initialize()
    {
        // Get all Rigidbodies in the agent's hierarchy
        allRigidbodies = GetComponentsInChildren<Rigidbody>();

        // Store initial positions and rotations of all body parts
        foreach (var rb in allRigidbodies)
        {
            initialPositions[rb.transform] = rb.transform.localPosition;
            initialRotations[rb.transform] = rb.transform.localRotation;
        }
		//remove that later if it doesnt turn out to work
		initialJointLocalRotations[leftHipJoint] = leftHipJoint.transform.localRotation;
        initialJointLocalRotations[rightHipJoint] = rightHipJoint.transform.localRotation;
        initialJointLocalRotations[leftKneeJoint] = leftKneeJoint.transform.localRotation;
        initialJointLocalRotations[rightKneeJoint] = rightKneeJoint.transform.localRotation;
		
		leftHipScript = GameObject.Find("LeftHipMotor").GetComponent<MotorController>();
    }

    public override void OnEpisodeBegin()
    {
        hasFallen = false;

        // Reset positions and rotations of all body parts
        foreach (var rb in allRigidbodies)
        {
            rb.transform.localPosition = initialPositions[rb.transform];
            rb.transform.localRotation = initialRotations[rb.transform];
        }

        // Reset velocities and angular velocities
        foreach (var rb in allRigidbodies)
        {
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // Reset joint target rotations
		// FIX THIS FOR HINGE JOINT
		ResetHingeJoint(leftHipJoint);
		ResetHingeJoint(rightHipJoint);
		ResetHingeJoint(leftKneeJoint);
		ResetHingeJoint(rightKneeJoint);
    }

    private void ResetHingeJoint(HingeJoint joint)
    {
        joint.transform.localRotation = Quaternion.identity;

        var rb = joint.GetComponent<Rigidbody>();
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(GetTargetRotationAngle(leftHipJoint));
        sensor.AddObservation(GetTargetRotationAngle(rightHipJoint));

        sensor.AddObservation(GetTargetRotationAngle(leftKneeJoint));
        sensor.AddObservation(GetTargetRotationAngle(rightKneeJoint));

        sensor.AddObservation(torso.up);

        sensor.AddObservation(torso.GetComponent<Rigidbody>().angularVelocity);
    }

		private float GetTargetRotationAngle(HingeJoint joint)
		{
			float angle = joint.angle;

			angle = Mathf.DeltaAngle(0, angle) / 180f;

			return angle;
		}


     public override void OnActionReceived(ActionBuffers actionBuffers)
    {
		//Debug.Log("Action received");
        var actions = actionBuffers.ContinuousActions;
        int i = 0;

		SetJointTargetRotationSingleAxis(leftHipJoint, actions[i]);
		i += 1;

		SetJointTargetRotationSingleAxis(rightHipJoint, actions[i]);
		i += 1;
		
        SetJointTargetRotationSingleAxis(leftKneeJoint, actions[i]);
        i += 1;

        SetJointTargetRotationSingleAxis(rightKneeJoint, actions[i]);
        i += 1;

        if (!hasFallen)
        {
            float uprightness = Vector3.Dot(torso.up, Vector3.up);
            float uprightReward = Mathf.Clamp01(uprightness);
            AddReward(uprightReward * 0.01f);
        }

        if (hasFallen)
        {
            SetReward(-1.0f);
            EndEpisode();
        }
    }

    private void SetJointTargetRotationSingleAxis(HingeJoint joint, float x)
		{
			// Clamp the input rotation value to be between -1 and 1 (normalized)
			float clampedX = Mathf.Clamp(x, -1f, 1f);
			
			// Convert the normalized value into a target angle (between -180° and 180°)
			float targetAngle = clampedX * 180f;

			// Set the target angle using the hinge motor
			JointMotor motor = joint.motor;
			motor.targetVelocity = targetAngle;  // Adjust velocity to move towards target angle
			motor.force = 100f;  // Set an appropriate force value (you can adjust this)

			// Apply motor to the hinge joint
			joint.motor = motor;
			joint.useMotor = true;  // Enable motor to move the joint towards the target

			// Optionally, you can log the target angle for debugging
			Debug.Log("Target angle for hinge joint: " + targetAngle);
		}

		public void OnBodyPartCollisionEnter(GameObject bodyPart, Collision collision)
		{
			// Check if collision is with the floor
			if (collision.collider.CompareTag("Floor"))
			{
				// Check if the body part is not a foot
				if (bodyPart != leftFootCollider.gameObject && bodyPart != rightFootCollider.gameObject)
				{
					hasFallen = true;
					//Debug.Log($"Agent has fallen due to collision with floor using {bodyPart.name}");
				}
			}
		}
		public override void Heuristic(in ActionBuffers actionsOut)
		{
			var actions = actionsOut.ContinuousActions;

			// Use input keys to control joint angles for testing
			actions[0] = Input.GetKey(KeyCode.W) ? 1f : (Input.GetKey(KeyCode.S) ? -1f : 0f); // Left thigh X
			actions[1] = Input.GetKey(KeyCode.Q) ? 1f : (Input.GetKey(KeyCode.X) ? -1f : 0f); // Left thigh Y 
			actions[2] = Input.GetKey(KeyCode.E) ? 1f : (Input.GetKey(KeyCode.D) ? -1f : 0f); // Left thigh Z 

			actions[3] = Input.GetKey(KeyCode.E) ? 1f : (Input.GetKey(KeyCode.D) ? -1f : 0f); // Right thigh X
			actions[4] = Input.GetKey(KeyCode.R) ? 1f : (Input.GetKey(KeyCode.F) ? -1f : 0f); // Right thigh Y 
			actions[5] = Input.GetKey(KeyCode.R) ? 1f : (Input.GetKey(KeyCode.F) ? -1f : 0f); // Right thigh Z 

			actions[6] = Input.GetKey(KeyCode.A) ? 1f : (Input.GetKey(KeyCode.Z) ? -1f : 0f); // Left knee X
			actions[7] = Input.GetKey(KeyCode.F) ? 1f : (Input.GetKey(KeyCode.G) ? -1f : 0f); // Right knee X
		}

}
