using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class Arm1Action : Agent
{
    [Header("References")]
    public Transform targetTransform; // Assign in the editor: the target object
    public Transform baseTransform;   // The base of the robot arm
    public Transform endEffector;     // The end-effector of the arm
    public Transform[] joints;  // A custom script or array that controls each joint

	public float angleOffset = 60f;
	public float height = 0.5f;
	public float maxJointAngle = 180f;

	public float reachThreshold = 0.1f;

	private int actionSize;
	private float[] jointAngles;

	private float maxReach;

    // NOTE: JointController is a hypothetical class you would create.
    // It could have a method SetAngle(float angle) that moves the joint to a target angle.
    // Ensure that the initialization and reading of angles is done in OnEpisodeBegin() or elsewhere.

    [Header("Training Parameters")]
    public bool singleStepEpisode = true;

    public override void Initialize()
    {
        // Optional: Initialize any joint states, etc.
		actionSize = joints.Length;
		for (int i=0;i<actionSize;i++)
		{
				height *= 2;
		}
		angleOffset = 90f - angleOffset;

		jointAngles = new float[actionSize];
		for (int i=0;i<actionSize;i++)
		{
			jointAngles[i] = 0f;
		}
		maxReach = joints.Length * 1.0f;
    }

    public override void OnEpisodeBegin()
    {
        // Reset the arm to a default position (e.g., all angles 0)
        /*foreach (var joint in joints)*/
        /*{*/
        /*    joint.SetAngle(0f);*/
        /*}*/

        // You could randomize the target position here if desired:
        // targetTransform.localPosition = new Vector3(Random.Range(-0.5f, 0.5f),
        //                                             Random.Range(0.0f, 0.5f),
        //                                             Random.Range(-0.5f, 0.5f));
		
		SetRandomTargetPosition();

        // Make sure the end-effector is in a known starting pose.
    }
	private void SetRandomTargetPosition()
    {

        Vector3 randomPosition = Random.insideUnitSphere * maxReach;
        randomPosition.y = Mathf.Abs(randomPosition.y);

        targetTransform.position = joints[0].position + randomPosition;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
	Vector3 relativePos = targetTransform.position - joints[0].position;
        
			/*Vector3 relativePos = baseTransform.InverseTransformPoint(targetTransform.position);*/
        // Add the 3D position of the target (relative) as observations
		
		//normalize the relative position based on the max reach
		relativePos /= maxReach;

        /*sensor.AddObservation(relativePos.x);*/
        /*sensor.AddObservation(relativePos.y);*/
        /*sensor.AddObservation(relativePos.z);*/
		sensor.AddObservation(relativePos);

        // If you find it difficult to learn, consider adding the current joint angles as well:
        // foreach (var joint in joints)
        // {
        //     sensor.AddObservation(joint.CurrentAngle);
        // }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions are 6 continuous values (angles).
        // Assume actions give you a delta or a target angle.
        // For a single-step scenario, you might interpret the action as the absolute angle to set.
        // Adjust this logic based on your hardware control approach.

        var actions = actionBuffers.ContinuousActions;
        for (int i = 0; i < actionSize; i++)
        {
            float actionValue = Mathf.Clamp(actions[i], -1f, 1f);
            float angleChange = actionValue * maxJointAngle * Time.deltaTime;

            jointAngles[i] = Mathf.Clamp(jointAngles[i] + angleChange, -maxJointAngle, maxJointAngle);
        }

		PositionJoints();

        // Compute reward: negative distance to target
        float distToTarget = Vector3.Distance(endEffector.position, targetTransform.position);
        float reward = -distToTarget;
        AddReward(reward);

        // If single step, end the episode here.
        // If you would like the agent to see the result and then improve, a single-step approach
        // might be too restrictive. But as requested:
        if (singleStepEpisode)
        {
            EndEpisode();
        }
        else
        {
            // If not single-step, you could add a termination condition if close enough:
            // if (distToTarget < 0.01f) { AddReward(1.0f); EndEpisode(); }
        }
    }

	private void PositionJoints()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            if (i != 0)
            {
                joints[i].position = joints[i - 1].position + joints[i - 1].up * height;

                float sign = ((i + 1) % 2 == 0) ? -1 : 1;
                Quaternion angleRotation = Quaternion.AngleAxis(angleOffset * sign, joints[i - 1].right);

                joints[i].rotation = joints[i - 1].rotation * angleRotation;
            }
            else
            {
                joints[i].localPosition = Vector3.zero;
                joints[i].localRotation = Quaternion.identity;
            }

            RotateJoint(i, jointAngles[i]);
        }
    }

    private void RotateJoint(int index, float angle)
    {
        if (index >= joints.Length) return;

        Vector3 originalAxis = Vector3.right;
        float sign = (index % 2 == 1) ? -1 : 1;
        Quaternion tilt = Quaternion.AngleAxis(sign * angleOffset, Vector3.forward);
        Vector3 customAxis = tilt * originalAxis;

        Quaternion rotationAroundCustomAxis = Quaternion.AngleAxis(angle, customAxis);
        joints[index].localRotation = rotationAroundCustomAxis;

        for (int i = index + 1; i < joints.Length; i++)
        {
            joints[i].position = joints[i - 1].position + joints[i - 1].up * height;
            joints[i].rotation = joints[i - 1].rotation;
        }
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Implement heuristic actions for testing (optional)
        var continuousActionsOut = actionsOut.ContinuousActions;
        for (int i = 0; i < actionSize; i++)
        {
            continuousActionsOut[i] = 0f;
        }
    }
}

