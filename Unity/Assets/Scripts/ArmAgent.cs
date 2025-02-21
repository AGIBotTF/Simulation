
//////////////////////////////////////////////////////////////////////////////
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class ArmAgent : Agent
{
    // Public variables to be set in the Unity Inspector
    public Transform[] joints; // Assign your joint GameObjects in order
    public Transform target;   // The target the arm is trying to reach
    public Transform endEffector; // The end of the arm (e.g., the last joint or tool)

    // Joint angle limits
    public float maxJointAngle = 180f;// Maximum absolute joint angle in degrees

    public float actionSpeed = 1.0f;

    // Training parameters
    public float reachThreshold = 0.2f;
    public float maxStepsPerEpisode = 1000;

    // Internal variables
    private int actionSize;         // Number of joints (actions)
    private float[] jointAngles;    // Current angles of each joint

    // Variables for reward calculation
    private Vector3 initialTargetPosition;
    private float initialDistanceToTarget;

    //public float angleOffset;
    // public float height;

    public float[] angleOffsetS;
    public float[] heights;

    public override void Initialize()
    {
        //height = height * 2;
        //angleOffset = 90f - angleOffset;
        actionSize = joints.Length;

        for (int i = 0; i < actionSize; i++)
        {
            heights[i] *= 2;
            angleOffsetS[i] = 90f - angleOffsetS[i];
        }

       
        jointAngles = new float[actionSize];

        for (int i = 0; i < actionSize; i++)
        {
            jointAngles[i] = 0f;
        }

        PositionJoints();
    }

    public override void OnEpisodeBegin()
    {
        for (int i = 0; i < actionSize; i++)
        {
            jointAngles[i] = Random.Range(-maxJointAngle, maxJointAngle);
        }

        PositionJoints();

        SetRandomTargetPosition();

        initialDistanceToTarget = Vector3.Distance(endEffector.position, target.position);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 relativePosition = target.position - endEffector.position;
        sensor.AddObservation(relativePosition / 10f);

        for (int i = 0; i < actionSize; i++)
        {
            float normalizedAngle = jointAngles[i] / maxJointAngle;
            sensor.AddObservation(normalizedAngle);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var actions = actionBuffers.ContinuousActions;

        // Update joint angles based on actions
        for (int i = 0; i < actionSize; i++)
        {
            float actionValue = Mathf.Clamp(actions[i], -1f, 1f);
            float angleChange = actionValue * maxJointAngle * actionSpeed;
            jointAngles[i] = Mathf.Clamp(jointAngles[i] + angleChange, -maxJointAngle, maxJointAngle);
        }

        PositionJoints();

        float distanceToTarget = Vector3.Distance(endEffector.position, target.position);

        // Reward for getting closer
        float progressReward = (initialDistanceToTarget - distanceToTarget) / initialDistanceToTarget;
        AddReward(progressReward);

        // Time penalty to encourage faster reaching
        AddReward(-0.001f);

        if (distanceToTarget < reachThreshold)
        {
            SetReward(1.0f);
            EndEpisode();
        }

        if (StepCount >= maxStepsPerEpisode)
        {
            EndEpisode();
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
    private void PositionJoints()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            if (i != 0)
            {
                joints[i].position = joints[i - 1].position + joints[i - 1].up * heights[i];

                float sign = ((i + 1) % 2 == 0) ? -1 : 1;
                Quaternion angleRotation = Quaternion.AngleAxis(angleOffsetS[i] * sign, joints[i - 1].right);

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
        Quaternion tilt = Quaternion.AngleAxis(sign * angleOffsetS[index], Vector3.forward);
        Vector3 customAxis = tilt * originalAxis;

        Quaternion rotationAroundCustomAxis = Quaternion.AngleAxis(angle, customAxis);
        joints[index].localRotation = rotationAroundCustomAxis;

        for (int i = index + 1; i < joints.Length; i++)
        {
            joints[i].position = joints[i - 1].position + joints[i - 1].up * heights[index];
            joints[i].rotation = joints[i - 1].rotation;
        }
    }

    private void SetRandomTargetPosition()
    {
        float maxReach = joints.Length * 1.0f;

        Vector3 randomPosition = Random.insideUnitSphere * maxReach;
        randomPosition.y = Mathf.Abs(randomPosition.y);

        target.position = joints[0].position + randomPosition;
    }
}
