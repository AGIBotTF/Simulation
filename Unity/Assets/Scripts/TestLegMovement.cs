using UnityEngine;

public class LegAgent : MonoBehaviour
{
    public GameObject leftThigh;
    public GameObject leftCalf;
    public GameObject rightThigh;
    public GameObject rightCalf;

    private ConfigurableJoint leftThighJoint;
    private ConfigurableJoint leftCalfJoint;
    private ConfigurableJoint rightThighJoint;
    private ConfigurableJoint rightCalfJoint;

    public float swingSpeed = 2f;
    public float swingAngle = 30f;
    public float bendAngle = 45f;

    void Start()
    {
        // Retrieve ConfigurableJoint components
        leftThighJoint = leftThigh.GetComponent<ConfigurableJoint>();
        leftCalfJoint = leftCalf.GetComponent<ConfigurableJoint>();
        rightThighJoint = rightThigh.GetComponent<ConfigurableJoint>();
        rightCalfJoint = rightCalf.GetComponent<ConfigurableJoint>();

        // Ensure joints are properly set up
        ConfigureJoint(leftThighJoint);
        ConfigureJoint(leftCalfJoint);
        ConfigureJoint(rightThighJoint);
        ConfigureJoint(rightCalfJoint);
    }

    void Update()
    {
        TestLegMovement();
    }

    void TestLegMovement()
    {
        float thighAngle = Mathf.Sin(Time.time * swingSpeed) * swingAngle;
        float calfAngle = Mathf.Cos(Time.time * swingSpeed) * bendAngle;

        Quaternion thighRotation = Quaternion.Euler(thighAngle, 0, 0);
        Quaternion calfRotation = Quaternion.Euler(calfAngle, 0, 0);

        if (leftThighJoint != null)
            ApplyRotationToJoint(leftThighJoint, thighRotation);

        if (leftCalfJoint != null)
            ApplyRotationToJoint(leftCalfJoint, calfRotation);

        if (rightThighJoint != null)
            ApplyRotationToJoint(rightThighJoint, Quaternion.Euler(-thighAngle, 0, 0));

        if (rightCalfJoint != null)
            ApplyRotationToJoint(rightCalfJoint, Quaternion.Euler(-calfAngle, 0, 0));
    }

    void ConfigureJoint(ConfigurableJoint joint)
    {
        if (joint == null) return;

        // Ensure angular motion is enabled on all axes
        joint.angularXMotion = ConfigurableJointMotion.Limited;
        joint.angularYMotion = ConfigurableJointMotion.Limited;
        joint.angularZMotion = ConfigurableJointMotion.Limited;

        // Set default angular limits
        SoftJointLimit limit = new SoftJointLimit { limit = 45f }; // Example limit value
        joint.lowAngularXLimit = limit;
        joint.highAngularXLimit = limit;
        joint.angularYLimit = limit;
        joint.angularZLimit = limit;

        // Ensure drive settings are configured
        JointDrive angularDrive = new JointDrive
        {
            positionSpring = 500f, // Adjust spring strength
            positionDamper = 10f,  // Adjust damping strength
            maximumForce = Mathf.Infinity
        };
        joint.angularXDrive = angularDrive;
        joint.angularYZDrive = angularDrive;
    }

    void ApplyRotationToJoint(ConfigurableJoint joint, Quaternion targetRotation)
    {
        if (joint == null) return;

        // Adjust rotation based on joint's local space
        joint.targetRotation = targetRotation;
    }
}

