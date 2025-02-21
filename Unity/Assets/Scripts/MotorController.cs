using UnityEngine;

/*[RequireComponent(typeof(HingeJoint))]*/
/*[RequireComponent(typeof(FixedJoint))]*/
[RequireComponent(typeof(Rigidbody))]
public class MotorController : MonoBehaviour{
    private HingeJoint hingeJoint;
	private FixedJoint fixedJoint;
    private JointMotor motor;

	public Rigidbody parentRigidbody;

    public float targetPosition = 0f;
    public float motorSpeed = 100f;
    public float motorForce = 1000f;

	private bool isAtLimit = false;

    void Start(){
        hingeJoint = GetComponent<HingeJoint>();
		fixedJoint = GetComponent<FixedJoint>();
        motor = hingeJoint.motor;

        motor.force = motorForce;
        hingeJoint.useMotor = true;

		fixedJoint.connectedBody = parentRigidbody;

		fixedJoint.connectedBody = parentRigidbody;
		hingeJoint.connectedBody = parentRigidbody; 

		motor.freeSpin = false;

		hingeJoint.useLimits = true; // uselsss
    }

    void Update(){
		float input = GetInput();

		/*Debug.Log(currentAngle);*/

		if (input != 0){
			/*toggleFixedJoint(false);*/
			/*toggleHingeJoint(true);*/

			float currentAngle = hingeJoint.angle;
			float minLimit = hingeJoint.limits.min;
		    float maxLimit = hingeJoint.limits.max;			

		    if ((currentAngle > maxLimit && input > 0)){
					motor.targetVelocity = 0;
					/*LockHingeAtLimit(maxLimit);*/

					isAtLimit = true;
		    }else if ((currentAngle < minLimit && input < 0)){
					motor.targetVelocity = 0;
					/*LockHingeAtLimit(minLimit);*/

					isAtLimit = true;
			}else{
				motor.targetVelocity = motorSpeed * input;

				isAtLimit = false;
			}
		}else{
			motor.targetVelocity = 0;
			/*toggleFixedJoint(true);*/
			/*toggleHingeJoint(false);*/
		}
		if (hingeJoint != null){
			hingeJoint.motor = motor;
		}
    }
		private void LockHingeAtLimit(float targetAngle){
			if (isAtLimit){
			return;
			}
			Rigidbody rb = hingeJoint.GetComponent<Rigidbody>();

			if (rb != null){
				float currentAngle = hingeJoint.angle;
				float angleCorrection = targetAngle - currentAngle;
				Vector3 correctionAxis = hingeJoint.transform.TransformDirection(hingeJoint.axis);
				rb.AddTorque(correctionAxis * angleCorrection * 10f, ForceMode.VelocityChange);
			}
		}

		private int GetInput(){
			if (Input.GetKey(KeyCode.A)){
				return -1;
			}else if (Input.GetKey(KeyCode.D)){
				return 1;
			}else{
				return 0;
			}
		}

		private void toggleFixedJoint(bool activaion){
				if (activaion){
					if (fixedJoint == null){
						fixedJoint = gameObject.AddComponent<FixedJoint>();
						fixedJoint.connectedBody = parentRigidbody;
					}
				}else{
					if (fixedJoint != null){
						Destroy(fixedJoint);
					}
				}
		}
		private void toggleHingeJoint(bool activaion){
				if (activaion){
					if (hingeJoint == null){
						hingeJoint = gameObject.AddComponent<HingeJoint>();
						hingeJoint.connectedBody = parentRigidbody;
					}
				}else{
					if (hingeJoint != null){
						Destroy(hingeJoint);
					}
				}
		}
}
