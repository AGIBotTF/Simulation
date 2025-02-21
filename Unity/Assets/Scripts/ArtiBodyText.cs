using UnityEngine;

public class KneeController : MonoBehaviour{
    private float targetRotation = 0; 
	private ArticulationBody calfBody;

	void Start(){
		calfBody = GetComponent<ArticulationBody>();
	}

    void Update(){
		if (Input.GetKey(KeyCode.Space)){
				targetRotation ++;
		}else if (Input.GetKey(KeyCode.LeftShift)){
				targetRotation --;
		}
		
        var drive = calfBody.xDrive; 
        drive.target = targetRotation;
        calfBody.xDrive = drive;
    }
}

