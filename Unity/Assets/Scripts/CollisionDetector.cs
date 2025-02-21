using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    private HumanoidAgentStand agent;

    void Start()
    {
        // Automatically find the agent in the parent hierarchy
        agent = GetComponentInParent<HumanoidAgentStand>();
        if (agent == null)
        {
            Debug.LogError("Agent script not found in parent hierarchy.");
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        // Notify the agent of the collision
        agent.OnBodyPartCollisionEnter(this.gameObject, collision);
    }
}

