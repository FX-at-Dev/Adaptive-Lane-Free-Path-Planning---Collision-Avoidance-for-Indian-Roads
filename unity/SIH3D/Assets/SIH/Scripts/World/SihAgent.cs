// Marks a road user in the scene with the id and class it has in the run.
// The replay finds agents by this id and moves them; everything under the
// object (the model, its materials, its collider) is free to change.

using UnityEngine;

namespace Sih
{
    public class SihAgent : MonoBehaviour
    {
        [Tooltip("Agent id in the run file (frames[].agent_id).")]
        public int agentId;

        [Tooltip("Road-user class: car, bus, truck, auto, two_wheeler, bicycle, pedestrian, cattle, pushcart or static.")]
        public string agentClass = "car";
    }
}
