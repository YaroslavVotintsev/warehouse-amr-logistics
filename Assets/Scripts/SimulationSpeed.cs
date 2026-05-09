using UnityEngine;

public class SimulationSpeed : MonoBehaviour
{
    [Range(0.1f, 20f)]
    public float speed = 1f;

    void Update()
    {
        Time.timeScale = speed;
    }
}