using UnityEngine;

public class SimulationSpeed : MonoBehaviour
{
    private float originalFixedDeltaTime;

    [Range(0.1f, 20f)]
    public float speed = 1f;

    [Header("Simulation Timer")]
    [SerializeField]
    private float elapsedSimulationTime;

    public float ElapsedSimulationTime => elapsedSimulationTime;

    void Awake()
    {
        originalFixedDeltaTime = Time.fixedDeltaTime;
    }

    void Update()
    {
        Time.timeScale = speed;
        Time.fixedDeltaTime = originalFixedDeltaTime / speed;

        elapsedSimulationTime = Time.time;
    }

    void OnDisable()
    {
        Time.timeScale = 1f;
        Time.fixedDeltaTime = originalFixedDeltaTime;
    }
}