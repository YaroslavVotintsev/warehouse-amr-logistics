using UnityEngine;

public class TestMove : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    public float moveSpeed = 5.0f;

    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // move left by moveSpeed
        transform.Translate(Vector3.right * moveSpeed * Time.deltaTime);
        
    }
}
