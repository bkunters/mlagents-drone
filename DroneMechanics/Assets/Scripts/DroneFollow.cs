using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneFollow : MonoBehaviour
{

    public Transform DroneTransform;
    private Vector3 m_cameraOffset;
    [Range(0.01f, 1.0f)]
    public float SmoothFactor;

    // Start is called before the first frame update
    void Start()
    {
        transform.LookAt(DroneTransform);
        m_cameraOffset = transform.position - DroneTransform.position;
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 newPos = DroneTransform.position + m_cameraOffset;
        transform.position = Vector3.Slerp(transform.position, newPos, SmoothFactor);
    }
}
