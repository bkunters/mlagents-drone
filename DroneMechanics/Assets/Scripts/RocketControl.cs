using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// add this script as component to the drone for manual control.
/// </summary>
public class RocketControl : MonoBehaviour
{

    private Rigidbody rb;
    [SerializeField]
    private List<GameObject> motors;

    // The force which must be applied for zero acceleration.
    private float baseForce;


    private void Start()
    {  
        rb = GetComponent<Rigidbody>();
        baseForce = rb.mass * -Physics.gravity.y;
    }

    /// <summary>
    /// We use FixedUpdate, as it is a physics simulation.
    /// </summary>
    private void FixedUpdate()
    {
        var currentRotationMatrix = Matrix4x4.Rotate(transform.rotation);

        if (Input.GetKey(KeyCode.Space))
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * Physics.gravity.sqrMagnitude, motors[8].transform.position, ForceMode.Force);
        }

        // Turn left.
        if (Input.GetKey(KeyCode.A))
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.forward * 1f, motors[0].transform.position, ForceMode.Impulse);
        }

        // Turn right.
        if (Input.GetKey(KeyCode.D))
        {
        }

        // Go forward.
        if (Input.GetKey(KeyCode.W))
        {
        }

        // Go backwards.
        if (Input.GetKey(KeyCode.S))
        {
        }


        // Rotate to left.
        if (Input.GetKey(KeyCode.Q))
        {
        }

        // Rotate to right.
        if (Input.GetKey(KeyCode.E))
        {
        }
        
    }
}
