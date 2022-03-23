using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;

/**
 * Task:
 * We want to teach a drone how to fly in a stable manner and to reach a given point on a terrain map by bypassing possible obstacles.
 * 
 * Observations:
 * 1) Current position and orientation of the drone
 * 2) Current position of the destination
 * 3) Camera vision
 * 4) Distance measurement using raycasts
 * 
 * 
 */
public class DroneAgentStableFlight : Agent
{

    [SerializeField] private Transform m_droneTransform;

    private Rigidbody rb;
    private float generalForce;

    public Vector3 startingPos;
    private float startingDistance;
    private Vector3 lastVelocity;
    private Vector3 acceleration;
    private float[] rotorThrusts;

    private readonly float ROTATION_TORQUE_COEFFICIENT = 64;

    public GameObject TargetObject;
    private Vector3 targetObjStartPos;

    public GameObject[] targetObjects;
    int currentTargetIndex;


    /// <summary>
    /// is called once before the training
    /// </summary>
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        var baseForce = rb.mass * -Physics.gravity.y;
        generalForce = baseForce * 3;
        //startingPos = m_droneTransform.position;
        startingDistance = Vector3.Distance(m_droneTransform.position, TargetObject.transform.position);
        lastVelocity = Vector3.zero;
        acceleration = Vector3.zero;
        rotorThrusts = new float[4];

        Physics.autoSimulation = true;

        currentTargetIndex = 0; // next gate after start
        //TargetObject = targetObjects[currentTargetIndex];
        targetObjStartPos = TargetObject.transform.position;
    }

    public GameObject prop1, prop2, prop3, prop4; // Related propellers: 1-3, 2-4
    public void FixedUpdate()
    {
        // Actively rotate the propellers(as simplfication, we do not simulate the propellers based on the real physical model).
        prop1.transform.Rotate(new Vector3(0, 33000 * Time.fixedDeltaTime, 0)); prop3.transform.Rotate(new Vector3(0, 33000 * Time.fixedDeltaTime, 0));
        prop2.transform.Rotate(new Vector3(0, -33000 * Time.fixedDeltaTime, 0)); prop4.transform.Rotate(new Vector3(0, -33000 * Time.fixedDeltaTime, 0));

        RequestDecision();
    }

    /// <summary>
    /// observations for the next actions
    /// </summary>
    /// <param name="sensor"></param>
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(m_droneTransform.rotation.eulerAngles);     // size 3, TODO: NORMALISIEREN
        sensor.AddObservation(rb.angularVelocity); // size 3, sigmoid normalization
        sensor.AddObservation(rb.velocity); // size 3, sigmoid normalization
        sensor.AddObservation((startingPos - transform.position).normalized); // size 3
        //sensor.AddObservation(Vector3.Distance(TargetObject.transform.position.normalized, transform.position.normalized)); // size 1
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
    }

    /// <summary>
    /// is called every time at the beginning of any episode
    /// </summary>
    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(UnityEngine.Random.Range(-22f, 22f), UnityEngine.Random.Range(3f, 15f), UnityEngine.Random.Range(-20f, 20f)); // TODO: convert the values to constant variables.

        //transform.rotation.SetEulerRotation(UnityEngine.Random.Range(0, 360f), UnityEngine.Random.Range(0, 360f), UnityEngine.Random.Range(0, 360f));
        //transform.Rotate(UnityEngine.Random.Range(0, 360f), UnityEngine.Random.Range(0, 360f), UnityEngine.Random.Range(0, 360f));

        // TODO: noisy episode beginnings ?
        //transform.rotation = Quaternion.identity;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        startingDistance = Vector3.Distance(m_droneTransform.position, TargetObject.transform.position);
        lastVelocity = Vector3.zero;
        acceleration = Vector3.zero;
        rotorThrusts = new float[4];

        currentTargetIndex = 0;
        //TargetObject = targetObjects[currentTargetIndex];
        //TargetObject.transform.position = targetObjStartPos;
    }

    /// <summary>
    /// Apply force to the rotors whenever an action is received.
    /// </summary>
    /// <param name="vectorAction"></param>
    public override void OnActionReceived(ActionBuffers vectorAction)
    {
        var currentRotationMatrix = Matrix4x4.Rotate(transform.rotation);

        /*for (int i = 0; i < vectorAction.DiscreteActions.Length; i++)
        {
            vectorAction.DiscreteActions[i] = (vectorAction[i] + 1f) * 0.5f;
            rotorThrusts[i] = vectorAction[i];
        }*/
        //Debug.LogWarningFormat("{0}, {1}, {2}, {3}", vectorAction[0], vectorAction[1], vectorAction[2], vectorAction[3]);
        // If it does not work, try with yaw, pitch, roll.
        if (vectorAction.DiscreteActions[0] == 0)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * generalForce, prop1.transform.position, ForceMode.Force);
        }
        if (vectorAction.DiscreteActions[1] == 0) { 
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * generalForce, prop2.transform.position, ForceMode.Force);
        }
        if (vectorAction.DiscreteActions[2] == 0) {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * generalForce, prop3.transform.position, ForceMode.Force);
        }
        if (vectorAction.DiscreteActions[3] == 0)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * generalForce, prop4.transform.position, ForceMode.Force);
        }

        #region Positive/Negative reward states

        // distance metric.
        var currentDistance = Vector3.Distance(transform.position, TargetObject.transform.position);
        var angle = Mathf.Abs(Quaternion.Angle(transform.rotation, Quaternion.identity));
        AddReward(Utils.Gaussian(angle));
        AddReward(Utils.Gaussian(rb.angularVelocity.sqrMagnitude));
        AddReward(Utils.Gaussian(currentDistance));
        AddReward(Utils.Gaussian(rb.velocity.sqrMagnitude));

        #endregion
    }

    /// <summary>
    /// Consider two cases:
    /// 1) The drone collides with an obstacle in the environment.
    /// 2) The drone collides with a surface.
    /// Both are rewarded negatively.
    /// </summary>
    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag.Equals("Obstacle"))
        {
            //Debug.Log("obstacle hit");
            AddReward(-3f);
            EndEpisode();
        }
        if (other.gameObject.tag.Equals("Plane"))
        {
            //Debug.Log("plane hit");
            AddReward(-3f);
            EndEpisode();
        }
        if (other.gameObject.tag.Equals("Checkpoint"))
        {
            AddReward(1f);
            /*GameObject parent = other.gameObject.transform.parent.gameObject;
            int nextTarget = Convert.ToInt32(parent.name);
                
            currentTargetIndex = (nextTarget + 1) % targetObjects.Length;
            Debug.Log(currentTargetIndex);
            TargetObject = targetObjects[currentTargetIndex];
            */
            EndEpisode();
            Debug.Log("checkpoint");
        }
    }

}
