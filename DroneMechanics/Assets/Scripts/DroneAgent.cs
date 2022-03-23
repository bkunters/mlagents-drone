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
public class DroneAgent : Agent
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

    private Gamepad ps4Controller;

    [SerializeField]
    [Range(-10, 10)]
    private float _xAxisP, _xAxisI, _xAxisD;

    [SerializeField]
    [Range(-10, 10)]
    private float _yAxisP, _yAxisI, _yAxisD;

    [SerializeField]
    [Range(-10, 10)]
    private float _zAxisP, _zAxisI, _zAxisD;

    private PID _xAxisPIDController;
    private PID _yAxisPIDController;
    private PID _zAxisPIDController;


    /// <summary>
    /// is called once before the training
    /// </summary>
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        var baseForce = rb.mass * -Physics.gravity.y;
        generalForce = baseForce * 3;
        startingPos = m_droneTransform.position;
        startingDistance = Vector3.Distance(m_droneTransform.position, TargetObject.transform.position);
        lastVelocity = Vector3.zero;
        acceleration = Vector3.zero;
        rotorThrusts = new float[4];

        Physics.autoSimulation = true;

        currentTargetIndex = 0; // next gate after start
        //TargetObject = targetObjects[currentTargetIndex];
        targetObjStartPos = TargetObject.transform.position;

        ps4Controller = Gamepad.current;

        _xAxisPIDController = new PID(_xAxisP, _xAxisI, _xAxisD);
        _yAxisPIDController = new PID(_yAxisP, _yAxisI, _yAxisD);
        _zAxisPIDController = new PID(_zAxisP, _zAxisI, _zAxisD);
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
        sensor.AddObservation(m_droneTransform.rotation.eulerAngles / 360f);     // size 3, TODO: NORMALISIEREN
        sensor.AddObservation(new Vector3(Utils.Sigmoid(rb.angularVelocity.x), Utils.Sigmoid(rb.angularVelocity.y), Utils.Sigmoid(rb.angularVelocity.z))); // size 3, sigmoid normalization
        sensor.AddObservation(new Vector3(Utils.Sigmoid(rb.velocity.x), Utils.Sigmoid(rb.velocity.y), Utils.Sigmoid(rb.velocity.z))); // size 3, sigmoid normalization
        sensor.AddObservation((TargetObject.transform.position - transform.position).normalized); // size 3
        sensor.AddObservation(Vector3.Distance(TargetObject.transform.position.normalized, transform.position.normalized)); // size 1
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var gamepad = Gamepad.current;
        if (gamepad == null)
            return; // No gamepad connected.

        Vector2 leftStick = gamepad.leftStick.ReadValue();
        Vector2 rightStick = gamepad.rightStick.ReadValue();

        Debug.Log(rightStick);

        var currentRotationMatrix = Matrix4x4.Rotate(transform.rotation);

        rb.AddForce(currentRotationMatrix * Vector3.up * generalForce * leftStick.y, ForceMode.Force);
        actionsOut.ContinuousActions.Array[0] = leftStick.y;

        var rotation = 50f;
        if (rightStick.y > 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * rightStick.y, prop1.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * rightStick.y, prop2.transform.position, ForceMode.Force);
            actionsOut.ContinuousActions.Array[1] = rightStick.y;
        }
        if (rightStick.y < 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -rightStick.y, prop3.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -rightStick.y, prop4.transform.position, ForceMode.Force);
            actionsOut.ContinuousActions.Array[2] = rightStick.y;
        }
        if (rightStick.x > 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * rightStick.x, prop1.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * rightStick.x, prop4.transform.position, ForceMode.Force);
            actionsOut.ContinuousActions.Array[3] = rightStick.x;
        }
        if (rightStick.x < 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -rightStick.x, prop2.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -rightStick.x, prop3.transform.position, ForceMode.Force);
            actionsOut.ContinuousActions.Array[4] = rightStick.x;
        }

        //Figure out the error for each asix
        float xAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.x, 0);
        float xTorqueCorrection = _xAxisPIDController.GetOutput(xAngleError, Time.fixedDeltaTime);

        float yAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.y, 0);
        float yTorqueCorrection = _yAxisPIDController.GetOutput(yAngleError, Time.fixedDeltaTime);

        float zAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.z, 0);
        float zTorqueCorrection = _zAxisPIDController.GetOutput(zAngleError, Time.fixedDeltaTime);
        rb.AddRelativeTorque((xTorqueCorrection * Vector3.right) + (yTorqueCorrection * Vector3.up) + (zTorqueCorrection * Vector3.forward));

        #region Positive/Negative reward states

        // distance metric.
        var currentDistance = Vector3.Distance(transform.position, TargetObject.transform.position);
        var angle = Mathf.Abs(Quaternion.Angle(transform.rotation, Quaternion.identity));
        //AddReward(Utils.Gaussian(angle));
        AddReward(Utils.Gaussian(rb.angularVelocity.sqrMagnitude));
        AddReward(Utils.Gaussian(currentDistance));
        float threshold = 1.0f;
        if (currentDistance < threshold)
        {
            AddReward(Utils.Gaussian(rb.velocity.sqrMagnitude));
        }
        else
            AddReward(-0.0001f); // Time penalty

        #endregion
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

        _xAxisPIDController.Kp = _xAxisP;
        _xAxisPIDController.Ki = _xAxisI;
        _xAxisPIDController.Kd = _xAxisD;

        _yAxisPIDController.Kp = _yAxisP;
        _yAxisPIDController.Ki = _yAxisI;
        _yAxisPIDController.Kd = _yAxisD;

        _zAxisPIDController.Kp = _zAxisP;
        _zAxisPIDController.Ki = _zAxisI;
        _zAxisPIDController.Kd = _zAxisD;
    }

    /// <summary>
    /// Apply force to the rotors whenever an action is received.
    /// </summary>
    /// <param name="vectorAction"></param>
    public override void OnActionReceived(ActionBuffers vectorAction)
    {
        var currentRotationMatrix = Matrix4x4.Rotate(transform.rotation);

        var rotation = 50f;
        rb.AddForce(currentRotationMatrix * Vector3.up * generalForce * vectorAction.ContinuousActions.Array[0], ForceMode.Force);

        if (vectorAction.ContinuousActions.Array[1] > 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * vectorAction.ContinuousActions.Array[1], prop1.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * vectorAction.ContinuousActions.Array[1], prop2.transform.position, ForceMode.Force);
        }
        if (vectorAction.ContinuousActions.Array[2] < 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -vectorAction.ContinuousActions.Array[2], prop3.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -vectorAction.ContinuousActions.Array[2], prop4.transform.position, ForceMode.Force);
        }
        if (vectorAction.ContinuousActions.Array[3] > 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * vectorAction.ContinuousActions.Array[3], prop1.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * vectorAction.ContinuousActions.Array[3], prop4.transform.position, ForceMode.Force);
        }
        if (vectorAction.ContinuousActions.Array[4] < 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -vectorAction.ContinuousActions.Array[4], prop2.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -vectorAction.ContinuousActions.Array[4], prop3.transform.position, ForceMode.Force);
        }

        //Figure out the error for each asix
        float xAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.x, 0);
        float xTorqueCorrection = _xAxisPIDController.GetOutput(xAngleError, Time.fixedDeltaTime);

        float yAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.y, 0);
        float yTorqueCorrection = _yAxisPIDController.GetOutput(yAngleError, Time.fixedDeltaTime);

        float zAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.z, 0);
        float zTorqueCorrection = _zAxisPIDController.GetOutput(zAngleError, Time.fixedDeltaTime);
        rb.AddRelativeTorque((xTorqueCorrection * Vector3.right) + (yTorqueCorrection * Vector3.up) + (zTorqueCorrection * Vector3.forward));


        #region Positive/Negative reward states

        // distance metric.
        var currentDistance = Vector3.Distance(transform.position, TargetObject.transform.position);
        var angle = Mathf.Abs(Quaternion.Angle(transform.rotation, Quaternion.identity));
        //AddReward(Utils.Gaussian(angle));
        //AddReward(Utils.Gaussian(rb.angularVelocity.sqrMagnitude));
        AddReward(Utils.Gaussian(currentDistance));
        float threshold = 1.0f;
        if(currentDistance < threshold)
        {
            AddReward(Utils.Gaussian(rb.velocity.sqrMagnitude));
        }
        else
            AddReward(-0.0001f); // Time penalty

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
