using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class DronePIDTest : MonoBehaviour
{

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

    private Gamepad ps4Controller;

    private Rigidbody rb;

    public GameObject prop1, prop2, prop3, prop4;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();

        _xAxisPIDController = new PID(_xAxisP, _xAxisI, _xAxisD);
        _yAxisPIDController = new PID(_yAxisP, _yAxisI, _yAxisD);
        _zAxisPIDController = new PID(_zAxisP, _zAxisI, _zAxisD);
    }

    private void FixedUpdate()
    {
        var gamepad = Gamepad.current;
        if (gamepad == null)
            return; // No gamepad connected.

        Vector2 leftStick = gamepad.leftStick.ReadValue();
        Vector2 rightStick = gamepad.rightStick.ReadValue();

        var baseForce = rb.mass * -Physics.gravity.y;
        var generalForce = baseForce * 3;

        Debug.Log(rightStick);

        var currentRotationMatrix = Matrix4x4.Rotate(transform.rotation);

        rb.AddForce(currentRotationMatrix * Vector3.up * generalForce * leftStick.y, ForceMode.Force);

        var rotation = 50f;
        if (rightStick.y > 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * rightStick.y, prop1.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * rightStick.y, prop2.transform.position, ForceMode.Force);
        }
        if (rightStick.y < 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -rightStick.y, prop3.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -rightStick.y, prop4.transform.position, ForceMode.Force);
        }
        if (rightStick.x > 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * rightStick.x, prop1.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * rightStick.x, prop4.transform.position, ForceMode.Force);
        }
        if (rightStick.x < 0f)
        {
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -rightStick.x, prop2.transform.position, ForceMode.Force);
            rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * rotation * -rightStick.x, prop3.transform.position, ForceMode.Force);
        }


        _xAxisPIDController.Kp = _xAxisP;
        _xAxisPIDController.Ki = _xAxisI;
        _xAxisPIDController.Kd = _xAxisD;

        _yAxisPIDController.Kp = _yAxisP;
        _yAxisPIDController.Ki = _yAxisI;
        _yAxisPIDController.Kd = _yAxisD;

        _zAxisPIDController.Kp = _zAxisP;
        _zAxisPIDController.Ki = _zAxisI;
        _zAxisPIDController.Kd = _zAxisD;

        //Figure out the error for each asix
        float xAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.x, 0);
        float xTorqueCorrection = _xAxisPIDController.GetOutput(xAngleError, Time.fixedDeltaTime);

        float yAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.y, 0);
        float yTorqueCorrection = _yAxisPIDController.GetOutput(yAngleError, Time.fixedDeltaTime);

        float zAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.z, 0);
        float zTorqueCorrection = _zAxisPIDController.GetOutput(zAngleError, Time.fixedDeltaTime);
        rb.AddRelativeTorque((xTorqueCorrection * Vector3.right) + (yTorqueCorrection * Vector3.up) + (zTorqueCorrection * Vector3.forward));
    }
}
