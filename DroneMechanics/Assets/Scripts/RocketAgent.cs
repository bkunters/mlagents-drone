using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.UI;

public class RocketAgent : Agent
{
    [SerializeField]
    private List<GameObject> motors; // 9 motors in total

    [SerializeField]
    private Rigidbody rocket_rb;

    [SerializeField]
    private GameObject target;

    private Vector3 startPosition;
    private Quaternion startRotation;

    private int stepCount;

    [SerializeField]
    private float discreteRotationCoefficient = 130f;
    [SerializeField]
    private float mainMotorPower = 200f;
    [SerializeField]
    private float startingAngle = 2.5f; // increase this while learning happens in curriculums.

    public Text velocityText;
    public Text distanceText;
    public Text landingAngleTxt;
    public Text landingVelocityTxt;

    private Vector3 currentTotalForce;

    private float currentAngle;
    private float currentvelocity;
    private float currentangularvelocity;

    public override void Initialize()
    {
        startPosition = rocket_rb.gameObject.transform.position;
        startRotation = rocket_rb.rotation;
        stepCount = 1;
        currentTotalForce = Vector3.zero;
    }

    private void FixedUpdate()
    {
        RequestDecision();

        currentAngle = Mathf.Abs(Quaternion.Angle(transform.rotation, Quaternion.identity));
        currentvelocity = rocket_rb.velocity.sqrMagnitude;
        currentangularvelocity = rocket_rb.angularVelocity.magnitude;
    }

    public override void OnEpisodeBegin()
    {
        //landingAngleTxt.text = "Landing Angle:";
        //landingVelocityTxt.text = "Landing Velocity:";

        stepCount++;
        rocket_rb.velocity = new Vector3(0f, -5f, 0f);
        rocket_rb.angularVelocity = Vector3.zero;
        currentTotalForce = Vector3.zero;
        transform.position = startPosition;
        //rocket_rb.rotation = startRotation;

        // First phase(Begin at 100m height with small random rotation, with random position on the x-z axis)
        //transform.rotation = Quaternion.identity;
        transform.localPosition = new Vector3(Random.Range(-10f, 10f), transform.localPosition.y, Random.Range(-10f, 10f));
        transform.rotation = Quaternion.Euler(Random.Range(-startingAngle, startingAngle), Random.Range(-startingAngle, startingAngle), Random.Range(-startingAngle, startingAngle));
        // Second phase(Begin with 500m height with small random rotation)
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //sensor.AddObservation(Vector3.Distance(transform.localPosition, target.transform.localPosition)); // 1
        sensor.AddObservation(rocket_rb.gameObject.transform.localPosition.x); // 3
        sensor.AddObservation(rocket_rb.gameObject.transform.localPosition.y);
        sensor.AddObservation(rocket_rb.gameObject.transform.localPosition.z);
        sensor.AddObservation(rocket_rb.velocity.x); // 3
        sensor.AddObservation(rocket_rb.velocity.y);
        sensor.AddObservation(rocket_rb.velocity.z);
        sensor.AddObservation(rocket_rb.angularVelocity.x); // 3
        sensor.AddObservation(rocket_rb.angularVelocity.y);
        sensor.AddObservation(rocket_rb.angularVelocity.z);
        sensor.AddObservation(rocket_rb.rotation.eulerAngles.x); // 3
        sensor.AddObservation(rocket_rb.rotation.eulerAngles.y);
        sensor.AddObservation(rocket_rb.rotation.eulerAngles.z);
        sensor.AddObservation(Quaternion.Angle(transform.rotation, Quaternion.identity)); // 1

        velocityText.text = "Velocity: " + rocket_rb.velocity.sqrMagnitude;
        distanceText.text = "Distance: " + Vector3.Distance(transform.position, target.transform.position).ToString();
    }
    

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (transform.position.y < target.transform.position.y - 5f)
        {
            EndEpisode();
        }

        var currentRotationMatrix = Matrix4x4.Rotate(transform.rotation);

        // Distinguish separate branches based on the current input.
        if (Input.GetKey(KeyCode.UpArrow))
        {
            actionsOut.DiscreteActions.Array[0] = 0;
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.forward * discreteRotationCoefficient, motors[0].transform.position, ForceMode.Force);
            currentTotalForce += (Vector3)(currentRotationMatrix* Vector3.forward* discreteRotationCoefficient);
        }
        else
            actionsOut.DiscreteActions.Array[0] = 1;
        
        if (Input.GetKey(KeyCode.DownArrow))
        {
            actionsOut.DiscreteActions.Array[1] = 0;
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.back * discreteRotationCoefficient, motors[1].transform.position, ForceMode.Force);
            currentTotalForce += (Vector3)(currentRotationMatrix * Vector3.back * discreteRotationCoefficient);
        }
        else
            actionsOut.DiscreteActions.Array[1] = 1;

        if (Input.GetKey(KeyCode.RightArrow))
        {
            actionsOut.DiscreteActions.Array[2] = 0;
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.right * discreteRotationCoefficient, motors[2].transform.position, ForceMode.Force);
            currentTotalForce += (Vector3)(currentRotationMatrix * Vector3.right * discreteRotationCoefficient);
        }
        else
            actionsOut.DiscreteActions.Array[2] = 1;

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            actionsOut.DiscreteActions.Array[3] = 0;
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.left * discreteRotationCoefficient, motors[3].transform.position, ForceMode.Force);
            currentTotalForce += (Vector3)(currentRotationMatrix * Vector3.left * discreteRotationCoefficient);
        }
        else
            actionsOut.DiscreteActions.Array[3] = 1;

        if (Input.GetKey(KeyCode.S))
        {
            actionsOut.DiscreteActions.Array[4] = 0;
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * mainMotorPower, motors[4].transform.position, ForceMode.Force);
            currentTotalForce += (Vector3)(currentRotationMatrix * Vector3.up * discreteRotationCoefficient);
        }
        else
            actionsOut.DiscreteActions.Array[4] = 1;

        if (Input.GetKey(KeyCode.Space))
        {
            EndEpisode();
        }
    }

    public override void OnActionReceived(ActionBuffers vectorAction)
    {

        if (transform.position.y < target.transform.position.y - 5f || transform.localPosition.x <= -50f 
            || transform.localPosition.x >= 50f || transform.localPosition.z <= -50f || transform.localPosition.z >= 50f)
        {
            EndEpisode();
        }

        var currentRotationMatrix = Matrix4x4.Rotate(transform.rotation);

        if(vectorAction.DiscreteActions[0] == 0)
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.forward * discreteRotationCoefficient, motors[0].transform.position, ForceMode.Force);
        if(vectorAction.DiscreteActions[1] == 0)
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.back * discreteRotationCoefficient, motors[1].transform.position, ForceMode.Force);
        if(vectorAction.DiscreteActions[2] == 0)
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.right * discreteRotationCoefficient, motors[2].transform.position, ForceMode.Force);
        if(vectorAction.DiscreteActions[3] == 0)
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.left * discreteRotationCoefficient, motors[3].transform.position, ForceMode.Force);
        if(vectorAction.DiscreteActions[4] == 0)
            rocket_rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * mainMotorPower, motors[4].transform.position, ForceMode.Force);
    }

    [SerializeField]
    private float currentTolerableVelocity = 15f;
    [SerializeField]
    private float currentTolerableAngle = 7f;
    // TODO: use in testing
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag.Equals("Goal"))
        {
            // Improving rewards are added after the rocket has hit the ground,
            // as it can otherwise learn to just float in the air 
            //AddReward(Utils.Gaussian(currentAngle));
            //AddReward(Utils.Gaussian(currentvelocity));
            //AddReward(Utils.Gaussian(currentangularvelocity));

            //AddReward(0.1f); // Signal for hitting the goal => faster learning

            landingAngleTxt.text = "Landing Angle: " + currentAngle;
            landingVelocityTxt.text = "Landing Velocity: " + currentvelocity;

            if (currentvelocity >= currentTolerableVelocity || currentAngle >= currentTolerableAngle)
            {
                EndEpisode();
            }
            if(currentvelocity < currentTolerableVelocity && currentAngle < currentTolerableAngle) // make it harder later(curriculum learning)
            {
                Debug.Log("goal");
                AddReward(1f);
                EndEpisode();
            }
        }
        if (!collision.gameObject.tag.Equals("Goal"))
        {
            EndEpisode();
        }
    }
}
