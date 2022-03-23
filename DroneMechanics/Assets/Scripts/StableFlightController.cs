using System.Collections;
using System.Collections.Generic;
using Unity.Barracuda;
using UnityEngine;

public class StableFlightController : MonoBehaviour
{
    public NNModel modelAsset;
    private Model m_RuntimeModel;
    private IWorker engine;

    private Rigidbody rb;
    private Vector3 targetPos;
    private float generalForce;

    public GameObject prop1, prop2, prop3, prop4; // Related propellers: 1-3, 2-4

    void Start()
    {
        m_RuntimeModel = ModelLoader.Load(modelAsset);
        engine = WorkerFactory.CreateWorker(WorkerFactory.Type.Compute, m_RuntimeModel);
        rb = GetComponent<Rigidbody>();

        var baseForce = rb.mass * -Physics.gravity.y;
        generalForce = baseForce * 3;
        targetPos = transform.position;
    }

    private void FixedUpdate()
    {
        float[] input = new float[] { transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y, transform.rotation.eulerAngles.z, rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z,
                                      rb.velocity.x, rb.velocity.y, rb.velocity.z, 
                                      (targetPos-transform.position).normalized.x, (targetPos-transform.position).normalized.y, (targetPos-transform.position).normalized.z};

        var inputs = new Dictionary<string, Tensor>();
        inputs["obs_0"] = new Tensor(1,12,input);
        inputs["action_masks"] = new Tensor(1,1,1,8);

        Tensor actions = engine.Execute(inputs).PeekOutput("discrete_actions");

        // Apply force through the rotors
        var currentRotationMatrix = Matrix4x4.Rotate(transform.rotation);
        rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * generalForce * (1 - actions[0, 0, 0, 0]), prop1.transform.position, ForceMode.Force);
        rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * generalForce * (1 - actions[0, 0, 0, 1]), prop2.transform.position, ForceMode.Force);
        rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * generalForce * (1 - actions[0, 0, 0, 2]), prop3.transform.position, ForceMode.Force);
        rb.AddForceAtPosition(currentRotationMatrix * Vector3.up * generalForce * (1 - actions[0, 0, 0, 3]), prop4.transform.position, ForceMode.Force);

        actions.Dispose();
        inputs["obs_0"].Dispose();
        inputs["action_masks"].Dispose();
    }
}
