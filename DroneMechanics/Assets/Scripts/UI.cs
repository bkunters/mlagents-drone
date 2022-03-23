using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UI : MonoBehaviour
{

    public GameObject Drone;
    public Button ResetButton;

    private void Start()
    {
        ResetButton.onClick.AddListener(OnClickReset);
    }

    private void OnClickReset()
    {
        Drone.transform.rotation = Quaternion.identity;
        Drone.transform.position = new Vector3(0, 1f, 0);
        Drone.GetComponent<Rigidbody>().velocity = Vector3.zero;
        Drone.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
    }
}
