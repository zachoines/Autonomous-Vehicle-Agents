using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rotate : MonoBehaviour
{
    [SerializeField]
    private float thrust = 1000;

    [SerializeField]
    private bool clockwise = false;
    private bool _rotating;
    private ArticulationBody rb;
    void Start()
    {
        _rotating = false;
        rb = GetComponent<ArticulationBody>();
    }

    void FixedUpdate()
    {
        if (Input.GetKey(KeyCode.E))
        {
            Debug.Log("Rotating");
        }
        if(Input.GetKey(KeyCode.E) && !_rotating)
        {
            _rotating = true;
           if (clockwise) {
               rb.AddRelativeTorque(Vector3.one * thrust);
           } else {
               rb.AddRelativeTorque(Vector3.one * -thrust);
           }
           
           _rotating = false;
        }
    }
    
    
    // [SerializeField]
    // private float _rotationTime = 2f;
    // [SerializeField]
    // private float _delayBetweenRotations = 1f;
    // private bool _rotating;
    
    // private WaitForSeconds _rotationDelay;
    // private Quaternion _targetRot;
 
    // private void Start()
    // {
    //     _rotationDelay = new WaitForSeconds(_delayBetweenRotations);
    //     _targetRot = transform.localRotation;
    // }
 
    // private void Update()
    // {
    //     if(Input.GetKey(KeyCode.E) && !_rotating)
    //     {
    //         _rotating = true;
 
    //         StartCoroutine(doRotate(_rotationTime));
    //     }
    // }
 
    // private IEnumerator doRotate(float rotateTime)
    // {
    //     var startRot = transform.localRotation;
    //     _targetRot *= Quaternion.AngleAxis(90, new Vector3(0, -1, 0));
 
    //     var time = 0f;
 
    //     while (time <= 1f)
    //     {
    //         transform.localRotation = Quaternion.Lerp(startRot, _targetRot, time);
    //         time += Time.deltaTime / rotateTime;
    //         yield return null;
    //     }
 
    //     transform.localRotation = _targetRot;
 
    //     yield return _rotationDelay;
 
    //     _rotating = false;
    // }
}
