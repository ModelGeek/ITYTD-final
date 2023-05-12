using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using static UnityEngine.GraphicsBuffer;

public class PhysicsHand : MonoBehaviour
{
    [SerializeField] Rigidbody _playerRigidbody;  
    Rigidbody m_Rigidbody;

    //Move함수에서 사용할 변수
    [SerializeField] float frequency = 50f;
    [SerializeField] float damping = 1f;  
    [SerializeField] Transform target;  
    

    //Rotate함수에서 사용할 변수
    [SerializeField] float rotfrequency = 100f;
    [SerializeField] float rotDamping = 0.9f;

    //실제 오를 때 사용할 변수
    [SerializeField] float climbForce = 1000f;
    [SerializeField] float climbDrag = 500f;

    Vector3 _previousPosition;
    bool _isColliding;

    // Start is called before the first frame update
    void Start()
    {
        m_Rigidbody= GetComponent<Rigidbody>();    
        m_Rigidbody.maxAngularVelocity = float.PositiveInfinity;
        _previousPosition = transform.position;
    }

   
    void FixedUpdate()  
    {
        HandMove();
        HandRotate();
        if (_isColliding) Climb();
    }

    void HandMove()
    {
        float kp = (6f * frequency) * (6f * frequency) * 0.25f;
        float kd = 4.5f * frequency * damping;
        float g = 1 / (1 + kd * Time.fixedDeltaTime + kp * Time.fixedDeltaTime * Time.fixedDeltaTime);
        float ksg = kp * g;
        float kdg = (kd + kp * Time.fixedDeltaTime) * g;

        Vector3 force = (target.position - transform.position) * ksg + (_playerRigidbody.velocity - m_Rigidbody.velocity) * kdg;
        m_Rigidbody.AddForce(force, ForceMode.Acceleration);
    }

    void HandRotate()
    {
        float kp = (6f * rotfrequency) * (6f * rotfrequency) * 0.25f;
        float kd = 4.5f * rotfrequency * rotDamping;
        float g = 1 / (1 + kd * Time.fixedDeltaTime + kp * Time.fixedDeltaTime * Time.fixedDeltaTime);
        float ksg = kp * g;
        float kdg = (kd + kp * Time.fixedDeltaTime) * g;
        Quaternion q = target.rotation * Quaternion.Inverse(transform.rotation); 
        if (q.w < 0)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        q.ToAngleAxis(out float angle, out Vector3 axis);
        axis.Normalize();
        axis *= Mathf.Deg2Rad;
        Vector3 torque = ksg * axis * angle + m_Rigidbody.angularVelocity * kdg;
        m_Rigidbody.AddTorque(torque, ForceMode.Acceleration);
    }

    void Climb()
    {
        Vector3 displacementFromResting = transform.position - target.position;
        Vector3 force = displacementFromResting * climbForce;
        float drag = GetDrag();

        _playerRigidbody.AddForce(force, ForceMode.Acceleration);
        _playerRigidbody.AddForce(drag * -_playerRigidbody.velocity * climbDrag, ForceMode.Acceleration);
    }

    float GetDrag()
    {
        Vector3 handVelocity = (target.localPosition - _previousPosition) / Time.fixedDeltaTime;
        float drag = 1 / handVelocity.magnitude + 0.01f;
        drag = drag > 1 ? 1 : drag;
        drag = drag < 0.03f ? 0.03f : drag;
        _previousPosition = transform.position;
        return drag;
    }

    void OnCollisionEnter(Collision collision)
    {
        _isColliding = true;
    }

    void OnCollisionExit(Collision other)
    {
        _isColliding = false;
    }
}
