using UnityEngine;

[RequireComponent(typeof(Transform))]
public class CustomRigidBody3D : MonoBehaviour
{
    [Header("Mass <= 0 => Static")]
    public float _mass = 1f;
    [Header("Friction [0..1]")]
    public float _friction = 0.6f;
    [Header("Restitution [0..1]")]
    public float _restitution = 0.0f;
    [Header("HalfSize for the box collider")]
    public Vector3 _halfSize = new Vector3(0.5f, 0.5f, 0.5f);
    [Header("Damping")]
    public float _linearDamping = 0.02f;  // linear velocity damping factor
    public float _angularDamping = 0.1f;  // angular velocity damping factor
    public float _maxAngularSpeed = 10.0f;

    private int _bodyID = -1;

    void Start()
    {
        // create body in C++ side
        Vector3 pos = transform.position;
        Quaternion rot = transform.rotation; // or Identity if you prefer
        _bodyID = Physics3DWrapper.AddBody(pos, rot, _halfSize, _mass, _friction, _restitution,
            _linearDamping, _angularDamping, _maxAngularSpeed);
    }

    void FixedUpdate()
    {
        // if dynamic, read back updated transform from C++ and apply
        if(_mass>0)
        {
            Vector3 pos; Quaternion rot;
            Physics3DWrapper.GetTransform(_bodyID, out pos, out rot);
            transform.position = pos;
            transform.rotation = rot;
        }
        // if static, no update needed
    }
}