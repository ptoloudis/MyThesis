using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent(typeof(Rigidbody))]
public class BaseRigibody : MonoBehaviour
{
    [Header("Rigidbody Properties")]
    [SerializeField] private float weight = 1.5f;
    protected float startDrags;
    protected float startAngularDrags;

    protected Rigidbody rb;

    // Awake
    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (rb)
        {
            rb.mass = weight;
            startDrags = rb.drag;
            startAngularDrags = rb.angularDrag;
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (!rb)
            return;

        HandlePhysics();

    }

    protected virtual void HandlePhysics() { }

}
