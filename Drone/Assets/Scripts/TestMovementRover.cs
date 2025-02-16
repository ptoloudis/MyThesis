using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestMovementRover : MonoBehaviour
{
    // List of positions to move to
    private Vector3[] positions = new Vector3[]
    {
        new Vector3(-114, 0, 15),  // Replace Y-axis with 0 or desired value
        new Vector3(-114, 0, 50),
        new Vector3(-114, 0, 70),
        new Vector3(-134, 0, 70),
        new Vector3(-154, 0, 70),
        new Vector3(-174, 0, 70),
        new Vector3(-194, 0, 70),
        new Vector3(-194, 0, 50)
    };

    public float speed = 1.25f; // Movement speed (1.25 m/s)
    public float rotationSpeed = 2.0f; // Rotation speed
    private int currentTarget = 0;
    private Quaternion targetRotation;
    private bool missionStarted = false;

    void Start()
    {
        targetRotation = transform.rotation;
    }

    void Update()
    {
        // Check if the space key is pressed to start the mission
        if (Input.GetKeyDown(KeyCode.Space))
        {
            missionStarted = true;
        }

        if (missionStarted && currentTarget < positions.Length)
        {
            // Move the object towards the target position
            transform.position = Vector3.MoveTowards(transform.position, positions[currentTarget], speed * Time.deltaTime);

            // Check if it has reached the target
            if (Vector3.Distance(transform.position, positions[currentTarget]) < 0.1f)
            {
                // Check if we are at (-114, 70) and set target rotation
                if (positions[currentTarget] == new Vector3(-114, 0, 70))
                {
                    targetRotation = Quaternion.Euler(0, -90, 0); // Set target rotation to Y = -90
                }
                else if (positions[currentTarget] == new Vector3(-194, 0, 70))
                {
                    targetRotation = Quaternion.Euler(0, -180, 0); // Set target rotation to Y = -180
                }

                currentTarget++; // Move to the next position
            }

            // Smoothly rotate towards the target rotation
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
        }
    }
}
