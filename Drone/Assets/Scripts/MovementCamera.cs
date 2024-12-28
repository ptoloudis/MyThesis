using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMove : MonoBehaviour
{
    public Transform player;
    [SerializeField] private Vector3 offset = Vector3.zero;
    [SerializeField] private bool rotateOnlyInY;

    // Update is called once per frame
    private void LateUpdate()
    {
        // Calculate the desired position in world space using TransformPoint
        Vector3 desiredPosition = player.TransformPoint(offset);

        // Move the camera to the desired position
        transform.position = desiredPosition;

        // Rotate the camera based on the rotateOnlyInY setting
        if (rotateOnlyInY)
        {
            // Only match the Y-axis rotation of the player
            //Quaternion targetRotation = Quaternion.Euler(0, player.eulerAngles.y, 0);
            //transform.rotation = targetRotation;
            transform.LookAt(player);
        }
        else
        {
            // Match the full rotation of the player
            transform.rotation = player.rotation;
        }
        


    }

    
}

