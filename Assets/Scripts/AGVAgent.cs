using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Unity.VisualScripting;

public class AGVAgent : Agent
{
    //[Header("AGV Position Setting")]
    //public Transform fixedStartPoint;   // 얘 굳이 필요한가?

    public Vector3 currentTarget;

    private bool isPausedByYield = false;
    private float pauseCooldown = 1.0f;
    private float pauseTimer = 0f;

    public float moveSpeed = 1.5f;
    public float rotationSpeed = 90f;

    public RayPerceptionSensorComponent3D raySensor;
    private Rigidbody rb;

    // 행동 정의
    const int Idle = 0;
    const int Forward = 1;
    const int Left = 2;
    const int Right = 3;

    // 회전 구현 부분
    private bool isRotating = false;
    private Quaternion targetRotation;
    public float rotationSpeedDegPerSec = 180f; // 90도 회전에 0.5초 정도..?

    private float prevDistToTarget = 0f;
    public Transform[] startPts;
    private int collisionCnt = 0;
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    // 회전 구현 부분 (안되면 삭제,,,?)
    private void Update()
    {
        if (isRotating)
        {
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation , rotationSpeedDegPerSec * Time.deltaTime);
            if (Quaternion.Angle(transform.rotation, targetRotation) < 0.1f)
            {
                transform.rotation = targetRotation;
                isRotating = false;
            }
        }
    }

    public override void OnEpisodeBegin()
    {
        isPausedByYield = false;
        pauseTimer = 0f;
        int randIdx = Random.Range(0, startPts.Length);

        transform.position = startPts[randIdx].position;
        transform.rotation = Quaternion.Euler(0f, -90f, 0f);
    
        SetNewRandomTarget();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.position);
        sensor.AddObservation(transform.forward);
        sensor.AddObservation(currentTarget);

        Vector3 dirToTarget = (currentTarget - transform.position).normalized;
        sensor.AddObservation(dirToTarget);

        float distanceToTarget = Vector3.Distance(transform.position, currentTarget);
        sensor.AddObservation(distanceToTarget);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        //Debug.Log("OnActionReceived 호출됨");

        if (isPausedByYield)
        {
            pauseTimer += Time.deltaTime;
            if (pauseTimer >= pauseCooldown)
            {
                isPausedByYield = false;
                pauseTimer = 0f;
            }
            return;
        }

        if (IsAGVOnRight())
        {
            isPausedByYield = true;
            pauseTimer = 0f;
            return;
        }

        int action = actions.DiscreteActions[0];
        AddReward(-0.01f);

        switch (action)
        {
            case Idle:
                break;
            case Forward:
                if (!IsPathBlocked())
                {
                    MoveForward();
                }
                else
                {
                    AddReward(-0.75f);
                }
                break;
            case Left:
                RotateLeft();
                AddReward(-0.02f);
                break;
            case Right:
                RotateRight();
                AddReward(-0.02f);
                break;
            default:
                Debug.LogWarning("Invalid acction received");
                break;
        }

        float currDist = Vector3.Distance(transform.position, currentTarget);
        float distDelta = prevDistToTarget - currDist;
        //AddReward(distDelta * 0.5f);
        if (distDelta > 0.01f)
        {
            AddReward(distDelta * 1.5f);
            //Debug.Log("Closer");
        }
        else
        {
            AddReward(-0.01f);      
        }

        prevDistToTarget = currDist;

        if (currDist < 1f)
        {
            AddReward(100f);
            Debug.Log("Get Reward +100");
            SetNewRandomTarget();
            prevDistToTarget = Vector3.Distance(transform.position, currentTarget);
        }
    }

    private bool IsPathBlocked()
    {
        Ray ray = new Ray(transform.position + Vector3.up * 0.5f, transform.forward);
        if (Physics.Raycast(ray, out RaycastHit hit, 1.5f))
        {
            if (hit.collider.CompareTag("Wall") || hit.collider.CompareTag("Obstacle"))
            {
                return true;
            }
        }
        return false;
    }

    private void SetNewRandomTarget()
    {
        GameObject[] targetCandidates = GameObject.FindGameObjectsWithTag("TargetPos");

        if (targetCandidates.Length == 0)
        {
            Debug.LogWarning("No TargetPos objects found in the scene.");
            return;
        }

        foreach (GameObject target in targetCandidates)
        {
            SpriteRenderer marker = target.GetComponent<SpriteRenderer>();
            if (marker != null) marker.enabled = false;
        }

        int randomIndex = Random.Range(0, targetCandidates.Length);
        GameObject selectedTarget = targetCandidates[randomIndex];
        currentTarget = selectedTarget.transform.position;

        SpriteRenderer selcetedMarker = selectedTarget.GetComponent<SpriteRenderer>();
        if (selcetedMarker != null) selcetedMarker.enabled = true;

        Debug.Log($"[Target Assigned] New target selected: {selectedTarget.name} at {currentTarget}");
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Obstacle") || collision.gameObject.CompareTag("AGV"))
        {
            float penalty = -10f;
            
            if (transform.position.z > -2f && transform.position.z < 20f)
            {
                penalty = -5f;
            }

            AddReward(penalty);

            int randIdx = Random.Range(0, startPts.Length);
            transform.position = startPts[randIdx].position;
            collisionCnt++;
            Debug.Log($"Collsion Count : {collisionCnt}");

            if (collisionCnt >= 3)
            {
                AddReward(-20f);
                Debug.Log("Get Penalty -20");
                collisionCnt = 0;
                EndEpisode();
            }
        }
    }

    private bool IsAGVOnRight()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        Vector3 direction = transform.right;    // 오른쪽?
        float rayLength = 2.0f;

        if (Physics.Raycast(origin, direction, out RaycastHit hit, rayLength))
        {
            if (hit.collider.CompareTag("AGV"))
            {
                return true;
            }
        }
        return false;
    }

    private void MoveForward()
    {
        Vector3 movement = transform.forward * moveSpeed * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + movement);
    }

    private void RotateLeft()
    {
        //transform.Rotate(Vector3.up, -rotationSpeed * Time.deltaTime);

        if (!isRotating)
        {
            targetRotation = Quaternion.Euler(0f, transform.eulerAngles.y - 90f, 0f);
            isRotating = true;
        }
    }

    private void RotateRight()
    {
        //transform.Rotate(Vector3.up, rotationSpeed * Time.deltaTime);

        if (!isRotating)
        {
            targetRotation = Quaternion.Euler(0f, transform.eulerAngles.y + 90f, 0f);
            isRotating = true;
        }
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //Debug.Log("Heuristic 실행됨.");

        var discreteActionsOut = actionsOut.DiscreteActions;
        discreteActionsOut[0] = Idle;

        if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = Forward;
            Debug.Log("Press W");
        }
        if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[0] = Left;
            Debug.Log("Press A");
        }
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[0] = Right;
            Debug.Log("Press D");
        }
    }
}
