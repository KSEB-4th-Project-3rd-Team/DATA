using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Unity.VisualScripting;

public class AGVAgent : Agent
{
    //[Header("AGV Position Setting")]
    //public Transform fixedStartPoint;   // �� ���� �ʿ��Ѱ�?

    public Vector3 currentTarget;

    private bool isPausedByYield = false;
    private float pauseCooldown = 1.0f;
    private float pauseTimer = 0f;

    public float moveSpeed = 1.5f;
    public float rotationSpeed = 90f;       // �� ���� ��� ����

    public RayPerceptionSensorComponent3D raySensor;
    private Rigidbody rb;

    // �ൿ ����
    const int Idle = 0;
    const int Forward = 1;
    const int Left = 2;
    const int Right = 3;

    // ȸ�� ���� �κ� & �� �κ��� AMR��� ������ �� �ֵ��� �����ҰŶ� ������ ����    ���߿� �ٲܶ� �ߴµ� �ٲ�� �н� ȿ�� �ö󰣴ٳ�
    // �ٵ� �ùķ����� �����°ǵ� �׷��� õõ�� ������...
    private bool isRotating = false;
    private Quaternion targetRotation;
    public float rotationSpeedDegPerSec = 180f; // 90�� ȸ���� 0.5�� ����..?

    private float prevDistToTarget = 0f;
    public Transform[] startPts;
    private int collisionCnt = 0;

    // ������Ʈ�� ���� ������ ��Ȯ�� �����ϱ� ���� ����
    //private Quaternion currentAgentRotation;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update���� ���� ȸ�� ������ ��� �ȵ�.
    // FixedUpdate( ) ���� �̵��� �̷������ OnActionReceived�� ȣ��
    //private void Update()
    //{
    //    if (isRotating)
    //    {
    //        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation , rotationSpeedDegPerSec * Time.deltaTime);
    //        if (Quaternion.Angle(transform.rotation, targetRotation) < 0.1f)
    //        {
    //            transform.rotation = targetRotation;
    //            isRotating = false;
    //        }
    //    }
    //}

    // FixedUpdate���� ȸ�� ���� ó�� (Update ���)
    void FixedUpdate() // FixedUpdate�� ����
    {
        // OnActionReceived���� ������ isRotating ���¿� ���� ȸ�� ����
        if (isRotating)
        {
            // ���� ȸ�������� ��ǥ ȸ�������� ���������� ȸ��
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotationSpeedDegPerSec * Time.fixedDeltaTime);

            // ��ǥ ȸ���� ���� �����ϸ� ȸ�� �Ϸ� ó��
            if (Quaternion.Angle(transform.rotation, targetRotation) < 0.1f)
            {
                transform.rotation = targetRotation; // ��Ȯ�� ��ǥ ������ ����
                isRotating = false; // ȸ�� �Ϸ� �÷��� ����
            }
        }
         else { Debug.Log("ȸ�� ����"); } // ����׿�
    }

    public override void OnEpisodeBegin()
    {
        isPausedByYield = false;
        pauseTimer = 0f;
        isRotating = false;
        int randIdx = Random.Range(0, startPts.Length);

        transform.position = startPts[randIdx].position;
        transform.rotation = Quaternion.Euler(0f, -90f, 0f);
        //currentAgentRotation = transform.rotation;  // ���� ������Ʈ�� ȸ���� �ʱ�ȭ
    
        SetNewRandomTarget();
        prevDistToTarget = Vector3.Distance(transform.position, currentTarget);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. AMR�� ���� ��ġ (���� ��ǥ)
        sensor.AddObservation(transform.position);

        // 2. AMR�� ���� ���� ���� (���� ��ǥ)
        sensor.AddObservation(transform.forward);

        // 3. ��ǥ���� ���� ��ǥ
        sensor.AddObservation(currentTarget);

        // 4. ��ǥ���� AMR ���� ��ǥ -> AMR�� �ڽ��� ��ġ���� ��ǥ���� ��� ���� (x, z)�� �ִ��� ���������� ����
        Vector3 targetLocalPos = transform.InverseTransformPoint(currentTarget);
        sensor.AddObservation(targetLocalPos.x);
        sensor.AddObservation(targetLocalPos.z);

        // 5. ��ǥ�������� ������� ���� (��/��, ��/��) -> AMR�� ���� �ٶ󺸴� ����� ��ǥ�� ������ ���� ���� ����
        Vector3 dirToTarget = (currentTarget - transform.position).normalized;
        float dotForward = Vector3.Dot(transform.forward, dirToTarget);     // 1 : ����, -1 : �ĸ�
        float dotRight = Vector3.Dot(transform.right, dirToTarget);         // 1: ������, -1 : ����
        sensor.AddObservation(dotForward);
        sensor.AddObservation(dotRight);

        // 6. ��ǥ�������� �Ÿ�
        float distanceToTarget = Vector3.Distance(transform.position, currentTarget);
        sensor.AddObservation(distanceToTarget);

        // RayPerceptionSensor�� ������ ������Ʈ�� �ڵ����� ó��.
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        //Debug.Log("OnActionReceived ȣ���");

        //if (isPausedByYield)      �ٸ� AMR�� ��� �н��� ���ص� �� ����. ���� �н��� ���� ��� ��Ȱ��ȭ��Ű��
        //{
        //    pauseTimer += Time.deltaTime;
        //    if (pauseTimer >= pauseCooldown)
        //    {
        //        isPausedByYield = false;
        //        pauseTimer = 0f;
        //    }
        //    return;
        //}

        // AMR �� ���� ���� �� ���ǿ� �� �� ������ Ȥ�� �� ��� ���..
        if (IsAGVOnRight())
        {
            isPausedByYield = true;
            pauseTimer = 0f;
            return;
        }

        // ȸ�� �߿��� �ٸ� �ൿ�� ���� �ʵ��� ó��
        if (isRotating)
        {
            // ȸ�� �߿��� �߰� ����/�г�Ƽ�� ���� �ʰų�, ���� ���� �г�Ƽ�� �� �� ���� (������)
             AddReward(-0.0001f); // ȸ�� �� �ð� ��� �г�Ƽ
            return; // �ٸ� �ൿ ó�� ���� ����
        }

        int action = actions.DiscreteActions[0];
        AddReward(-0.001f);     // ���� 0.01f���� ����, �Ÿ� ��� ���� �� ������ �� �ֵ���

        switch (action)
        {
            case Idle:
                AddReward(-0.0001f);        // �ƹ��͵� ���� ���� �߰� �г�Ƽ (�ʿ��Ѱ�?)
                break;
            case Forward:
                // Raycast�� ����Ͽ� �浹 �̸� ����, ������Ʈ�� �̸� ��ֹ� �����ϰ� ȸ���ϵ��� ����
                if (!IsPathBlocked())
                {
                    MoveForward();
                }
                else
                {
                    AddReward(-1f);     // ���� ��� ������ �� ū �г�Ƽ (���� -0.75f)
                }
                break;
            case Left:
                // RotateLeft �Լ��� ȣ���ϴ� ���, ���⼭ ��ǥ ȸ���� ���� �� ȸ�� ����
                targetRotation = transform.rotation * Quaternion.Euler(0f, -90f, 0f);
                isRotating = true;
                AddReward(-0.005f);     // ȸ�� �ൿ�� ���� ���� �г�Ƽ, -0.02f���� ����
                break;
            case Right:
                // RotateRight �Լ��� ȣ���ϴ� ���, ���⼭ ��ǥ ȸ���� ���� �� ȸ�� ����
                targetRotation = transform.rotation * Quaternion.Euler(0f, 90f, 0f);
                isRotating = true;
                AddReward(-0.005f);
                break;
            default:
                Debug.LogWarning("Invalid acction received");
                AddReward(-0.1f);       // �� �� ���� �ൿ�� ���� �г�Ƽ
                break;
        }

        float currDist = Vector3.Distance(transform.position, currentTarget);
        float distDelta = prevDistToTarget - currDist;      // ���� ��ġ�� �Ÿ��� �پ����� ���, �ø� ����
        
        if (distDelta > 0.01f)
        {
            AddReward(distDelta * 2.0f);        // 1.5f���� ��ȭ (�ִ� ��� ����)
            // Debug.Log($"Reward: Closer by {distDelta:F2}, current reward: {GetCumulativeReward():F2}");
        }

        else if (distDelta < -0.01f)    // ��ǥ���� �־����� ��
        {
            AddReward(distDelta * 3.0f);        // �־����� �Ϳ� ���� �г�Ƽ ��ȭ (��������? �ʹ� ��ū��)
             // Debug.Log($"Penalty: Further by {-distDelta:F2}, current reward: {GetCumulativeReward():F2}");                                                
        }

        prevDistToTarget = currDist;    // ���� ������ ���� ���� �Ÿ� ����

        if (currDist < 1f)
        {
            float finalReward = 100f;

            int maxPossibleSteps = MaxStep;     // Academy ������ Max Steps
            if (maxPossibleSteps == 0) maxPossibleSteps = 5000;     // �����Ϳ��� ���� ���ߴٸ� �⺻�� 5000���� ����

            if (StepCount <= maxPossibleSteps / 2)      // ���� ���� �̳��� ���� �� ���ʽ�
            {
                float bonusRatio = 1f - ((float)StepCount / (maxPossibleSteps / 2));    // 0 (�ʰ�) ~ 1 (����), ���� �����Ҽ��� ū ���ʽ�
                finalReward += bonusRatio * 50f;
            }
            else            // ���� ���� ���� �г�Ƽ
            {
                float penaltyRatio = ((float)StepCount - (maxPossibleSteps / 2)) / (maxPossibleSteps / 2);      // 0 (�ʰ�) ~ 1 (�� �ʰ�), �ʰ� �����Ҽ��� �� ū �г�Ƽ
                finalReward -= penaltyRatio * 75f;      // �ִ� 75 �г�Ƽ
            }


            AddReward(finalReward);
            Debug.Log($"[Step] : {StepCount}, [FinalReward] : {finalReward}");  
            SetNewRandomTarget();
            prevDistToTarget = Vector3.Distance(transform.position, currentTarget);
            EndEpisode();
        }
    }

    private bool IsPathBlocked()
    {
        // AMR�� �߾� �ణ ������ AMR ���� �������� Raycast
        Ray ray = new Ray(transform.position + Vector3.up * 0.5f, transform.forward);

        // Ray ���̴� AMR�� ���� ���ܿ� �̵��� �Ÿ����� ���� �� ��� ����
        float currentRayLength = moveSpeed * Time.fixedDeltaTime * 2.0f;       // ���� 2���� ������ �̸� ����

        // Raycast�� ��� ��� ���͸��ϵ��� LayerMask�� ���
        if (Physics.Raycast(ray, out RaycastHit hit, currentRayLength))
        {
            // Debug.DrawRay(ray.origin, ray.direction * currentRayLength, Color.yellow, Time.fixedDeltaTime); // ������
            if (hit.collider.CompareTag("Wall") || hit.collider.CompareTag("Obstacle"))     // �ϴ� �� ������ ���°Ŷ� "AGV" �±״� ���������� �����
            {
                return true;
            }
        }
        // Debug.DrawRay(ray.origin, ray.direction * currentRayLength, Color.green, Time.fixedDeltaTime); // ������
        return false;
    }

    private void SetNewRandomTarget()
    {
        Transform platform = transform.parent;
        if (platform == null)
        {
            Debug.LogWarning("AGV has no parent platform assigned.");
        }

        List<Transform> targetCandidates = new List<Transform>();
        foreach (Transform child in platform.GetComponentsInChildren<Transform>())
        {
            if (child.CompareTag("TargetPos"))  targetCandidates.Add(child);
        }

        if (targetCandidates.Count == 0)
        {
            Debug.LogWarning("No TargetPos objects found in the scene.");
            return;
        }

        foreach (Transform target in targetCandidates)
        {
            SpriteRenderer marker = target.GetComponent<SpriteRenderer>();
            if (marker != null) marker.enabled = false;
        }

        // ���� AMR�� ��ġ�� �ʹ� ����� ��ǥ������ ����
        List<Transform> validTargets = new List<Transform>();
        foreach(Transform target in targetCandidates)
        {
            if (Vector3.Distance(transform.position, target.position) > 2.0f)       // �ּ� 2m �̻� ������ Ÿ�ٵ鸸 ����
            {
                validTargets.Add(target);
            }
        }

        if (validTargets.Count == 0)
        {
            Debug.LogWarning("No valid TargetPos objects far enough from current position. Resetting to any random target.");
            validTargets = targetCandidates; // ������� ��¿ �� ���� ����
        }

        int randomIndex = Random.Range(0, validTargets.Count);
        Transform selectedTarget = validTargets[randomIndex];
        currentTarget = selectedTarget.position;

        SpriteRenderer selcetedMarker = selectedTarget.GetComponent<SpriteRenderer>();
        if (selcetedMarker != null) selcetedMarker.enabled = true;

        Debug.Log($"[Target Assigned] {this.name}'s target selected: {selectedTarget.name} at {currentTarget}");
    }

    private void OnCollisionEnter(Collision collision)
    {
        // �������� �ε����°� ���Ϸ��� ���ڸ����� ���� ���淡 2�������� ���� �ְ� �� ���Ŀ��� �г�Ƽ ��� ��.
        // �н��� ���� ���Ǽҵ带 �����Ű�� ū �г�Ƽ�� �ο��϶��

        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Obstacle") || collision.gameObject.CompareTag("AGV"))
        {
            AddReward(-100f);
            Debug.Log($"[COLLISION] Hit {collision.gameObject.name} (Tag: {collision.gameObject.tag}). Ending episode. Penalty: -100");
            EndEpisode();
        }
    }

    private bool IsAGVOnRight()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        Vector3 direction = transform.right;    // ������?
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
        // rb.MovePosition�� FixedUpdate���� ȣ���ϴ� ���� �����ϴ�.
        rb.MovePosition(rb.position + transform.forward * moveSpeed * Time.fixedDeltaTime);
    }

    //private void RotateLeft()
    //{
    //    // --- �ൿ ���� 1: ��� 90�� ȸ�� ���� ---
    //    // isRotating �÷��� �� Update ������ �����ϰ� OnActionReceived���� ��� ������ �����մϴ�.
    //    currentAgentRotation *= Quaternion.Euler(0f, -90f, 0f);
    //    transform.rotation = currentAgentRotation;
    //}

    //private void RotateRight()
    //{
    //    // --- �ൿ ���� 1: ��� 90�� ȸ�� ���� ---
    //    currentAgentRotation *= Quaternion.Euler(0f, 90f, 0f);
    //    transform.rotation = currentAgentRotation;
    //}

    // Heuristic ���: ���� ����
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        discreteActionsOut[0] = Idle;

        // ȸ�� �߿��� ����� �Է� ����
        if (isRotating)
        {
            return;
        }

        if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = Forward;
        }
        else if (Input.GetKeyDown(KeyCode.A)) // GetKeyDown ����
        {
            discreteActionsOut[0] = Left;
        }
        else if (Input.GetKeyDown(KeyCode.D)) // GetKeyDown ����
        {
            discreteActionsOut[0] = Right;
        }
    }
}
