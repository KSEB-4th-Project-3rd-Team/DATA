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
    public float rotationSpeed = 90f;       // 얘 딱히 사용 안함

    public RayPerceptionSensorComponent3D raySensor;
    private Rigidbody rb;

    // 행동 정의
    const int Idle = 0;
    const int Forward = 1;
    const int Left = 2;
    const int Right = 3;

    // 회전 구현 부분 & 이 부분은 AMR답게 움직일 수 있도록 수정할거라 삭제될 예정    나중에 바꿀라 했는데 바꿔야 학습 효율 올라간다네
    // 근데 시뮬레이터 돌리는건데 그래도 천천히 돌리는...
    private bool isRotating = false;
    private Quaternion targetRotation;
    public float rotationSpeedDegPerSec = 180f; // 90도 회전에 0.5초 정도..?

    private float prevDistToTarget = 0f;
    public Transform[] startPts;
    private int collisionCnt = 0;

    // 에이전트의 현재 각도를 정확히 추적하기 위한 변수
    //private Quaternion currentAgentRotation;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update문은 이제 회전 로직에 사용 안됨.
    // FixedUpdate( ) 에서 이동이 이루어지고 OnActionReceived가 호췰
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

    // FixedUpdate에서 회전 로직 처리 (Update 대신)
    void FixedUpdate() // FixedUpdate로 변경
    {
        // OnActionReceived에서 설정된 isRotating 상태에 따라 회전 진행
        if (isRotating)
        {
            // 현재 회전값에서 목표 회전값으로 점진적으로 회전
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotationSpeedDegPerSec * Time.fixedDeltaTime);

            // 목표 회전에 거의 도달하면 회전 완료 처리
            if (Quaternion.Angle(transform.rotation, targetRotation) < 0.1f)
            {
                transform.rotation = targetRotation; // 정확히 목표 각도로 설정
                isRotating = false; // 회전 완료 플래그 해제
            }
        }
         else { Debug.Log("회전 안함"); } // 디버그용
    }

    public override void OnEpisodeBegin()
    {
        isPausedByYield = false;
        pauseTimer = 0f;
        isRotating = false;
        int randIdx = Random.Range(0, startPts.Length);

        transform.position = startPts[randIdx].position;
        transform.rotation = Quaternion.Euler(0f, -90f, 0f);
        //currentAgentRotation = transform.rotation;  // 현재 에이전트의 회전값 초기화
    
        SetNewRandomTarget();
        prevDistToTarget = Vector3.Distance(transform.position, currentTarget);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. AMR의 현재 위치 (월드 좌표)
        sensor.AddObservation(transform.position);

        // 2. AMR의 현재 정면 방향 (월드 좌표)
        sensor.AddObservation(transform.forward);

        // 3. 목표점의 월드 좌표
        sensor.AddObservation(currentTarget);

        // 4. 목표점의 AMR 로컬 좌표 -> AMR이 자신의 위치에서 목표점이 어느 방향 (x, z)에 있는지 직관적으로 이해
        Vector3 targetLocalPos = transform.InverseTransformPoint(currentTarget);
        sensor.AddObservation(targetLocalPos.x);
        sensor.AddObservation(targetLocalPos.z);

        // 5. 목표점까지의 상대적인 방향 (앞/뒤, 좌/우) -> AMR이 현재 바라보는 방향과 목표점 사이의 각도 정보 제공
        Vector3 dirToTarget = (currentTarget - transform.position).normalized;
        float dotForward = Vector3.Dot(transform.forward, dirToTarget);     // 1 : 정면, -1 : 후면
        float dotRight = Vector3.Dot(transform.right, dirToTarget);         // 1: 오른쪽, -1 : 왼쪽
        sensor.AddObservation(dotForward);
        sensor.AddObservation(dotRight);

        // 6. 목표점까지의 거리
        float distanceToTarget = Vector3.Distance(transform.position, currentTarget);
        sensor.AddObservation(distanceToTarget);

        // RayPerceptionSensor의 관측은 컴포넌트가 자동으로 처리.
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        //Debug.Log("OnActionReceived 호출됨");

        //if (isPausedByYield)      다른 AMR이 없어서 학습에 방해될 수 있음. 단일 학습할 때는 잠시 비활성화시키기
        //{
        //    pauseTimer += Time.deltaTime;
        //    if (pauseTimer >= pauseCooldown)
        //    {
        //        isPausedByYield = false;
        //        pauseTimer = 0f;
        //    }
        //    return;
        //}

        // AMR 한 대일 때는 이 조건에 들어갈 일 없지만 혹시 모를 경우 대비..
        if (IsAGVOnRight())
        {
            isPausedByYield = true;
            pauseTimer = 0f;
            return;
        }

        // 회전 중에는 다른 행동을 받지 않도록 처리
        if (isRotating)
        {
            // 회전 중에는 추가 보상/패널티를 주지 않거나, 아주 작은 패널티만 줄 수 있음 (선택적)
             AddReward(-0.0001f); // 회전 중 시간 경과 패널티
            return; // 다른 행동 처리 없이 종료
        }

        int action = actions.DiscreteActions[0];
        AddReward(-0.001f);     // 이전 0.01f보다 감소, 거리 기반 보상에 더 집중할 수 있도록

        switch (action)
        {
            case Idle:
                AddReward(-0.0001f);        // 아무것도 안할 때에 추가 패널티 (필요한가?)
                break;
            case Forward:
                // Raycast를 사용하여 충돌 미리 감지, 에이전트가 미리 장애물 인지하고 회피하도록 유도
                if (!IsPathBlocked())
                {
                    MoveForward();
                }
                else
                {
                    AddReward(-1f);     // 막힌 길로 가려할 때 큰 패널티 (기존 -0.75f)
                }
                break;
            case Left:
                // RotateLeft 함수를 호출하는 대신, 여기서 목표 회전값 설정 및 회전 시작
                targetRotation = transform.rotation * Quaternion.Euler(0f, -90f, 0f);
                isRotating = true;
                AddReward(-0.005f);     // 회전 행동에 대한 작은 패널티, -0.02f보다 감소
                break;
            case Right:
                // RotateRight 함수를 호출하는 대신, 여기서 목표 회전값 설정 및 회전 시작
                targetRotation = transform.rotation * Quaternion.Euler(0f, 90f, 0f);
                isRotating = true;
                AddReward(-0.005f);
                break;
            default:
                Debug.LogWarning("Invalid acction received");
                AddReward(-0.1f);       // 알 수 없는 행동에 대한 패널티
                break;
        }

        float currDist = Vector3.Distance(transform.position, currentTarget);
        float distDelta = prevDistToTarget - currDist;      // 이전 위치와 거리가 줄었으면 양수, 늘면 음수
        
        if (distDelta > 0.01f)
        {
            AddReward(distDelta * 2.0f);        // 1.5f보다 강화 (최단 경로 유도)
            // Debug.Log($"Reward: Closer by {distDelta:F2}, current reward: {GetCumulativeReward():F2}");
        }

        else if (distDelta < -0.01f)    // 목표물과 멀어졌을 때
        {
            AddReward(distDelta * 3.0f);        // 멀어지는 것에 대한 패널티 강화 (괜찮은가? 너무 안큰가)
             // Debug.Log($"Penalty: Further by {-distDelta:F2}, current reward: {GetCumulativeReward():F2}");                                                
        }

        prevDistToTarget = currDist;    // 다음 스탭을 위해 현재 거리 저장

        if (currDist < 1f)
        {
            float finalReward = 100f;

            int maxPossibleSteps = MaxStep;     // Academy 설정의 Max Steps
            if (maxPossibleSteps == 0) maxPossibleSteps = 5000;     // 에디터에서 설정 안했다면 기본값 5000으로 가정

            if (StepCount <= maxPossibleSteps / 2)      // 절반 스텝 이내에 도달 시 보너스
            {
                float bonusRatio = 1f - ((float)StepCount / (maxPossibleSteps / 2));    // 0 (늦게) ~ 1 (빨리), 빨리 도달할수록 큰 보너스
                finalReward += bonusRatio * 50f;
            }
            else            // 절반 스탭 이후 패널티
            {
                float penaltyRatio = ((float)StepCount - (maxPossibleSteps / 2)) / (maxPossibleSteps / 2);      // 0 (늦게) ~ 1 (더 늦게), 늦게 도달할수록 더 큰 패널티
                finalReward -= penaltyRatio * 75f;      // 최대 75 패널티
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
        // AMR의 중앙 약간 위에서 AMR 정면 방향으로 Raycast
        Ray ray = new Ray(transform.position + Vector3.up * 0.5f, transform.forward);

        // Ray 길이는 AMR이 다음 스텝에 이동할 거리보다 조금 더 길게 설정
        float currentRayLength = moveSpeed * Time.fixedDeltaTime * 2.0f;       // 전방 2스텝 정도를 미리 감지

        // Raycast가 닿는 대상만 필터링하도록 LayerMask를 사용
        if (Physics.Raycast(ray, out RaycastHit hit, currentRayLength))
        {
            // Debug.DrawRay(ray.origin, ray.direction * currentRayLength, Color.yellow, Time.fixedDeltaTime); // 디버깅용
            if (hit.collider.CompareTag("Wall") || hit.collider.CompareTag("Obstacle"))     // 일단 길 막힌지 보는거라 "AGV" 태그는 넣을지말지 고민중
            {
                return true;
            }
        }
        // Debug.DrawRay(ray.origin, ray.direction * currentRayLength, Color.green, Time.fixedDeltaTime); // 디버깅용
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

        // 현재 AMR의 위치와 너무 가까운 목표점들은 제외
        List<Transform> validTargets = new List<Transform>();
        foreach(Transform target in targetCandidates)
        {
            if (Vector3.Distance(transform.position, target.position) > 2.0f)       // 최소 2m 이상 떨어진 타겟들만 선택
            {
                validTargets.Add(target);
            }
        }

        if (validTargets.Count == 0)
        {
            Debug.LogWarning("No valid TargetPos objects far enough from current position. Resetting to any random target.");
            validTargets = targetCandidates; // 가까워도 어쩔 수 없이 선택
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
        // 기존에는 부딪히는걸 피하려고 제자리에서 뺑뺑 돌길래 2번까지는 적게 주고 그 이후에는 패널티 쎄게 줌.
        // 학습을 위해 에피소드를 종료시키고 큰 패널티를 부여하라네

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
        // rb.MovePosition은 FixedUpdate에서 호출하는 것이 좋습니다.
        rb.MovePosition(rb.position + transform.forward * moveSpeed * Time.fixedDeltaTime);
    }

    //private void RotateLeft()
    //{
    //    // --- 행동 개선 1: 즉시 90도 회전 적용 ---
    //    // isRotating 플래그 및 Update 로직을 제거하고 OnActionReceived에서 즉시 각도를 변경합니다.
    //    currentAgentRotation *= Quaternion.Euler(0f, -90f, 0f);
    //    transform.rotation = currentAgentRotation;
    //}

    //private void RotateRight()
    //{
    //    // --- 행동 개선 1: 즉시 90도 회전 적용 ---
    //    currentAgentRotation *= Quaternion.Euler(0f, 90f, 0f);
    //    transform.rotation = currentAgentRotation;
    //}

    // Heuristic 모드: 수동 제어
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        discreteActionsOut[0] = Idle;

        // 회전 중에는 사용자 입력 무시
        if (isRotating)
        {
            return;
        }

        if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = Forward;
        }
        else if (Input.GetKeyDown(KeyCode.A)) // GetKeyDown 유지
        {
            discreteActionsOut[0] = Left;
        }
        else if (Input.GetKeyDown(KeyCode.D)) // GetKeyDown 유지
        {
            discreteActionsOut[0] = Right;
        }
    }
}
