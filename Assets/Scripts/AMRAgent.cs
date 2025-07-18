using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using System.Collections.Generic;

public class AMRAgent : Agent
{
    public WarehouseManager manager;        // WarhouseManager는 NewPlatform에 붙일 스크립트
    public float moveSpeed = 1.5f;

    private Vector2Int agvGridPos;          // AMR의 현재 위치를 그리드 형태로 나타낸 변수
                                            // 근데 이거는 좌표만 이동시키는것, 시뮬 상에서는 운전하는 것처럼 보여야 할 것 같은데 일단 나중에 고려.
    private RackState carryingRack = null;  // RackState 랙 상태 나타내는 클래스, WarehouseManager.cs에 들어있음. carryingRack은 AMR이 운반하고 있는 랙

    public void SetGridPos(Vector2Int gridPos)
    {
        this.agvGridPos = gridPos;      // AMR 에이전트를 격자 좌표계 상의 위치에 배치
        transform.position = manager.GridToWorld(gridPos);      // 유니티 월드 좌표로 실제 이동시켜줌.
    }


    public override void OnEpisodeBegin()
    {
        manager.ResetEnvironment(this);     // 환경 재설정
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. AGV 위치 (2차원)
        sensor.AddObservation(agvGridPos);

        // 2. 현재 목표 랙 위치
        Vector2Int targetPos = manager.GetTargetPosition(carryingRack);
        sensor.AddObservation(targetPos);

        // 3. AGV가 들고 있는 랙 ID (없으면 -1)
        sensor.AddObservation(carryingRack != null ? carryingRack.rackID : -1);

        // 4. 주문 큐 길이
        sensor.AddObservation(manager.OrderQueueCount());

        // 5. 랙 상태들 (0~3)
        foreach (var rack in manager.racks)
        {
            sensor.AddObservation((int)rack.status);
        }

        // 6. 주변 7x7 영역 정보 (장애물/공간)
        int[,] localMap = manager.GetLocalMap(agvGridPos);
        foreach (var val in localMap)
            sensor.AddObservation(val);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int action = actions.DiscreteActions[0];
        float reward = -1f;

        Vector2Int newPos = agvGridPos;

        if (action >= 0 && action <= 4)
        {
            if (action == 0) newPos += new Vector2Int(0, 1);   // forward(z+)
            else if (action == 1) newPos += new Vector2Int(0, -1);  // backward(z?)
            else if (action == 2) newPos += new Vector2Int(-1, 0);  // left(x?)
            else if (action == 3) newPos += new Vector2Int(1, 0);   // right(x+)
            // action == 4 → 정지

            bool collided = manager.CheckCollision(newPos);
            if (collided)
            {
                reward -= 100f;
            }
            else
            {
                agvGridPos = newPos;
                transform.position = manager.GridToWorld(agvGridPos);
            }
        }

        else if (action == 5) // PickUp
        {
            bool success = manager.TryPickUpRack(agvGridPos, ref carryingRack);
            reward += success ? 10f : -1f;
        }

        else if (action == 6) // Drop
        {
            var dropResult = manager.TryDropRack(agvGridPos, ref carryingRack);
            if (dropResult == DropResult.CorrectDelivery) reward += 100f;
            else if (dropResult == DropResult.CorrectReplenish) reward += 50f;
            else if (dropResult == DropResult.WrongDelivery) reward -= 50f;
            else reward -= 10f;
        }

        SetReward(reward);

        if (manager.AllOrdersCompleted())
        {
            SetReward(+500f);
            EndEpisode();
        }

        if (StepCount >= manager.maxStepPerEpisode)
        {
            AddReward(-10f); // 시간 초과 패널티
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // 키보드 수동 조작
        var discreteActions = actionsOut.DiscreteActions;
        discreteActions[0] = 4;
        if (Input.GetKey(KeyCode.UpArrow)) discreteActions[0] = 0;
        else if (Input.GetKey(KeyCode.DownArrow)) discreteActions[0] = 1;
        else if (Input.GetKey(KeyCode.LeftArrow)) discreteActions[0] = 2;
        else if (Input.GetKey(KeyCode.RightArrow)) discreteActions[0] = 3;
        else if (Input.GetKey(KeyCode.Z)) discreteActions[0] = 5;
        else if (Input.GetKey(KeyCode.X)) discreteActions[0] = 6;
    }
}
