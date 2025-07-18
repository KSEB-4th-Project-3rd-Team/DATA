using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using System.Collections.Generic;

public class AMRAgent : Agent
{
    public WarehouseManager manager;        // WarhouseManager�� NewPlatform�� ���� ��ũ��Ʈ
    public float moveSpeed = 1.5f;

    private Vector2Int agvGridPos;          // AMR�� ���� ��ġ�� �׸��� ���·� ��Ÿ�� ����
                                            // �ٵ� �̰Ŵ� ��ǥ�� �̵���Ű�°�, �ù� �󿡼��� �����ϴ� ��ó�� ������ �� �� ������ �ϴ� ���߿� ���.
    private RackState carryingRack = null;  // RackState �� ���� ��Ÿ���� Ŭ����, WarehouseManager.cs�� �������. carryingRack�� AMR�� ����ϰ� �ִ� ��

    public void SetGridPos(Vector2Int gridPos)
    {
        this.agvGridPos = gridPos;      // AMR ������Ʈ�� ���� ��ǥ�� ���� ��ġ�� ��ġ
        transform.position = manager.GridToWorld(gridPos);      // ����Ƽ ���� ��ǥ�� ���� �̵�������.
    }


    public override void OnEpisodeBegin()
    {
        manager.ResetEnvironment(this);     // ȯ�� �缳��
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. AGV ��ġ (2����)
        sensor.AddObservation(agvGridPos);

        // 2. ���� ��ǥ �� ��ġ
        Vector2Int targetPos = manager.GetTargetPosition(carryingRack);
        sensor.AddObservation(targetPos);

        // 3. AGV�� ��� �ִ� �� ID (������ -1)
        sensor.AddObservation(carryingRack != null ? carryingRack.rackID : -1);

        // 4. �ֹ� ť ����
        sensor.AddObservation(manager.OrderQueueCount());

        // 5. �� ���µ� (0~3)
        foreach (var rack in manager.racks)
        {
            sensor.AddObservation((int)rack.status);
        }

        // 6. �ֺ� 7x7 ���� ���� (��ֹ�/����)
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
            // action == 4 �� ����

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
            AddReward(-10f); // �ð� �ʰ� �г�Ƽ
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Ű���� ���� ����
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
