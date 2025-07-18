using System.Collections.Generic;
using UnityEngine;

public enum RackStatus { Empty = 0, Filled = 1, WaitingDelivery = 2, InTransit = 3 }
public enum DropResult { None, CorrectDelivery, CorrectReplenish, WrongDelivery, Invalid }
// ���� �������� ���� ��� : None, ��� ����, ��� ���� ����, �߸� ���, �߸� �õ�

public class RackState : MonoBehaviour
{
    public int rackID;
    public Vector2Int gridPos;
    public RackStatus status;   // ���� ���¸� ��Ÿ���� enum : Empty 0��, Filled 1��, ����� 2��, ��� �� 3��
}

public class WarehouseManager : MonoBehaviour
{
    public int maxStepPerEpisode = 500; // �� ���Ǽҵ� �� �ִ� ���� ��

    public int width = 76;      // ��� ���� 51, 40���� �ٲٵ簡..? �� ���̽� �ڵ�� ��ġ��Ű�� �� �� ���⵵
    public int height = 52;
    public float cellSize = 1.0f;

    public List<RackState> racks;       // ���� ���� ����Ʈ
    public Queue<Order> orderQueue = new Queue<Order>();        // �ֹ� ���� ť

    public Vector2Int receivingZonePos = new Vector2Int(3, 3);      // �޴� ����? �԰���
    public Vector2Int deliveryZonePos = new Vector2Int(17, 17);     // ��� ����? ���� �̰� �����

    public GameObject wallPrefab;   // ��������..? ��� �� ��� ��Դ°�
    public int[,] warehouseMap;     // �긦 2���� Grid�� ���� AMR�� ��ġ?�� ��� ���ΰ�. ���̽� �� np �����ϸ� �ɵ�

    public void Awake()
    {
        warehouseMap = new int[height, width];
        // ���⿡ ��, ���̺�, �� ��ġ �� �� ���� ä���
        InitializeWarehouseMapFromScene();  // �� �ڵ� �� ����
        PrintWarehouseMap(); // �� �ֿܼ� ���
    }

    public void InitializeWarehouseMapFromScene()
    {
        warehouseMap = new int[height, width];

        var tagToCode = new Dictionary<string, int> 
        {
            { "Wall", 1 },
            { "Rack", 2 },
            { "Reprocessing", 3 },
            { "Table", 4 },
            { "Charging", 5 }
        };

        foreach (var kvp in tagToCode)
        {
            string tag = kvp.Key;
            int code = kvp.Value;

            GameObject[] taggedObjects = GameObject.FindGameObjectsWithTag(tag);
            foreach (GameObject obj in taggedObjects)
            {
                Collider col = obj.GetComponent<Collider>();
                if (col == null)
                {
                    Debug.LogWarning($"No collider found on {obj.name}");
                    continue;
                }

                Bounds bounds = col.bounds;
                //Vector2Int minGrid = WorldToGrid(bounds.min);
                //Vector2Int maxGrid = WorldToGrid(bounds.max);

                int XGrid = (int)(bounds.max.x - bounds.min.x);
                int ZGrid = (int)(bounds.max.z - bounds.min.z);

                for (int x = (int)bounds.min.x; x <= XGrid; x++)
                {
                    for (int y = (int)bounds.min.z; y <= ZGrid; y++)
                    {
                        Vector2Int grid = new Vector2Int(x, y);
                        if (IsInGrid(grid))
                            warehouseMap[y, x] = code; // (y, x)
                    }
                }
            }
        }
    }

    public void PrintWarehouseMap()
    {
        string mapStr = "";
        for (int y = height - 1; y >= 0; y--) // ����� ���� ���� ���
        {
            for (int x = 0; x < width; x++)
            {
                mapStr += warehouseMap[y, x].ToString();
            }
            mapStr += "\n";
        }
        Debug.Log(mapStr);
    }


    public Vector2Int WorldToGrid(Vector3 worldPos)
    {
        Vector3 origin = transform.position;  // platform ����
        int x = Mathf.RoundToInt((worldPos.x - origin.x) / cellSize);
        int y = Mathf.RoundToInt((worldPos.z - origin.z) / cellSize);
        return new Vector2Int(x, y);
    }

    public bool IsInGrid(Vector2Int grid)
    {
        return grid.x >= 0 && grid.x < width && grid.y >= 0 && grid.y < height;
    }

    public void ResetEnvironment(AMRAgent agent)
    {
        // AGV ���� ��ġ
        Vector2Int start = new Vector2Int(0, 0);
        agent.transform.position = GridToWorld(start);
        agent.SetGridPos(start);

        foreach (var rack in racks)
        {
            rack.status = RackStatus.Filled;    // ������ ���� ���´� ��� �� ä���� ���·� �ʱ�ȭ
        }

        orderQueue.Clear();         // �ֹ� ���� ����
        orderQueue.Enqueue(new Order { requiredRackID = 0 });   // �ϳ� �߰�?
    }

    public Vector2Int GetTargetPosition(RackState carrying)     // AMR�� ����ϰ� �ִ� ���� �� ��ġ ���ϴ� �Լ�
    {
        if (carrying != null)
        {
            if (carrying.status == RackStatus.WaitingDelivery)  // ������� ���¸� ��� ��������
                return deliveryZonePos;
            else if (carrying.status == RackStatus.InTransit)   // ��� ���̸� �� �ڸ� ã��.
                return FindEmptyRackPosition(); 
        }

        if (orderQueue.Count > 0)
        {
            int targetID = orderQueue.Peek().requiredRackID;        // �ֹ����� �� ID
            foreach (var r in racks)
                if (r.rackID == targetID)       // ����Ʈ�� �ִ� r�� ID�� ������ �� ��ġ��.
                    return r.gridPos;
        }

        if (AnyEmptyRack())     // ��� ����
            return receivingZonePos;

        return new Vector2Int(-1, -1);
    }

    public bool TryPickUpRack(Vector2Int agvPos, ref RackState carrying)    // �� �Ⱦ��ϴ� �Լ�
    {
        foreach (var rack in racks)
        {
            if (rack.gridPos == agvPos)     // �� ��ġ�� amrPos�� ������
            {
                if (rack.status == RackStatus.WaitingDelivery || (rack.status == RackStatus.Filled && agvPos == receivingZonePos))
                {   // ���°� ����� ���̰ų� ��ǰ�� ���ִ� ���̰� AMR ��ġ�� �԰� ������ Ȯ��
                    carrying = rack;    // �� ���� ��� ���� ������ ����
                    if (rack.status == RackStatus.Filled) rack.status = RackStatus.InTransit;   // ��ǰ�� �� �ִ� ���̸� ���¸� ��� ������ ����
                    return true;
                }
            }
        }
        return false;
    }

    public DropResult TryDropRack(Vector2Int agvPos, ref RackState carrying)    // ���� �������� �Լ�
    {
        if (carrying == null) return DropResult.Invalid;    // ����ϴ� ���� ���� �� �Լ� ȣ���ϸ� �߸� �õ�

        if (agvPos == deliveryZonePos)  // AMR ��ġ�� ������̸� 
        {
            if (orderQueue.Count > 0 && carrying.rackID == orderQueue.Peek().requiredRackID)    // ���� ��� ���� ���� ID�� �ֹ� ������ ��ġ�ϴ� ID Ȯ��
            {
                orderQueue.Dequeue();                   // �ֹ� ���� ����
                carrying.status = RackStatus.Empty;     // ���.. �����Ŵϱ� ���� �������
                carrying = null;                        
                return DropResult.CorrectDelivery;      // ��� ����
            }
            else
            {
                carrying = null;                        // ��� �׳� �ֹ� ������ ���µ� ������
                return DropResult.WrongDelivery;        // �߸��� ���
            }
        }
        
        else
        {
            foreach (var rack in racks)
            {
                if (rack.gridPos == agvPos && rack.status == RackStatus.Empty && rack.rackID == carrying.rackID)
                {
                    rack.status = RackStatus.Filled;
                    carrying = null;
                    if (orderQueue.Count == 0) GenerateNewOrder();
                    return DropResult.CorrectReplenish;
                }
            }
        }

        return DropResult.Invalid;
    }

    public bool CheckCollision(Vector2Int pos)      // â����� ��ǥ Ȯ���ϸ鼭 �浹�ߴ���?
    {   // 0���� �۴ٰ� �ϸ� �Ʒ�ĭ���� ��������. �׷��ٰ� < -height ���� �ϸ� 
        if (pos.x < 0 || pos.x >= height || pos.y < 0 || pos.y >= width)
            return true;

        return warehouseMap[pos.x, pos.y] == 1;     // ���⼭ index ���� �� ��ī��
    }

    public Vector2Int FindEmptyRackPosition()
    {
        foreach (var r in racks)
            if (r.status == RackStatus.Empty)   // ������ ����ִ� ������ ã�Ƽ�
                return r.gridPos;               // �� ���� ��ġ�� ��ȯ

        return receivingZonePos;                // �ƴϸ� �� �԰����� ��ȯ? ��?
    }

    public bool AnyEmptyRack()                  // ����ִ� ������ Ȯ��
    {
        foreach (var r in racks)
            if (r.status == RackStatus.Empty)
                return true;

        return false;
    }

    public int OrderQueueCount() => orderQueue.Count;

    public int[,] GetLocalMap(Vector2Int agvPos)
    {
        int size = 7;
        int[,] local = new int[size, size];
        int half = size / 2;

        for (int dr = -half; dr <= half; dr++)
        {
            for (int dc = -half; dc <= half; dc++)
            {
                int r = agvPos.x + dr;
                int c = agvPos.y + dc;
                if (r >= 0 && r < height && c >= 0 && c < width)
                    local[dr + half, dc + half] = warehouseMap[r, c];
                else
                    local[dr + half, dc + half] = 1;
            }
        }
        return local;
    }

    public bool AllOrdersCompleted()
    {
        foreach (var r in racks)
        {
            if (r.status != RackStatus.Empty && r.status != RackStatus.Filled)
                return false;
        }

        return orderQueue.Count == 0;
    }

    public Vector3 GridToWorld(Vector2Int gridPos)
    {
        Vector3 worldOrigin = transform.position;   // NewPlatform ��ġ
        return new Vector3(worldOrigin.x + gridPos.x * cellSize, 0f, worldOrigin.z + gridPos.y * cellSize);
    }

    public void GenerateNewOrder()
    {
        foreach (var r in racks)
        {
            if (r.status == RackStatus.Filled)
            {
                r.status = RackStatus.WaitingDelivery;
                orderQueue.Enqueue(new Order { requiredRackID = r.rackID });
                break;
            }
        }
    }
}

[System.Serializable]
public class Order
{
    public int requiredRackID;
}
