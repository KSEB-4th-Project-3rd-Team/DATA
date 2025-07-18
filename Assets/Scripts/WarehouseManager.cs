using System.Collections.Generic;
using UnityEngine;

public enum RackStatus { Empty = 0, Filled = 1, WaitingDelivery = 2, InTransit = 3 }
public enum DropResult { None, CorrectDelivery, CorrectReplenish, WrongDelivery, Invalid }
// 랙을 내려놨을 떄의 결과 : None, 출고 성공, 재고 보충 성공, 잘못 출고, 잘못 시도

public class RackState : MonoBehaviour
{
    public int rackID;
    public Vector2Int gridPos;
    public RackStatus status;   // 랙의 상태를 나타내는 enum : Empty 0번, Filled 1번, 출고대기 2번, 운송 중 3번
}

public class WarehouseManager : MonoBehaviour
{
    public int maxStepPerEpisode = 500; // 한 에피소드 당 최대 스텝 수

    public int width = 76;      // 얘네 둘은 51, 40으로 바꾸든가..? 그 파이썬 코드랑 일치시키면 될 것 같기도
    public int height = 52;
    public float cellSize = 1.0f;

    public List<RackState> racks;       // 랙들 담을 리스트
    public Queue<Order> orderQueue = new Queue<Order>();        // 주문 담을 큐

    public Vector2Int receivingZonePos = new Vector2Int(3, 3);      // 받는 구역? 입고존
    public Vector2Int deliveryZonePos = new Vector2Int(17, 17);     // 배달 구역? 뭐고 이건 출고존

    public GameObject wallPrefab;   // 벽프리팹..? 얘는 뭐 어따 써먹는겨
    public int[,] warehouseMap;     // 얘를 2차원 Grid로 랙과 AMR의 위치?를 담는 곳인가. 파이썬 그 np 생각하면 될듯

    public void Awake()
    {
        warehouseMap = new int[height, width];
        // 여기에 벽, 테이블, 랙 위치 등 맵 정보 채우기
        InitializeWarehouseMapFromScene();  // ← 자동 맵 구성
        PrintWarehouseMap(); // ← 콘솔에 출력
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
        for (int y = height - 1; y >= 0; y--) // 상단이 위로 오게 출력
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
        Vector3 origin = transform.position;  // platform 기준
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
        // AGV 시작 위치
        Vector2Int start = new Vector2Int(0, 0);
        agent.transform.position = GridToWorld(start);
        agent.SetGridPos(start);

        foreach (var rack in racks)
        {
            rack.status = RackStatus.Filled;    // 랙들의 기존 상태는 모두 다 채워진 상태로 초기화
        }

        orderQueue.Clear();         // 주문 내역 비우고
        orderQueue.Enqueue(new Order { requiredRackID = 0 });   // 하나 추가?
    }

    public Vector2Int GetTargetPosition(RackState carrying)     // AMR이 운반하고 있는 랙을 둘 위치 정하는 함수
    {
        if (carrying != null)
        {
            if (carrying.status == RackStatus.WaitingDelivery)  // 출고대기중 상태면 배달 구역으로
                return deliveryZonePos;
            else if (carrying.status == RackStatus.InTransit)   // 운송 중이면 빈 자리 찾기.
                return FindEmptyRackPosition(); 
        }

        if (orderQueue.Count > 0)
        {
            int targetID = orderQueue.Peek().requiredRackID;        // 주문내역 랙 ID
            foreach (var r in racks)
                if (r.rackID == targetID)       // 리스트에 있는 r의 ID가 같으면 그 위치로.
                    return r.gridPos;
        }

        if (AnyEmptyRack())     // 얘는 뭐냐
            return receivingZonePos;

        return new Vector2Int(-1, -1);
    }

    public bool TryPickUpRack(Vector2Int agvPos, ref RackState carrying)    // 랙 픽업하는 함수
    {
        foreach (var rack in racks)
        {
            if (rack.gridPos == agvPos)     // 랙 위치와 amrPos와 같으면
            {
                if (rack.status == RackStatus.WaitingDelivery || (rack.status == RackStatus.Filled && agvPos == receivingZonePos))
                {   // 상태가 출고대기 중이거나 물품이 차있는 랙이고 AMR 위치가 입고 존인지 확인
                    carrying = rack;    // 그 랙을 운반 중인 랙으로 설정
                    if (rack.status == RackStatus.Filled) rack.status = RackStatus.InTransit;   // 물품이 차 있는 랙이면 상태를 운송 중으로 변경
                    return true;
                }
            }
        }
        return false;
    }

    public DropResult TryDropRack(Vector2Int agvPos, ref RackState carrying)    // 랙을 내려놓는 함수
    {
        if (carrying == null) return DropResult.Invalid;    // 운반하는 랙이 없을 때 함수 호출하면 잘못 시도

        if (agvPos == deliveryZonePos)  // AMR 위치가 출고존이면 
        {
            if (orderQueue.Count > 0 && carrying.rackID == orderQueue.Peek().requiredRackID)    // 현재 운반 중인 랙의 ID와 주문 내역과 일치하는 ID 확인
            {
                orderQueue.Dequeue();                   // 주문 내역 제거
                carrying.status = RackStatus.Empty;     // 출고.. 나간거니깐 랙은 비어있음
                carrying = null;                        
                return DropResult.CorrectDelivery;      // 출고 성공
            }
            else
            {
                carrying = null;                        // 얘는 그냥 주문 내역에 없는데 나간거
                return DropResult.WrongDelivery;        // 잘못된 출고
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

    public bool CheckCollision(Vector2Int pos)      // 창고맵의 좌표 확인하면서 충돌했는지?
    {   // 0보다 작다고 하면 아래칸으로 못내려감. 그렇다고 < -height 으로 하면 
        if (pos.x < 0 || pos.x >= height || pos.y < 0 || pos.y >= width)
            return true;

        return warehouseMap[pos.x, pos.y] == 1;     // 여기서 index 에러 남 어카노
    }

    public Vector2Int FindEmptyRackPosition()
    {
        foreach (var r in racks)
            if (r.status == RackStatus.Empty)   // 랙에서 비어있는 랙들을 찾아서
                return r.gridPos;               // 그 랙의 위치를 반환

        return receivingZonePos;                // 아니면 걍 입고존을 반환? 왜?
    }

    public bool AnyEmptyRack()                  // 비어있는 랙인지 확인
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
        Vector3 worldOrigin = transform.position;   // NewPlatform 위치
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
