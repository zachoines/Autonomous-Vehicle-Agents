using UnityEngine;

public class EnvController : MonoBehaviour {


    [SerializeField]
    [Header("Goal Spawn Radias", order = 999)]
    private float spawnRadius = 2f;

    [SerializeField]
    [Header("Arena", order = 999)]
    public GameObject arena;

    [SerializeField]
    [Header("Arena bounds", order = 999)]
    public Bounds arenaBounds;


    void Start()
    {
        arenaBounds = arena.GetComponent<Collider>().bounds;
    }
    
    public Vector3 GenerateNewSpawn()
    {
        var foundNewSpawnLocation = false;
        var newSpawnPos = Vector3.zero;
        while (foundNewSpawnLocation == false)
        {
            float randomPosX = Random.Range(-arenaBounds.extents.x * spawnRadius,
                arenaBounds.extents.x * spawnRadius);

            float randomPosZ = Random.Range(-arenaBounds.extents.z * spawnRadius,
                arenaBounds.extents.z * spawnRadius);
            newSpawnPos = arena.transform.position + new Vector3(randomPosX, 1f, randomPosZ);
            if (Physics.CheckBox(newSpawnPos, new Vector3(1.5f, 0.01f, 1.5f)) == false)
            {
                foundNewSpawnLocation = true;
            }
        }
       
        return newSpawnPos;
    }
}
    
    