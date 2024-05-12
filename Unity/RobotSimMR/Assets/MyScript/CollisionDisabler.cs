using UnityEngine;

public class CollisionDisabler : MonoBehaviour
{
    [SerializeField] GameObject _parent;
    [SerializeField] GameObject _omni1;
    [SerializeField] GameObject _omni2;
    [SerializeField] GameObject _omni3;
    [SerializeField] GameObject _omni4;

    void Start()
    {
        SetActiveCollision(false, _parent, _omni1);
        SetActiveCollision(false, _parent, _omni2);
        SetActiveCollision(false, _parent, _omni3);
        SetActiveCollision(false, _parent, _omni4);
    }
    //�Փ˂��邩�ǂ����A�ΏۃI�u�W�F�N�g1�A�ΏۃI�u�W�F�N�g2���w��
    public static void SetActiveCollision(bool isCollide, GameObject targetObj1, GameObject targetObj2)
    {
        var colliders1 = targetObj1.GetComponentsInChildren<Collider>();
        var colliders2 = targetObj2.GetComponentsInChildren<Collider>();

        foreach (var col1 in colliders1)
        {
            foreach (var col2 in colliders2)
            {
                Physics.IgnoreCollision(col1, col2, !isCollide);
            }
        }
    }
}