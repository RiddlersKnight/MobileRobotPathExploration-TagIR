using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Light_Tag : MonoBehaviour
{
    Vector3 noAngle;
    Quaternion frontAngle;
    Quaternion leftAngle_30;
    Quaternion rightAngle_30;
    Quaternion leftAngle_45;
    Quaternion rightAngle_45;
    Quaternion leftAngle_60;
    Quaternion rightAngle_60;
    Vector3 Left_30degVector;
    Vector3 Right_30degVector;
    Vector3 Left_45degVector;
    Vector3 Right_45degVector;
    Vector3 Left_60degVector;
    Vector3 Right_60degVector;
    Vector3 Front_Vector;

    float rayLength =5;
    public int TagID;
    
    public Light_Tag NextTag;
    public List<Transform> NextWaypoints;

    public const int numCardinalDirections = 4;
    // Starting at East because atan2 returns 0 for East. Also, rotates left because Lidar rotates left.
    public enum Direction {East = 0, North = 1, West = 2, South = 3}
    public enum NodeState {New = 0, VisitedByOther = 1, VisitedByMe = 2, DeadEnd = 3}
    public NodeState[] dirStates = new NodeState[numCardinalDirections];
    public int[] dirVisitor = new int[numCardinalDirections];
    
    // Awake is called at the start of the scene
    private void Awake()
    {
      noAngle = this.transform.forward;
      leftAngle_30 = Quaternion.AngleAxis(-30,new Vector3(0f,2f,0f));
      rightAngle_30 = Quaternion.AngleAxis(30,new Vector3(0f,2f,0f));
      leftAngle_45 = Quaternion.AngleAxis(-45,new Vector3(0f,2f,0f));
      rightAngle_45 = Quaternion.AngleAxis(45,new Vector3(0f,2f,0f));
      leftAngle_60 = Quaternion.AngleAxis(-60,new Vector3(0f,2f,0f));
      rightAngle_60 = Quaternion.AngleAxis(60,new Vector3(0f,2f,0f));
      
      Front_Vector = frontAngle*noAngle;
      Left_30degVector = leftAngle_30*noAngle;
      Right_30degVector = rightAngle_30*noAngle;
      Left_45degVector = leftAngle_45*noAngle;
      Right_45degVector = rightAngle_45*noAngle;
      Left_60degVector = leftAngle_60*noAngle;
      Right_60degVector = rightAngle_60*noAngle;
      
      for(int i = 0; i < numCardinalDirections; i++) {
        dirStates[i] = NodeState.New;
        dirVisitor[i] = -1;
      }
    }

    // Update is called once per frame
    void Update()
    {
        Debug.DrawRay(this.transform.position, Front_Vector*rayLength, Color.black);
        Debug.DrawRay(this.transform.position, Left_30degVector*rayLength, Color.red);
        Debug.DrawRay(this.transform.position, Right_30degVector*rayLength, Color.red);
        Debug.DrawRay(this.transform.position, Left_45degVector*rayLength, Color.green);
        Debug.DrawRay(this.transform.position, Right_45degVector*rayLength, Color.green);
        Debug.DrawRay(this.transform.position, Left_60degVector*rayLength, Color.blue);
        Debug.DrawRay(this.transform.position, Right_60degVector*rayLength, Color.blue);
    }
}
