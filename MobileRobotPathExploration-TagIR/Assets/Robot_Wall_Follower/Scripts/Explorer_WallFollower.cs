using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.UrdfImporter.Control;


public class Explorer_WallFollower : MonoBehaviour
{
	public int Id;

	public GameObject wheel1;
	public GameObject wheel2;

	private ArticulationBody wA1;
	private ArticulationBody wA2;

	public float maxLinearSpeed = 2; //  m/s
	public float maxRotationalSpeed = 1;//
	public float wheelRadius = 0.033f; //meters
	public float trackWidth = 0.288f; // meters Distance between tyres
	public float forceLimit = 10;
	public float damping = 10;

	public float ROSTimeout = 0.5f;
	private float lastCmdReceived = 0f;

	ROSConnection ros;
	private RotationDirection direction;
	private float rosLinear = 0f;
	private float rosAngular = 0f;

	const float k_Nav2InitializeTime = 5.0f;
	bool m_RosConnected = false;
	Transform robot;
	const string k_RobotBaseName = "base_footprint/base_link";

	public string ScanTopic = "/scan";

	float[] realRanges = new float[0];
	float[] realAngles = new float[0];
	float[] distances = new float[0];

	public GameObject rangeVisualizationCubePrefab;
	private List<GameObject> rangeVisualizationCubes = new List<GameObject>();
	private List<MeshRenderer> rangeVisualizationRenderers = new List<MeshRenderer>();
	public GameObject goalVisualizationCapsulePrefab;
	private Transform goalVisualizer = null;

	public float avgCellWidth = 1.4f;
	public float avgWallWidth = 0.4f;
	private float avgDistCellCenterToCellCenter;
	private float avgThreeQuarterCellWidth;
	private float avgHalfCellWidth;
	private float avgQuarterCellWidth;

	private bool isTurning = false;
	private bool isStuck = false;

	public const int numCardinalDirections = 4;
	// Starting at East because atan2 returns 0 for East. Also, rotates left because Lidar rotates left.
	public enum Direction {East = 0, North = 1, West = 2, South = 3}
	public Vector3[] CardinalDirectionVectors = new Vector3[] {Vector3.right, Vector3.forward, Vector3.left, Vector3.back};
	public enum LocalDirection {Forward = 0, Left = 1, Back = 2, Right = 3}
	// Make sure to update this public var if you change the starting rotation of the actual robot in the scene view
	public Direction currentPrimaryDirection = Direction.North;

	private enum NodeState {New = 0, VisitedByOther = 1, VisitedByMe = 2, DeadEnd = 3}
	private Color[] NodeStateColors = new Color[] {Color.white, Color.red, Color.blue, Color.black};
	private Vector2Int[] NodeDirectionVectors = new Vector2Int[] {Vector2Int.right, Vector2Int.up, Vector2Int.left, Vector2Int.down};
	public int numNodesPerSide = 10;
	private NodeState[,] nodes; // = new NodeState[numNodesPerSide,numNodesPerSide];
	public Vector2Int currentNode = Vector2Int.zero;
	public Vector2Int nextNode = Vector2Int.zero;
	//  For a 10x10 maze, (0,0) is the bottom left corner, (9,9) is the top right, coords are (column, row)
	public Vector3 startCoord;
	public Vector3 cornerOffset;
	public MeshRenderer[,] nodeStateVisualizers; // = new MeshRenderer[numNodesPerSide,numNodesPerSide];
	public bool CreateNodeStateVisualizers = false;
	public Explorer_Mover NodeStateVisualizerSource;

	public LaserScanSensor sensor;

	public Vector3 goalPos;

	public float StartDelay = 0;

	// True for left wall follower, False for right wall follower
	public bool LeftWallFollower;

	// Awake is called at the start of the scene
	private void Awake() {
		nodes = new NodeState[numNodesPerSide,numNodesPerSide];
		nodeStateVisualizers = new MeshRenderer[numNodesPerSide,numNodesPerSide];
		
		avgDistCellCenterToCellCenter = avgCellWidth + avgWallWidth;
		avgThreeQuarterCellWidth = avgCellWidth * 0.75f;
		avgHalfCellWidth = avgCellWidth * 0.5f;
		avgQuarterCellWidth = avgCellWidth * 0.25f;

		for(int i = 0; i < numNodesPerSide; i++) {
			for(int j = 0; j < numNodesPerSide; j++) {
				nodes[i,j] = NodeState.New;
				if(CreateNodeStateVisualizers) {
					nodeStateVisualizers[i,j] = Instantiate(goalVisualizationCapsulePrefab, new Vector3(startCoord.x + i * avgDistCellCenterToCellCenter, 0, startCoord.z + j * avgDistCellCenterToCellCenter), Quaternion.identity).GetComponent<MeshRenderer>();
					nodeStateVisualizers[i,j].material.color = NodeStateColors[(int) nodes[i,j]];
				}
			}
		}
		nodes[currentNode.x,currentNode.y] = NodeState.VisitedByMe;
		if(CreateNodeStateVisualizers) {
			nodeStateVisualizers[currentNode.x,currentNode.y].material.color = NodeStateColors[(int) NodeState.VisitedByMe];
		}
		goalPos = startCoord + cornerOffset * avgQuarterCellWidth;
		Debug.Log("GOAL POS: " + goalPos);
	}

	// Start is called before the first frame update but after Awake
	void Start()
	{
		wA1 = wheel1.GetComponent<ArticulationBody>();
		wA2 = wheel2.GetComponent<ArticulationBody>();
		SetParameters(wA1);
		SetParameters(wA2);

		if(!CreateNodeStateVisualizers) {
			nodeStateVisualizers = NodeStateVisualizerSource.nodeStateVisualizers;
		}

		// Start the Control Function
		robot = transform.Find(k_RobotBaseName);
		StartCoroutine(ControlFunction());
	}

	void FixedUpdate()
	{
		ROSUpdate();
	}

	void ScanCallback(RosMessageTypes.Sensor.LaserScanMsg msg) {
		realRanges = msg.ranges;
		realAngles = sensor.angles.ToArray();
	}


	IEnumerator ControlFunction(){
		var ros = ROSConnection.GetOrCreateInstance();
		ros.ConnectOnStart = true;
		
		yield return new WaitForSeconds(k_Nav2InitializeTime);
		
		ros.Subscribe<RosMessageTypes.Sensor.LaserScanMsg>(ScanTopic, ScanCallback);
		
		m_RosConnected = true;

		yield return new WaitForSeconds(StartDelay);

		while(m_RosConnected) {
			int numRanges = realRanges.Length;

			if(numRanges > 0) {
				if(goalVisualizer == null) {
					goalVisualizer = Instantiate(goalVisualizationCapsulePrefab).transform;
					goalVisualizer.position = goalPos;
				}

				while(rangeVisualizationCubes.Count < numRanges) {
					GameObject cube = Instantiate(rangeVisualizationCubePrefab);
					rangeVisualizationCubes.Add(cube);
					rangeVisualizationRenderers.Add(cube.GetComponent<MeshRenderer>());
				}

				int[] cardinalDirectionIndices = new int[numCardinalDirections];
				float[] cardinalDirectionRanges = new float[numCardinalDirections];
				Vector3[] cardinalDirectionHits = new Vector3[numCardinalDirections];
				float rotAngleStep = (-360f / (float) numRanges);
				float closestDistance = 99999;
				for(int i = 0; i < numRanges; i++) {
					if(realRanges[i] < closestDistance) {
						closestDistance = realRanges[i];
					}

					float LaserAngle = rotAngleStep * (float) i;
					Vector3 hitPosition = robot.position + (Quaternion.Euler(0, LaserAngle, 0) * robot.forward * realRanges[i]);

					// Visualizer Cubes
					rangeVisualizationCubes[i].SetActive(true);
					rangeVisualizationCubes[i].transform.position = hitPosition;
					rangeVisualizationRenderers[i].material.color = Color.HSVToRGB(Mathf.Clamp01(realRanges[i] / 50f), 1, 1);
					
					// Find the closest reading to the world coordinate directions
					float worldAngle = Mathf.Atan2(hitPosition.z - robot.position.z, hitPosition.x - robot.position.x) * Mathf.Rad2Deg;
					// Debug.Log("World Angle: " + worldAngle);
					float worldMod90Angle = Modulus(worldAngle, 90);
					// Debug.Log("World Angle mod 90: " + worldMod90Angle);
					if(worldMod90Angle < 5 || worldMod90Angle > 85) {
						// Debug.Log("World Angle: Effectively a 90 multiple");
						for(int j = 0; j < numCardinalDirections; j++) {
							if(Mathf.Abs((90 * j) - Modulus(worldAngle, 360)) < 5 ) {
								// Debug.Log("World Angle: It was " + (Direction) j);
								cardinalDirectionIndices[j] = i;
								cardinalDirectionRanges[j] = realRanges[i];
								cardinalDirectionHits[j] = hitPosition;
							}
						}
					}
				}
				float robotWorldForwardAngleRad = Mathf.Atan2(robot.forward.z, robot.forward.x);

				// Keep this. Deactivates any extra visualization cubes.
				for(int i = numRanges; i < rangeVisualizationCubes.Count; i++) {
					rangeVisualizationCubes[i].SetActive(false);
				}

				if(isTurning) { // The robot is currently turning.
					if(isStuck) {
						if(closestDistance < 0.25f) { // Too close to a wall! (Assumption that it's a wall in front)
							rosLinear = -0.5f;
							rosAngular = 0f;
						} else {
							isStuck = false;
						}
					} else {
						// Debug.Log("TURNING");
						rosLinear = 0f;
						float goalAngleRad = Mathf.Atan2(goalPos.z - robot.position.z, goalPos.x - robot.position.x);
						float rotAngleRad = goalAngleRad - robotWorldForwardAngleRad;
						if(rotAngleRad > Mathf.PI) {
							rotAngleRad -= 2 * Mathf.PI;
						} else {
							if(rotAngleRad < -Mathf.PI) {
								rotAngleRad += 2 * Mathf.PI;
							}
						}
						rosAngular = 1.5f * rotAngleRad;
						// Debug.Log("TURNING Speed: " + rosAngular);

						if(Mathf.Abs(rotAngleRad) < 0.15f) {
							isTurning = false;
							rosAngular = 0f;
						}
					}
				} else { // Assume that the robot is mostly facing "forward" and is mostly aligned with the walls
					// Are we at the "center" of a node? If so, determine where to go next.
					float distToGoal = Vector3.Distance(robot.position, goalPos);
					if(distToGoal < 0.1f) {
						Debug.Log("PICKING GOAL");
						rosLinear = 0f;
						rosAngular = 0f;

						// Loop through the four main directions and look for an open path
						currentPrimaryDirection = (Direction) (((int)currentPrimaryDirection + (int) (LeftWallFollower ? LocalDirection.Left : LocalDirection.Right)) % numCardinalDirections);
						for(int i = 0; i < numCardinalDirections; i++) {
							if(cardinalDirectionRanges[(int) currentPrimaryDirection] < avgCellWidth) { // There's a wall
								currentPrimaryDirection = (Direction) (((int)currentPrimaryDirection + (int) (LeftWallFollower ? LocalDirection.Right : LocalDirection.Left)) % numCardinalDirections);
							} else { // It's open
								break;
							}
						}

						// Move in the chosen direction
						goalPos = goalPos + CardinalDirectionVectors[(int) currentPrimaryDirection] * avgDistCellCenterToCellCenter;
						goalVisualizer.position = goalPos;
						Debug.Log("GOAL POS: " + goalPos);

						isTurning = true;
					} else {
						// Debug.Log("MOVING FORWARD");
						if(distToGoal < 0.3f) {
							rosLinear = 0.1f;
						} else {
							rosLinear = 0.5f;
						}

						float goalAngleRad = Mathf.Atan2(goalPos.z - robot.position.z, goalPos.x - robot.position.x);
						float rotAngleRad = goalAngleRad - robotWorldForwardAngleRad;
						if(rotAngleRad > Mathf.PI) {
							rotAngleRad -= 2 * Mathf.PI;
						} else {
							if(rotAngleRad < -Mathf.PI) {
								rotAngleRad += 2 * Mathf.PI;
							}
						}

						if(Mathf.Abs(rotAngleRad) > Mathf.PI / 4) { // Robot is currently facing more than 45* from target
							isTurning = true;
							isStuck = true;
						} else {
							rosAngular = 0.8f * rotAngleRad; // Robot is at least mostly facing towards target
						}
					}
				}


				lastCmdReceived = Time.time;
			}
			yield return null;
		}
		
		yield return null;
	}

	private void SetParameters(ArticulationBody joint)
	{
		ArticulationDrive drive = joint.xDrive;
		drive.forceLimit = forceLimit;
		drive.damping = damping;
		joint.xDrive = drive;
	}

	private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
	{
		ArticulationDrive drive = joint.xDrive;
		if (float.IsNaN(wheelSpeed))
		{
			drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
		}
		else
		{
			drive.targetVelocity = wheelSpeed;
		}
		joint.xDrive = drive;
	}

	private void ROSUpdate()
	{
		if (Time.time - lastCmdReceived > ROSTimeout)
		{
			rosLinear = 0f;
			rosAngular = 0f;
		}
		RobotInput(rosLinear, -rosAngular);
	} 

	private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
	{
		if (speed > maxLinearSpeed)
		{
			speed = maxLinearSpeed;
		}
		if (rotSpeed > maxRotationalSpeed)
		{
			rotSpeed = maxRotationalSpeed;
		}
		float wheel1Rotation = (speed / wheelRadius);
		float wheel2Rotation = wheel1Rotation;
		float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
		if (rotSpeed != 0)
		{
			wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
			wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
		}
		else
		{
			wheel1Rotation *= Mathf.Rad2Deg;
			wheel2Rotation *= Mathf.Rad2Deg;
		}
		SetSpeed(wA1, wheel1Rotation);
		SetSpeed(wA2, wheel2Rotation);
	}

	// C# Modulus (%) isn't a true modulus. Use this instead.
	public float Modulus(float a, float b) {
		return ((a % b) + b) % b;
	}
}