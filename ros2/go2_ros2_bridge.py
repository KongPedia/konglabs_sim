import omni.graph.core as og
import omni.replicator.core as rep
import omni.kit.commands
import rclpy
import carb
import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from geometry_msgs.msg import Twist
import go2.go2_ctrl as go2_ctrl
from sensor_msgs.msg import JointState
from rclpy.time import Time

class RobotDataManager:
    def __init__(self, env, lidar_sensors_3d, lidar_sensors_2d, cameras, cfg, global_turrets=None):
        self.env = env
        self.num_envs = cfg.num_envs
        self.lidar_sensors_3d = lidar_sensors_3d
        self.lidar_sensors_2d = lidar_sensors_2d
        self.cameras = cameras
        self.step_size = 20
        self.global_turrets = global_turrets
        self.turret_targets = None
        self.turret_joint_pubs = []
        self.turret_joint_subs = []
        self.turret_joint_names = []
        self._turret_joint_name_to_idx = {}

        if self.global_turrets is not None:
            self.turret_targets = self.global_turrets.data.joint_pos.clone()
            self.turret_joint_names = self._get_turret_joint_names()
            self._turret_joint_name_to_idx = {
                name: idx for idx, name in enumerate(self.turret_joint_names)
            }

        # ROS 2 노드 초기화 (구독자용)
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("go2_robot_data_manager")

        # 1. 카메라 발행기 설정 (루프 처리)
        self._setup_camera_publishers()
        
        # 2. 라이다 발행기 설정 (루프 처리)
        self._setup_3d_lidar_publishers()

        self._setup_2d_lidar_publishers()
        
        self._setup_global_omnigraph()

        self._setup_odom_publishers()

        self._setup_cmd_vel_subscribers()
        
        self._setup_joint_state_publishers()

        self._setup_turret_pubs_subs()


    def _setup_joint_state_publishers(self):
            self.joint_pubs = []
            for i in range(self.num_envs):

                topic_name = f"env{i}/joint_states"
                # 퍼블리셔 생성 (메시지 타입: JointState, 큐 사이즈: 10)
                pub = self.node.create_publisher(JointState, topic_name, 10)
                self.joint_pubs.append(pub)
                print(f"[Bridge] Created JointState publisher: {topic_name}")

    def _get_turret_joint_names(self):
        if self.global_turrets is None:
            return []
        joint_names = getattr(self.global_turrets.data, "joint_names", None)
        if joint_names is None:
            joint_names = getattr(self.global_turrets, "joint_names", [])
        return list(joint_names)

    def _setup_turret_pubs_subs(self):
        """터렛용 joint_state 발행 및 joint_command 구독 설정"""
        if self.global_turrets is None:
            return

        num_turrets = int(
            getattr(self.global_turrets, "num_instances", self.global_turrets.data.joint_pos.shape[0])
        )

        for i in range(num_turrets):
            pub_topic = f"/turret{i}/joint_states"
            pub = self.node.create_publisher(JointState, pub_topic, 10)
            self.turret_joint_pubs.append(pub)

            sub_topic = f"/turret{i}/joint_command"
            sub = self.node.create_subscription(
                JointState,
                sub_topic,
                lambda msg, idx=i: self._turret_cmd_callback(msg, idx),
                10,
            )
            self.turret_joint_subs.append(sub)

        print(f"[Bridge] Created Publishers/Subscribers for {num_turrets} Turrets.")

    def _turret_cmd_callback(self, msg, idx):
        """수신된 turret 명령을 내부 텐서(turret_targets)에 반영"""
        if self.turret_targets is None:
            return
        if idx >= self.turret_targets.shape[0]:
            return

        if len(msg.name) > 0:
            for name, pos in zip(msg.name, msg.position):
                j_idx = self._turret_joint_name_to_idx.get(name)
                if j_idx is not None:
                    self.turret_targets[idx, j_idx] = float(pos)
            return

        max_len = min(len(msg.position), self.turret_targets.shape[1])
        for j_idx in range(max_len):
            self.turret_targets[idx, j_idx] = float(msg.position[j_idx])


    def _setup_cmd_vel_subscribers(self):
        """각 환경별 cmd_vel 토픽 구독 설정"""
        self._velocity_subs = []
        for i in range(self.num_envs):

            topic_name = f"robot{i}/cmd_vel"
            # 클로저를 사용하여 인덱스 i를 캡처
            sub = self.node.create_subscription(
                Twist,
                topic_name,
                lambda msg, idx=i: self._cmd_vel_callback(msg, idx),
                10
            )
            self._velocity_subs.append(sub)
            print(f"[Bridge] Subscribed to velocity commands on: {topic_name}")

    def _cmd_vel_callback(self, msg, idx):
        """수신된 Twist 메시지를 go2_ctrl의 전역 텐서에 주입"""
        if go2_ctrl.base_vel_cmd_input is not None:
            go2_ctrl.base_vel_cmd_input[idx, 0] = msg.linear.x
            go2_ctrl.base_vel_cmd_input[idx, 1] = msg.linear.y
            go2_ctrl.base_vel_cmd_input[idx, 2] = msg.angular.z




    def _setup_camera_publishers(self):
        """라이터 대신 OmniGraph의 ROS2 Camera Helper 노드를 직접 생성"""
        if self.cameras is not None:
            for i in range(self.num_envs):
                # Render Product 경로 가져오기
                render_product_path = self.cameras.render_product_paths[i]
                
                # 각 환경별 고유한 그래프 경로
                graph_path = f"/World/Graph/Camera/Camera_ROS_Graph_env_{i}"
                

                topic_name = f"robot{i}/camera/image_raw"
                info_topic = f"robot{i}/camera/camera_info"
                frame_id = f"front_camera"

                try:
                    # OmniGraph 구성
                    og.Controller.edit(
                        {"graph_path": graph_path, "evaluator_name": "push"},
                        {
                            og.Controller.Keys.CREATE_NODES: [
                                ("OnTick", "omni.graph.action.OnTick"),
                                ("Gate", "isaacsim.core.nodes.IsaacSimulationGate"), 
                                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                            ],
                            og.Controller.Keys.CONNECT: [
                                ("OnTick.outputs:tick", "Gate.inputs:execIn"),
                                ("Gate.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                                ("Gate.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                            ],
                            og.Controller.Keys.SET_VALUES: [
                                ("Gate.inputs:step", self.step_size),
                                ("cameraHelperRgb.inputs:renderProductPath", render_product_path),
                                ("cameraHelperRgb.inputs:frameId", frame_id),
                                ("cameraHelperRgb.inputs:topicName", topic_name),
                                ("cameraHelperRgb.inputs:type", "rgb"),
                                ("cameraHelperInfo.inputs:renderProductPath", render_product_path),
                                ("cameraHelperInfo.inputs:frameId", frame_id), 
                                ("cameraHelperInfo.inputs:topicName", info_topic),
                                ("cameraHelperInfo.inputs:useSystemTime", False), 
                            ],
                        },
                    )
                    print(f"[Bridge] RGB & Info Publisher created for Env {i} via OmniGraph")
                except Exception as e:
                    print(f"[Error] Failed to create Camera Helper for Env {i}: {e}")




    def _setup_3d_lidar_publishers(self):
        if self.lidar_sensors_3d is not None:
            for i, annotator in enumerate(self.lidar_sensors_3d):
                # 1. 렌더 제품 경로 가져오기 (문서의 renderProductPath 입력값)
                render_product_obj = rep.create.render_product(annotator.GetPath(), resolution=[1, 1], name="Isaac")
                render_product_path = render_product_obj.path
                # 2. 각 환경별 고유 그래프 경로
                graph_path = f"/World/Graph/Lidar/Lidar_3d_ROS_Graph_env_{i}"
                topic_name = f"env{i}/point_cloud2"
                frame_id = f"env{i}/lidar_link"

                # 3. OmniGraph 노드 생성 및 설정
                og.Controller.edit(
                    {"graph_path": graph_path, "evaluator_name": "push"},
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ("OnTick", "omni.graph.action.OnTick"),
                            ("Gate", "isaacsim.core.nodes.IsaacSimulationGate"),
                            ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                            ("LidarQoS", "isaacsim.ros2.bridge.ROS2QoSProfile"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ("OnTick.outputs:tick", "Gate.inputs:execIn"),
                            ("Gate.outputs:execOut", "LidarHelper.inputs:execIn"),
                            ("LidarQoS.outputs:qosProfile", "LidarHelper.inputs:qosProfile"),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ("Gate.inputs:step", self.step_size),
                            ("LidarHelper.inputs:renderProductPath", render_product_path),
                            ("LidarHelper.inputs:topicName", topic_name),
                            ("LidarHelper.inputs:frameId", frame_id),
                            ("LidarHelper.inputs:type", "point_cloud"),
                            ("LidarQoS.inputs:createProfile", "Sensor Data"), 
                            ("LidarHelper.inputs:fullScan", True), 
                            ("LidarHelper.inputs:resetSimulationTimeOnStop", True),
                        ],
                    },
                )
                print(f"[Bridge] LiDAR Helper Node (Env {i}) created successfully.")


    def _setup_2d_lidar_publishers(self):
        if self.lidar_sensors_2d is not None:
            for i, annotator in enumerate(self.lidar_sensors_2d):
                # 1. 렌더 제품 경로 가져오기 (문서의 renderProductPath 입력값)
                render_product_obj = rep.create.render_product(annotator.GetPath(), resolution=[1, 1], name="Isaac")
                render_product_path = render_product_obj.path
                # 2. 각 환경별 고유 그래프 경로
                graph_path = f"/World/Graph/Lidar/Lidar_2d_ROS_Graph_env_{i}"
                topic_name = f"env{i}/scan"
                frame_id = f"env{i}/scan_link"
                # 3. OmniGraph 노드 생성 및 설정
                og.Controller.edit(
                    {"graph_path": graph_path, "evaluator_name": "push"},
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ("OnTick", "omni.graph.action.OnTick"),
                            ("Gate", "isaacsim.core.nodes.IsaacSimulationGate"),
                            ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                            ("LidarQoS", "isaacsim.ros2.bridge.ROS2QoSProfile"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ("OnTick.outputs:tick", "Gate.inputs:execIn"),
                            ("Gate.outputs:execOut", "LidarHelper.inputs:execIn"),
                            ("LidarQoS.outputs:qosProfile", "LidarHelper.inputs:qosProfile"),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ("Gate.inputs:step", self.step_size),
                            ("LidarHelper.inputs:renderProductPath", render_product_path),
                            ("LidarHelper.inputs:topicName", topic_name),
                            ("LidarHelper.inputs:frameId", frame_id),
                            ("LidarHelper.inputs:type", "laser_scan"),
                            ("LidarQoS.inputs:createProfile", "Sensor Data"), 
                            ("LidarHelper.inputs:fullScan", True), 
                            ("LidarHelper.inputs:resetSimulationTimeOnStop", True),
                        ],
                    },
                )
                print(f"[Bridge] LiDAR Helper Node (Env {i}) created successfully.")


    def _setup_odom_publishers(self):
        """데이터 주입을 위한 ROS 2 Publish Odometry 노드만 생성"""
        for i in range(self.num_envs):
            graph_path = f"/World/Graph/Odom/Odom_ROS_Graph_env_{i}"
            

            topic_name = f"env{i}/odom"
            odom_frame_id = f"env{i}/odom"
            chassis_frame_id = f"env{i}/base_link" 

            try:
                og.Controller.edit(
                    {"graph_path": graph_path, "evaluator_name": "push"},
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            # OnTick은 이제 필요하지 않습니다. Python에서 직접 실행(execIn)을 때려줄 것이기 때문입니다.
                            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                        ],
                        og.Controller.Keys.SET_VALUES: [
                            ("PublishOdom.inputs:topicName", topic_name),
                            ("PublishOdom.inputs:odomFrameId", odom_frame_id),
                            ("PublishOdom.inputs:chassisFrameId", chassis_frame_id),
                        ],
                    },
                )
                print(f"[Bridge] Odometry Publisher Node created for Env {i}")
            except Exception as e:
                print(f"[Error] Failed to create Odometry Publisher for Env {i}: {e}")




    def _setup_global_omnigraph(self):
        """시뮬레이션 시간(/clock) 발행 (모든 환경 공통)"""
        try:
            og.Controller.edit(
                {"graph_path": "/World/Graph/Clock/Push_ROS2_Clock", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishClock.inputs:topicName", f"clock")
                    ],
                },
            )
            print("[Bridge] Clock Publisher Graph created successfully.")
        except Exception as e:
            print(f"[Error] Global OmniGraph setup error: {e}")

    # RobotDataManager 클래스 내부
    def update(self, character):
        """매 프레임마다 호출되어 ROS 2 데이터를 업데이트"""
        # ROS 2 콜백 처리 (비동기 메시지 수신)
        rclpy.spin_once(self.node, timeout_sec=0)

        # Isaac Lab Tensor 데이터 가져오기 (unitree_go2는 Scene에 등록된 이름)
        robot_data = self.env.unwrapped.scene["go2"].data

        # 시뮬레이션 시간 가져오기
        current_sim_time = self.env.unwrapped.sim.current_time
        ros_time_msg = Time(seconds = current_sim_time).to_msg()
        for i in range(self.num_envs):
            # 그래프 상의 PublishOdom 노드 경로
            odom_node_path = f"/World/Graph/Odom/Odom_ROS_Graph_env_{i}/PublishOdom"
            pos = robot_data.root_state_w[i, :3].tolist()
            # Isaac Lab(WXYZ) -> ROS 2(XYZW) 변환
            quat_wxyz = robot_data.root_state_w[i, 3:7]
            quat_xyzw = [quat_wxyz[1].item(), quat_wxyz[2].item(), quat_wxyz[3].item(), quat_wxyz[0].item()]
            
            # 2. 선속도 및 각속도 (Body Frame 기준)
            lin_vel = robot_data.root_lin_vel_b[i].tolist()
            ang_vel = robot_data.root_ang_vel_b[i].tolist()

            try:
                # OmniGraph 노드 속성에 직접 값 주입 (GPU -> ROS Bridge 데이터 흐름)
                og.Controller.set(og.Controller.attribute(f"{odom_node_path}.inputs:position"), pos)
                og.Controller.set(og.Controller.attribute(f"{odom_node_path}.inputs:orientation"), quat_xyzw)
                og.Controller.set(og.Controller.attribute(f"{odom_node_path}.inputs:linearVelocity"), lin_vel)
                og.Controller.set(og.Controller.attribute(f"{odom_node_path}.inputs:angularVelocity"), ang_vel)
                og.Controller.set(og.Controller.attribute(f"{odom_node_path}.inputs:timeStamp"), current_sim_time)
                
                # TF 업데이트가 필요한 경우 여기서 추가로 처리 가능
            except Exception as e:
                # 시뮬레이션 초기화 단계에서 노드가 아직 없을 때 에러 방지
                pass
            try:
                msg = JointState()
                
                # Header 채우기
                msg.header.stamp = ros_time_msg
                msg.header.frame_id = f"env{i}/base_link"
                
                # 데이터 채우기 (Tensor -> List 변환 필수!)
                msg.name = robot_data.joint_names 
                msg.position = robot_data._joint_pos.data[i].tolist()
                msg.velocity = robot_data._joint_vel.data[i].tolist()
                msg.effort = robot_data.applied_torque[i].tolist()

                # 발행 (Publish)
                self.joint_pubs[i].publish(msg)

            except Exception as e:
                pass

        if self.global_turrets is not None and len(self.turret_joint_pubs) > 0:
            turret_data = self.global_turrets.data
            if len(self.turret_joint_names) == 0:
                self.turret_joint_names = self._get_turret_joint_names()
            for i, pub in enumerate(self.turret_joint_pubs):
                if i >= turret_data.joint_pos.shape[0]:
                    break
                try:
                    msg = JointState()
                    msg.header.stamp = ros_time_msg
                    msg.header.frame_id = f"turret{i}_base_link"
                    msg.name = self.turret_joint_names
                    msg.position = turret_data.joint_pos[i].tolist()
                    msg.velocity = turret_data.joint_vel[i].tolist()
                    if hasattr(turret_data, "applied_torque"):
                        msg.effort = turret_data.applied_torque[i].tolist()
                    pub.publish(msg)
                except Exception:
                    pass


        if character is not None:
            vec3 = go2_ctrl.get_keyboard_cmd()
            # Tensor/Numpy 타입을 Python float으로 명시적 변환 (carb 호환성 확보)
            carb_vec3 = carb.Float3(float(vec3[0]), float(vec3[1]), 0.0)
            # print(f"Current action vector: {carb_vec3}")

            character.set_variable("move", carb_vec3)
            character.set_variable("forward", carb_vec3)



    def destroy_node(self):
        """시뮬레이션 종료 시 노드 정리"""
        if hasattr(self, 'node'):
            self.node.destroy_node()
