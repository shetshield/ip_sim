import omni.graph.core as og
from omni.isaac.core.articulations import Articulation, ArticulationSubset
import numpy as np
import omni.kit.commands
import omni.usd
import time
from pxr import Gf, Usd, UsdGeom
import inspect

try:
    from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
    LULA_IK_AVAILABLE = True
except ImportError:
    try:
        from omni.isaac.motion_generation.lula.ik_solver import LulaKinematicsSolver
        ArticulationKinematicsSolver = None
        LULA_IK_AVAILABLE = True
    except ImportError:
        LulaKinematicsSolver = None
        ArticulationKinematicsSolver = None
        LULA_IK_AVAILABLE = False

# [Interface Import]
try:
    from isaacsim.robot.surface_gripper._surface_gripper import acquire_surface_gripper_interface
    sg_interface = acquire_surface_gripper_interface()
    IS_INTERFACE_AVAILABLE = True
except ImportError:
    IS_INTERFACE_AVAILABLE = False
    sg_interface = None

class DualRobotController:
    def __init__(self):
        # ---------------------------------------------------------
        self.world_root = "/World/ip_model/ip_model"

        # Robot Paths
        self.root_1 = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/tn__AR070PRN150010NNV3EL211_jONg6No0QW3AEIKNlHm2/tn__AR070PRN150010NNV3EL_03_VY0AEIKN"
        self.root_2 = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/tn__MY1C25400L6LZ7311_XIHq4d0W2DfBh1"
        self.table_root = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/tn__MY1C25400L6LZ7311_XIHq4d0W2DfBh1/tn___MY1C250_TABLE_nIP/tn___MY1C250_TABLE_nIP"
        self.rotation_root = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A101_tCX59b7o0/Rotation_Part"
        self.assembly_root = (
            self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A3011_uDDl3V0l19d9V1/Lid_Assem_Table"
        )
        self.assembly_rotation_root = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A3011_uDDl3V0l19d9V1/rot"
        self.lid_assy_damping_attr_path = (
            "/World/ip_model/ip_model/tn__NT251101A001_tCX59b7o0/tn__NT251101A3011_uDDl3V0l19d9V1/"
            "Lid_Assem_Table/Moving/Lid_Assy_p_joint.drive:linear:physics:damping"
        )

        # [Surface Gripper Prim Path]
        self.sg_prim_path = \
            "/World/SurfaceGripper"

        # 별도로 동작하는 Surface Gripper Prim Path
        self.mold_sg_prim_path = "/World/mold_sg"
        self.mold_sg2_prim_path = "/World/mold_sg2"

        self.unlock_target_path = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/moldcap"

        self.cylinder_prim_path = (
            "/World/ip_model/ip_model/tn__NT251101A001_tCX59b7o0/"
            "tn__NT251101A101_tCX59b7o0/tn__moldA181_k88X2Lu0a6i0/mold/mold/Cylinder_01"
        )

        self.assembly_gripper_cylinder_path = (
            "/World/ip_model/ip_model/tn__NT251101A001_tCX59b7o0/"
            "tn__NT251101A3011_uDDl3V0l19d9V1/gripper/Cylinder"
        )

        # [M1013 Robot]
        self.m1013_root_prim = "/World/ip_model/m1013/root_joint"
        self.m1013_config_path = "/Users/shets/Downloads/software/isaac_sim/model/m1013_lula.yaml"
        self.m1013_urdf_path = "/Users/shets/Downloads/software/isaac_sim/model/m1013.urdf"
        self.m1013_default_revolute_deg = np.array([-90.0, 5.0, -145.0, 0.0, -40.0, 0.0])
        self.m1013_default_prismatic = np.zeros(4, dtype=float)
        self.m1013_end_effector_frame = "eef"
        self.m1013_robot = Articulation(self.m1013_root_prim)

        self.final_eef_target = np.array([-736.948, -11.0, -669.432]) / 1000.0
        self.eef_path_steps = 20
        self.eef_default_orientation = None
        self.eef_waypoints = []
        self.eef_motion_started = False
        self.eef_motion_finished = False

        self.cone_prim_path = (
            "/World/ip_model/ip_model/tn__NT251101A001_tCX59b7o0/tn__HA980DW1_l8d3o4Z0/"
            "tn__HA980DWHOPPER1_xEt58c8u0/Cone/pd"
        )
        
        # Settings
        self.joint_1 = "Lid_Lifter_p_joint"; self.joint_2 = "UpDn_p_joint"
        self.vel_1 = -0.1; self.vel_2 = -0.3
        self.threshold_1 = -0.022 - 0.02*2.6; self.threshold_2 = -0.042
        self.hold_duration = 0.5
        self.up_vel_2 = 0.3
        self.position_tolerance = 1e-3
        self.table_joint = "Body_Table_p_joint"
        self.table_up_vel = 0.5
        self.table_stop_threshold = 0.3799
        self.rotation_joint = "idx1_r_joint"
        self.rotation_target = np.deg2rad(-45.0)
        self.rotation_final_target = self.rotation_target + np.deg2rad(-90.0)
        self.rotation_tolerance = np.deg2rad(0.01)
        self.rotation_kp = 6.0
        self.rotation_max_speed = 1.0
        self.lid_assy_joint = "Lid_Assy_p_joint"
        self.lid_assy_speed = 0.1
        self.lid_assy_kp = 5.0
        self.lid_assy_first_stop = 0.057
        self.lid_assy_second_stop = 0.07
        self.assembly_move_timeout = 2.0
        self.assembly_rotation_joint = "Assy_Assem_r_joint"
        self.assembly_rotation_speed = -15.0

        self.robot_1 = Articulation(self.root_1)
        self.robot_2 = Articulation(self.root_2)
        self.table = Articulation(self.table_root)
        self.rotation = Articulation(self.rotation_root)
        self.lid_assy = Articulation(self.assembly_root)
        self.assembly_rotation = Articulation(self.assembly_rotation_root)
        self.subset_1 = ArticulationSubset(self.robot_1, [self.joint_1])
        self.subset_2 = ArticulationSubset(self.robot_2, [self.joint_2])
        self.table_subset = ArticulationSubset(self.table, [self.table_joint])
        self.rotation_subset = ArticulationSubset(self.rotation, [self.rotation_joint])
        self.lid_assy_subset = ArticulationSubset(self.lid_assy, [self.lid_assy_joint])
        self.assembly_rotation_subset = ArticulationSubset(self.assembly_rotation, [self.assembly_rotation_joint])
        self.stage = omni.usd.get_context().get_stage()

        self._apply_m1013_default_configuration()

        self.reset_logic()
        print("Controller Ready: Wait-Verify-Move Logic")

    def _apply_m1013_default_configuration(self):
        """Teleport the M1013 robot to its default configuration at sim start."""
        prim = self.stage.GetPrimAtPath(self.m1013_root_prim)
        if not prim.IsValid():
            return

        articulation = Articulation(self.m1013_root_prim)
        default_positions = np.concatenate([
            np.deg2rad(self.m1013_default_revolute_deg),
            self.m1013_default_prismatic,
        ])
        articulation.set_joint_positions(default_positions)
        print(f"M1013 default configuration applied to {self.m1013_root_prim}.")

    def _ensure_m1013_ik_solver(self):
        if not LULA_IK_AVAILABLE:
            if not getattr(self, "m1013_ik_unavailable_notified", False):
                print(
                    "[M1013 IK] LulaKinematicsSolver is not available. Skipping end-effector motion.")
                self.m1013_ik_unavailable_notified = True

            # Skip the motion rather than raising a TypeError when the solver is missing.
            self.eef_motion_finished = True
            return False

        if self.m1013_ik_solver is None:
            self.m1013_ik_solver = self._create_m1013_ik_solver()

            if self.m1013_ik_solver is None:
                # Avoid raising a TypeError when the signature differs across Isaac Sim versions.
                self.eef_motion_finished = True
                return False

        return True

    def _create_m1013_ik_solver(self):
        # Isaac Sim 5.1: LulaKinematicsSolver(robot_description_path=YAML, urdf_path=URDF)
        try:
            solver = LulaKinematicsSolver(
                robot_description_path=self.m1013_config_path,
                urdf_path=self.m1013_urdf_path,
            )
            # 디버그: URDF에 존재하는 frame 목록 확인 (eef가 있어야 함)
            print("[M1013 IK] Valid frame names:", solver.get_all_frame_names())
            return solver
        except Exception as exc:
            print(f"[M1013 IK] LulaKinematicsSolver creation failed: {exc}")
            return None

    def _extract_pose(self, pose):
        if hasattr(pose, "p") and hasattr(pose, "q"):
            position = np.array(pose.p)
            orientation = np.array(pose.q)
        elif isinstance(pose, (list, tuple)) and len(pose) == 2:
            position = np.array(pose[0])
            orientation = np.array(pose[1])
        else:
            raise ValueError("Unsupported pose format from IK solver")
        return position, orientation

    def _get_current_m1013_pose(self):
        joint_positions = self.m1013_robot.get_joint_positions()
        # Isaac Sim 5.1 introduced a keyword-only signature for the Lula solver's
        # compute_forward_kinematics method. Use keyword arguments for the
        # joint positions and fall back to positional arguments for older
        # versions to avoid "missing positional argument" TypeErrors.
        try:
            pose = self.m1013_ik_solver.compute_forward_kinematics(
                joint_positions=joint_positions.tolist(),
                frame_name=self.m1013_end_effector_frame,
            )
        except TypeError:
            pose = self.m1013_ik_solver.compute_forward_kinematics(joint_positions.tolist())

        return self._extract_pose(pose)

    def _prepare_eef_waypoints(self):
        current_pos, current_orientation = self._get_current_m1013_pose()
        if self.eef_default_orientation is None:
            self.eef_default_orientation = current_orientation

        direction = self.final_eef_target - current_pos
        for step in range(1, self.eef_path_steps + 1):
            alpha = step / self.eef_path_steps
            self.eef_waypoints.append(current_pos + alpha * direction)

    def _solve_and_apply_m1013(self, target_position):
        """Compute and apply IK, keeping orientations as numpy arrays for Lula."""

        joint_guess = self.m1013_robot.get_joint_positions()

        # Lula expects NumPy arrays for quaternions (it accesses ``shape``), so avoid
        # converting to lists even when juggling different API signatures.
        target_pos_np = np.asarray(target_position, dtype=float)
        orientation_np = self._normalize_orientation(self.eef_default_orientation)
        joint_guess_list = joint_guess.tolist()

        def _call_inverse_kinematics():
            """Call Lula IK while adapting to differing function signatures."""
            ik_fn = self.m1013_ik_solver.compute_inverse_kinematics
            signature = inspect.signature(ik_fn)
            params = list(signature.parameters.values())
            positional_params = [
                p for p in params if p.kind in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD)
            ]

            orientation_param_names = {"target_orientation", "orientation", "quat", "target_quaternion"}
            initial_guess_names = {
                "initial_joint_positions",
                "initial_position_guess",
                "initial_joint_guess",
                "initial_joint_state",
            }

            def _keyword_attempt(name):
                if name in signature.parameters:
                    return ik_fn(
                        target_position=target_pos_np,
                        target_orientation=orientation_np,
                        **{name: joint_guess_list},
                    )
                return None

            # Prefer keyword calls when the signature explicitly lists the parameter.
            for guess_name in initial_guess_names:
                try:
                    result = _keyword_attempt(guess_name)
                    if result is not None:
                        return result
                except TypeError:
                    pass

            # Without an initial guess keyword, try a simple keyword call.
            try:
                if {"target_position", "target_orientation"}.issubset(signature.parameters):
                    return ik_fn(target_position=target_pos_np, target_orientation=orientation_np)
            except TypeError:
                pass

            # Fall back to positional calls based on the declared parameter order so the
            # orientation argument is placed correctly for both (pos, ori, guess) and
            # (pos, guess, ori) layouts.
            orientation_index = None
            for idx, param in enumerate(positional_params):
                if param.name in orientation_param_names:
                    orientation_index = idx
                    break

            if orientation_index is not None:
                args = [None] * max(len(positional_params), orientation_index + 1)
                args[0] = target_pos_np
                args[orientation_index] = orientation_np

                if len(args) > orientation_index + 1:
                    args[orientation_index + 1] = joint_guess_list

                try:
                    return ik_fn(*args)
                except TypeError:
                    pass

            # Last resort: rely on the legacy positional layout.
            try:
                return ik_fn(target_pos_np, orientation_np, joint_guess_list)
            except TypeError:
                return ik_fn(target_pos_np, orientation_np)

        try:
            ik_result = _call_inverse_kinematics()
        except Exception as exc:
            print(f"[M1013 IK] Failed to compute inverse kinematics: {exc}")
            self.eef_motion_finished = True
            return

        if hasattr(ik_result, "joint_positions"):
            solved_positions = np.array(ik_result.joint_positions)
        else:
            solved_positions = np.array(ik_result)

        self.m1013_robot.set_joint_positions(solved_positions)

    def _normalize_orientation(self, orientation):
        """Ensure target orientation is a quaternion the IK solver can consume."""
        orientation_np = np.asarray(orientation, dtype=float)

        # Lula's IK expects a quaternion; some Isaac Sim versions return a
        # 3-length vector for Euler angles from forward kinematics. Pad with a
        # neutral w-component so the solver doesn't index past the array.
        if orientation_np.shape[-1] == 3:
            orientation_np = np.concatenate([orientation_np, np.array([1.0])])

        return orientation_np

    def _drive_m1013_to_final_pose(self):
        if self.eef_motion_finished:
            return

        if not self._ensure_m1013_ik_solver():
            return

        if not self.eef_motion_started:
            self._prepare_eef_waypoints()
            self.eef_motion_started = True

        if not self.eef_waypoints:
            self.eef_motion_finished = True
            return

        target_position = self.eef_waypoints[0]
        self._solve_and_apply_m1013(target_position)
        self.eef_waypoints.pop(0)

    def reset_logic(self):
        self.phase_1_done = False; self.phase_2_done = False
        self.suction_on = False
        self.robot2_up_started = False; self.robot2_up_done = False
        self.phase_1_hold_start = None; self.phase_2_hold_start = None
        self.suction_hold_start = None; self.post_up_hold_start = None
        self.unlock_triggered = False
        self.hold_pos_1 = None; self.hold_pos_2 = None
        self.table_hold_pos = None; self.table_motion_done = False
        self.initial_pos_1 = None; self.initial_pos_2 = None
        self.post_table_sequence_stage = 0
        self.post_table_stage_start = None
        self.rotation_hold_start = None; self.rotation_hold_pos = None
        self.rotation_done = False; self.rotation_started = False
        self.rotation_final_hold_start = None; self.rotation_final_hold_pos = None; self.rotation_final_done = False
        self.mold_gripper_closed = False; self.mold_gripper_released = False
        self.mold_sg2_gripper_closed = False
        self.assembly_sequence_stage = 0
        self.assembly_stage_start = None
        self.lid_assy_hold_pos = None
        self.assembly_rotation_hold_pos = None; 
        self.assembly_gripper_closed = False; self.assembly_gripper_released = False
        self.m1013_ik_solver = None
        self.m1013_ik_unavailable_notified = False
        self.eef_waypoints = []
        self.eef_motion_started = False
        self.eef_motion_finished = False
        self.eef_default_orientation = None
        self._apply_m1013_default_configuration()

        self._set_lid_assy_damping(1e3)

        cone_prim = self.stage.GetPrimAtPath(self.cone_prim_path)
        if cone_prim.IsValid():
            rigid_body_attr = cone_prim.GetAttribute("physics:rigidBodyEnabled")
            if rigid_body_attr and rigid_body_attr.IsValid():
                rigid_body_attr.Set(True)

        # 초기화 시 열기
        self.send_gripper_command(False)
        self.send_mold_gripper_command(False)
        self.send_mold_sg2_gripper_command(False)
        self.send_assembly_gripper_command(False)

        self.mold_sg2_gripper_closed = False

        cylinder_prim = self.stage.GetPrimAtPath(self.cylinder_prim_path)
        if cylinder_prim.IsValid():
            collision_attr = cylinder_prim.GetAttribute("physics:collisionEnabled")
            if collision_attr and collision_attr.IsValid():
                collision_attr.Set(True)
        
        assembly_gripper_cylinder_prim = self.stage.GetPrimAtPath(self.assembly_gripper_cylinder_path)
        if assembly_gripper_cylinder_prim.IsValid():
            collision_attr = assembly_gripper_cylinder_prim.GetAttribute("physics:collisionEnabled")
            if collision_attr and collision_attr.IsValid():
                collision_attr.Set(True)

        prim = self.stage.GetPrimAtPath(self.unlock_target_path)
        if prim.IsValid():
            attr = prim.GetAttribute("physxRigidBody:lockedPosAxis")
            if attr: attr.Set(3)

    def capture_initial_positions(self):
        if self.initial_pos_1 is None:
            self.initial_pos_1 = self.subset_1.get_joint_positions()[0]
        if self.initial_pos_2 is None:
            self.initial_pos_2 = self.subset_2.get_joint_positions()[0]

    def unlock_axis(self):
        prim = self.stage.GetPrimAtPath(self.unlock_target_path)
        if prim.IsValid():
            attr = prim.GetAttribute("physxRigidBody:lockedPosAxis")
            if attr: attr.Set(0)

    def send_gripper_command(self, close: bool):
        """명령만 전송 (상태 확인은 나중에)"""
        if not IS_INTERFACE_AVAILABLE or sg_interface is None:
            return

        try:
            if close:
                sg_interface.close_gripper(self.sg_prim_path)
                print(f">>> [Command] CLOSE Signal Sent.")
            else:
                sg_interface.open_gripper(self.sg_prim_path)
                print(f">>> [Command] OPEN Signal Sent.")
        except Exception as e:
            print(f"[ERROR] Gripper Command Failed: {e}")

    def send_mold_gripper_command(self, close: bool):
        """/World/mold_sg 전용 Surface Gripper 제어"""
        if not IS_INTERFACE_AVAILABLE or sg_interface is None:
            return

        try:
            if close:
                sg_interface.close_gripper(self.mold_sg_prim_path)
                print(">>> [Mold SG Command] CLOSE Signal Sent.")
            else:
                sg_interface.open_gripper(self.mold_sg_prim_path)
                print(">>> [Mold SG Command] OPEN Signal Sent.")
        except Exception as e:
            print(f"[ERROR] Mold Gripper Command Failed: {e}")

    def send_mold_sg2_gripper_command(self, close: bool):
        """/World/mold_sg2 Surface Gripper 제어"""
        if not IS_INTERFACE_AVAILABLE or sg_interface is None:
            return

        try:
            if close:
                sg_interface.close_gripper(self.mold_sg2_prim_path)
                print(">>> [Mold SG2 Command] CLOSE Signal Sent.")
            else:
                sg_interface.open_gripper(self.mold_sg2_prim_path)
                print(">>> [Mold SG2 Command] OPEN Signal Sent.")
        except Exception as e:
            print(f"[ERROR] Mold SG2 Gripper Command Failed: {e}")

    def get_gripper_status(self):
        """현재 그리퍼의 실제 상태(Closed/Open)를 Attribute에서 읽어옴"""
        prim = self.stage.GetPrimAtPath(self.sg_prim_path)
        if not prim.IsValid():
            return "Unknown"
        attr = prim.GetAttribute("isaac:status")
        return attr.Get() if attr.IsValid() else "Unknown"

    def send_assembly_gripper_command(self, close: bool):
        """/World/assem_sg 전용 Surface Gripper 제어"""
        if not IS_INTERFACE_AVAILABLE or sg_interface is None:
            return

        try:
            if close:
                sg_interface.close_gripper("/World/assem_sg")
                print(">>> [Assembly SG Command] CLOSE Signal Sent.")
            else:
                sg_interface.open_gripper("/World/assem_sg")
                print(">>> [Assembly SG Command] OPEN Signal Sent.")
        except Exception as e:
            print(f"[ERROR] Assembly Gripper Command Failed: {e}")

    def _post_table_stage_elapsed(self):
        if self.post_table_stage_start is None:
            self.post_table_stage_start = time.time()
            return False
        return (time.time() - self.post_table_stage_start) >= self.hold_duration

    def _set_lid_assy_damping(self, damping_value: float):
        """Safely set the lid assembly joint damping attribute if it exists."""
        damping_attr = self.stage.GetAttributeAtPath(self.lid_assy_damping_attr_path)
        if damping_attr and damping_attr.IsValid():
            damping_attr.Set(damping_value)

    def _update_orientation_xy(self, prim, x_deg: float = 90.0, y_deg: float = 0.0):
        """Update rotateXYZ so x/y match requested angles while preserving z."""
        rotate_attr = prim.GetAttribute("xformOp:rotateXYZ")
        if rotate_attr and rotate_attr.IsValid():
            current_rotate = rotate_attr.Get()
            z_value = current_rotate[2] if current_rotate is not None else 0.0
            rotate_attr.Set(Gf.Vec3d(x_deg, y_deg, z_value))

    def run_post_table_sequence(self):
        # Sequence starts after Body_Table_p_joint reaches stop threshold
        curr_table = self.table_subset.get_joint_positions()[0]
        current_pos_2 = self.subset_2.get_joint_positions()[0]

        if self.post_table_sequence_stage == 0:
            if self.table_hold_pos is not None:
                self.table_subset.apply_action(joint_velocities=np.array([(self.table_hold_pos - curr_table) * 10.0]))
            return

        # 1. UpDn_p_joint 에 self.vel_2 할당
        if self.post_table_sequence_stage == 1:
            self.subset_2.apply_action(joint_velocities=np.array([self.vel_2]))
            if current_pos_2 <= self.threshold_2 + 0.02:
                self.hold_pos_2 = current_pos_2
                self.subset_2.apply_action(joint_velocities=np.array([0.0]))
                self.post_table_sequence_stage = 2
                self.post_table_stage_start = time.time()
            return

        # 2. self.threshold_2 도달 후 hold_duration 대기, 그 후 Surface Gripper Open
        if self.post_table_sequence_stage == 2:
            if self._post_table_stage_elapsed():
                prim = self.stage.GetPrimAtPath(self.cone_prim_path)
                if prim.IsValid():
                    translate_attr = prim.GetAttribute("xformOp:translate")
                    if translate_attr and translate_attr.IsValid():
                        translate = translate_attr.Get()
                        if translate is not None:
                            translate_attr.Set(Gf.Vec3d(translate[0], 0.0, translate[2]))

                    rigid_body_attr = prim.GetAttribute("physics:rigidBodyEnabled")
                    if rigid_body_attr and rigid_body_attr.IsValid():
                        rigid_body_attr.Set(False)

                self.send_gripper_command(False)
                self.post_table_sequence_stage = 3
                self.post_table_stage_start = time.time()
            return

        # 3. hold_duration 후 UpDn_p_joint 에 self.up_vel_2 할당
        if self.post_table_sequence_stage == 3:
            if self._post_table_stage_elapsed():
                self.post_table_sequence_stage = 4
                self.post_table_stage_start = None
            return

        if self.post_table_sequence_stage == 4:
            target_pos = self.initial_pos_2 if self.initial_pos_2 is not None else 0.0
            error = target_pos - current_pos_2

            # Use a simple proportional controller to pull the joint back to its start position.
            # This avoids getting stuck slightly below zero with only a constant velocity command.
            self.subset_2.apply_action(joint_velocities=np.array([error * 10.0]))

            if abs(error) <= self.position_tolerance:
                self.hold_pos_2 = target_pos
                self.subset_2.apply_action(joint_velocities=np.array([0.0]))
                self.post_table_sequence_stage = 5
                self.post_table_stage_start = time.time()
            return

        # 4. fully up 이후 hold_duration 대기 후 테이블 내려가기 시작
        if self.post_table_sequence_stage == 5:
            if self._post_table_stage_elapsed():
                self.post_table_sequence_stage = 6
            return

        # 5. Body_Table_p_joint 에 -self.table_up_vel 적용하여 원위치 복귀
        if self.post_table_sequence_stage == 6:
            if curr_table <= self.position_tolerance:
                self.table_subset.apply_action(joint_velocities=np.array([0.0]))
                self.table_hold_pos = 0.0
                self.post_table_sequence_stage = 7
            else:
                self.table_subset.apply_action(joint_velocities=np.array([-self.table_up_vel]))
            return

        # 6. 완료 후 위치 유지
        if self.post_table_sequence_stage >= 7:
            if self.table_hold_pos is not None:
                self.table_subset.apply_action(joint_velocities=np.array([(self.table_hold_pos - curr_table) * 10.0]))

            current_rotation = self.rotation_subset.get_joint_positions()[0]

            if not self.rotation_started:
                self.send_mold_gripper_command(True)
                self.rotation_started = True
                self.mold_gripper_closed = True

            if self.rotation_done:
                if self.mold_gripper_closed and not self.mold_gripper_released:
                    self.send_mold_gripper_command(False)
                    self.mold_gripper_released = True

                if self.rotation_hold_pos is not None and self.assembly_sequence_stage < 7:
                    self.rotation_subset.apply_action(
                        joint_velocities=np.array([(self.rotation_hold_pos - current_rotation) * 10.0])
                    )

                if self.assembly_sequence_stage == 0:
                    self.assembly_sequence_stage = 1

                self.run_assembly_sequence()

                if self.assembly_sequence_stage >= 7:
                    if self.rotation_final_done:
                        if self.rotation_final_hold_pos is not None:
                            self.rotation_subset.apply_action(
                                joint_velocities=np.array([
                                    (self.rotation_final_hold_pos - current_rotation) * 10.0
                                ])
                            )
                    else:
                        final_error = self.rotation_final_target - current_rotation

                        if abs(final_error) <= self.rotation_tolerance:
                            self.rotation_subset.apply_action(joint_velocities=np.array([0.0]))

                            if self.rotation_final_hold_start is None:
                                self.rotation_final_hold_start = time.time()
                                self.rotation_final_hold_pos = current_rotation

                            if (time.time() - self.rotation_final_hold_start) >= self.hold_duration:
                                self.rotation_final_done = True
                        else:
                            self.rotation_final_hold_start = None
                            self.rotation_final_hold_pos = None
                            commanded_speed = np.clip(
                                final_error * self.rotation_kp,
                                -self.rotation_max_speed,
                                self.rotation_max_speed,
                            )
                            self.rotation_subset.apply_action(joint_velocities=np.array([commanded_speed]))

                return

            rotation_error = self.rotation_target - current_rotation

            if abs(rotation_error) <= self.rotation_tolerance:
                self.rotation_subset.apply_action(joint_velocities=np.array([0.0]))

                if self.rotation_hold_start is None:
                    self.rotation_hold_start = time.time()
                    self.rotation_hold_pos = current_rotation

                if (time.time() - self.rotation_hold_start) >= self.hold_duration:
                    self.rotation_done = True
            else:
                self.rotation_hold_start = None
                self.rotation_hold_pos = None
                commanded_speed = np.clip(
                    rotation_error * self.rotation_kp,
                    -self.rotation_max_speed,
                    self.rotation_max_speed,
                )
                self.rotation_subset.apply_action(joint_velocities=np.array([commanded_speed]))

    def _assembly_stage_elapsed(self):
        if self.assembly_stage_start is None:
            self.assembly_stage_start = time.time()
            return False
        return (time.time() - self.assembly_stage_start) >= self.hold_duration

    def _hold_lid_assy_position(self, current_pos):
        if self.lid_assy_hold_pos is not None:
            self.lid_assy_subset.apply_action(
                joint_velocities=np.array([(self.lid_assy_hold_pos - current_pos) * 10.0])
            )

    def _command_lid_assy_to_target(self, target_pos, max_speed):
        """Drive the lid assembly joint toward a target using a saturated P controller."""
        current_pos = self.lid_assy_subset.get_joint_positions()[0]
        error = target_pos - current_pos
        commanded = np.clip(error * self.lid_assy_kp, -max_speed, max_speed)
        self.lid_assy_subset.apply_action(joint_velocities=np.array([commanded]))
        return current_pos, abs(error) <= self.position_tolerance

    def _hold_assembly_rotation(self, current_rot):
        if self.assembly_rotation_hold_pos is not None:
            self.assembly_rotation_subset.apply_action(
                joint_velocities=np.array([(self.assembly_rotation_hold_pos - current_rot) * 10.0])
            )

    def run_assembly_sequence(self):
        lid_pos = self.lid_assy_subset.get_joint_positions()[0]
        assembly_rot_pos = self.assembly_rotation_subset.get_joint_positions()[0]

        # Idle hold
        if self.assembly_sequence_stage == 0:
            self._hold_lid_assy_position(lid_pos)
            self._hold_assembly_rotation(assembly_rot_pos)
            return

        # 1. Move to 0.56 with velocity 0.1
        if self.assembly_sequence_stage == 1:
            _, reached = self._command_lid_assy_to_target(self.lid_assy_first_stop, self.lid_assy_speed)

            if self.assembly_stage_start is None:
                self.assembly_stage_start = time.time()

            elapsed = time.time() - self.assembly_stage_start
            if reached or elapsed >= self.assembly_move_timeout:
                self.lid_assy_hold_pos = self.lid_assy_subset.get_joint_positions()[0]
                self.lid_assy_subset.apply_action(joint_velocities=np.array([0.0]))
                self.assembly_stage_start = time.time()
                self.assembly_sequence_stage = 2
            return

        # 2. Hold at 0.56 for hold_duration
        if self.assembly_sequence_stage == 2:
            self._hold_lid_assy_position(lid_pos)
            if self._assembly_stage_elapsed():
                cylinder_prim = self.stage.GetPrimAtPath(self.cylinder_prim_path)
                if cylinder_prim.IsValid():
                    collision_attr = cylinder_prim.GetAttribute("physics:collisionEnabled")
                    if collision_attr and collision_attr.IsValid():
                        collision_attr.Set(False)
                self.assembly_sequence_stage = 3
                self.assembly_stage_start = None
            return

        # 3. Close /World/assem_sg and hold
        if self.assembly_sequence_stage == 3:
            self._hold_lid_assy_position(lid_pos)
            if not self.assembly_gripper_closed:
                self.send_assembly_gripper_command(True)
                self.assembly_gripper_closed = True
                self.assembly_stage_start = time.time()
            elif (self.assembly_stage_start is not None) and self._assembly_stage_elapsed():
                self._set_lid_assy_damping(1e5)
                self.assembly_sequence_stage = 4
                self.assembly_stage_start = None
            return

        # 4. Move to 0.7 while rotating assembly joint at speed 15
        if self.assembly_sequence_stage == 4:
            lid_pos, reached_target = self._command_lid_assy_to_target(
                self.lid_assy_second_stop, 0.25*self.lid_assy_speed
            )
            self.assembly_rotation_subset.apply_action(
                joint_velocities=np.array([self.assembly_rotation_speed])
            )

            if self.assembly_stage_start is None:
                self.assembly_stage_start = time.time()

            # Fall back after a timeout so the sequence can continue even if the
            # prismatic joint is unable to reach its target (e.g. due to
            # collisions or limits).
            elapsed = time.time() - self.assembly_stage_start

            if reached_target or elapsed >= 1.7 * self.assembly_move_timeout:
                self.lid_assy_hold_pos = self.lid_assy_subset.get_joint_positions()[0]
                self.assembly_rotation_hold_pos = assembly_rot_pos
                self.lid_assy_subset.apply_action(joint_velocities=np.array([0.0]))
                self.assembly_rotation_subset.apply_action(joint_velocities=np.array([0.0]))
                self.assembly_sequence_stage = 5
                self.assembly_stage_start = None
            return

        # 5. Stop rotation, open gripper, hold for hold_duration
        if self.assembly_sequence_stage == 5:
            self._hold_lid_assy_position(lid_pos)
            self._hold_assembly_rotation(assembly_rot_pos)

            if not self.assembly_gripper_released:
                assembly_gripper_cylinder_prim = self.stage.GetPrimAtPath(self.assembly_gripper_cylinder_path)
                if assembly_gripper_cylinder_prim.IsValid():
                    collision_attr = assembly_gripper_cylinder_prim.GetAttribute("physics:collisionEnabled")
                    if collision_attr and collision_attr.IsValid():
                        collision_attr.Set(False)

                source_prim = self.stage.GetPrimAtPath(
                    "/World/ip_model/ip_model/tn__NT251101A001_tCX59b7o0/"
                    "tn__NT251101A101_tCX59b7o0/tn__moldA181_k88X2Lu0a6i0/mold"
                )
                target_prim = self.stage.GetPrimAtPath(
                    "/World/ip_model/ip_model/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/"
                    "moldcap"
                )

                if source_prim.IsValid() and target_prim.IsValid():
                    source_translate_attr = source_prim.GetAttribute("xformOp:translate")
                    target_translate_attr = target_prim.GetAttribute("xformOp:translate")
                    target_rigid_attr = target_prim.GetAttribute("physics:rigidBodyEnabled")

                    if (
                        source_translate_attr
                        and source_translate_attr.IsValid()
                        and target_translate_attr
                        and target_translate_attr.IsValid()
                    ):
                        source_translate = source_translate_attr.Get()
                        target_translate = target_translate_attr.Get()

                        if source_translate is not None:
                            target_y = target_translate[1] if target_translate is not None else 0.0
                            new_translate = Gf.Vec3d(source_translate[0], target_y, source_translate[2])
                            if target_rigid_attr and target_rigid_attr.IsValid():
                                target_rigid_attr.Set(False)

                            target_translate_attr.Set(new_translate)
                            self._update_orientation_xy(target_prim)

                            if target_rigid_attr and target_rigid_attr.IsValid():
                                target_rigid_attr.Set(True)
                            if not self.mold_sg2_gripper_closed:
                                self.send_mold_sg2_gripper_command(True)
                                self.mold_sg2_gripper_closed = True
                            print(
                                f"Updated moldcap translate to {new_translate} based on source {source_translate}"
                            )
                self.assembly_gripper_released = True

                if self.assembly_stage_start is None:
                    self.assembly_stage_start = time.time()

            if self._assembly_stage_elapsed():
                self.assembly_sequence_stage = 6
                self.assembly_stage_start = None
            return

        # 6. Return to 0 with velocity -0.1
        if self.assembly_sequence_stage == 6:
            self.assembly_rotation_subset.apply_action(joint_velocities=np.array([0.0]))
            self.lid_assy_subset.apply_action(joint_velocities=np.array([-self.lid_assy_speed]))

            if not self.mold_sg2_gripper_closed:
                self.send_mold_sg2_gripper_command(True)
                self.mold_sg2_gripper_closed = True

            if lid_pos <= self.position_tolerance:
                self.lid_assy_hold_pos = 0.0
                self.lid_assy_subset.apply_action(joint_velocities=np.array([0.0]))
                self.assembly_sequence_stage = 7
                self.assembly_stage_start = time.time()
            return

        # 7. Hold final position
        if self.assembly_sequence_stage >= 7:
            self._hold_lid_assy_position(lid_pos)
            self._hold_assembly_rotation(assembly_rot_pos)
            if self.rotation_final_done:
                self._drive_m1013_to_final_pose()

    def step(self):
        needs_init = False
        if not self.robot_1.handles_initialized:
            self.robot_1.initialize()
            needs_init = True
        if not self.robot_2.handles_initialized:
            self.robot_2.initialize()
            needs_init = True
        if not self.table.handles_initialized:
            self.table.initialize()
            needs_init = True
        if not self.rotation.handles_initialized:
            self.rotation.initialize()
            needs_init = True
        if not self.lid_assy.handles_initialized:
            self.lid_assy.initialize()
            needs_init = True
        if not self.assembly_rotation.handles_initialized:
            self.assembly_rotation.initialize()
            needs_init = True
        
        if needs_init:
            self.reset_logic()
            self.capture_initial_positions()
            return

        # [Phase 4]
        if self.robot2_up_done:
            if self.post_up_hold_start is None:
                self.post_up_hold_start = time.time()
            if ((time.time() - self.post_up_hold_start) >= self.hold_duration and not self.unlock_triggered):
                self.unlock_axis()
                self.unlock_triggered = True
            if not self.table_motion_done:
                curr_table = self.table_subset.get_joint_positions()[0]
                if curr_table >= self.table_stop_threshold:
                    self.table_motion_done = True
                    self.table_hold_pos = curr_table
                    self.table_subset.apply_action(joint_velocities=np.array([0.0]))
                    self.post_table_sequence_stage = 1
                    self.post_table_stage_start = None
                else:
                    self.table_subset.apply_action(joint_velocities=np.array([self.table_up_vel]))
            else:
                self.run_post_table_sequence()
            if self.hold_pos_1 is not None:
                curr_1 = self.subset_1.get_joint_positions()[0]
                self.subset_1.apply_action(joint_velocities=np.array([(self.hold_pos_1 - curr_1) * 10.0]))
            if self.hold_pos_2 is not None and not (self.post_table_sequence_stage in [1, 4]):
                curr_2 = self.subset_2.get_joint_positions()[0]
                self.subset_2.apply_action(joint_velocities=np.array([(self.hold_pos_2 - curr_2) * 10.0]))
            return

        # [Phase 3] Suction Hold & Robot 2 Up
        if self.phase_2_done:
            # 1. 아직 Suction 명령을 안 보냈으면 -> 보냄
            if not self.suction_on:
                self.send_gripper_command(True) # Close 명령
                self.suction_on = True
                self.suction_hold_start = time.time()
                print(">>> Suction ON. Waiting for hold duration...")

            # 2. hold_duration 동안 대기 (로봇 2는 멈춤 상태 유지)
            elapsed_time = 0
            if self.suction_hold_start is not None:
                elapsed_time = time.time() - self.suction_hold_start

            # 대기 시간이 끝나고, 아직 올라가지 않았다면
            if elapsed_time >= self.hold_duration and not self.robot2_up_started:
                # [중요] 여기서 실제 상태 확인
                status = self.get_gripper_status()
                print(f"!!! Hold Duration Over. Checking Status: {status}")
                
                if status == "Closed":
                    print("!!! GRIP CONFIRMED. Moving Robot 2 Up !!!")
                    self.robot2_up_started = True
                else:
                    # 실패 시 경고 출력 (계속 시도할지, 그냥 올릴지 결정 필요. 여기선 경고 후 올림)
                    print(f"!!! WARNING: Gripper Status is '{status}'. Object might not be attached.")
                    # 재시도 로직을 원하면 여기서 return을 해서 계속 대기할 수도 있음
                    # 현재는 그냥 진행
                    self.robot2_up_started = True

            # Robot 1 Hold
            if self.hold_pos_1 is not None:
                curr_1 = self.subset_1.get_joint_positions()[0]
                self.subset_1.apply_action(joint_velocities=np.array([(self.hold_pos_1 - curr_1) * 10.0]))

            # Robot 2 Control
            if self.robot2_up_started:
                current_pos_2 = self.subset_2.get_joint_positions()[0]
                if (self.initial_pos_2 is not None and current_pos_2 >= self.initial_pos_2 - self.position_tolerance):
                    print(f"!!! Robot 2 Up Complete at {current_pos_2:.4f} !!!")
                    self.robot2_up_done = True
                    self.hold_pos_2 = self.initial_pos_2
                    self.subset_2.apply_action(joint_velocities=np.array([0.0]))
                else:
                    self.subset_2.apply_action(joint_velocities=np.array([self.up_vel_2]))
            else:
                # 대기 중 (Hold Position)
                if self.hold_pos_2 is not None:
                    curr_2 = self.subset_2.get_joint_positions()[0]
                    self.subset_2.apply_action(joint_velocities=np.array([(self.hold_pos_2 - curr_2) * 10.0]))
            return

        # [Phase 2] (기존 동일)
        if self.phase_1_done:
            curr_1 = self.subset_1.get_joint_positions()[0]
            if self.hold_pos_1 is not None:
                self.subset_1.apply_action(joint_velocities=np.array([(self.hold_pos_1 - curr_1) * 10.0]))
            
            current_pos_2 = self.subset_2.get_joint_positions()[0]
            if (self.phase_1_hold_start is not None and (time.time() - self.phase_1_hold_start) < self.hold_duration):
                self.subset_2.apply_action(joint_velocities=np.array([0.0]))
                return
            
            if current_pos_2 <= self.threshold_2:
                print(f"!!! Phase 2 Done. Robot 2 Reached {current_pos_2:.4f} !!!")
                self.phase_2_done = True
                self.phase_2_hold_start = time.time()
                self.hold_pos_2 = current_pos_2
                self.subset_2.apply_action(joint_velocities=np.array([0.0]))
            else:
                self.subset_2.apply_action(joint_velocities=np.array([self.vel_2]))
            return

        # [Phase 1] (기존 동일)
        current_pos_1 = self.subset_1.get_joint_positions()[0]
        if current_pos_1 <= self.threshold_1:
            print(f"!!! Phase 1 Done. Holding {self.joint_1}, Moving {self.joint_2} !!!")
            self.phase_1_done = True
            self.phase_1_hold_start = time.time()
            self.hold_pos_1 = current_pos_1
            self.subset_1.apply_action(joint_velocities=np.array([0.0]))
        else:
            self.subset_1.apply_action(joint_velocities=np.array([self.vel_1]))

# --- Hooks ---
def setup(db: og.Database):
    db.per_instance_state.controller = DualRobotController()

def compute(db: og.Database):
    if hasattr(db.per_instance_state, "controller"):
        db.per_instance_state.controller.step()
    return True
