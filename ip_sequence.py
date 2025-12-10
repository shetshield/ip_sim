import omni.graph.core as og
from omni.isaac.core.articulations import Articulation, ArticulationSubset
import numpy as np
import omni.usd
import time

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

        # [Prim Path]
        self.sg_prim_path = \
            "/World/SurfaceGripper"
        
        self.unlock_target_path = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/tn__moldACAP_23_mG6"
        
        # Settings
        self.joint_1 = "Lid_Lifter_p_joint"
        self.joint_2 = "UpDn_p_joint"
        self.vel_1 = -0.1
        self.threshold_1 = -0.022 - 0.02*2.6
        self.vel_2 = -0.3
        self.threshold_2 = -0.043
        self.hold_duration = 0.5
        self.up_vel_2 = 0.3
        self.position_tolerance = 1e-3

        self.robot_1 = Articulation(self.root_1)
        self.robot_2 = Articulation(self.root_2)
        self.subset_1 = ArticulationSubset(self.robot_1, [self.joint_1])
        self.subset_2 = ArticulationSubset(self.robot_2, [self.joint_2])
        self.stage = omni.usd.get_context().get_stage()

        self.reset_logic()
        print("Controller Ready: Wait-Verify-Move Logic")

    def reset_logic(self):
        self.phase_1_done = False
        self.phase_2_done = False
        self.suction_on = False
        self.robot2_up_started = False
        self.robot2_up_done = False
        self.phase_1_hold_start = None
        self.phase_2_hold_start = None
        self.suction_hold_start = None
        self.post_up_hold_start = None
        self.unlock_triggered = False
        self.hold_pos_1 = None
        self.hold_pos_2 = None
        self.initial_pos_1 = None
        self.initial_pos_2 = None
        
        # 초기화 시 열기
        self.send_gripper_command(False)

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

    def get_gripper_status(self):
        """현재 그리퍼의 실제 상태(Closed/Open)를 Attribute에서 읽어옴"""
        prim = self.stage.GetPrimAtPath(self.sg_prim_path)
        if not prim.IsValid():
            return "Unknown"
        attr = prim.GetAttribute("isaac:status")
        return attr.Get() if attr.IsValid() else "Unknown"

    def step(self):
        needs_init = False
        if not self.robot_1.handles_initialized:
            self.robot_1.initialize()
            needs_init = True
        if not self.robot_2.handles_initialized:
            self.robot_2.initialize()
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
            if self.hold_pos_1 is not None:
                curr_1 = self.subset_1.get_joint_positions()[0]
                self.subset_1.apply_action(joint_velocities=np.array([(self.hold_pos_1 - curr_1) * 10.0]))
            if self.hold_pos_2 is not None:
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
