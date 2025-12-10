import omni.graph.core as og
from omni.isaac.core.articulations import Articulation, ArticulationSubset
import numpy as np
import omni.usd

class DualRobotController:
    def __init__(self):
        # ---------------------------------------------------------
        # [경로 설정]
        self.world_root = "/World/ip_model/ip_model"

        # Robot 1: Lid Lifter Root
        self.root_1 = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/tn__AR070PRN150010NNV3EL211_jONg6No0QW3AEIKNlHm2/tn__AR070PRN150010NNV3EL_03_VY0AEIKN" 
        
        # Robot 2: UpDn Root
        self.root_2 = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/tn__MY1C25400L6LZ7311_XIHq4d0W2DfBh1" 

        # Unlock 대상 Prim 경로
        self.unlock_target_path = self.world_root + "/tn__NT251101A001_tCX59b7o0/tn__NT251101A2011_uDDl3V0l19d9V1/tn__moldACAP_23_mG6"

        # Joint 이름
        self.joint_1 = "Lid_Lifter_p_joint"
        self.joint_2 = "UpDn_p_joint"

        # 설정값
        self.vel_1 = -0.1
        self.threshold_1 = -0.022 - 0.02*2.6
        
        self.vel_2 = -0.3
        self.threshold_2 = -0.043 
        # ---------------------------------------------------------

        self.robot_1 = Articulation(self.root_1)
        self.robot_2 = Articulation(self.root_2)

        self.subset_1 = ArticulationSubset(self.robot_1, [self.joint_1])
        self.subset_2 = ArticulationSubset(self.robot_2, [self.joint_2])

        self.stage = omni.usd.get_context().get_stage()

        self.reset_logic()
        print("Controller Ready: Phase 1 -> Phase 2 -> Unlock (Bitmask Fixed)")

    def reset_logic(self):
        self.phase_1_done = False
        self.phase_2_done = False
        self.hold_pos_1 = None
        self.hold_pos_2 = None
        prim = self.stage.GetPrimAtPath(self.unlock_target_path)
        attr = prim.GetAttribute("physxRigidBody:lockedPosAxis")
        attr.Set(3) # X, Y 잠금
        print(">>> RESET: Logic variables have been reset. <<<")

    def unlock_axis(self):
        """지정된 Prim의 X, Y 축 잠금을 해제하는 함수"""
        prim = self.stage.GetPrimAtPath(self.unlock_target_path)
        if not prim.IsValid():
            print(f"[ERROR] Unlock Target Prim not found: {self.unlock_target_path}")
            return

        attr = prim.GetAttribute("physxRigidBody:lockedPosAxis")
        if attr:
            # [수정됨] 벡터가 아닌 정수(Bitmask)를 입력해야 합니다.
            # 0: 모두 해제
            # 1: X 잠금
            # 2: Y 잠금
            # 4: Z 잠금
            # 7: 모두 잠금 (1+2+4)
            
            # 사용자 요청: X, Y, Z 모두 해제 -> 값 0
            attr.Set(0) 
            print(f">>> UNLOCKED: X, Y axis unlocked (Z locked) for {prim.GetName()}")
        else:
            print("[ERROR] Attribute 'physxRigidBody:lockedPosAxis' not found on target prim.")

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
            return

        # [Phase 3] Unlock & Hold Both
        if self.phase_2_done:
            if self.hold_pos_1 is not None:
                curr_1 = self.subset_1.get_joint_positions()[0]
                self.subset_1.apply_action(joint_velocities=np.array([(self.hold_pos_1 - curr_1) * 10.0]))
            
            if self.hold_pos_2 is not None:
                curr_2 = self.subset_2.get_joint_positions()[0]
                self.subset_2.apply_action(joint_velocities=np.array([(self.hold_pos_2 - curr_2) * 10.0]))
            return

        # [Phase 2] Robot 1 Hold & Robot 2 Move
        if self.phase_1_done:
            curr_1 = self.subset_1.get_joint_positions()[0]
            if self.hold_pos_1 is not None:
                self.subset_1.apply_action(joint_velocities=np.array([(self.hold_pos_1 - curr_1) * 10.0]))

            current_pos_2 = self.subset_2.get_joint_positions()[0]

            if current_pos_2 <= self.threshold_2:
                print(f"!!! Phase 2 Done. Robot 2 Reached {current_pos_2:.4f} !!!")
                self.phase_2_done = True
                self.hold_pos_2 = current_pos_2
                self.subset_2.apply_action(joint_velocities=np.array([0.0]))
                
                # Unlock 함수 호출
                self.unlock_axis()
            else:
                self.subset_2.apply_action(joint_velocities=np.array([self.vel_2]))
            return

        # [Phase 1] Robot 1 Move
        current_pos_1 = self.subset_1.get_joint_positions()[0]

        if current_pos_1 <= self.threshold_1:
            print(f"!!! Phase 1 Done. Holding {self.joint_1}, Moving {self.joint_2} !!!")
            self.phase_1_done = True
            self.hold_pos_1 = current_pos_1
            self.subset_1.apply_action(joint_velocities=np.array([0.0]))
        else:
            self.subset_1.apply_action(joint_velocities=np.array([self.vel_1]))

# --- Action Graph Hooks ---

def setup(db: og.Database):
    db.per_instance_state.controller = DualRobotController()

def compute(db: og.Database):
    if hasattr(db.per_instance_state, "controller"):
        db.per_instance_state.controller.step()
    return True