import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np

kPi = 3.141592654
kPi_2 = 1.57079632

class G1JointIndex:
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof

    kNotUsedJoint = 29 # NOTE: Weight

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02  
        # Increased duration so each stage is slower than original 3.0 s
        self.duration_ = 5.0   
        self.counter_ = 0
        self.weight = 0.
        self.weight_rate = 0.2
        # Slightly lower gains to make motion softer / less snappy
        self.kp = 40.
        self.kd = 1.0
        self.dq = 0.
        self.tau_ff = 0.
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False

        # target_pos used in one of the early stages (arms up)
        self.target_pos = [
            0., kPi_2,  0., kPi_2, 0., 0., 0.,
            0., -kPi_2, 0., kPi_2, 0., 0., 0., 
            0., 0., 0.
        ]

        # arm & waist joints we control
        self.arm_joints = [
          G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
          G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
          G1JointIndex.LeftWristRoll,      G1JointIndex.LeftWristPitch,
          G1JointIndex.LeftWristYaw,
          G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
          G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
          G1JointIndex.RightWristRoll,     G1JointIndex.RightWristPitch,
          G1JointIndex.RightWristYaw,
          G1JointIndex.WaistYaw,
          G1JointIndex.WaistRoll,
          G1JointIndex.WaistPitch
        ]

    def Init(self):
        # create publisher #
        self.arm_sdk_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.arm_sdk_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.first_update_low_state == False:
            time.sleep(1)

        if self.first_update_low_state == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.first_update_low_state == False:
            self.first_update_low_state = True

    def _set_joint_cmd(self, joint, desired_q):
        """Helper: set PD command for a single joint toward desired_q."""
        self.low_cmd.motor_cmd[joint].tau = 0.
        self.low_cmd.motor_cmd[joint].q = desired_q
        self.low_cmd.motor_cmd[joint].dq = 0.
        self.low_cmd.motor_cmd[joint].kp = self.kp
        self.low_cmd.motor_cmd[joint].kd = self.kd

    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        t = self.time_
        D = self.duration_  # shorthand

        # Helper to compute smooth blend between current state and a target
        def blend(joint, start_time, span, target_q):
            ratio = np.clip((t - start_time) / span, 0.0, 1.0)
            current_q = self.low_state.motor_state[joint].q
            q_cmd = ratio * target_q + (1.0 - ratio) * current_q
            self._set_joint_cmd(joint, q_cmd)

        # By default, keep SDK enabled unless we taper it off explicitly
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

        # -------------------
        # Stage 1: Move to zero posture (arms to neutral)
        # 0–D
        # -------------------
        if t < D:
            if self.counter_ == 0:
                print(f"Stage 1: Setting to zero posture (0-{D}s)")
            for i, joint in enumerate(self.arm_joints):
                ratio = np.clip(t / D, 0.0, 1.0)
                current_q = self.low_state.motor_state[joint].q
                q_cmd = (1.0 - ratio) * current_q
                self._set_joint_cmd(joint, q_cmd)

        # -------------------
        # Stage 2: Lift arms up (using target_pos)
        # D–3D
        # -------------------
        elif t < 3 * D:
            if self.counter_ == int(D / self.control_dt_):
                print(f"Stage 2: Lifting arms up ({D}-{3*D}s)")
            for i, joint in enumerate(self.arm_joints):
                ratio = np.clip((t - D) / (2 * D), 0.0, 1.0)
                current_q = self.low_state.motor_state[joint].q
                target_q = self.target_pos[i]
                q_cmd = ratio * target_q + (1.0 - ratio) * current_q
                self._set_joint_cmd(joint, q_cmd)

        # -------------------
        # Stage 3: Return arms to zero posture
        # 3D–6D
        # -------------------
        elif t < 6 * D:
            if self.counter_ == int(3 * D / self.control_dt_):
                print(f"Stage 3: Returning to zero posture ({3*D}-{6*D}s)")
            for i, joint in enumerate(self.arm_joints):
                ratio = np.clip((t - 3 * D) / (3 * D), 0.0, 1.0)
                current_q = self.low_state.motor_state[joint].q
                q_cmd = (1.0 - ratio) * current_q
                self._set_joint_cmd(joint, q_cmd)

        # -------------------
        # Stage 4: Release arm_sdk slowly (no big motion, just disabling)
        # 6D–7D
        # -------------------
        elif t < 7 * D:
            if self.counter_ == int(6 * D / self.control_dt_):
                print(f"Stage 4: Releasing arm_sdk ({6*D}-{7*D}s)")
            ratio = np.clip((t - 6 * D) / D, 0.0, 1.0)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = (1.0 - ratio)

        # ==================================================================
        # NEW DANCE & "ACROBATICS" STAGES (arms & waist only, legs untouched)
        # ==================================================================

        # Stage 5: Re-enable & move to wide T-pose (arms straight out)
        # 7D–8D
        elif t < 8 * D:
            if self.counter_ == int(7 * D / self.control_dt_):
                print(f"Stage 5: Moving to T-pose ({7*D}-{8*D}s)")
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
            for joint in self.arm_joints:
                target_q = 0.0
                if joint == G1JointIndex.LeftShoulderRoll:
                    target_q = kPi_2 * 0.8   # slightly less than 90°
                elif joint == G1JointIndex.RightShoulderRoll:
                    target_q = -kPi_2 * 0.8
                blend(joint, 7 * D, D, target_q)

        # Stage 6: Arm wave (dance) – elbows bend-unbend while in T-pose
        # 8D–9D
        elif t < 9 * D:
            if self.counter_ == int(8 * D / self.control_dt_):
                print(f"Stage 6: Arm wave ({8*D}-{9*D}s)")
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

            # Keep shoulder rolls wide (T-pose)
            blend(G1JointIndex.LeftShoulderRoll, 8 * D, D, kPi_2 * 0.8)
            blend(G1JointIndex.RightShoulderRoll, 8 * D, D, -kPi_2 * 0.8)

            # Simple sinusoidal elbow wave (small amplitude)
            wave_phase = (t - 8 * D) / D * 2.0 * kPi  # 1 full cycle
            elbow_offset = 0.3 * np.sin(wave_phase)

            for joint in [G1JointIndex.LeftElbow, G1JointIndex.RightElbow]:
                current_q = self.low_state.motor_state[joint].q
                target_q = elbow_offset
                ratio = np.clip((t - 8 * D) / D, 0.0, 1.0)
                q_cmd = ratio * target_q + (1.0 - ratio) * current_q
                self._set_joint_cmd(joint, q_cmd)

        # Stage 7: Cross arms in front of chest (dance move)
        # 9D–10D
        elif t < 10 * D:
            if self.counter_ == int(9 * D / self.control_dt_):
                print(f"Stage 7: Crossing arms ({9*D}-{10*D}s)")
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

            for joint in self.arm_joints:
                target_q = 0.0
                if joint == G1JointIndex.LeftShoulderYaw:
                    target_q = 0.4
                elif joint == G1JointIndex.RightShoulderYaw:
                    target_q = -0.4
                elif joint in (G1JointIndex.LeftElbow, G1JointIndex.RightElbow):
                    target_q = -kPi / 3.0  # bend inward
                blend(joint, 9 * D, D, target_q)

        # Stage 8: Open back to neutral from cross
        # 10D–11D
        elif t < 11 * D:
            if self.counter_ == int(10 * D / self.control_dt_):
                print(f"Stage 8: Open from cross to neutral ({10*D}-{11*D}s)")
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
            for joint in self.arm_joints:
                blend(joint, 10 * D, D, 0.0)

        # Stage 9: "Acrobatic" overhead circle – arms make a big overhead sweep
        # 11D–12D
        elif t < 12 * D:
            if self.counter_ == int(11 * D / self.control_dt_):
                print(f"Stage 9: Overhead arm circle ({11*D}-{12*D}s)")
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

            # Use sine/cosine to keep it smooth and slow
            phase = (t - 11 * D) / D * kPi  # half-circle in time D
            left_pitch = 0.5 * kPi_2 * (1 - np.cos(phase))   # 0 -> ~90°
            right_pitch = 0.5 * kPi_2 * (1 - np.cos(phase))

            for joint in self.arm_joints:
                target_q = 0.0
                if joint == G1JointIndex.LeftShoulderPitch:
                    target_q = left_pitch
                elif joint == G1JointIndex.RightShoulderPitch:
                    target_q = right_pitch
                blend(joint, 11 * D, D, target_q)

        # Stage 10: "Acrobatic" twist – gentle waist yaw twist with arms up
        # 12D–13D
        elif t < 13 * D:
            if self.counter_ == int(12 * D / self.control_dt_):
                print(f"Stage 10: Gentle waist twist ({12*D}-{13*D}s)")
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

            twist_phase = (t - 12 * D) / D * 2.0 * kPi
            waist_yaw = 0.25 * (kPi / 6.0) * np.sin(twist_phase)  # small, safe amplitude

            # Keep arms generally up while twisting
            for joint in self.arm_joints:
                target_q = 0.0
                if joint in (G1JointIndex.LeftShoulderPitch, G1JointIndex.RightShoulderPitch):
                    target_q = kPi_2 * 0.6
                if joint == G1JointIndex.WaistYaw:
                    target_q = waist_yaw
                blend(joint, 12 * D, D, target_q)

        # Stage 11: "Acrobatic" side lean – small waist roll, arms out for balance
        # 13D–14D
        elif t < 14 * D:
            if self.counter_ == int(13 * D / self.control_dt_):
                print(f"Stage 11: Side lean with arms out ({13*D}-{14*D}s)")
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

            lean_phase = (t - 13 * D) / D * 2.0 * kPi
            waist_roll = 0.25 * (kPi / 9.0) * np.sin(lean_phase)  # very small lean

            for joint in self.arm_joints:
                target_q = 0.0
                if joint == G1JointIndex.LeftShoulderRoll:
                    target_q = kPi_2 * 0.7
                elif joint == G1JointIndex.RightShoulderRoll:
                    target_q = -kPi_2 * 0.7
                elif joint == G1JointIndex.WaistRoll:
                    target_q = waist_roll
                blend(joint, 13 * D, D, target_q)

        # Stage 12: Return gently to neutral & slowly release arm_sdk
        # 14D–15D
        elif t < 15 * D:
            if self.counter_ == int(14 * D / self.control_dt_):
                print(f"Stage 12: Final neutral & release ({14*D}-{15*D}s)")
            # Gradual release of arm_sdk
            ratio_release = np.clip((t - 14 * D) / D, 0.0, 1.0)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0 - ratio_release
            for joint in self.arm_joints:
                blend(joint, 14 * D, D, 0.0)

        else:
            if not self.done:
                print(f"All stages completed! Total time: {t:.2f}s")
            self.done = True
  
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)
        self.counter_ += 1

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        # If interface is "lo", use domain ID 1 for simulation, otherwise use 0 for real robot
        domain_id = 1 if sys.argv[1] == "lo" else 0
        ChannelFactoryInitialize(domain_id, sys.argv[1])
    else:
        ChannelFactoryInitialize(1,"lo")  # Default to domain 1 for simulation

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:        
        time.sleep(1)
        if custom.done: 
           print("Done!")
           sys.exit(0)
