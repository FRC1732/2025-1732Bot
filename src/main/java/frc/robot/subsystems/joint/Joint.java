// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.joint;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joint extends SubsystemBase {

  public Joint() {}

  public void setJointPose(JointPosition jointPosition) {}

  public void setJointPosition(double setpoint) {}

  public double getJointPosition() {
    return 0;
  }

  public boolean isAtGoal() {
    return true;
    // return jointPID.atGoal();
  }

  private double getAbsolutePosition() {
    return 0;
  }

  @Override
  public void periodic() {}

  public void resetToAbsoluteEncoder() {}

  public void runJointForward() {}

  public void runJointBackward() {}

  public void stopJoint() {}

  public double getSetPoint() {
    return 0;
  }

  public void setupShuffleboard() {}
}
