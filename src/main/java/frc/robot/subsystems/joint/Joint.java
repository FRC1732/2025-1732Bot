// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.joint;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joint extends SubsystemBase {
  /** Creates a new Joint. */
  private final SparkMax jointMotor;

  private ShuffleboardTab jointTab;
  private Double setpoint;

  public Joint() {
    jointMotor = new SparkMax(JointConstants.JOINT_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);
    jointMotor.stopMotor();
    setpoint = 0.0;
  }

  public enum JointPosition {
    CORAL_STATION,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4,
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runJointForward() {
    jointMotor.set(JointConstants.JOINT_FORWARD_SPEED);
  }

  public void runJointBackward() {
    jointMotor.set(JointConstants.JOINT_REVERSE_SPEED);
  }

  public void stopJoint() {
    jointMotor.stopMotor();
    ;
  }

  public Double getSetPoint() {
    return setpoint;
  }

  public void JointTabs() {
    jointTab = Shuffleboard.getTab("Joint Tab");
    jointTab.addDouble("Setpoint", () -> getSetPoint());
  }
}
