// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.joint;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joint extends SubsystemBase {
  /** Creates a new Joint. */
  private final SparkMax jointMotor;

  private ShuffleboardTab tab;
  private Double setpoint;
  private RelativeEncoder encoder;

  public Joint() {
    jointMotor = new SparkMax(JointConstants.JOINT_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);
    jointMotor.stopMotor();
    encoder = jointMotor.getEncoder();
    setpoint = 0.0;
    setupShuffleboard();
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

  public void setSetPoint(double newSetpoint) {
    setpoint = newSetpoint;
    jointMotor.set(newSetpoint);
  }

  public void runJointForward() {
    setSetPoint(JointConstants.JOINT_FORWARD_SPEED);
  }

  public void runJointBackward() {
    setSetPoint(JointConstants.JOINT_REVERSE_SPEED);
  }

  public void stopJoint() {
    setSetPoint(JointConstants.JOINT_STOPPED_SPEED);
  }

  public Double getSetPoint() {
    return setpoint;
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public void setupShuffleboard() {
    tab = Shuffleboard.getTab("Joint Tab");
    tab.addDouble("Setpoint", this::getSetPoint);
    tab.addDouble("Encoder Position", this::getEncoderPosition);
  }
}
