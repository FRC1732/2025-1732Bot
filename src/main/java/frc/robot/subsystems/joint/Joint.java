// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.joint;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Joint extends SubsystemBase {
  /** Creates a new Joint. */
  private final SparkMax jointMotor;

  private JointPosition jointPosition;
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
    Logger.recordOutput(
        JointConstants.SUBSYSTEM_NAME + "/Joint Encoder Position", this.getEncoderPosition());
    Logger.recordOutput(JointConstants.SUBSYSTEM_NAME + "/Joint Set Point", this.getSetPoint());
    Logger.recordOutput(
        JointConstants.SUBSYSTEM_NAME + "/Joint Position", this.getJointPositionInt());
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

  public JointPosition getJointPosition() {
    return jointPosition;
  }

  public int getJointPositionInt() {
    int switchPositon = 0;

    switch (getJointPosition()) {
      case LEVEL_1 -> switchPositon = 1;
      case LEVEL_2 -> switchPositon = 2;
      case LEVEL_3 -> switchPositon = 3;
      case LEVEL_4 -> switchPositon = 4;
      case CORAL_STATION -> switchPositon = 5;
      default -> switchPositon = -1;
    }

    return switchPositon;
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
