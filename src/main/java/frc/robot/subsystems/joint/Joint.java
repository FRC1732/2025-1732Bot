// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.joint;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joint extends SubsystemBase {
  /** Creates a new Joint. */
  private final SparkMax jointMotor;

  private ShuffleboardTab jointTab;
  private Double setpoint;
  private RelativeEncoder jointEncoder;
  private DutyCycleEncoder jointAbsoluteEncoder;
  private ProfiledPIDController jointPID;
  private ArmFeedforward jointFeedforward;

  public Joint() {
    jointMotor = new SparkMax(JointConstants.JOINT_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);
    jointAbsoluteEncoder = new DutyCycleEncoder(8);
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(JointConstants.JOINT_DEGREES_PER_ROTATION)
        .velocityConversionFactor(JointConstants.JOINT_RPM_TO_DEGREES_PER_SECOND);
    // config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 2, 3);

    jointMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    jointMotor.stopMotor();
    jointEncoder = jointMotor.getEncoder();
    jointEncoder.setPosition(angleModulusDeg(JointConstants.JOINT_START_SETPOINT));

    jointPID =
        new ProfiledPIDController(
            JointConstants.JOINT_KP,
            JointConstants.JOINT_KI,
            JointConstants.JOINT_KD,
            new TrapezoidProfile.Constraints(
                JointConstants.JOINT_MAX_VELOCITY, JointConstants.JOINT_MAX_ACCELERATION),
            JointConstants.JOINT_PID_PERIOD_SEC);
    jointPID.setTolerance(JointConstants.JOINT_GOAL_TOLERANCE_DEGREES);
    jointPID.reset(jointEncoder.getPosition());
    jointPID.setGoal(JointConstants.JOINT_START_SETPOINT);

    jointFeedforward =
        new ArmFeedforward(
            JointConstants.JOINT_KS,
            JointConstants.JOINT_KG,
            JointConstants.JOINT_KV,
            JointConstants.JOINT_KA);
  }

  public enum JointPosition {
    START_POSITION,
    CORAL_STATION,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4,
  }

  public void setJointPose(JointPosition jointPosition) {
    switch (jointPosition) {
      case START_POSITION:
        jointPID.setGoal(JointConstants.JOINT_START_SETPOINT);
        break;

      case CORAL_STATION:
        jointPID.setGoal(JointConstants.JOINT_CORAL_SETPOINT);
        break;

      case LEVEL_1:
        jointPID.setGoal(JointConstants.JOINT_LV1_SETPOINT);
        break;

      case LEVEL_2:
        jointPID.setGoal(JointConstants.JOINT_LV2_SETPOINT);
        break;

      case LEVEL_3:
        jointPID.setGoal(JointConstants.JOINT_LV3_SETPOINT);
        break;

      case LEVEL_4:
        jointPID.setGoal(JointConstants.JOINT_LV4_SETPOINT);
        break;

      default:
        break;
    }
  }

  public boolean isAtGoal() {
    return jointPID.atGoal();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      jointPID.reset(jointEncoder.getPosition());
    }

    jointMotor.set(
        MathUtil.clamp(jointPID.calculate(jointEncoder.getPosition()), -0.5, 0.5)
            + jointFeedforward.calculate(
                Math.toRadians(jointEncoder.getPosition() + JointConstants.JOINT_COG_OFFSET),
                jointEncoder.getVelocity()));
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

  private double angleModulusDeg(double angleDeg) {
    return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(angleDeg)));
  }
}
