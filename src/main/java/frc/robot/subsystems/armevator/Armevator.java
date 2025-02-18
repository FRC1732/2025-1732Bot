// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Armevator extends SubsystemBase {
  /** Creates a new Armevator. */
  private HashMap<ArmevatorPose, Double> poseAngleMap;

  private HashMap<ArmevatorPose, Double> poseCarriageHeightMap;

  // private double armAngleSetpoint;
  // private double carriageHeightSetpoint;

  private ArmevatorPose pose;

  private int limitSwitchCounter;
  private boolean elevatorPIDOverride;

  private ProfiledPIDController armPID;
  private ArmFeedforward armFeedforward;

  private ProfiledPIDController elevatorPID;
  private ElevatorFeedforward elevatorHeightFeedforward;

  private SparkMax elevatorRightMotor;
  private SparkMax elevatorLeftMotor;
  private SparkMax armMotor;

  private DigitalInput elevatorLimitSwitch;
  private RelativeEncoder elevatorRelativeEncoder;

  private AbsoluteEncoder armAbsoluteEncoder;
  private RelativeEncoder armRelativeEncoder;

  public Armevator() {
    // setup poses
    poseAngleMap = new HashMap<>();

    poseAngleMap.put(ArmevatorPose.STARTING, 96.9);
    poseAngleMap.put(ArmevatorPose.CLIMB, -110.0);
    poseAngleMap.put(ArmevatorPose.CORAL_HP_LOAD, -125.0);
    poseAngleMap.put(ArmevatorPose.CORAL_L4_SCORE, 55.0);
    poseAngleMap.put(ArmevatorPose.CORAL_L3_SCORE, 45.0);
    poseAngleMap.put(ArmevatorPose.CORAL_L2_SCORE, 72.0);
    poseAngleMap.put(ArmevatorPose.CORAL_L1_SCORE, 80.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_INTAKE, 90.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_HANDOFF, 90.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_NET_SCORE, -55.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 25.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_L3_DROP, 5.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 25.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_L2_DROP, 5.0);

    poseCarriageHeightMap = new HashMap<>(); // in inches
    poseCarriageHeightMap.put(ArmevatorPose.STARTING, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.CLIMB, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_HP_LOAD, 3.25);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_L4_SCORE, 32.0);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_L3_SCORE, 4.5);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_L2_SCORE, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_L1_SCORE, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_INTAKE, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_HANDOFF, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_NET_SCORE, 32.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 15.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_L3_DROP, 20.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_L2_DROP, 4.5);

    // setup elevator motors
    elevatorRightMotor = new SparkMax(ArmevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    elevatorLeftMotor = new SparkMax(ArmevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.follow(elevatorRightMotor, true);

    rightConfig.idleMode(IdleMode.kBrake);
    leftConfig.idleMode(IdleMode.kBrake);

    elevatorRightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorLeftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // setup arm motor
    armMotor = new SparkMax(ArmevatorConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.idleMode(IdleMode.kBrake);

    armMotor.configure(
        armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // motor extras
    elevatorLimitSwitch = new DigitalInput(ArmevatorConstants.ELEVATOR_LIMIT_SWITCH_ID);
    elevatorRelativeEncoder = elevatorRightMotor.getEncoder();

    armRelativeEncoder = armMotor.getEncoder();
    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();

    elevatorHeightFeedforward =
        new ElevatorFeedforward(
            ArmevatorConstants.ELEVATOR_HEIGHT_KS,
            ArmevatorConstants.ELEVATOR_HEIGHT_KG,
            ArmevatorConstants.ELEVATOR_HEIGHT_KV,
            ArmevatorConstants.ELEVATOR_HEIGHT_KA);

    // elevatorPID =
    // new ProfiledPIDController(
    // ArmevatorConstants.ELEVATOR_KP,
    // ArmevatorConstants.ELEVATOR_KI,
    // ArmevatorConstants.ELEVATOR_KD,
    // new TrapezoidProfile.Constraints(
    // ArmevatorConstants.ELEVATOR_MAX_VELOCITY,
    // ArmevatorConstants.ELEVATOR_MAX_ACCELERATION),
    // ArmevatorConstants.ELEVATOR_PERIOD_SEC);

    armFeedforward =
        new ArmFeedforward(
            ArmevatorConstants.ARM_HEIGHT_KS,
            ArmevatorConstants.ARM_HEIGHT_KG,
            ArmevatorConstants.ARM_HEIGHT_KV,
            ArmevatorConstants.ARM_HEIGHT_KA);

    // armPID =
    // new ProfiledPIDController(
    // ArmevatorConstants.ARM_KP,
    // ArmevatorConstants.ARM_KI,
    // ArmevatorConstants.ARM_KD,
    // new TrapezoidProfile.Constraints(
    // ArmevatorConstants.ARM_MAX_VELOCITY,
    // ArmevatorConstants.ARM_MAX_ACCELERATION),
    // ArmevatorConstants.ARM_PERIOD_SEC);

  }

  public void setPose(ArmevatorPose pose) {
    this.pose = pose;
    // elevatorPID.setGoal(poseCarriageHeightMap.get(pose));
    // armPID.setGoal(poseAngleMap.get(pose));

    // armAngleSetpoint = poseAngleMap.get(pose);
    // carriageHeightSetpoint = poseCarriageHeightMap.get(pose);
  }

  public ArmevatorPose getPose() {
    return pose;
  }

  public boolean isAtGoal() {
    // return armPID.atGoal() && elevatorPID.atGoal();
    return false;
  }

  public void resetToAbsoluteEncoder() {
    armRelativeEncoder.setPosition(getAbsolutePosition());
  }

  // motion validation methods. Remove once verified
  public void runArmForward() {
    armMotor.set(0.1);
  }

  public void runArmBackward() {
    armMotor.set(-0.1);
  }

  public void stopJoint() {
    armMotor.stopMotor();
  }

  public void runElevatorUp() {
    elevatorRightMotor.set(0.1);
  }

  public void runElevatorDown() {
    elevatorRightMotor.set(-0.1);
  }

  public void stopElevator() {
    elevatorRightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // if (DriverStation.isDisabled()) {
    // armPID.reset(armRelativeEncoder.getPosition());
    // }

    if (elevatorLimitSwitch.get()) {
      limitSwitchCounter++;
    } else {
      limitSwitchCounter = 0;
    }

    // turn off elevator when limit switch is pressed, leave it off if goal isn't
    // changed
    // if (elevatorPID.getGoal().position != 0) {
    // elevatorPIDOverride = false;
    // } else if (limitSwitchCounter > 10) {
    // elevatorPIDOverride = true;
    // }

    // if (elevatorPIDOverride) {
    // elevatorRightMotor.stopMotor();
    // elevatorRelativeEncoder.setPosition(0);
    // } else {
    // elevatorRightMotor.set(
    // elevatorPID.calculate(elevatorRelativeEncoder.getPosition())
    // +
    // elevatorHeightFeedforward.calculate(elevatorRelativeEncoder.getVelocity()));
    // }

    // armMotor.set(
    // MathUtil.clamp(armPID.calculate(armRelativeEncoder.getPosition()), -0.5, 0.5)
    // + armFeedforward.calculate(
    // Math.toRadians(
    // armRelativeEncoder.getPosition() + ArmevatorConstants.ARM_COG_OFFSET),
    // armRelativeEncoder.getVelocity()));

    doLogging();
    setupNT();
  }

  private double getAbsolutePosition() {
    return armAbsoluteEncoder.getPosition(); // TODO: needs an offset
  }

  private double getAbsoluteDegrees() {
    return ArmevatorConstants.ARM_COG_OFFSET * 360 - getAbsolutePosition() * 360;
  }

  private void doLogging() {
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Position",
        elevatorRelativeEncoder.getPosition());
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Velocity",
        elevatorRelativeEncoder.getVelocity());
    // Logger.recordOutput(
    // ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Goal",
    // elevatorPID.getGoal().position);

    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Position", armRelativeEncoder.getPosition());
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Velocity", armRelativeEncoder.getVelocity());
    // Logger.recordOutput(ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Goal",
    // armPID.getGoal().position);

    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Limit Switch", elevatorLimitSwitch.get());
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Absolute Encoder", getAbsolutePosition());
  }

  private void setupNT() {
    // SmartDashboard.putData(ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator PID",
    // elevatorPID);
    // SmartDashboard.putData(ArmevatorConstants.SUBSYSTEM_NAME + "/Arm PID",
    // armPID);

    SmartDashboard.putNumber(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Position",
        elevatorRelativeEncoder.getPosition());
    SmartDashboard.putNumber(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Velocity",
        elevatorRelativeEncoder.getVelocity());
    // SmartDashboard.putNumber(
    // ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Goal",
    // elevatorPID.getGoal().position);

    SmartDashboard.putNumber(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Position", armRelativeEncoder.getPosition());
    SmartDashboard.putNumber(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Velocity", armRelativeEncoder.getVelocity());

    // SmartDashboard.putNumber(
    // ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Goal", armPID.getGoal().position);

    SmartDashboard.putBoolean(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Limit Switch", elevatorLimitSwitch.get());
    SmartDashboard.putNumber(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Absolute Encoder", getAbsolutePosition());
    SmartDashboard.putNumber(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Absolute Degrees", getAbsoluteDegrees());

    // Does recordOutput also make these available from network tables?
  }
}
