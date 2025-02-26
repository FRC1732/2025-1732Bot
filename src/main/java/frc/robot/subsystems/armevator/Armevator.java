// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Armevator extends SubsystemBase {
  /** Creates a new Armevator. */
  private Map<ArmevatorPose, Double> armMap;

  private Map<ArmevatorPose, Double> elevatorMap;

  // private double armAngleSetpoint;
  // private double carriageHeightSetpoint;

  private ArmevatorPose currentPose;

  private int limitSwitchCounter;
  private boolean elevatorPIDOverride;

  private ProfiledPIDController armPID;
  private ArmFeedforward armFeedforward;

  private ProfiledPIDController elevatorPID;
  private ElevatorFeedforward elevatorFeedforward;
  private SparkLimitSwitch elevatorLimitSwitch;

  private SparkMax elevatorRightMotor;
  private SparkMax elevatorLeftMotor;
  private SparkMax armMotor;

  private RelativeEncoder elevatorEncoder;

  private AbsoluteEncoder armAbsoluteEncoder;
  private RelativeEncoder armRelativeEncoder;

  // tuning variables

  private NetworkTableInstance table = NetworkTableInstance.getDefault();
  private NetworkTable networkTable = table.getTable("ArmevatorConstants");

  private GenericEntry subscriberArmVelocity =
      networkTable.getTopic("armMaxVelocity").getGenericEntry();
  private GenericEntry subscriberArmMaxAcceleration =
      networkTable.getTopic("armMaxAcceleration").getGenericEntry();
  private GenericEntry subscriberArmGoalTolerance =
      networkTable.getTopic("armGoalTolerance").getGenericEntry();
  private GenericEntry subscriberArmKG = networkTable.getTopic("armKG").getGenericEntry();

  private GenericEntry subscriberElevatorVelocity =
      networkTable.getTopic("elevatorMaxVelocity").getGenericEntry();
  private GenericEntry subscriberElevatormMaxAcceleration =
      networkTable.getTopic("elevatorMaxAcceleration").getGenericEntry();
  private GenericEntry subscriberElevatorGoalTolerance =
      networkTable.getTopic("elevatorGoalTolerance").getGenericEntry();
  private GenericEntry subscriberElevatorKG = networkTable.getTopic("elevatorKG").getGenericEntry();

  public Armevator() {
    // ensure network tables are visible on elastic (unsure if this is needed)
    subscriberArmVelocity.setDouble(ArmevatorConstants.ARM_MAX_VELOCITY);
    subscriberArmMaxAcceleration.setDouble(ArmevatorConstants.ARM_MAX_ACCELERATION);
    subscriberArmGoalTolerance.setDouble(ArmevatorConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    subscriberArmKG.setDouble(ArmevatorConstants.ARM_KG);

    subscriberElevatorVelocity.setDouble(ArmevatorConstants.ELEVATOR_MAX_VELOCITY);
    subscriberElevatormMaxAcceleration.setDouble(ArmevatorConstants.ELEVATOR_MAX_ACCELERATION);
    subscriberElevatorGoalTolerance.setDouble(ArmevatorConstants.HEIGHT_GOAL_TOLERANCE_INCHES);
    subscriberElevatorKG.setDouble(ArmevatorConstants.ELEVATOR_KG);

    // setup poses
    armMap = new HashMap<>();
    armMap.put(ArmevatorPose.STARTING, 96.9);
    armMap.put(ArmevatorPose.CLIMB, -110.0);
    armMap.put(ArmevatorPose.CORAL_HP_LOAD, -125.0);
    armMap.put(ArmevatorPose.CORAL_L4_SCORE, 55.0);
    armMap.put(ArmevatorPose.CORAL_L3_SCORE, 45.0);
    armMap.put(ArmevatorPose.CORAL_L2_SCORE, 72.0);
    armMap.put(ArmevatorPose.CORAL_L1_SCORE, 80.0);
    armMap.put(ArmevatorPose.CORAL_POST_SCORE, 55.0);
    armMap.put(ArmevatorPose.ALGAE_INTAKE, 90.0);
    armMap.put(ArmevatorPose.ALGAE_HANDOFF, 90.0);
    armMap.put(ArmevatorPose.ALGAE_NET_SCORE, -70.0);
    armMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 15.0);
    armMap.put(ArmevatorPose.ALGAE_L3_DROP, 5.0);
    armMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 15.0);
    armMap.put(ArmevatorPose.ALGAE_L2_DROP, 5.0);

    elevatorMap = new HashMap<>(); // in inches
    elevatorMap.put(ArmevatorPose.STARTING, 0.0);
    elevatorMap.put(ArmevatorPose.CLIMB, 0.0);
    elevatorMap.put(ArmevatorPose.CORAL_HP_LOAD, 4.0);
    elevatorMap.put(ArmevatorPose.CORAL_L4_SCORE, 32.0);
    elevatorMap.put(ArmevatorPose.CORAL_L3_SCORE, 4.5);
    elevatorMap.put(ArmevatorPose.CORAL_L2_SCORE, 0.0);
    elevatorMap.put(ArmevatorPose.CORAL_L1_SCORE, 0.0);
    elevatorMap.put(ArmevatorPose.CORAL_POST_SCORE, 4.0);
    elevatorMap.put(ArmevatorPose.ALGAE_INTAKE, 0.0);
    elevatorMap.put(ArmevatorPose.ALGAE_HANDOFF, 0.0);
    elevatorMap.put(ArmevatorPose.ALGAE_NET_SCORE, 32.0);
    elevatorMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 18.0);
    elevatorMap.put(ArmevatorPose.ALGAE_L3_DROP, 20.0);
    elevatorMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 3.0);
    elevatorMap.put(ArmevatorPose.ALGAE_L2_DROP, 4.5);

    /////////////////////////
    // setup elevator motors
    /////////////////////////
    elevatorRightMotor = new SparkMax(ArmevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    elevatorLeftMotor = new SparkMax(ArmevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig leftConfig = new SparkMaxConfig();

    leftConfig.follow(elevatorRightMotor, true);

    rightConfig.idleMode(IdleMode.kBrake);
    leftConfig.idleMode(IdleMode.kBrake);

    rightConfig.voltageCompensation(12.0);
    leftConfig.voltageCompensation(12.0);

    rightConfig.softLimit.forwardSoftLimitEnabled(true);
    rightConfig.softLimit.reverseSoftLimitEnabled(true);
    rightConfig.softLimit.forwardSoftLimit(ArmevatorConstants.MAX_HEIGHT_INCHES);
    rightConfig.softLimit.reverseSoftLimit(ArmevatorConstants.MIN_HEIGHT_INCHES);

    rightConfig.encoder.positionConversionFactor(ArmevatorConstants.ELEVATOR_INCHES_PER_ROTATION);
    rightConfig.encoder.velocityConversionFactor(
        ArmevatorConstants.ELEVATOR_RPM_TO_INCHES_PER_SECOND);

    rightConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
    rightConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    elevatorRightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorLeftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorEncoder = elevatorRightMotor.getEncoder();
    elevatorEncoder.setPosition(0.0);

    elevatorLimitSwitch = elevatorRightMotor.getReverseLimitSwitch();

    elevatorFeedforward =
        new ElevatorFeedforward(
            ArmevatorConstants.ELEVATOR_KS,
            ArmevatorConstants.ELEVATOR_KG,
            ArmevatorConstants.ELEVATOR_KV,
            ArmevatorConstants.ELEVATOR_KA);

    elevatorPID =
        new ProfiledPIDController(
            ArmevatorConstants.ELEVATOR_KP,
            ArmevatorConstants.ELEVATOR_KI,
            ArmevatorConstants.ELEVATOR_KD,
            new TrapezoidProfile.Constraints(
                ArmevatorConstants.ELEVATOR_MAX_VELOCITY,
                ArmevatorConstants.ELEVATOR_MAX_ACCELERATION),
            ArmevatorConstants.PID_PERIOD_SEC);
    elevatorPID.setTolerance(ArmevatorConstants.HEIGHT_GOAL_TOLERANCE_INCHES);
    elevatorPID.reset(elevatorEncoder.getPosition());
    elevatorPID.setGoal(0.0);

    ////////////////////
    // setup arm motors
    ////////////////////
    armMotor = new SparkMax(ArmevatorConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.inverted(true);
    armConfig.idleMode(IdleMode.kBrake);
    armConfig.voltageCompensation(12.0);

    armConfig.softLimit.forwardSoftLimitEnabled(true);
    armConfig.softLimit.reverseSoftLimitEnabled(true);
    armConfig.softLimit.forwardSoftLimit(ArmevatorConstants.MAX_ANGLE_DEGREES);
    armConfig.softLimit.reverseSoftLimit(ArmevatorConstants.MIN_ANGLE_DEGREES);

    armConfig.encoder.positionConversionFactor(ArmevatorConstants.ARM_DEGREES_PER_ROTATION);
    armConfig.encoder.velocityConversionFactor(ArmevatorConstants.ARM_RPM_TO_DEGREES_PER_SECOND);

    armMotor.configure(
        armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    armRelativeEncoder = armMotor.getEncoder();
    armRelativeEncoder.setPosition(armMap.get(ArmevatorPose.STARTING));
    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();

    armFeedforward =
        new ArmFeedforward(
            ArmevatorConstants.ARM_KS,
            ArmevatorConstants.ARM_KG,
            ArmevatorConstants.ARM_KV,
            ArmevatorConstants.ARM_KA);

    armPID =
        new ProfiledPIDController(
            ArmevatorConstants.ARM_KP,
            ArmevatorConstants.ARM_KI,
            ArmevatorConstants.ARM_KD,
            new TrapezoidProfile.Constraints(
                ArmevatorConstants.ARM_MAX_VELOCITY, ArmevatorConstants.ARM_MAX_ACCELERATION),
            ArmevatorConstants.PID_PERIOD_SEC);
    armPID.setTolerance(ArmevatorConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    armPID.reset(armRelativeEncoder.getPosition());
    armPID.setGoal(armMap.get(ArmevatorPose.STARTING));

    elevatorRightMotor.stopMotor();
    armMotor.stopMotor();

    setupNT();
  }

  public void setTargetPose(ArmevatorPose pose) {
    this.currentPose = pose;
    elevatorPID.setGoal(elevatorMap.get(pose));
    armPID.setGoal(armMap.get(pose));
  }

  public void updateScoringLevel(ArmevatorPose pose) {
    if (currentPose == ArmevatorPose.CORAL_L1_SCORE
        || currentPose == ArmevatorPose.CORAL_L2_SCORE
        || currentPose == ArmevatorPose.CORAL_L3_SCORE
        || currentPose == ArmevatorPose.CORAL_L4_SCORE) {
      setTargetPose(pose);
    }
  }

  public ArmevatorPose getCurrentPose() {
    return currentPose;
  }

  public boolean isAtGoal() {
    return elevatorPID.atGoal() && armPID.atGoal();
  }

  public void resetToAbsoluteEncoder() {
    armRelativeEncoder.setPosition(getAbsoluteDegrees());
  }

  public void doConstantChecks() {
    double newArmMaxVelocity = subscriberArmVelocity.getDouble(ArmevatorConstants.ARM_MAX_VELOCITY);
    double newArmMaxAcceleration =
        subscriberArmMaxAcceleration.getDouble(ArmevatorConstants.ARM_MAX_ACCELERATION);

    if (armPID.getConstraints().maxVelocity != newArmMaxVelocity
        || armPID.getConstraints().maxAcceleration != newArmMaxAcceleration) {
      armPID.setConstraints(new Constraints(newArmMaxVelocity, newArmMaxAcceleration));
      System.out.println(
          "Updated arm velocity and accel: " + newArmMaxVelocity + ", " + newArmMaxAcceleration);
    }

    double setGoalTolerance =
        subscriberArmGoalTolerance.getDouble(ArmevatorConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    if (armPID.getPositionTolerance() != setGoalTolerance) {
      armPID.setTolerance(setGoalTolerance);
      System.out.println("Updated arm degree tolerance: " + setGoalTolerance);
    }

    double setArmKG = subscriberArmKG.getDouble(ArmevatorConstants.ARM_KG);
    if (armFeedforward.getKg() != setArmKG) {
      armFeedforward =
          new ArmFeedforward(
              ArmevatorConstants.ARM_KS,
              setArmKG,
              ArmevatorConstants.ARM_KV,
              ArmevatorConstants.ARM_KA);

      System.out.println("Updated arm KG: " + setArmKG);
    }

    double newElevatorMaxVelocity =
        subscriberElevatorVelocity.getDouble(ArmevatorConstants.ELEVATOR_MAX_VELOCITY);
    double newElevatorMaxAcceleration =
        subscriberElevatormMaxAcceleration.getDouble(ArmevatorConstants.ELEVATOR_MAX_ACCELERATION);
    if (elevatorPID.getConstraints().maxVelocity != newElevatorMaxVelocity
        || elevatorPID.getConstraints().maxAcceleration != newElevatorMaxAcceleration) {
      elevatorPID.setConstraints(
          new Constraints(newElevatorMaxVelocity, newElevatorMaxAcceleration));
      System.out.println(
          "Updated elevator velocity and accel: "
              + newElevatorMaxVelocity
              + ", "
              + newElevatorMaxAcceleration);
    }

    double setElevatorGoalTolerance =
        subscriberElevatorGoalTolerance.getDouble(ArmevatorConstants.HEIGHT_GOAL_TOLERANCE_INCHES);
    if (elevatorPID.getPositionTolerance() != setElevatorGoalTolerance) {
      elevatorPID.setTolerance(setElevatorGoalTolerance);
      System.out.println("Updated elevator height tolerance: " + setElevatorGoalTolerance);
    }

    double setElevatorKG = subscriberElevatorKG.getDouble(ArmevatorConstants.ELEVATOR_KG);
    if (elevatorFeedforward.getKg() != setElevatorKG) {
      elevatorFeedforward =
          new ElevatorFeedforward(
              ArmevatorConstants.ELEVATOR_KS,
              setElevatorKG,
              ArmevatorConstants.ELEVATOR_KV,
              ArmevatorConstants.ELEVATOR_KA);

      System.out.println("Updated elevator KG: " + setElevatorKG);
    }
  }

  // 250, 900, 0.01

  @Override
  public void periodic() {
    // doConstantChecks();

    if (DriverStation.isDisabled()) {
      elevatorPID.reset(elevatorEncoder.getPosition());
      armPID.reset(armRelativeEncoder.getPosition());
    }

    // filter out false positives
    if (elevatorLimitSwitch.isPressed()) {
      limitSwitchCounter++;
    } else {
      limitSwitchCounter = 0;
    }

    // turn off elevator when limit switch is pressed, leave it off if goal isn't
    // changed
    if (elevatorPID.getGoal().position != 0) {
      elevatorPIDOverride = false;
    } else if (limitSwitchCounter > 10) {
      elevatorPIDOverride = true;
    }

    if (elevatorPIDOverride) {
      elevatorRightMotor.stopMotor();
      elevatorEncoder.setPosition(0);
    } else {
      elevatorRightMotor.set(
          elevatorPID.calculate(elevatorEncoder.getPosition())
              + elevatorFeedforward.calculate(elevatorEncoder.getVelocity()));
    }

    armMotor.set(
        MathUtil.clamp(armPID.calculate(armRelativeEncoder.getPosition()), -0.5, 0.5)
            + armFeedforward.calculate(
                MathUtil.angleModulus(
                    Math.toRadians(
                        armRelativeEncoder.getPosition()
                            + ArmevatorConstants.ANGLE_COG_OFFSET
                            + 90.0)),
                armRelativeEncoder.getVelocity()));

    doLogging();
  }

  private double getAbsolutePosition() {
    return armAbsoluteEncoder.getPosition();
  }

  private double getAbsoluteDegrees() {
    return Rotation2d.fromDegrees(ArmevatorConstants.ANGLE_ABSOLUTE_OFFSET_DEG)
        .minus(Rotation2d.fromRotations(getAbsolutePosition()))
        .getDegrees();
  }

  private void doLogging() {
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Position", elevatorEncoder.getPosition());
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Velocity", elevatorEncoder.getVelocity());
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Goal", elevatorPID.getGoal().position);

    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Position", armRelativeEncoder.getPosition());
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Velocity", armRelativeEncoder.getVelocity());
    Logger.recordOutput(ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Goal", armPID.getGoal().position);

    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Elevator Limit Switch",
        elevatorLimitSwitch.isPressed());
    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Absolute Encoder", getAbsolutePosition());

    Logger.recordOutput(
        ArmevatorConstants.SUBSYSTEM_NAME + "/Arm Absolute Degrees", getAbsoluteDegrees());
  }

  private void setupNT() {
    ShuffleboardTab tab = Shuffleboard.getTab(ArmevatorConstants.SUBSYSTEM_NAME);

    tab.add("Elevator PID", elevatorPID);
    tab.add("Arm PID", armPID);

    tab.addDouble("Elevator Position", () -> elevatorEncoder.getPosition());
    tab.addDouble("Elevator Velocity", () -> elevatorEncoder.getVelocity());
    tab.addDouble("Elevator Goal", () -> elevatorPID.getGoal().position);

    tab.addDouble("Arm Position", () -> armRelativeEncoder.getPosition());
    tab.addDouble("Arm Velocity", () -> armRelativeEncoder.getVelocity());

    tab.addDouble("Arm Goal", () -> armPID.getGoal().position);

    tab.addBoolean("Elevator Limit Switch", () -> elevatorLimitSwitch.isPressed());
    tab.addDouble("Arm Absolute Encoder", () -> getAbsolutePosition());
    tab.addDouble("Arm Absolute Degrees", () -> getAbsoluteDegrees());
  }

  private double angleModulusDeg(double angleDeg) {
    return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(angleDeg)));
  }
}
