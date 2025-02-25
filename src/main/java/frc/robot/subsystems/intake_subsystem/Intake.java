// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake_subsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.armevator.ArmevatorPose;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private HashMap<ArmevatorPose, Double> intakeMap;
  private double algaeAngleSetpoint;

  private ProfiledPIDController intakePID;
  private ArmFeedforward intakeFeedforward;

  private RelativeEncoder tiltEncoder;

  private TalonFX rollerMotor;
  private TalonFX intakeMotor;

  private ArmevatorPose pose;

  private NetworkTableInstance table = NetworkTableInstance.getDefault();

  // private NetworkTable networkTable = table.getTable("IntakeConstants");

  // private GenericEntry subscriberIntakeVelocity =
  //     networkTable.getTopic("intakeMaxVelocity").getGenericEntry();
  // private GenericEntry subscriberIntakeMaxAcceleration =
  //     networkTable.getTopic("intakeMaxAcceleration").getGenericEntry();
  // private GenericEntry subscriberIntakeGoalTolerance =
  //     networkTable.getTopic("intakeGoalTolerance").getGenericEntry();
  // private GenericEntry subscriberIntakeKG = networkTable.getTopic("intakeKG").getGenericEntry();

  public Intake() {
    // ensure network tables are visible on elastic (unsure if this is needed)
    // subscriberIntakeVelocity.setDouble(IntakeConstants.INTAKE_MAX_VELOCITY);
    // subscriberIntakeMaxAcceleration.setDouble(IntakeConstants.INTAKE_MAX_ACCELERATION);
    // subscriberIntakeGoalTolerance.setDouble(IntakeConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    // subscriberIntakeKG.setDouble(IntakeConstants.INTAKE_KG);

    rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    intakeMotor = new TalonFX(IntakeConstants.TILT_MOTOR_ID);

    intakeMap = new HashMap<>();
    intakeMap.put(ArmevatorPose.STARTING, -4.0);
    intakeMap.put(ArmevatorPose.CLIMB, 35.0);
    intakeMap.put(ArmevatorPose.CORAL_HP_LOAD, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L4_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L3_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L2_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L1_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_POST_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.ALGAE_INTAKE, 55.0);
    intakeMap.put(ArmevatorPose.ALGAE_HANDOFF, 10.0);
    intakeMap.put(ArmevatorPose.ALGAE_NET_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 5.0);
    intakeMap.put(ArmevatorPose.ALGAE_L3_DROP, 15.0);
    intakeMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 5.0);
    intakeMap.put(ArmevatorPose.ALGAE_L2_DROP, 10.0);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    intakeConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    intakeConfig.Voltage.withPeakForwardVoltage(12.0);
    intakeConfig.Voltage.withPeakReverseVoltage(-12.0);
    intakeConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    intakeConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    intakeConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
        IntakeConstants.MAX_ANGLE_DEGREES / IntakeConstants.INTAKE_DEGREES_PER_ROTATION);
    intakeConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(
        IntakeConstants.MIN_ANGLE_DEGREES / IntakeConstants.INTAKE_DEGREES_PER_ROTATION);
    intakeConfig.CurrentLimits.withStatorCurrentLimit(40.0);

    intakeMotor.getConfigurator().apply(intakeConfig);

    intakeMotor.setPosition(
        intakeMap.get(ArmevatorPose.STARTING) / IntakeConstants.INTAKE_DEGREES_PER_ROTATION);

    intakeFeedforward =
        new ArmFeedforward(
            IntakeConstants.INTAKE_KS,
            IntakeConstants.INTAKE_KG,
            IntakeConstants.INTAKE_KV,
            IntakeConstants.INTAKE_KA);

    intakePID =
        new ProfiledPIDController(
            IntakeConstants.INTAKE_KP,
            IntakeConstants.INTAKE_KI,
            IntakeConstants.INTAKE_KD,
            new TrapezoidProfile.Constraints(
                IntakeConstants.INTAKE_MAX_VELOCITY, IntakeConstants.INTAKE_MAX_ACCELERATION),
            IntakeConstants.PID_PERIOD_SEC);
    intakePID.setTolerance(IntakeConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    intakePID.reset(intakeMap.get(ArmevatorPose.STARTING));
    intakePID.setGoal(intakeMap.get(ArmevatorPose.CORAL_L1_SCORE));

    intakeMotor.stopMotor();

    setupNT();
  }

  public boolean isAtGoal() {
    return intakePID.atGoal();
  }

  public void runIntake() {
    rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED);
  }

  public void ejectIntake() {
    rollerMotor.set(IntakeConstants.ROLLER_EJECT_SPEED);
  }

  public void tiltForward() {
    intakeMotor.set(0.3);
  }

  public void tiltBackwards() {
    intakeMotor.set(-0.3);
  }

  public void stopTilt() {
    intakeMotor.set(0);
  }

  public void stopIntake() {
    rollerMotor.set(0);
  }

  public double getTiltPosition() {
    return intakeMotor.getPosition().getValueAsDouble();
  }

  public double getTiltVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  // public void doConstantChecks() {
  //   double newIntakeMaxVelocity =
  //       subscriberIntakeVelocity.getDouble(IntakeConstants.INTAKE_MAX_VELOCITY);
  //   double newIntakeMaxAcceleration =
  //       subscriberIntakeMaxAcceleration.getDouble(IntakeConstants.INTAKE_MAX_ACCELERATION);

  //   if (intakePID.getConstraints().maxVelocity != newIntakeMaxVelocity
  //       || intakePID.getConstraints().maxAcceleration != newIntakeMaxAcceleration) {
  //     intakePID.setConstraints(new Constraints(newIntakeMaxVelocity, newIntakeMaxAcceleration));
  //     System.out.println(
  //         "Updated intake velocity and accel: "
  //             + newIntakeMaxVelocity
  //             + ", "
  //             + newIntakeMaxAcceleration);
  //   }

  //   double setGoalTolerance =
  //       subscriberIntakeGoalTolerance.getDouble(IntakeConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
  //   if (intakePID.getPositionTolerance() != setGoalTolerance) {
  //     intakePID.setTolerance(setGoalTolerance);
  //     System.out.println("Updated intake degree tolerance: " + setGoalTolerance);
  //   }

  //   double setIntakeKG = subscriberIntakeKG.getDouble(IntakeConstants.INTAKE_KG);
  //   if (intakeFeedforward.getKg() != setIntakeKG) {
  //     intakeFeedforward =
  //         new ArmFeedforward(
  //             IntakeConstants.INTAKE_KS,
  //             setIntakeKG,
  //             IntakeConstants.INTAKE_KV,
  //             IntakeConstants.INTAKE_KA);

  //     System.out.println("Updated intake KG: " + setIntakeKG);
  //   }
  // }

  @Override
  public void periodic() {
    // doConstantChecks();

    if (DriverStation.isDisabled()) {
      intakePID.reset(getAngle());
    }

    intakeMotor.set(
        MathUtil.clamp(intakePID.calculate(getAngle()), -0.5, 0.5)
            + intakeFeedforward.calculate(
                MathUtil.angleModulus(Math.toRadians(getAngle() + 90.0)), getVelocity()));

    doLogging();
  }

  public double getAngle() {
    return new Rotation2d(intakeMotor.getPosition().getValue()).getRotations()
        * IntakeConstants.INTAKE_DEGREES_PER_ROTATION;
  }

  public double getVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble()
        * IntakeConstants.INTAKE_RPM_TO_DEGREES_PER_SECOND
        * 60;
  }

  public void setTargetPose(ArmevatorPose pose) {
    this.pose = pose;
    intakePID.setGoal(intakeMap.get(pose));
  }

  public ArmevatorPose getPose() {
    return pose;
  }

  private void doLogging() {
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Position", getTiltPosition());
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Velocity", getTiltVelocity());
    Logger.recordOutput(
        IntakeConstants.SUBSYSTEM_NAME + "/Tilt Goal", intakePID.getGoal().position);
  }

  private void setupNT() {

    ShuffleboardTab tab = Shuffleboard.getTab(IntakeConstants.SUBSYSTEM_NAME);

    tab.addDouble("Tilt Position", this::getAngle);
    tab.addDouble("Tilt Velocity", this::getVelocity);
    tab.add("Tilt PID", intakePID);
  }
}
