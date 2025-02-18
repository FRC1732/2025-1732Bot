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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.armevator.ArmevatorConstants;
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

  public Intake() {
    rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    intakeMotor = new TalonFX(IntakeConstants.TILT_MOTOR_ID);

    intakeMap = new HashMap<>();
    intakeMap.put(ArmevatorPose.STARTING, 0.0);
    intakeMap.put(ArmevatorPose.CLIMB, 35.0);
    intakeMap.put(ArmevatorPose.CORAL_HP_LOAD, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L4_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L3_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L2_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L1_SCORE, 5.0);
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
    intakePID.setTolerance(ArmevatorConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    intakePID.reset(intakeMap.get(ArmevatorPose.STARTING));
    intakePID.setGoal(intakeMap.get(ArmevatorPose.STARTING));

    intakeMotor.setPosition(intakeMap.get(ArmevatorPose.STARTING));

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
    intakeMotor.set(0.1);
  }

  public void tiltBackwards() {
    intakeMotor.set(-0.1);
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

  @Override
  public void periodic() {
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
    return new Rotation2d(intakeMotor.getPosition().getValue()).getDegrees()
        * IntakeConstants.INTAKE_DEGREES_PER_ROTATION;
  }

  public double getVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble()
        * IntakeConstants.INTAKE_RPM_TO_DEGREES_PER_SECOND;
  }

  public void setPose(ArmevatorPose pose) {
    this.pose = pose;
    intakePID.setGoal(intakeMap.get(pose));
  }

  public ArmevatorPose getPose() {
    return pose;
  }

  private void doLogging() {
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Position", getTiltPosition());
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Velocity", getTiltVelocity());
    // Logger.recordOutput(
    //     IntakeConstants.SUBSYSTEM_NAME + "/Tilt Goal", intakePID.getGoal().position);
  }

  private void setupNT() {
    // SmartDashboard.putData(IntakeConstants.SUBSYSTEM_NAME + "/Tilt PID", intakePID);

    SmartDashboard.putNumber(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Position", getTiltPosition());
    SmartDashboard.putNumber(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Velocity", getTiltVelocity());
    // SmartDashboard.putNumber(
    //     IntakeConstants.SUBSYSTEM_NAME + "/Tilt Goal", intakePID.getGoal().position);
  }
}
