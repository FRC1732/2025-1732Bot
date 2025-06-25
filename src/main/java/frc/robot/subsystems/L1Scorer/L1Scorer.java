// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.L1Scorer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class L1Scorer extends SubsystemBase {
  /** Creates a new L1Scorer. */
  private double l1ScorerSetpoint;

  private L1ScorerPose l1ScorerPose = L1ScorerPose.Start;

  private PIDController tiltPID;

  private SparkMax tiltMotor;
  private TalonFX intakeMotor;

  private RelativeEncoder tiltEncoder;

  public L1Scorer() {
    intakeMotor = new TalonFX(L1ScorerConstants.INTAKE_MOTOR_ID);
    tiltMotor = new SparkMax(L1ScorerConstants.TILT_MOTOR_ID, MotorType.kBrushless);
    tiltEncoder = tiltMotor.getEncoder();

    tiltPID =
        new PIDController(
            L1ScorerConstants.TILT_KP, L1ScorerConstants.TILT_KI, L1ScorerConstants.TILT_KD);
    tiltPID.setTolerance(L1ScorerConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    tiltPID.reset();

    intakeMotor.stopMotor();
    tiltMotor.stopMotor();

    setupNT();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      tiltPID.reset();
    }

    double output = tiltPID.calculate(getTiltPosition(), l1ScorerSetpoint);
    // intakeMotor.set(output);

    doLogging();
  }

  public void tiltForward() {
    tiltMotor.set(L1ScorerConstants.TILT_SPEED);
  }

  public void tiltBackwards() {
    tiltMotor.set(L1ScorerConstants.TILT_SPEED * -1);
  }

  public void stopTilt() {
    tiltMotor.set(0);
  }

  public void runIntake() {
    intakeMotor.set(L1ScorerConstants.INTAKE_SPEED);
  }

  public void ejectIntake() {
    intakeMotor.set(L1ScorerConstants.EJECT_SPEED);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public double getTiltPosition() {
    return tiltEncoder.getPosition();
  }

  public double getTiltVelocity() {
    return tiltEncoder.getVelocity();
  }

  public void setL1Pose(L1ScorerPose l1ScorerPose) {
    this.l1ScorerPose = l1ScorerPose;
    switch (this.l1ScorerPose) {
      case Start:
        l1ScorerSetpoint = L1ScorerConstants.START_ANGLE;
        break;
      case Intake:
        l1ScorerSetpoint = L1ScorerConstants.INTAKE_ANGLE;
        break;
      case Score:
        l1ScorerSetpoint = L1ScorerConstants.SCORE_ANGLE;
        break;
      case Hold:
        l1ScorerSetpoint = L1ScorerConstants.HOLD_ANGLE;
        break;
    }
    tiltPID.setSetpoint(l1ScorerSetpoint);
  }

  private void doLogging() {
    Logger.recordOutput(L1ScorerConstants.SUBSYSTEM_NAME + "/Tilt Position", getTiltPosition());
    Logger.recordOutput(L1ScorerConstants.SUBSYSTEM_NAME + "/Tilt Velocity", getTiltVelocity());
    Logger.recordOutput(L1ScorerConstants.SUBSYSTEM_NAME + "/Tilt Goal", l1ScorerSetpoint);
  }

  private void setupNT() {

    ShuffleboardTab tab = Shuffleboard.getTab(L1ScorerConstants.SUBSYSTEM_NAME);

    tab.addDouble("Tilt Position", this::getTiltPosition);
    tab.addDouble("Tilt Velocity", this::getTiltVelocity);

    tab.add("Tilt PID", tiltPID);
  }
}
