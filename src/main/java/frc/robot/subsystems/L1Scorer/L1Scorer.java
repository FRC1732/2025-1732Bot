// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.L1Scorer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class L1Scorer extends SubsystemBase {
  /** Creates a new L1Scorer. */
  private double L1ScorerSetpoint;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
  
}
