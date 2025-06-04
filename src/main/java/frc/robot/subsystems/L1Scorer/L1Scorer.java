// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.L1Scorer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class L1Scorer extends SubsystemBase {
  /** Creates a new L1Scorer. */
  private double L1ScorerSetpoint;

  private PIDController rotatePID;

  private SparkMax tiltMotor;
  private TalonFX intakeMotor;

  public L1Scorer() {
    intakeMotor = new TalonFX(L1ScorerConstants.INTAKE_MOTOR_ID);
    tiltMotor = new SparkMax(L1ScorerConstants.TILT_MOTOR_ID, null);

    rotatePID =
        new PIDController(
            L1ScorerConstants.INTAKE_KP, L1ScorerConstants.INTAKE_KI, L1ScorerConstants.INTAKE_KD);
    rotatePID.setTolerance(L1ScorerConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    rotatePID.reset();

    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tiltForward() {
    tiltMotor.set(0.20);
  }

  public void tiltBackwards() {
    tiltMotor.set(-0.20);
  }

  public void stopTilt() {
    tiltMotor.set(0);
  }

  public void runIntake() {
    intakeMotor.set(0.2);
  }

  public void ejectIntake() {
    intakeMotor.set(-0.2);
  }

}
