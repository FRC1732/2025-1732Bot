// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  public final SparkMax clawMotor;

  public Claw() {
    clawMotor = new SparkMax(ClawConstants.Claw_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);
    Timer.delay(0.050);
    // clawMotor.enableVoltageCompensation(12);
    // clawMotor.setIdleMode(IdleMode.kBrake);
    clawMotor.stopMotor();
  }

  public void runClaw() {
    clawMotor.set(ClawConstants.Claw_MOTOR_SPEED);
  }

  public void brakeClaw() {
    clawMotor.set(ClawConstants.Claw_BRAKE_SPEED);
  }

  public void reverseClaw() {
    clawMotor.set(ClawConstants.Claw_MOTOR_SPEED * -1);
  }

  public void stopClaw() {
    clawMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
