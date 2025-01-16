// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  public final CANSparkMax clawMotor;

  public Claw() {
    clawMotor = new CANSparkMax(ClawConstants.Claw_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
