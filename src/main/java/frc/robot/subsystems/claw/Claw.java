// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private SparkMax clawMotor;

  private RelativeEncoder encoder;
  private ShuffleboardTab tab;
  private DigitalInput digitalInput;
  private DutyCycleEncoder clawAbsoluteEncoder;

  public Claw() {
    clawMotor = new SparkMax(ClawConstants.Claw_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);
    Timer.delay(0.050);
    // clawMotor.setInverted(ClawConstants.Claw_MOTOR_INVERTED);
    encoder = clawMotor.getEncoder();
    // clawMotor.enableVoltageCompensation(12);

    // clawMotor.setIdleMode(IdleMode.kBrake);
    digitalInput = new DigitalInput(ClawConstants.BEAMBREAK_ID);
    clawAbsoluteEncoder = new DutyCycleEncoder(ClawConstants.CLAW_ABSOLUTE_ENCODER);
    clawMotor.stopMotor();
    setupShuffleboard();
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

  public double getClawSpeed() {
    return clawMotor.get();
  }

    private double angleModulusDeg(double angleDeg) {
    return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(angleDeg)));
  }

  private double getAbsolutePosition() {
    return angleModulusDeg(
      clawAbsoluteEncoder.get() * -360
            + ClawConstants.SHOOTER_TILT_ABSOLUTE_OFFSET);
  }

  public void resetToAbsoluteEncoder() {
    if (clawAbsoluteEncoder.isConnected()) {
      encoder.setPosition(getAbsolutePosition());
    }
  }


  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public void setupShuffleboard() {
    tab = Shuffleboard.getTab("Claw");
    tab.addDouble("Claw Encoder Position", this::getEncoderPosition);
    tab.addDouble("Claw Speed", this::getClawSpeed);
    tab.addBoolean("Has Coral", this::hasCoral);
    tab.addDouble("Absolute Position", this::getAbsolutePosition);
  }

  public boolean hasCoral() {
    return digitalInput.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(ClawConstants.SUBSYSTEM_NAME + "/Claw Encoder Position", this::getEncoderPosition);
    Logger.recordOutput(ClawConstants.SUBSYSTEM_NAME + "/Claw Speed", this::getClawSpeed);
    Logger.recordOutput(ClawConstants.SUBSYSTEM_NAME + "/Has Coral", this::hasCoral);
    Logger.recordOutput(ClawConstants.SUBSYSTEM_NAME + "/Absolute Position", this::getAbsolutePosition);
  }
}
