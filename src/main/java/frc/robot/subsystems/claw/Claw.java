// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private SparkMax clawMotor;

  private RelativeEncoder encoder;
  private SparkAnalogSensor beamBreakSensor;
  private ShuffleboardTab tab;

  public Claw() {
    clawMotor = new SparkMax(ClawConstants.Claw_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);
    Timer.delay(0.050);
    // clawMotor.setInverted(ClawConstants.Claw_MOTOR_INVERTED);
    encoder = clawMotor.getEncoder();
    beamBreakSensor = clawMotor.getAnalog();
    // clawMotor.enableVoltageCompensation(12);

    // clawMotor.setIdleMode(IdleMode.kBrake);
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

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public double getBeambreakVoltage() {
    return beamBreakSensor.getVoltage();
  }

  public void setupShuffleboard() {
    tab = Shuffleboard.getTab("Claw");
    tab.addDouble("Claw Encoder Position", this::getEncoderPosition);
    tab.addDouble("Claw Speed", this::getClawSpeed);
    tab.addDouble("Beambreak Voltage", this::getBeambreakVoltage);
    tab.addBoolean("Has Coral", this::hasCoral);
  }

  public boolean hasCoral() {
    return beamBreakSensor.getVoltage() > ClawConstants.BEAMBREAK_THRESHOLD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(
        ClawConstants.SUBSYSTEM_NAME + "/Claw Encoder Position", this.getEncoderPosition());
    Logger.recordOutput(ClawConstants.SUBSYSTEM_NAME + "/Claw Speed", this.getClawSpeed());
    Logger.recordOutput(ClawConstants.SUBSYSTEM_NAME + "/Has Coral", this.hasCoral());
  }
}
