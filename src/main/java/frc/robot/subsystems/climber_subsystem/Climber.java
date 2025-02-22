// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber_subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climberMotor;

  private SparkMax windmillMotor;
  private RelativeEncoder windmillEncoder;

  private ShuffleboardTab tab;

  private RelativeEncoder climberRelativeEncoder;

  private RelativeEncoder windmillRelativeEncoder;

  private boolean windmillEngaged;

  public Climber() {
    climberMotor = new SparkMax(ClimberConstants.CLIMBER_CAN_ID, MotorType.kBrushless);
    windmillMotor = new SparkMax(ClimberConstants.WINDMILL_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig climberConfig = new SparkMaxConfig();
    SparkMaxConfig windmillConfig = new SparkMaxConfig();

    climberConfig.inverted(true);
    windmillConfig.inverted(true);

    climberConfig.idleMode(IdleMode.kBrake);
    windmillConfig.idleMode(IdleMode.kBrake);

    EncoderConfig climberEncoderConfig = new EncoderConfig();
    climberEncoderConfig.positionConversionFactor(ClimberConstants.CLIMBER_INCHES_PER_ROTATION);
    climberEncoderConfig.velocityConversionFactor(
        ClimberConstants.CLIMBER_RPM_TO_INCHES_PER_SECOND);
    climberConfig.apply(climberEncoderConfig);

    EncoderConfig windmillEncoderConfig = new EncoderConfig();
    windmillEncoderConfig.positionConversionFactor(ClimberConstants.WINDMILL_DEGREES_PER_ROTATION);
    windmillEncoderConfig.velocityConversionFactor(
        ClimberConstants.WINDMILL_RPM_TO_DEGREES_PER_SECOND);
    windmillConfig.apply(windmillEncoderConfig);

    climberMotor.configure(
        climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    windmillMotor.configure(
        windmillConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    climberRelativeEncoder = climberMotor.getEncoder();
    windmillRelativeEncoder = windmillMotor.getEncoder();

    climberRelativeEncoder.setPosition(0);
    windmillRelativeEncoder.setPosition(0);

    climberMotor.stopMotor();
    windmillMotor.stopMotor();

    setupShuffleboard();
  }

  public void runClimber() {
    climberMotor.set(ClimberConstants.CLIMBER_SPEED);
  }

  public void runWindmill() {
    windmillMotor.set(ClimberConstants.WINDMILL_SPEED);
  }

  public void brakeClimber() {
    climberMotor.set(ClimberConstants.CLIMBER_BRAKE_SPEED);
  }

  public void reverseClimber() {
    climberMotor.set(ClimberConstants.CLIMBER_SPEED * -0.75);
  }

  public void reverseWindmill() {
    windmillMotor.set(ClimberConstants.WINDMILL_SPEED * -1);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }

  public void stopWindmill() {
    windmillMotor.stopMotor();
  }

  public void engageWindmill() {
    windmillEngaged = true;
  }

  public void disengageWindmill() {
    windmillEngaged = false;
  }

  public void extendClimber() {
    if (getClimberPosition() < ClimberConstants.FULLY_EXTENDED_SETPOINT) {
      runClimber();
    } else {
      stopClimber();
    }
  }

  public void retractClimber() {
    if (getClimberPosition() > ClimberConstants.CLIMB_SETPOINT) {
      reverseClimber();
    } else {
      brakeClimber();
    }
  }

  public double getClimberPosition() {
    return climberRelativeEncoder.getPosition();
  }

  public double getWindmillPosition() {
    return windmillRelativeEncoder.getPosition();
  }

  public boolean isClimberAtPoint(double point) {
    return climberRelativeEncoder.getPosition() + ClimberConstants.CLIMBER_TOLERANCE >= point
        && climberRelativeEncoder.getPosition() - ClimberConstants.CLIMBER_TOLERANCE <= point;
  }

  public boolean isAtClimbSetpoint() {
    return isClimberAtPoint(ClimberConstants.CLIMB_SETPOINT);
  }

  public boolean isClimberFullyExtended() {
    return isClimberAtPoint(ClimberConstants.FULLY_EXTENDED_SETPOINT);
  }

  public boolean isWindmillEngaged() {
    return windmillEncoder.getPosition() >= ClimberConstants.WINDMILL_TOLERANCE;
  }

  public boolean isWindmillFullyEngaged() {
    return windmillEncoder.getPosition()
        >= ClimberConstants.WINDMILL_FULLY_ENGAGED_SETPOINT - ClimberConstants.WINDMILL_TOLERANCE;
  }

  public void setupShuffleboard() {
    tab = Shuffleboard.getTab(ClimberConstants.SUBSYSTEM_NAME);
    tab.addDouble("climber position", this::getClimberPosition);
    // tab.addDouble("climber speed", this::getClimberPivot);
    tab.addDouble("windmill position", this::getWindmillPosition);
    // tab.addDouble("windmill speed", this::getClimberWindmill);
  }

  @Override
  public void periodic() {
    if (windmillEngaged
        && getWindmillPosition()
            < ClimberConstants.WINDMILL_FULLY_ENGAGED_SETPOINT
                - ClimberConstants.WINDMILL_TOLERANCE) {
      runWindmill();
    } else if (!windmillEngaged && getWindmillPosition() > ClimberConstants.WINDMILL_TOLERANCE) {
      reverseWindmill();
    } else {
      stopWindmill();
    }
  }
}
