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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climber_commands.RetractIntoSafeBounds;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climberMotor;

  private SparkMax windmillMotor;
  private RelativeEncoder windmillEncoder;

  private ShuffleboardTab tab;
  private boolean runningReturnCommand;

  private RelativeEncoder climberRelativeEncoder;

  private RelativeEncoder windmillRelativeEncoder;

  public Climber() {
    climberMotor = new SparkMax(ClimberConstants.CLIMBER_CAN_ID, MotorType.kBrushless);
    windmillMotor = new SparkMax(ClimberConstants.WINDMILL_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig climberConfig = new SparkMaxConfig();
    SparkMaxConfig windmillConfig = new SparkMaxConfig();

    climberConfig.inverted(true);

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

    climberMotor.stopMotor();
    windmillMotor.stopMotor();
    setupShuffleboard();

    runningReturnCommand = false;
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
    climberMotor.set(ClimberConstants.CLIMBER_SPEED * -0.5);
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
    return windmillEncoder.getPosition() >= ClimberConstants.WINDMILL_FULLY_ENGAGED_SETPOINT;
  }

  public void setRunningReturnCommand(boolean running) {
    runningReturnCommand = running;
  }

  public void setupShuffleboard() {
    tab = Shuffleboard.getTab("Climber");
    tab.addDouble(ClimberConstants.SUBSYSTEM_NAME + "/climber position", this::getClimberPosition);
    // tab.addDouble("climber speed", this::getClimberPivot);
    tab.addDouble(
        ClimberConstants.SUBSYSTEM_NAME + "/windmill position", this::getWindmillPosition);
    // tab.addDouble("windmill speed", this::getClimberWindmill);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (climberRelativeEncoder.getPosition() > ClimberConstants.MAX_CLIMBER_EXTEND
        && !runningReturnCommand) {
      climberMotor.stopMotor();

      CommandScheduler.getInstance()
          .schedule(
              new RetractIntoSafeBounds(this, true, ClimberConstants.RETREAT_TO_SAFE_BOUNDS_TIME));
    }

    if (climberRelativeEncoder.getPosition() < ClimberConstants.MAX_CLIMBER_RETRACT
        && !runningReturnCommand) {
      climberMotor.stopMotor();

      CommandScheduler.getInstance()
          .schedule(
              new RetractIntoSafeBounds(this, false, ClimberConstants.RETREAT_TO_SAFE_BOUNDS_TIME));
    }
  }
}
