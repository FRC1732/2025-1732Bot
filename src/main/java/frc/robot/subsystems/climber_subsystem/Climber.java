// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber_subsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climber_commands.RetractIntoSafeBounds;
import frc.robot.subsystems.armevator.ArmevatorConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climberMotorPivot;

  private SparkMax climberMotorWindmill;  
  private RelativeEncoder pivotEncoder;
  private RelativeEncoder windmillEncoder;

  private ShuffleboardTab tab;
  private boolean runningReturnCommand;

  public Climber() {
    climberMotorPivot = new SparkMax(ClimberConstants.CLIMBER_PIVOT_CAN_ID, MotorType.kBrushless);
    climberMotorWindmill = new SparkMax(ClimberConstants.CLIMBER_WINDMILL_CAN_ID, MotorType.kBrushless);

    climberMotorPivot.stopMotor();
    climberMotorWindmill.stopMotor();

    pivotEncoder = climberMotorPivot.getEncoder();
    pivotEncoder.setPosition(0);

    windmillEncoder = climberMotorWindmill.getEncoder();
    windmillEncoder.setPosition(0);
    setupShuffleboard();

    runningReturnCommand = false;
  }

  public void runClimberPivot() {
    climberMotorPivot.set(ClimberConstants.PIVOT_SPEED);
  }

  public void runClimberWindmill() {
    climberMotorWindmill.set(ClimberConstants.WINDMILL_SPEED);
  }

  public void brakeClimberPivot() {
    climberMotorPivot.set(ClimberConstants.CLIMBER_BRAKE_SPEED);
  }

  public void brakeClimberWindmill() {
    climberMotorWindmill.set(ClimberConstants.CLIMBER_BRAKE_SPEED);
  }

  public void reverseClimberPivot() {
    climberMotorPivot.set(ClimberConstants.PIVOT_SPEED * -0.5);
  }

  public void reverseClimberWindmill() {
    climberMotorWindmill.set(ClimberConstants.WINDMILL_SPEED * -1);
  }

  public void stopClimberPivot() {
    climberMotorPivot.stopMotor();
  }

  public void stopClimberWindmill() {
    climberMotorWindmill.stopMotor();
  }

  public double getClimberPivot() {
    return climberMotorPivot.get();
  }

  public double getClimberWindmill() {
    return climberMotorWindmill.get();
  }

  public boolean isPivotAtPoint(double point) {
    return pivotEncoder.getPosition() + ClimberConstants.PIVOT_TOLERANCE >= point && pivotEncoder.getPosition() - ClimberConstants.PIVOT_TOLERANCE <= point;
  }

  public boolean isAtClimbSetpoint() {
    return isPivotAtPoint(ClimberConstants.CLIMB_SETPOINT);
  }

  public boolean isPivotFullyExtended() {
    return isPivotAtPoint(ClimberConstants.FULLY_EXTENDED_SETPOINT);
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
    tab.addDouble(ClimberConstants.SUBSYSTEM_NAME + "/Claw Pivot Speed", this::getClimberPivot);
    tab.addDouble(ClimberConstants.SUBSYSTEM_NAME + "/Claw Windmill Speed", this::getClimberWindmill);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (pivotEncoder.getPosition() > ClimberConstants.MAX_PIVOT_EXTEND && !runningReturnCommand) {
      climberMotorPivot.stopMotor();

      CommandScheduler.getInstance().schedule(new RetractIntoSafeBounds(this, true, ClimberConstants.RETREAT_TO_SAFE_BOUNDS_TIME));
    }

    if (pivotEncoder.getPosition() < ClimberConstants.MAX_PIVOT_RETRACT && !runningReturnCommand) {
      climberMotorPivot.stopMotor();

      CommandScheduler.getInstance().schedule(new RetractIntoSafeBounds(this, false, ClimberConstants.RETREAT_TO_SAFE_BOUNDS_TIME));

    }
  }
}
