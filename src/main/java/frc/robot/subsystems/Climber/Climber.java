// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climberMotorPivot;
  private SparkMax climberMotorWindmill;

  private ShuffleboardTab tab;

  public Climber() {
    climberMotorPivot = new SparkMax(ClimberConstants.CLIMBER_PIVOT_CAN_ID, MotorType.kBrushless);
    climberMotorWindmill = new SparkMax(ClimberConstants.CLIMBER_WINDMILL_CAN_ID, MotorType.kBrushless);

    climberMotorPivot.stopMotor();
    climberMotorWindmill.stopMotor();
    setupShuffleboard();
  }

    public void runClimberPivot() {
    climberMotorPivot.set(ClimberConstants.CLIMBER_SPEED_1);
  }

  public void runClimberWindmill() {
    climberMotorWindmill.set(ClimberConstants.CLIMBER_SPEED_2);
  }

  public void brakeClimberPivot() {
    climberMotorPivot.set(ClimberConstants.CLIMBER_BRAKE_SPEED);
  }

  public void brakeClimberWindmill() {
    climberMotorWindmill.set(ClimberConstants.CLIMBER_BRAKE_SPEED);
  }

  public void reverseClimberPivot() {
    climberMotorPivot.set(ClimberConstants.CLIMBER_SPEED_1 * -1);
  }

  public void reverseClimberWindmill() {
    climberMotorWindmill.set(ClimberConstants.CLIMBER_SPEED_2 * -1);
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

  public void setupShuffleboard() {
    tab = Shuffleboard.getTab("Climber");
    tab.addDouble("Claw Pivot Speed", this::getClimberPivot);
    tab.addDouble("Claw Windmill Speed", this::getClimberWindmill);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
