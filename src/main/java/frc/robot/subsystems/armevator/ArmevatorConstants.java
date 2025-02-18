// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armevator;

/** Add your docs here. */
public class ArmevatorConstants {
  public static final String SUBSYSTEM_NAME = "Armevator";

  public static final int LEFT_MOTOR_ID = 60;
  public static final int RIGHT_MOTOR_ID = 61;

  public static final int ARM_MOTOR_ID = 56;

  public static final int ELEVATOR_LIMIT_SWITCH_ID = 6;

  public static final double ELEVATOR_KP = 0; // TODO find values for these
  public static final double ELEVATOR_KI = 0;
  public static final double ELEVATOR_KD = 0;
  public static final double ELEVATOR_MAX_VELOCITY = 0;
  public static final double ELEVATOR_MAX_ACCELERATION = 0;
  public static final double ELEVATOR_PERIOD_SEC = 0.02;

  public static final double ELEVATOR_HEIGHT_KS = 0;
  public static final double ELEVATOR_HEIGHT_KG = 0;
  public static final double ELEVATOR_HEIGHT_KV = 0;
  public static final double ELEVATOR_HEIGHT_KA = 0;

  public static final double ARM_KP = 0.04; // TODO: find values for these, copied from joint
  public static final double ARM_KI = 0;
  public static final double ARM_KD = 0;
  public static final double ARM_MAX_VELOCITY = 300;
  public static final double ARM_MAX_ACCELERATION = 600;
  public static final double ARM_PERIOD_SEC = 0.02;

  public static final double ARM_HEIGHT_KS = 0;
  public static final double ARM_HEIGHT_KG = 0;
  public static final double ARM_HEIGHT_KV = 0;
  public static final double ARM_HEIGHT_KA = 0;

  /////////////////////////////////////////
  public static final double PID_PERIOD_SEC = 0.02;

  public static final double MIN_HEIGHT_INCHES = 0;
  public static final double MAX_HEIGHT_INCHES = 16; // @todo
  public static final double HEIGHT_GOAL_TOLERANCE_INCHES = 1; // @todo

  public static final double MIN_ANGLE_CLEARANCE_DEGREES = -46.4 + 6.0; // @todo
  public static final double MIN_ANGLE_DEGREES = -46.4; // @todo
  public static final double MAX_ANGLE_DEGREES = 102.5; // @todo
  public static final double ANGLE_ABSOLUTE_OFFSET = 126.1374 - 60.0 - 11; // @todo
  public static final double ANGLE_GOAL_TOLERANCE_DEGREES = 2;
  public static final double ANGLE_COG_OFFSET = 54.3;

  /*
  public static final double SHOOTER_HEIGHT_MAX_VELOCITY = 35.17; // in/s
  public static final double SHOOTER_HEIGHT_MAX_ACCELERATION = 100; // in/s^2 838.75 calculated max
  public static final double SHOOTER_HEIGHT_KP = 0.1;
  public static final double SHOOTER_HEIGHT_KI = 0;
  public static final double SHOOTER_HEIGHT_KD = 0;
  public static final double SHOOTER_HEIGHT_KG = 0.23 / 12 * .8; // V
  public static final double SHOOTER_HEIGHT_KV = 0; // 0.334010 / 12 * .8; // V*s/in
  public static final double SHOOTER_HEIGHT_KA = 0; // 0.000762 / 12 * .8; // V*s^2/in
  public static final double SHOOTER_HEIGHT_KS = 0;

  public static final double SHOOTER_ANGLE_MAX_VELOCITY = 300; // deg/s
  public static final double SHOOTER_ANGLE_MAX_ACCELERATION = 600; // deg/s^2 4500 calculated max
  public static final double SHOOTER_ANGLE_KP = .04;
  public static final double SHOOTER_ANGLE_KI = 0;
  public static final double SHOOTER_ANGLE_KD = 0;
  public static final double SHOOTER_ANGLE_KG = 0.77 / 12 * .45; // V
  public static final double SHOOTER_ANGLE_KV = 0; // 0.021778; // V*s/deg
  public static final double SHOOTER_ANGLE_KA = 0; // 0.000361; // V*s^2/deg
  public static final double SHOOTER_ANGLE_KS = 0;
  */

  // @todo
  public static final double ARM_DEGREES_PER_ROTATION =
      5.625; // degrees per motor revolution (360 / reduction = 360 / 64)
  public static final double ARM_RPM_TO_DEGREES_PER_SECOND =
      0.09375; // RPM to deg/sec (360 / reduction / 60 = 360 / 64 / 60)
  public static final double ELEVATOR_INCHES_PER_ROTATION =
      0.366519; // inches per motor revolution (spool diam * 3.14 / reduction = 1.75 * 3.14 / 15)
  public static final double ELEVATOR_RPM_TO_INCHES_PER_SECOND =
      0.006109; // RPM to inch/sec (spool diam * 3.14 / reduction / 60 = 1.75 * 3.14 / 15 / 60)
}
