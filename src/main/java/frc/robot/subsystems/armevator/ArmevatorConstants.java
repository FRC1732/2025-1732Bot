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

  /////////////////////////////////////////
  public static final double PID_PERIOD_SEC = 0.02;

  public static final double MIN_HEIGHT_INCHES = 0.0;
  public static final double MAX_HEIGHT_INCHES = 32.0;
  public static final double HEIGHT_GOAL_TOLERANCE_INCHES = 1.0;

  public static final double MIN_ANGLE_CLEARANCE_DEGREES = -108.0;
  public static final double MIN_HEIGHT_CLEARANCE_INCHES = 2.5;
  public static final double MIN_ANGLE_DEGREES = -127.0;
  public static final double MAX_ANGLE_DEGREES = 96.6;
  public static final double ANGLE_ABSOLUTE_OFFSET_DEG = 99.0;
  public static final double ANGLE_GOAL_TOLERANCE_DEGREES = 2.0;
  public static final double ANGLE_COG_OFFSET = 54.3;

  public static final double ELEVATOR_MAX_VELOCITY = 10; // in/s 35.17 last year
  public static final double ELEVATOR_MAX_ACCELERATION = 25; // in/s^2 100 last year
  public static final double ELEVATOR_KP = 0; // 0.1 last year
  public static final double ELEVATOR_KI = 0;
  public static final double ELEVATOR_KD = 0;
  public static final double ELEVATOR_KG = 0.1 / 12.0; // V 0.184 / 12 last year
  public static final double ELEVATOR_KV = 0; // 0.334010 / 12 * .8; // V*s/in
  public static final double ELEVATOR_KA = 0; // 0.000762 / 12 * .8; // V*s^2/in
  public static final double ELEVATOR_KS = 0;

  public static final double ARM_MAX_VELOCITY = 75; // deg/s 300 last year
  public static final double ARM_MAX_ACCELERATION = 150; // deg/s^2 600 last year
  public static final double ARM_KP = 0; // 0.04 last year
  public static final double ARM_KI = 0;
  public static final double ARM_KD = 0;
  public static final double ARM_KG = 0.1 / 12; // V 0.3465 / 12 last year
  public static final double ARM_KV = 0; // 0.021778; // V*s/deg
  public static final double ARM_KA = 0; // 0.000361; // V*s^2/deg
  public static final double ARM_KS = 0;

  public static final double ARM_DEGREES_PER_ROTATION =
      9.0; // degrees per motor revolution (360 / reduction = 360 / 40)
  public static final double ARM_RPM_TO_DEGREES_PER_SECOND =
      0.15; // RPM to deg/sec (360 / reduction / 60 = 360 / 40 / 60)
  public static final double ELEVATOR_INCHES_PER_ROTATION =
      0.3333576; // inches per motor revolution (spool diam * 3.14 / reduction = 0.955 * 3.14 / 9)
  public static final double ELEVATOR_RPM_TO_INCHES_PER_SECOND =
      0.00555596; // RPM to inch/sec (spool diam * 3.14 / reduction / 60 = 0.955 * 3.14 / 9 / 60)
}
