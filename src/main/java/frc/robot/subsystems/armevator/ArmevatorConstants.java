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

  public static final double ARM_GOAL_TOLERANCE_DEGREES = 2;
  public static final double ARM_MAX_ABOLUTE_RELATIVE_ERROR_DEG = 10;
  public static final double ARM_COG_OFFSET = 0;
}
