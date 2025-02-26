package frc.robot.subsystems.intake_subsystem;

public class IntakeConstants {
  public static final String SUBSYSTEM_NAME = "Intake";

  public static final int ROLLER_MOTOR_ID = 50;
  public static final int TILT_MOTOR_ID = 51;

  public static final double PID_PERIOD_SEC = 0.02;

  public static final double MIN_ANGLE_DEGREES = -5.0;
  public static final double MAX_ANGLE_DEGREES = 57.0;
  public static final double ANGLE_GOAL_TOLERANCE_DEGREES = 2.0;

  public static final double INTAKE_MAX_VELOCITY = 60; // deg/s
  public static final double INTAKE_MAX_ACCELERATION = 200; // deg/s^2
  public static final double INTAKE_KP = 0.01;
  public static final double INTAKE_KI = 0;
  public static final double INTAKE_KD = 0;
  public static final double INTAKE_KG = 0.025; // V
  public static final double INTAKE_KV = 0; // V*s/deg
  public static final double INTAKE_KA = 0; // V*s^2/deg
  public static final double INTAKE_KS = 0;

  public static final double INTAKE_DEGREES_PER_ROTATION = 20.336;
  // 22.5; // degrees per motor revolution (360 / reduction = 360 / 16)
  public static final double INTAKE_RPM_TO_DEGREES_PER_SECOND =
      0.3389; // RPM to deg/sec (360 / reduction / 60 = 360 / 16 / 60)

  public static final double ROLLER_INTAKE_SPEED = -0.50;
  public static final double ROLLER_EJECT_SPEED = 0.4;
}
