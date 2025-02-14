package frc.robot.subsystems.Intake;

public class IntakeConstants {
  public static final String SUBSYSTEM_NAME = "Intake";

  public static final int ROLLER_MOTOR_ID = 50; // TODO: verify left/right motor IDs as roller/tilt
  public static final int TILT_MOTOR_ID = 51;

  public static final double TILT_KP = 0; // TODO find values for these
  public static final double TILT_KI = 0;
  public static final double TILT_KD = 0;
  public static final double TILT_MAX_VELOCITY = 0;
  public static final double TILT_MAX_ACCELERATION = 0;
  public static final double TILT_PERIOD_SEC = 0;

  public static final double TILT_KS = 0;
  public static final double TILT_KG = 0;
  public static final double TILT_KV = 0;
  public static final double TILT_KA = 0;

  public static final double TILT_GOAL_TOLERANCE_DEGREES = 2;
  public static final double TILT_MAX_ABOLUTE_RELATIVE_ERROR_DEG = 10;
  public static final double TILT_COG_OFFSET = 0;

  public static final double ROLLER_INTAKE_SPEED = 0.2;
  public static final double ROLLER_EJECT_SPEED = -0.2;
}
