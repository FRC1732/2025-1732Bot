package frc.robot.subsystems.L1Scorer;

public class L1ScorerConstants {
  public static final String SUBSYSTEM_NAME = "L1Scorer";

  public static final int TILT_MOTOR_ID = 53;
  public static final int INTAKE_MOTOR_ID = 54; // to be assigned

  public static final double MIN_ANGLE_DEGREES = 0.0; // to be assigned
  public static final double MAX_ANGLE_DEGREES = 103.0;
  public static final double ANGLE_GOAL_TOLERANCE_DEGREES = 1.5;

  public static final double TILT_KP = 0; // to be assigned
  public static final double TILT_KI = 0; // to be assigned
  public static final double TILT_KD = 0; // to be assigned
  public static final double TILT_KG = 0; // to be assigned

  public static final double INTAKE_SPEED = -0.3;
  public static final double EJECT_SPEED = 0.3;
  public static final double TILT_SPEED = 0.10;

  public static final double START_ANGLE = 0.0; // to be assigned
  public static final double HOLD_ANGLE = 0.0; // to be assigned
  public static final double SCORE_ANGLE = 0.0; // to be assigned
  public static final double INTAKE_ANGLE = 0.0; // to be assigned

  public static final double INTAKE_DEGREES_PER_ROTATION = 360d / (50d / 26d * 25d);
  // degrees per motor revolution (360 / reduction = 360 / (50 / 26 * 25))
  public static final double INTAKE_RPM_TO_DEGREES_PER_SECOND = 360d / (50d / 26d * 25d) / 60d;
  // RPM to deg/sec (360 / reduction / 60 = 360 / (50 / 26 * 25) / 60)
}
