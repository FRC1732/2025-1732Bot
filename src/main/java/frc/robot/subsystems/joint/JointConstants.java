package frc.robot.subsystems.joint;

public class JointConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private JointConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "JOINT";
  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = true;

  public static final int JOINT_MOTOR_CAN_ID = 56;

  public static final Double JOINT_FORWARD_SPEED = 0.2;
  public static final Double JOINT_REVERSE_SPEED = -0.2;

  public static final boolean JOINT_LOGGING = true;

  public static final double JOINT_PID_PERIOD_SEC = 0.02;
  public static final double JOINT_GOAL_TOLERANCE_DEGREES = 2;
  public static final double JOINT_MAX_ABOLUTE_RELATIVE_ERROR_DEG = 10;
  public static final double JOINT_COG_OFFSET = 37.79;
  public static final double JOINT_MAX_VELOCITY = 300; // deg/s
  public static final double JOINT_MAX_ACCELERATION = 600; // deg/s^2 4500 calculated max
  public static final double JOINT_KP = .04;
  public static final double JOINT_KI = 0;
  public static final double JOINT_KD = 0;
  public static final double JOINT_KG = 0.77 / 12 * .45; // V
  public static final double JOINT_KV = 0; // 0.021778; // V*s/deg
  public static final double JOINT_KA = 0; // 0.000361; // V*s^2/deg
  public static final double JOINT_KS = 0;

  public static final double JOINT_DEGREES_PER_ROTATION =
      5.625; // degrees per motor revolution (360 / reduction = 360 / 64)
  public static final double JOINT_RPM_TO_DEGREES_PER_SECOND =
      0.09375; // RPM to deg/sec (360 / reduction / 60 = 360 / 64 / 60)
  public static final double JOINT_HEIGHT_INCHES_PER_ROTATION =
      0.366519; // inches per motor revolution (spool diam * 3.14 / reduction = 1.75 * 3.14 / 15)
  public static final double JOINT_HEIGHT_RPM_TO_INCHES_PER_SECOND =
      0.006109; // RPM to inch/sec (spool diam * 3.14 / reduction / 60 = 1.75 * 3.14 / 15 / 60)

  public static final double JOINT_START_SETPOINT = 196;
  public static final double JOINT_CORAL_SETPOINT = 55;
  public static final double JOINT_LV1_SETPOINT = 0;
  public static final double JOINT_LV2_SETPOINT = 55;
  public static final double JOINT_LV3_SETPOINT = 55;
  public static final double JOINT_LV4_SETPOINT = 90;
}
