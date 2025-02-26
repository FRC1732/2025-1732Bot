package frc.robot.subsystems.climber_subsystem;

public class ClimberConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClimberConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = true;
  public static final String SUBSYSTEM_NAME = "Climber";

  public static final int CLIMBER_CAN_ID = 20;
  public static final int WINDMILL_CAN_ID = 12;

  public static final Double WINDMILL_SPEED = 0.2;
  public static final Double CLIMBER_SPEED = 0.9;
  public static final Double CLIMBER_BRAKE_SPEED = 0.05;

  public static final boolean CLIMBER_LOGGING = true;
  public static final double CLIMBER_TOLERANCE = 0.1;

  public static final double MAX_CLIMBER_EXTEND = 0.0;
  public static final double MAX_CLIMBER_RETRACT = 0.0;

  public static final double CLIMB_SETPOINT = 7.5; // 10.72"
  public static final double FULLY_EXTENDED_SETPOINT = 15.4; // 17.65"
  public static final double WINDMILL_TOLERANCE = 3.0;
  public static final double WINDMILL_FULLY_ENGAGED_SETPOINT = 90.0;
  public static final double RETREAT_TO_SAFE_BOUNDS_TIME = 0.2;

  public static final double WINDMILL_DEGREES_PER_ROTATION =
      90.0; // degrees per motor revolution (360 / reduction = 360 / 4)
  public static final double WINDMILL_RPM_TO_DEGREES_PER_SECOND =
      1.5; // RPM to deg/sec (360 / reduction / 60 = 360 / 4 / 60)
  public static final double CLIMBER_INCHES_PER_ROTATION =
      0.0593411; // inches per motor revolution (spool diam * 3.14 / reduction = 0.75 * 3.14 /
  // 39.70588)
  public static final double CLIMBER_RPM_TO_INCHES_PER_SECOND =
      0.0009890; // RPM to inch/sec (spool diam * 3.14 / reduction / 60 = 0.75 * 3.14 / 39.70588 /
  // 60)
}
