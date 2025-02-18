package frc.robot.subsystems.climber_subsystem;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ClimberConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClimberConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = true;
  public static final String SUBSYSTEM_NAME = "Climber";

  public static final int CLIMBER_PIVOT_CAN_ID = 20;
  public static final int CLIMBER_WINDMILL_CAN_ID = 12;

  public static final Double WINDMILL_SPEED = -0.2;
  public static final Double PIVOT_SPEED = -0.8;
  public static final Double CLIMBER_BRAKE_SPEED = 0.0;

  public static final boolean CLIMBER_LOGGING = true;
  public static final double PIVOT_TOLERANCE = 0.05;

  public static final double MAX_PIVOT_EXTEND = 0.0;
  public static final double MAX_PIVOT_RETRACT = 0.0;

  public static final double CLIMB_SETPOINT = 0; // TODO get value for this
  public static final double FULLY_EXTENDED_SETPOINT = 0; // TODO get value for this
  public static final double WINDMILL_TOLERANCE = 0.05;
  public static final double WINDMILL_FULLY_ENGAGED_SETPOINT = 0.0; // TODO get value for this
  public static final double RETREAT_TO_SAFE_BOUNDS_TIME = 0.2;
}
