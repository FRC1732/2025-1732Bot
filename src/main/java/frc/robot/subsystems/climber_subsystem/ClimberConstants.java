package frc.robot.subsystems.climber_subsystem;

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
}
