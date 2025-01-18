package frc.robot.subsystems.claw;

public class ClawConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClawConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = true;
  public static final String SUBSYSTEM_NAME = "Claw";

  public static final int Claw_MOTOR_CAN_ID = 51;

  public static final Double Claw_MOTOR_SPEED = -.2;
  public static final Double Claw_BRAKE_SPEED = 0.0; // -.04;

  public static final boolean Claw_LOGGING = true;
  public static final int BEAMBREAK_ID = 9;
}
