package frc.robot.subsystems.claw;

public class ClawConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClawConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = true;
  public static final String SUBSYSTEM_NAME = "Claw";

  public static final int CLAW_MOTOR_CAN_ID = 52;

  public static final Double CLAW_MOTOR_SPEED = -.5;
  public static final Double CLAW_BRAKE_SPEED = 0.0; // -.04;

  public static final boolean CLAW_LOGGING = true;
  public static final int BEAMBREAK_ID = 9;
  public static final int CLAW_ABSOLUTE_ENCODER = 8;
  public static final int SHOOTER_TILT_ABSOLUTE_OFFSET = 0; // TODO find value for this
  public static final double BEAMBREAK_THRESHOLD = 1.5;
}
