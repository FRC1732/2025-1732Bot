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

  public static final Double JOINT_FORWARD_SPEED = 0.4;
  public static final Double JOINT_REVERSE_SPEED = -0.4;

  public static final boolean JOINT_LOGGING = true;
}
