package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class DriveToPoseSlew extends Command {
  public final CommandSwerveDrivetrain drivetrain;
  public final SwerveRequest.FieldCentricFacingAngle driveRequest;
  private final Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose;
  private Pose2d lastPose = new Pose2d();
  private int atTargetCount = 0;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;

  private boolean running = false;
  private Timer timer;

  private static final double driveKp = 7.5;
  private static final double driveKd = 0.0;
  private static final double driveKi = 0.0;
  private static final double driveMaxVelocity = 3.5;
  private static final double driveMaxAcceleration = 3.0;
  private static final double driveTolerance = 0.04;
  private static final double velocityTolerance = 0.3;
  private static final double angularVelocityTolerance = 0.6;
  private static final double thetaTolerance = 5.0;
  private static final double timeout = 5.0;

  private final PIDController xController =
      new PIDController(driveKp, driveKi, driveKd, LOOP_PERIOD_SECS);
  private final PIDController yController =
      new PIDController(driveKp, driveKi, driveKd, LOOP_PERIOD_SECS);

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToPoseSlew(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> poseSupplier,
      SwerveRequest.FieldCentricFacingAngle driveRequest) {
    this.driveRequest = driveRequest;
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.timer = new Timer();

    // Initialize slew rate limiters with the maximum acceleration
    this.xLimiter = new SlewRateLimiter(driveMaxAcceleration);
    this.yLimiter = new SlewRateLimiter(driveMaxAcceleration);

    addRequirements(drivetrain);
  }

  /**
   * This method is invoked once when this command is scheduled. It resets all the PID controllers
   * and initializes the current and target poses.
   */
  @Override
  public void initialize() {
    // Reset all controllers
    Pose2d currentPose = drivetrain.getPose();

    xController.reset();
    yController.reset();

    xController.setSetpoint(0);
    yController.setSetpoint(0);

    xController.setTolerance(driveTolerance);
    yController.setTolerance(driveTolerance);

    this.targetPose = poseSupplier.get();

    // Reset slew rate limiters
    this.xLimiter =
        new SlewRateLimiter(driveMaxAcceleration, -driveMaxAcceleration, currentPose.getX());
    this.yLimiter =
        new SlewRateLimiter(driveMaxAcceleration, -driveMaxAcceleration, currentPose.getY());

    Logger.recordOutput("DriveToPose/targetPose", targetPose);

    this.timer.restart();
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    running = true;

    Pose2d currentPose = drivetrain.getPose();

    // Calculate the error between current position and target position
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();

    // Calculate raw velocity commands from PID controllers
    double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());

    // Limit velocity to maximum
    xVelocity = Math.min(Math.max(xVelocity, -driveMaxVelocity), driveMaxVelocity);
    yVelocity = Math.min(Math.max(yVelocity, -driveMaxVelocity), driveMaxVelocity);

    // Apply slew rate limiting to smooth acceleration
    xVelocity = xLimiter.calculate(xVelocity);
    yVelocity = yLimiter.calculate(yVelocity);

    // If we're at the goal, stop moving
    if (Math.abs(xError) < driveTolerance) xVelocity = 0.0;
    if (Math.abs(yError) < driveTolerance) yVelocity = 0.0;

    drivetrain.setControl(
        driveRequest
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withTargetDirection(targetPose.getRotation()));
  }

  /**
   * This method returns true if the command has finished.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    Pose2d currentPose = drivetrain.getPose();
    double xError = Math.abs(targetPose.getX() - currentPose.getX());
    double yError = Math.abs(targetPose.getY() - currentPose.getY());
    double thetaError =
        Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
    double currentVelocity =
        Math.hypot(
            drivetrain.getState().Speeds.vxMetersPerSecond,
            drivetrain.getState().Speeds.vxMetersPerSecond);
    double currentAngularVelocity = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);
    boolean xAtGoal = xError < driveTolerance;
    boolean yAtGoal = yError < driveTolerance;
    boolean thetaAtGoal = thetaError < thetaTolerance;
    boolean velocityAtGoal = currentVelocity < velocityTolerance;
    boolean angularVelocityAtGoal = currentAngularVelocity < thetaTolerance;

    Logger.recordOutput("DriveToPose/xErr", xAtGoal);
    Logger.recordOutput("DriveToPose/yErr", yAtGoal);
    Logger.recordOutput("DriveToPose/tErr", thetaAtGoal);
    Logger.recordOutput("DriveToPose/vErr", velocityAtGoal);
    Logger.recordOutput("DriveToPose/tvErr", angularVelocityAtGoal);

    boolean isAtTarget =
        (running
            && xAtGoal
            && yAtGoal
            && velocityAtGoal
            && angularVelocityAtGoal
            && isThetaAtGoal());
    if (isAtTarget) {
      atTargetCount++;
    } else {
      atTargetCount = 0;
    }

    return this.timer.hasElapsed(timeout) || atTargetCount >= 2;
  }

  private boolean isThetaAtGoal() {
    return Math.abs(targetPose.getRotation().minus(drivetrain.getPose().getRotation()).getDegrees())
        < thetaTolerance;
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(drivetrain.getPose().getRotation()));
    running = false;
  }
}
