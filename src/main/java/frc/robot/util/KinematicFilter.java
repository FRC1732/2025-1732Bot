package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class KinematicFilter {
  private double maxSpeed = 5.39496; // Max linear speed in m/s
  private double maxAcceleration = 6.1; // Max linear acceleration in m/s^2
  private double maxJerk = 75.0; // Max linear jerk in m/s^3
  private double maxRotationalSpeed = 1.5 * 2 * Math.PI; // rad/sec
  private double maxAngularAcceleration = Math.toRadians(1587); // rad/s^2
  private double maxAngularJerk = Math.toRadians(10000); // rad/s^3

  private Translation2d previousPosition = new Translation2d();
  private Rotation2d previousRotation = new Rotation2d();
  private double previousLinearVelocity = 0.0;
  private double previousAngularVelocity = 0.0;
  private double previousLinearAcceleration = 0.0;
  private double previousAngularAcceleration = 0.0;
  private double previousTime = -1.0;

  private double unwrapAngle(double current, double previous) {
    double diff = current - previous;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    return previous + diff;
  }

  public Pose2d update(Pose2d currentPose, double currentTime) {
    if (previousTime < 0) {
      previousPosition = currentPose.getTranslation();
      previousRotation = currentPose.getRotation();
      previousTime = currentTime;
      return currentPose; // No filtering on the first update
    }

    double deltaTime = currentTime - previousTime;
    if (deltaTime <= 0) {
      return currentPose; // Avoid division by zero or negative time steps
    }

    // Calculate linear velocity
    Translation2d deltaPosition = currentPose.getTranslation().minus(previousPosition);
    double linearVelocity = deltaPosition.getNorm() / deltaTime;

    // Calculate rotational velocity with angle unwrapping
    double unwrappedCurrentRotation =
        unwrapAngle(currentPose.getRotation().getRadians(), previousRotation.getRadians());
    double angularVelocity = (unwrappedCurrentRotation - previousRotation.getRadians()) / deltaTime;

    // Calculate linear acceleration
    double linearAcceleration = (linearVelocity - previousLinearVelocity) / deltaTime;

    // Calculate angular acceleration
    double angularAcceleration = (angularVelocity - previousAngularVelocity) / deltaTime;

    // Calculate linear jerk
    double linearJerk = (linearAcceleration - previousLinearAcceleration) / deltaTime;

    // Calculate angular jerk
    double angularJerk = (angularAcceleration - previousAngularAcceleration) / deltaTime;

    // Clamp values within limits
    linearJerk = Math.min(Math.abs(linearJerk), maxJerk) * Math.signum(linearJerk);
    linearAcceleration =
        Math.min(Math.abs(linearAcceleration), maxAcceleration) * Math.signum(linearAcceleration);
    linearVelocity =
        Math.min(Math.abs(previousLinearVelocity + linearAcceleration * deltaTime), maxSpeed)
            * Math.signum(linearVelocity);

    angularJerk = Math.min(Math.abs(angularJerk), maxAngularJerk) * Math.signum(angularJerk);
    angularAcceleration =
        Math.min(Math.abs(angularAcceleration), maxAngularAcceleration)
            * Math.signum(angularAcceleration);
    angularVelocity =
        Math.min(
                Math.abs(previousAngularVelocity + angularAcceleration * deltaTime),
                maxRotationalSpeed)
            * Math.signum(angularVelocity);

    // Compute new position and rotation based on constrained values
    Translation2d newPosition =
        previousPosition.plus(new Translation2d(linearVelocity * deltaTime, 0));
    Rotation2d newRotation =
        new Rotation2d(
            unwrapAngle(
                previousRotation.getRadians() + angularVelocity * deltaTime,
                previousRotation.getRadians()));

    // Update stored values
    previousPosition = newPosition;
    previousRotation = newRotation;
    previousLinearVelocity = linearVelocity;
    previousAngularVelocity = angularVelocity;
    previousLinearAcceleration = linearAcceleration;
    previousAngularAcceleration = angularAcceleration;
    previousTime = currentTime;

    return new Pose2d(newPosition, newRotation);
  }

  public void reset(Pose2d curPose, double curTime) {
    previousPosition = curPose.getTranslation();
    previousRotation = curPose.getRotation();
    previousTime = curTime;
  }
}
