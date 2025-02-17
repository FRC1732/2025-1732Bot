package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SmoothedPoseFilter {
  private double alpha;
  private double beta;
  private double gamma;
  private double delta;
  private double dt; // Time step in seconds

  // State variables
  private Translation2d position;
  private Translation2d velocity;
  private Translation2d acceleration;
  private Translation2d jerk;
  private double angle;
  private double angularVelocity;
  private double angularAcceleration;
  private double angularJerk;

  private double maxSpeed = 5.39496; // Max linear speed in m/s
  private double maxAcceleration = 6.1; // Max linear acceleration in m/s^2
  private double maxJerk = 75.0; // Max linear jerk in m/s^3 (example value)
  private double maxRotationalSpeed = 1.5 * 2 * Math.PI; // Convert rotations/sec to rad/sec
  private double maxAngularAcceleration = Math.toRadians(1587); // Convert deg/s^2 to rad/s^2
  private double maxAngularJerk =
      Math.toRadians(10000); // Max angular jerk in rad/s^3 (example value)

  public SmoothedPoseFilter(double alpha, double beta, double gamma, double delta, double dt) {
    this.alpha = alpha;
    this.beta = beta;
    this.gamma = gamma;
    this.dt = dt;
    this.delta = delta;
    this.position = new Translation2d(0, 0);
    this.velocity = new Translation2d(0, 0);
    this.acceleration = new Translation2d(0, 0);
    this.jerk = new Translation2d(0, 0);
    this.angle = 0;
    this.angularVelocity = 0;
    this.angularAcceleration = 0;
    this.angularJerk = 0;
  }

  public SmoothedPoseFilter() {
    this(0.85, 0.005, 0.0001, 0.00001, 0.02);
  }

  public Pose2d update(Pose2d measuredPose) {
    // Translation update
    Translation2d measuredPosition = measuredPose.getTranslation();
    Translation2d predictedPosition =
        position.plus(velocity.times(dt)).plus(acceleration.times(0.5 * dt * dt));
    Translation2d predictedVelocity = velocity.plus(acceleration.times(dt));
    Translation2d predictedAcceleration = acceleration.plus(jerk.times(dt));
    Translation2d residualTranslation = measuredPosition.minus(predictedPosition);

    position = predictedPosition.plus(residualTranslation.times(alpha));
    velocity = predictedVelocity.plus(residualTranslation.times(beta / dt));
    acceleration = predictedAcceleration.plus(residualTranslation.times(gamma / (dt * dt)));
    jerk = residualTranslation.times(delta / (dt * dt * dt));

    if (velocity.getNorm() > maxSpeed) {
      velocity = velocity.times(maxSpeed / velocity.getNorm());
    }
    if (acceleration.getNorm() > maxAcceleration) {
      acceleration = acceleration.times(maxAcceleration / acceleration.getNorm());
    }
    if (jerk.getNorm() > maxJerk) {
      jerk = jerk.times(maxJerk / jerk.getNorm());
    }

    // Rotation update
    double measuredAngle = measuredPose.getRotation().getRadians();
    double residualAngle = measuredAngle - angle;
    while (residualAngle > Math.PI) residualAngle -= 2 * Math.PI;
    while (residualAngle < -Math.PI) residualAngle += 2 * Math.PI;

    double predictedAngle = angle + angularVelocity * dt + 0.5 * angularAcceleration * dt * dt;
    double predictedAngularVelocity = angularVelocity + angularAcceleration * dt;
    double predictedAngularAcceleration = angularAcceleration + angularJerk * dt;

    angle = predictedAngle + alpha * residualAngle;
    angularVelocity = predictedAngularVelocity + (beta / dt) * residualAngle;
    angularAcceleration = predictedAngularAcceleration + (gamma / (dt * dt)) * residualAngle;
    angularJerk = (delta / (dt * dt * dt)) * residualAngle;

    if (Math.abs(angularVelocity) > maxRotationalSpeed) {
      angularVelocity = Math.signum(angularVelocity) * maxRotationalSpeed;
    }
    if (Math.abs(angularAcceleration) > maxAngularAcceleration) {
      angularAcceleration = Math.signum(angularAcceleration) * maxAngularAcceleration;
    }
    if (Math.abs(angularJerk) > maxAngularJerk) {
      angularJerk = Math.signum(angularJerk) * maxAngularJerk;
    }

    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;

    return new Pose2d(position, new Rotation2d(angle));
  }

  public void resetPose(Pose2d curPose) {
    position = curPose.getTranslation();
    angle = curPose.getRotation().getRadians();
  }
}
