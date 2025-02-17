package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseLowPassFilter {
  private Pose2d filteredPose;
  private double alpha;

  public PoseLowPassFilter() {
    this.alpha = 0.17;
  }

  public PoseLowPassFilter(double alpha, Pose2d initialPose) {
    this.alpha = alpha;
    this.filteredPose = initialPose;
  }

  public Pose2d update(Pose2d measuredPose) {
    Translation2d newPosition = measuredPose.getTranslation();
    Rotation2d newRotation = measuredPose.getRotation();

    // Apply exponential smoothing for position
    Translation2d filteredPosition =
        filteredPose.getTranslation().times(1 - alpha).plus(newPosition.times(alpha));

    // Handle discontinuity in rotation
    double currentAngle = filteredPose.getRotation().getRadians();
    double newAngle = newRotation.getRadians();

    // Compute shortest angular difference
    double deltaAngle = newAngle - currentAngle;
    deltaAngle = Math.atan2(Math.sin(deltaAngle), Math.cos(deltaAngle)); // Wrap between -π and π

    // Apply smoothing to the angle
    double filteredAngle = currentAngle + alpha * deltaAngle;

    Rotation2d filteredRotation = new Rotation2d(filteredAngle);

    // Update the pose
    filteredPose = new Pose2d(filteredPosition, filteredRotation);
    return filteredPose;
  }

  public void reset(Pose2d curPose2d) {
    filteredPose = curPose2d;
  }
}
