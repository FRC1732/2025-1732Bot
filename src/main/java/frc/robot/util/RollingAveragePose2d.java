package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.LinkedList;
import java.util.Queue;

public class RollingAveragePose2d {
  private final int windowSize;
  private final Queue<Pose2d> poses;
  private double sumX, sumY, sumTheta;

  public RollingAveragePose2d(int windowSize) {
    this.windowSize = windowSize;
    this.poses = new LinkedList<>();
    this.sumX = 0;
    this.sumY = 0;
    this.sumTheta = 0;
  }

  public void addPose(Pose2d pose) {
    poses.add(pose);
    sumX += pose.getX();
    sumY += pose.getY();
    sumTheta += pose.getRotation().getRadians();

    if (poses.size() > windowSize) {
      Pose2d removed = poses.poll();
      sumX -= removed.getX();
      sumY -= removed.getY();
      sumTheta -= removed.getRotation().getRadians();
    }
  }

  public Pose2d getAveragePose() {
    if (poses.isEmpty()) {
      return new Pose2d();
    }
    int size = poses.size();
    return new Pose2d(sumX / size, sumY / size, new Rotation2d(sumTheta / size));
  }

  public void reset() {
    this.poses.clear();
    this.sumX = 0;
    this.sumY = 0;
    this.sumTheta = 0;
  }
}
