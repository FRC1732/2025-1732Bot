package frc.robot.field;

public enum FieldPoseObject {
  ROBOT_POSE("RobotPose"),
  LIMELIGHT_POSE("LimelightPose"),
  QUEST_POSE("QuestPose");

  private final String poseName;

  FieldPoseObject(String poseName) {
    this.poseName = poseName;
  }

  public String getPoseName() {
    return poseName;
  }
}
