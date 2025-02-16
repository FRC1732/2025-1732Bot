package frc.robot.field;

public enum FieldObject {
  ROBOT_POSE("RobotPose"),
  LIMELIGHT_POSE("LimelightPose"),
  QUEST_POSE("QuestPose");

  private final String poseName;

  FieldObject(String poseName) {
    this.poseName = poseName;
  }

  public String getPoseName() {
    return poseName;
  }
}
