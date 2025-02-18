package frc.robot.subsystems.rgb;

public enum ScoringLevel {
  NONE(0),
  LEVEL_1(1),
  LEVEL_2(2),
  LEVEL_3(3),
  LEVEL_4(4);

  private final int level;

  ScoringLevel(int level) {
    this.level = level;
  }

  public int getLevel() {
    return level;
  }
}
