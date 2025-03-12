package frc.robot.subsystems.rgb;

public enum ScoringPosition {
  NONE(0),
  B1(12),
  B2(1),
  BR2(2),
  BR1(3),
  FR2(4),
  FR1(5),
  F2(6),
  F1(7),
  FL1(8),
  FL2(9),
  BL1(10),
  BL2(11);

  private final int position;

  ScoringPosition(int position) {
    this.position = position;
  }

  public int getPosition() {
    return position;
  }
}
