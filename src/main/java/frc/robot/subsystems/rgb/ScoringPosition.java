package frc.robot.subsystems.rgb;

public enum ScoringPosition {
  B1(1),
  B2(2),
  BR2(3),
  BR1(4),
  FR2(5),
  FR1(6),
  F2(7),
  F1(8),
  FL1(9),
  FL2(10),
  BL1(11),
  BL2(12);

  private final int position;

  ScoringPosition(int position) {
    this.position = position;
  }

  public int getPosition() {
    return position;
  }
}
