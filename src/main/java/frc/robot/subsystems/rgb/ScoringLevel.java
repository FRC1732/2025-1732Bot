package frc.robot.subsystems.rgb;

public enum ScoringLevel {
    L1(1),
    L2(2),
    L3(3),
    L4(4);

    private final int level;
    
    ScoringLevel(int level) {
        this.level = level;
    }

    public int getLevel() {
        return level;
    }
}
