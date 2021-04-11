package frc.robot;

public enum Direction {
    Forward (1),
    Stop (0),
    Reverse (-1);

    private final int multiplier;

    private Direction(int m) {
        this.multiplier = m;
    }

    public int getMultiplier() {
        return multiplier;
    }


}
