package frc.robot.units;

import java.util.EnumMap;

public interface Unit {
    public double to(Unit to);
    public EnumMap<Dimension, Integer> getDimension();
    public double getScalar();
}
