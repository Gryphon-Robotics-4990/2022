package frc.robot.units;

import java.util.EnumMap;

public interface Unit {
    public double to(Unit to) throws UnitDimensionException;
    public EnumMap<Dimension, Integer> getDimension();
    public double getScalar();
}
