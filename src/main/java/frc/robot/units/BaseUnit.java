package frc.robot.units;

import java.util.EnumMap;

public class BaseUnit implements Unit {
    private EnumMap<Dimension, Integer> dimension;
    private double scalar;

    public BaseUnit(Dimension dimension, double scalar) {
        this.dimension = new EnumMap<>(Dimension.class);
        this.dimension.replace(dimension, 1);
        this.scalar = scalar;
    }

    @Override
    public double to(Unit to){
        return scalar / to.getScalar();
    }

    //TODO Change getter below to deep copy.
    @Override
    public EnumMap<Dimension, Integer> getDimension() {
        return dimension.clone();
    }

    @Override
    public double getScalar() {
        return scalar;
    }
}
