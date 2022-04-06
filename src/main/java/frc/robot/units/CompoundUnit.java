package frc.robot.units;

import java.util.EnumMap;

public class CompoundUnit implements Unit {
    //Stores the powers of each dimension
    private EnumMap<Dimension, Integer> dimensions;

    //Stores the scalar multiplier from the base units
    private double scalar;

    public CompoundUnit(Unit[] numerators, Unit[] denominators) {
        dimensions = new EnumMap<Dimension, Integer>(Dimension.class);
        scalar = 1;

        for (Unit u : numerators) {
            for (Dimension d : Dimension.values()) {
                int cur = dimensions.containsKey(d) ? dimensions.get(d) : 0;
                int add = u.getDimension().containsKey(d) ? u.getDimension().get(d) : 0;
                dimensions.put(d, cur + add);
            }
            scalar *= u.getScalar();
        }

        for (Unit u : denominators) {
            for (Dimension d : Dimension.values()) {
                int cur = dimensions.containsKey(d) ? dimensions.get(d) : 0;
                int add = u.getDimension().containsKey(d) ? u.getDimension().get(d) : 0;
                dimensions.put(d, cur - add);
            }
            scalar /= u.getScalar();
        }
    }

    public CompoundUnit(Unit numerator, Unit[] denominators) {
        this(new Unit[] {numerator}, denominators);
    }

    public CompoundUnit(Unit[] numerators, Unit denominator) {
        this(numerators, new Unit[] {denominator});
    }
    
    public CompoundUnit(Unit numerator, Unit denominator) {
        this(new Unit[] {numerator}, new Unit[] {denominator});
    }

    @Override
    public double to(Unit to) {
        return scalar / to.getScalar();
    }

    
    //TODO Change getter below to deep copy.
    @Override
    public EnumMap<Dimension, Integer> getDimension() {
        return dimensions.clone();
    }

    @Override
    public double getScalar() {
        return scalar;
    }


}
