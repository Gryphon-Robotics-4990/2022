package frc.robot.units;

public class UnitDimensionException extends Exception {
    private static final long serialVersionUID = 2528602015917702706L;
    
    public UnitDimensionException(String message) {
        super(message);
    }

    public UnitDimensionException() {
        this("Incorrect dimensions for unit conversions.");
    }

    public UnitDimensionException(String message, Throwable t) {
        super(message, t);
    }

    public UnitDimensionException(Throwable t) {
        super("Incorrect dimensions for unit conversions.", t);
    }
}
