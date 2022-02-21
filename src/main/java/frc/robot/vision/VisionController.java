package frc.robot.vision;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.wpilibj.util.Units;


// https://www.wired.com/2012/01/projectile-motion-primer-for-first-robotics/
// http://docs.limelightvision.io/en/latest/cs_aimandrange.html

public class VisionController {

    private static ControlPoint[][] controlPoints;

    public static void setControlPoints(ControlPoint[] points) {
        int distancePerBucket = 2; // The interval of distance in each bucket
        // Assumes they are sorted in increasing order by distance
        int bucketCounter = 0;
        double startDistance = points[0].getDistance(); // should probably start at 0
        int countWithinBucket = 0;
        for (ControlPoint p : points) {
            if (p.getDistance() - startDistance <= distancePerBucket) {
                controlPoints[bucketCounter][countWithinBucket] = p;
                countWithinBucket += 1;
            }
            else {
                bucketCounter += 1;
                countWithinBucket = 0;
                controlPoints[bucketCounter][countWithinBucket] = p;
            }
        }
    }

    public static double getShooterSpeedFromDistance(double dist) {
        int distancePerBucket = 2; // Repeat of value in setControlPoints()
        //Try to interpolate.
        int closestBucket = (int)Math.floor(dist / distancePerBucket);
        ControlPoint last = new ControlPoint(0, 0);
        for (ControlPoint p : controlPoints[closestBucket]){
            if (p.getDistance() == dist){
                return p.getSpeed();
            }
            if (p.getDistance() > dist) {
                double speed = LinearInterpolate.interpolate(last, p, dist);
                return speed;
            }
            last = p;

        }
        

        //If we couldn't interpolate, it means that the distance isn't in the correct range.
        System.err.println("[ERROR] Couldn't interpolate shooter speed, distance out of range.");
        return 0;
    }

    public static double getShooterSpeedFromLimelight() {
        return getShooterSpeedFromDistance(findDistanceToTarget());
    }

    // http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    public static double findDistanceToTarget() {
        return (Vision.TARGET_HEIGHT_METERS - RobotMeasurements.LIMELIGHT_HEIGHT_METERS) / Math.tan(findAngleToTarget());
    }

    public static double findAngleToTarget() {
        return RobotMeasurements.LIMELIGHT_ANGLE_RADIANS + Units.degreesToRadians(Limelight.getCrosshairVerticalOffset());
    }

    private static class LinearInterpolate {
        // Fits a line between two points, and plugs in an x-value along that line to find the corresponding y-value
        public static double interpolate(ControlPoint p1, ControlPoint p2, double dist) {
            double x1 = p1.getX();
            double y1 = p1.getY();
            double m = (p2.getY() - p1.getY()) / (p2.getX() - p1.getX());
            Function<Double, Double> f = (Double x) -> (m * (x - x1) + y1);
            double result = f.apply(dist);
            return result;
        }
    }
}
