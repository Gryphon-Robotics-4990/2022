package frc.robot.vision;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import frc.robot.units.UnitDimensionException;


// https://www.wired.com/2012/01/projectile-motion-primer-for-first-robotics/
// http://docs.limelightvision.io/en/latest/cs_aimandrange.html

public class VisionController {
    // http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    public static double findDistanceToTarget() {
        return (Vision.TARGET_HEIGHT_METERS - RobotMeasurements.LIMELIGHT_HEIGHT_METERS) / Math.tan(findVerticalAngleToTarget());
    }

    public static double findVerticalAngleToTarget() {
        double target_offset = 0d;
        try {
            double factor = Units.DEGREE.to(Units.RADIAN);
            target_offset = factor * Limelight.getCrosshairVerticalOffset();
        } catch (UnitDimensionException e) {/* Will never throw this error because we know it can convert degrees to radians*/}
        return RobotMeasurements.LIMELIGHT_ANGLE_RADIANS + target_offset;
    }

    public static class ShooterVision {
        // 2D array of control points, with multiple "buckets" 
        // corresponding to a set of control points
        private static ControlPoint[][] controlPoints;
        // The interval of distance in each bucket, used to configure how
        // we interpolate based on the control points
        private static final int distancePerBucket = 2;

        public static void setControlPoints(ControlPoint[] points) {
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

        private static double[] getShooterSpeedsFromDistance(double dist) {
            int closestBucket = (int)Math.floor(dist / distancePerBucket);
            if (closestBucket > controlPoints.length - 1) {
                //Input distance is out of range, so can't interpolate shooter speed
                System.err.println("[ERROR] Couldn't interpolate shooter speed, distance out of range.");
                return new double[]{0.0, 0.0};
            }
            ControlPoint last = new ControlPoint(0, 0);
            for (ControlPoint p : controlPoints[closestBucket]){
                if (p.getDistance() == dist) {
                    // We don't want top roller if it's a lob
                    double topSpeed = isLob(dist) ? 0.0 : 0.5;
                    return new double[]{topSpeed, p.getSpeed()};
                }
                if (p.getDistance() > dist) {
                    double bottomSpeed = interpolate(last, p, dist);
                    double topSpeed = isLob(dist) ? 0.0 : 0.5;
                    return new double[]{topSpeed, bottomSpeed};
                }
                last = p;
            }
            System.err.println("[ERROR] Couldn't interpolate shooter speed");
            return new double[]{0.0, 0.0};
        }

        // TODO figure out better way to transition over lob threshold
        private static boolean isLob(double dist) {
            return dist <= Vision.LOB_THRESHOLD;
        }

        // Returns top speed, bottom speed
        public static double[] getShooterSpeedFromLimelight() {
            return getShooterSpeedsFromDistance(findDistanceToTarget());
        }

        // Fits a line between two points, and plugs in an x-value along that line to find the corresponding y-value
        public static double interpolate(ControlPoint p1, ControlPoint p2, double dist) {
            double x1 = p1.getX();
            double y1 = p1.getY();
            double m = (p2.getY() - p1.getY()) / (p2.getX() - p1.getX());
            // Creating function of line using point-slope form
            Function<Double, Double> f = (Double x) -> (m * (x - x1) + y1);
            double result = f.apply(dist);
            return result;
        }
    }

    public static class TurretVision {
        public static double getTurretPositionFromAngle() {
            double position = 0d;
            try {
                double factor = Units.DEGREE.to(Units.ENCODER_ANGLE);
                position = factor * Limelight.getCrosshairHorizontalOffset();
            } catch (UnitDimensionException e) {/* Won't throw exception because both dimensions are angles*/}
            
            return position;
        }
    }
}
