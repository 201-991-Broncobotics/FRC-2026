package frc.robot.utility;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Functions {

    /** Finds the smallest difference between two angles or gets the equivalent angle between -180 and
     * 180 when the currentAngle is 0 (and wrapAngle is 360).
     * A wrapAngle of 180 treats the targetAngle and the angle directly opposite of targetAngle the same
     *
     * @param currentAngle in degrees
     * @param targetAngle in degrees
     * @param wrapAngle Should be 360 unless you are doing a swerve module
     */
    public static double angleDifferenceDeg(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }
    /** Finds the smallest difference between two angles or gets the equivalent angle between -pi and
     * pi when the currentAngle is 0 (and wrapAngle is 2pi).
     * A wrapAngle of pi treats the targetAngle and the angle directly opposite of targetAngle the same
     *
     * @param currentAngle in radians
     * @param targetAngle in radians
     * @param wrapAngle Should be 2pi unless you are doing a swerve module
     */
    public static double angleDifference(double currentAngle, double targetAngle, double wrapAngle) {
        return Math.toRadians(angleDifferenceDeg(Math.toDegrees(currentAngle), Math.toDegrees(targetAngle), (int) Math.round(Math.toDegrees(wrapAngle))));
    }
    /**
     * Finds the equivalent angle between -180 and 180
     * 
     * @param angle in degrees
     */
    public static double normalizeAngleDeg(double angle) {
        return angleDifferenceDeg(0, angle, 360);
    }
    /**
     * Finds the equivalent angle between -pi and pi
     * 
     * @param angle in radians
     */
    public static double normalizeAngle(double angle) {
        return angleDifference(0, angle, 2*Math.PI);
    }
    /**
     * Finds the equivalent angle between 0 and 360
     * 
     * @param angle in degrees
     */
    public static double normalizeAngleDegPositive(double angle) {
        return angleDifferenceDeg(-180, angle, 360) + 180;
    }
    /**
     * Finds the equivalent angle between 0 and 2pi
     * 
     * @param angle in radians
     */
    public static double normalizeAnglePositive(double angle) {
        return angleDifference(-2*Math.PI, angle, 2*Math.PI) + 2*Math.PI;
    }
    /**
     * Works the same way as the round function in python. 
     * @param input double
     * @param decimalPlaces Number of decimal places to round to
     */
    public static double round(double input, int decimalPlaces) {
        return Math.round(input * (Math.pow(10, decimalPlaces))) / (Math.pow(10, decimalPlaces));
    }

    /**
     * Limits a value to between the min and max. 
     */
    public static double minMaxValue(double min, double max, double value) {
        if (value > max) return max;
        if (value < min) return min;
        else return value;
    }

    /**
     * Applies a polynomial curve to the input value while keeping the same sign. 
     * @param value input
     * @param mag power of the curve
     */
    public static double throttleCurve(double value, double mag) {
        return Math.pow(Math.abs(value), Math.abs(mag)) * Math.signum(value); 
    }

    /**
     * Applies a deadband to the input and also normalizes the output in the remaining range of 0 to 1
     */
    public static double deadbandValue(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0.0;
        else return (1 - deadband) * value + deadband * Math.signum(value);
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static String stringifyPose(Pose2d RobotPose){
        return "X:" + Functions.round(RobotPose.getX(), 3) + " Y:" + Functions.round(RobotPose.getY(), 3) + " R:" + Functions.round(RobotPose.getRotation().getDegrees(), 3);
    }

    public static String stringifyTrans(Translation3d translation){
        return "X:" + Functions.round(translation.getX(), 3) + " Y:" + Functions.round(translation.getY(), 3) + " Z:" + Functions.round(translation.getZ(), 3);
    }


    public static String stringifyTrans(Translation2d translation){
        return "X:" + Functions.round(translation.getX(), 3) + " Y:" + Functions.round(translation.getY(), 3);
    }


    public static double min(double... value) {
        double finalValue = value[0];
        for (double nextValue : value) finalValue = Math.min(finalValue, nextValue);
        return finalValue;
    }

    public static double max(double... value) {
        double finalValue = value[0];
        for (double nextValue : value) finalValue = Math.max(finalValue, nextValue);
        return finalValue;
    }

    public static double closestToZero(double value1, double value2) {
        if (Math.abs(value1) < Math.abs(value2)) return value1;
        else return value2;
    }

}
