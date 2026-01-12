package frc.robot.utility;


/**
 * Basically directly copied from Roadrunner
 */
public class Vector2d {

    public double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d() {
        this.x = 0;
        this.y = 0;
    }

    public Vector2d plus(Vector2d v) {
        return new Vector2d(x + v.x, y + v.y);
    }

    public Vector2d minus(Vector2d v) {
        return new Vector2d(x - v.x, y - v.y);
    }

    public Vector2d unaryMinus() {
        return new Vector2d(-x, -y);
    }

    public Vector2d times(double z) {
        return new Vector2d(x * z, y * z);
    }

    public Vector2d div(double z) {
        return new Vector2d(x / z, y / z);
    }


    // My methods
    /**
     * Returns the angle in radians of the vector
     */
    public double angle() {
        if (y == 0 && x == 0) {
            return 0;
        } else return Math.atan2(y, x);
    }

    /**
     * Returns the magnitude of the vector
     */
    public double mag() {
        return Math.hypot(x, y);
    }

    /**
     * Return the distance between the current vector and the inputted vector
     */
    public double distFrom(Vector2d v) {
        return Math.hypot(x - v.x, y - v.y);
    }

    /**
     * Returns a vector at the same angle but at the inputted magnitude. DOES NOT CHANGE ORIGINAL VECTOR
     */
    public Vector2d withMag(double m) {
        double angle = Math.atan2(y, x);
        return new Vector2d(m * Math.cos(angle), m * Math.sin(angle));
    }

    /**
     * Returns a vector at the same magnitude but at the inputted angle in radians. DOES NOT CHANGE ORIGINAL VECTOR
     */
    public Vector2d withAngle(double a) {
        double mag = Math.hypot(x, y);
        return new Vector2d(mag * Math.cos(a), mag * Math.sin(a));
    }

    /**
     * Set the vector to the same angle but at the inputted magnitude.
     */
    public void setMag(double m) {
        double angle = Math.atan2(y, x);
        x = m * Math.cos(angle);
        y = m * Math.sin(angle);
    }

    /**
     * Sets the vector to the same magnitude but at the inputted angle in radians.
     */
    public void setAngle(double a) {
        double mag = Math.hypot(x, y);
        x = mag * Math.cos(a);
        y = mag * Math.sin(a);
    }
}