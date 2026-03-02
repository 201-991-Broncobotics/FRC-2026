package frc.robot.utility;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ThroughBoreEncoder {

    private Encoder relativeEncoder;
    private DutyCycleEncoder absoluteEncoder;

    private final double absoluteResolution = 1024;
    private final double relativeResolution = 8192;

    private double absoluteEncoderZero = 0, relativeEncoderZero = 0;

    public ThroughBoreEncoder(int channelA, int channelB, int channelC) {

        if (channelA >= 0) {
            absoluteEncoder = new DutyCycleEncoder(channelA);
        }
        if (channelB >= 0 && channelC >= 0) {
            relativeEncoder = new Encoder(channelB, channelC);
        }

    }

    /**
     * For using only the duty cycle / absolute encoder
     * @param channelA
     */
    public ThroughBoreEncoder(int channelA) {
        this(channelA, -1, -1);
    }

    /**
     * For using only the quadrature / relative encoder
     * @param channelB
     * @param channelC
     */
    public ThroughBoreEncoder(int channelB, int channelC) {
        this(-1, channelB, channelC);
    }

    public double getRelativeRaw() { return (relativeEncoder != null) ? relativeEncoder.getRaw() : 0.0; }
    public double getAbsoluteRaw() { return (absoluteEncoder != null) ? absoluteEncoder.get() : 0.0; }

    public double getRelativeTicks() { return (relativeEncoder != null) ? relativeEncoder.getRaw() : 0.0; }
    public double getAbsoluteTicks() { return (absoluteEncoder != null) ? absoluteEncoder.get() * absoluteResolution : 0.0; }

    /**
     * Gets the relative angle in radians
     * @return angle in radians
     */
    public double getRelativeAngle() { return getRelativeRaw() / relativeResolution * 2*Math.PI - relativeEncoderZero; }
    /**
     * Returns absolute angle between 0 and 2pi
     * @return angle in radians between 0 and 2pi
     */
    public double getAbsoluteAngle() { return Functions.normalizeAnglePositive(getAbsoluteRaw() * 2*Math.PI - absoluteEncoderZero); }
    /**
     * Normalizes the angle between -pi and pi
     * @return angle in radians between -pi and pi
     */
    public double getAbsoluteAngleNorm() { return Functions.normalizeAngle(getAbsoluteRaw() * 2*Math.PI - absoluteEncoderZero); }


    public double getAbsoluteAngleWithoutZero() { return Functions.normalizeAnglePositive(getAbsoluteRaw() * 2*Math.PI); }

    public void resetRelative() { relativeEncoder.reset(); }
    public void setRelativeZero(double radians) { relativeEncoderZero = radians; }
    public void setAbsoluteZero(double radians) { absoluteEncoderZero = radians; }

    public boolean encoderExists() { 
        return this.relativeEncoder != null || this.absoluteEncoder != null; 
    }

    public boolean encoderConnected() { 
        if (absoluteEncoder != null) {
            return absoluteEncoder.isConnected();
        }
        // Relative WPILib Encoders don't have a built-in connection check, 
        // so we can only verify if the object was instantiated.
        return relativeEncoder != null; 
    }


}
