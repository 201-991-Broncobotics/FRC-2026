package frc.robot.utility;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ThroughBoreEncoder {

    private Encoder relativeEncoder;
    private DutyCycleEncoder absoluteEncoder;

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
        new ThroughBoreEncoder(channelA, -1, -1);
    }

    /**
     * For using only the quadrature / relative encoder
     * @param channelB
     * @param channelC
     */
    public ThroughBoreEncoder(int channelB, int channelC) {
        new ThroughBoreEncoder(-1, channelB, channelC);
    }

    public double getRelativeRaw() { return relativeEncoder.getRaw(); }
    public double getAbsoluteRaw() { return absoluteEncoder.get(); }

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
    public void setRelativeZero(double relativeZero) { relativeEncoderZero = relativeZero; }
    public void setAbsoluteZero(double absoluteZero) { absoluteEncoderZero = absoluteZero; }

    public boolean encoderExists() { return this.relativeEncoder != null; }
    public boolean encoderConnected() { return absoluteEncoder.isConnected(); }


}
