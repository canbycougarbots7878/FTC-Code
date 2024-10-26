package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;


/**
 * Helper class for configuring the Brushland Labs Color Rangefinder.
 * Online documentation: <a href="https://docs.brushlandlabs.com">...</a>
 */
class ColorRangefinder {
    public final RevColorSensorV3 emulator;
    private final I2cDeviceSynchSimple i2c;

    public ColorRangefinder(RevColorSensorV3 emulator) {
        this.emulator = emulator;
        this.i2c = emulator.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    /**
     * Configure Pin 0 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin0Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound);
    }

    /**
     * Configure Pin 1 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin1Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 0's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin0DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin0Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 1's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin1DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin1Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin1InvertHue() {
        setPin1DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    public float[] getCalibration() {
        ByteBuffer bytes =
                ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(ByteOrder.LITTLE_ENDIAN);
        return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    private void setDigital(
            PinNum pinNum,
            DigitalMode digitalMode,
            double lowerBound,
            double higherBound
    ) {
        int lo, hi;
        if (lowerBound == higherBound) {
            lo = (int) lowerBound;
            hi = (int) higherBound;
        } else if (digitalMode.value <= DigitalMode.HSV.value) { // color value 0-255
            lo = (int) Math.round(lowerBound / 255.0 * 65535);
            hi = (int) Math.round(higherBound / 255.0 * 65535);
        } else { // distance in mm
            float[] calib = getCalibration();
            System.out.println(Arrays.toString(calib));
            if (lowerBound < .5) hi = 2048;
            else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound);
            lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound);
        }

        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        i2c.write(pinNum.modeAddress, new byte[]{digitalMode.value, lo0, lo1, hi0, hi1});
    }

    private double root(double n, double v) {
        double val = Math.pow(v, 1.0 / Math.abs(n));
        if (n < 0) val = 1.0 / val;
        return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
        return (int) (root(b, (mm - c) / a) + x0);
    }

    private enum PinNum {
        PIN0(0x28), PIN1(0x2D);

        private final byte modeAddress;

        PinNum(int modeAddress) {
            this.modeAddress = (byte) modeAddress;
        }
    }

    // other writeable registers
    private static final byte CALIB_A_VAL_0 = 0x32;

    public enum DigitalMode {
        HSV(5);
        public final byte value;

        DigitalMode(int value) {
            this.value = (byte) value;
        }
    }

}
