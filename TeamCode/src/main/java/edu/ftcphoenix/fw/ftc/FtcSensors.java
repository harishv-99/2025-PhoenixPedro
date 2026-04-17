package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.ftcphoenix.fw.core.color.NormalizedRgba;
import edu.ftcphoenix.fw.core.color.Rgba;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * FTC-boundary adapters from FTC SDK devices to Phoenix sources.
 *
 * <p>This class mirrors {@link FtcHardware}, but for <b>sensor inputs</b> instead of actuator
 * outputs. Returned sources are memoized per loop by default so a device is sampled at most once
 * per {@link LoopClock#cycle()}, even when multiple consumers read it during the same loop.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * ScalarSource armTicks = FtcSensors.motorPositionTicks(hardwareMap, "arm");
 * BooleanSource limit = FtcSensors.digitalLow(hardwareMap, "armLimit");
 * Source<NormalizedRgba> markColor = FtcSensors.normalizedRgba(hardwareMap, "markSensor");
 * }</pre>
 */
public final class FtcSensors {

    private FtcSensors() {
        // utility class
    }

    // ------------------------------------------------------------------------------------------------
    // Distance sensors
    // ------------------------------------------------------------------------------------------------

    /**
     * Create a memoized scalar source that reads distance in the requested FTC unit.
     *
     * @param sensor FTC distance sensor to sample
     * @param unit FTC distance unit to request from the SDK
     * @return memoized scalar source producing distance readings in {@code unit}
     */
    public static ScalarSource distance(DistanceSensor sensor, DistanceUnit unit) {
        if (sensor == null) {
            throw new IllegalArgumentException("sensor is required");
        }
        if (unit == null) {
            throw new IllegalArgumentException("unit is required");
        }
        return ScalarSource.of(() -> sensor.getDistance(unit)).memoized();
    }

    /**
     * Create a memoized scalar source that reads distance in centimeters.
     *
     * @param sensor FTC distance sensor to sample
     * @return memoized scalar source producing distance readings in centimeters
     */
    public static ScalarSource distanceCm(DistanceSensor sensor) {
        return distance(sensor, DistanceUnit.CM);
    }

    /**
     * Create a memoized scalar source that reads distance in inches.
     *
     * @param sensor FTC distance sensor to sample
     * @return memoized scalar source producing distance readings in inches
     */
    public static ScalarSource distanceIn(DistanceSensor sensor) {
        return distance(sensor, DistanceUnit.INCH);
    }

    /**
     * Create a memoized scalar source that reads distance in centimeters from a named FTC device.
     *
     * @param hw FTC hardware map used to look up the sensor
     * @param name configured hardware name of the {@link DistanceSensor}
     * @return memoized scalar source producing centimeter readings from the named sensor
     */
    public static ScalarSource distanceCm(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return distanceCm(hw.get(DistanceSensor.class, name));
    }

    /**
     * Create a memoized scalar source that reads distance in inches from a named FTC device.
     *
     * @param hw FTC hardware map used to look up the sensor
     * @param name configured hardware name of the {@link DistanceSensor}
     * @return memoized scalar source producing inch readings from the named sensor
     */
    public static ScalarSource distanceIn(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return distanceIn(hw.get(DistanceSensor.class, name));
    }

    // ------------------------------------------------------------------------------------------------
    // Color sensors
    // ------------------------------------------------------------------------------------------------

    /**
     * Create a memoized source of raw RGBA channels from an FTC {@link ColorSensor}.
     *
     * <p>The returned value uses Phoenix {@link Rgba} so robot code does not depend on FTC SDK color
     * container types. Channel values are sensor-native integers and may not be limited to
     * {@code 0..255}.</p>
     *
     * @param sensor FTC color sensor to sample
     * @return memoized source of raw RGBA readings
     */
    public static Source<Rgba> rgba(ColorSensor sensor) {
        if (sensor == null) {
            throw new IllegalArgumentException("sensor is required");
        }
        return new Source<Rgba>() {
            @Override
            public Rgba get(LoopClock clock) {
                return new Rgba(sensor.red(), sensor.green(), sensor.blue(), sensor.alpha());
            }
        }.memoized();
    }

    /**
     * Create a memoized source of raw RGBA channels from a named FTC {@link ColorSensor}.
     *
     * @param hw FTC hardware map used to look up the sensor
     * @param name configured hardware name of the color sensor
     * @return memoized source of raw RGBA readings from the named sensor
     */
    public static Source<Rgba> rgba(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return rgba(hw.get(ColorSensor.class, name));
    }

    /**
     * Create a memoized source of normalized RGBA channels from an FTC
     * {@link NormalizedColorSensor}.
     *
     * <p>This is often the best starting point for color classification because the FTC SDK normalizes
     * the channels into a stable range (typically {@code 0..1}) and exposes sensor gain control.
     * The alpha channel is commonly useful as an overall brightness or confidence signal.</p>
     *
     * @param sensor FTC normalized color sensor to sample
     * @return memoized source of normalized RGBA readings
     */
    public static Source<NormalizedRgba> normalizedRgba(NormalizedColorSensor sensor) {
        if (sensor == null) {
            throw new IllegalArgumentException("sensor is required");
        }
        return new Source<NormalizedRgba>() {
            @Override
            public NormalizedRgba get(LoopClock clock) {
                NormalizedRGBA c = sensor.getNormalizedColors();
                return new NormalizedRgba(c.red, c.green, c.blue, c.alpha);
            }
        }.memoized();
    }

    /**
     * Create a memoized source of normalized RGBA channels from a named FTC
     * {@link NormalizedColorSensor}.
     *
     * @param hw FTC hardware map used to look up the sensor
     * @param name configured hardware name of the normalized color sensor
     * @return memoized source of normalized RGBA readings from the named sensor
     */
    public static Source<NormalizedRgba> normalizedRgba(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return normalizedRgba(hw.get(NormalizedColorSensor.class, name));
    }

    // ------------------------------------------------------------------------------------------------
    // Touch sensors
    // ------------------------------------------------------------------------------------------------

    /**
     * Create a memoized boolean source that is true while the touch sensor is pressed.
     *
     * @param sensor FTC touch sensor to sample
     * @return memoized boolean source that mirrors {@link TouchSensor#isPressed()}
     */
    public static BooleanSource touchPressed(TouchSensor sensor) {
        if (sensor == null) {
            throw new IllegalArgumentException("sensor is required");
        }
        return BooleanSource.of(sensor::isPressed).memoized();
    }

    /**
     * Create a memoized boolean source that is true while a named FTC touch sensor is pressed.
     *
     * @param hw FTC hardware map used to look up the sensor
     * @param name configured hardware name of the touch sensor
     * @return memoized boolean source that mirrors the named sensor's pressed state
     */
    public static BooleanSource touchPressed(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return touchPressed(hw.get(TouchSensor.class, name));
    }

    // ------------------------------------------------------------------------------------------------
    // Digital inputs
    // ------------------------------------------------------------------------------------------------

    private static DigitalChannel requireDigitalInput(DigitalChannel ch) {
        if (ch == null) {
            throw new IllegalArgumentException("digital channel is required");
        }
        ch.setMode(DigitalChannel.Mode.INPUT);
        return ch;
    }

    /**
     * Create a memoized boolean source that reports whether a digital input pin is HIGH.
     *
     * <p>FTC {@link DigitalChannel#getState()} returns {@code true} when the pin is HIGH. Some
     * sensors are wired active-low, so prefer {@link #digitalLow(DigitalChannel)} when LOW means
     * "active".</p>
     *
     * @param channel FTC digital channel configured for input use
     * @return memoized boolean source that is true when the input pin is HIGH
     */
    public static BooleanSource digitalHigh(DigitalChannel channel) {
        DigitalChannel ch = requireDigitalInput(channel);
        return BooleanSource.of(ch::getState).memoized();
    }

    /**
     * Create a memoized boolean source that reports whether a digital input pin is LOW.
     *
     * @param channel FTC digital channel configured for input use
     * @return memoized boolean source that is true when the input pin is LOW
     */
    public static BooleanSource digitalLow(DigitalChannel channel) {
        DigitalChannel ch = requireDigitalInput(channel);
        return BooleanSource.of(() -> !ch.getState()).memoized();
    }

    /**
     * Create a memoized boolean source that reports whether a named digital input is HIGH.
     *
     * @param hw FTC hardware map used to look up the digital channel
     * @param name configured hardware name of the digital channel
     * @return memoized boolean source that is true when the named input pin is HIGH
     */
    public static BooleanSource digitalHigh(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return digitalHigh(hw.get(DigitalChannel.class, name));
    }

    /**
     * Create a memoized boolean source that reports whether a named digital input is LOW.
     *
     * @param hw FTC hardware map used to look up the digital channel
     * @param name configured hardware name of the digital channel
     * @return memoized boolean source that is true when the named input pin is LOW
     */
    public static BooleanSource digitalLow(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return digitalLow(hw.get(DigitalChannel.class, name));
    }

    // ------------------------------------------------------------------------------------------------
    // Analog inputs
    // ------------------------------------------------------------------------------------------------

    /**
     * Create a memoized scalar source that reads analog voltage.
     *
     * @param input FTC analog input to sample
     * @return memoized scalar source producing voltage in volts
     */
    public static ScalarSource analogVoltage(AnalogInput input) {
        if (input == null) {
            throw new IllegalArgumentException("input is required");
        }
        return ScalarSource.of(input::getVoltage).memoized();
    }

    /**
     * Create a memoized scalar source that reads analog voltage from a named FTC device.
     *
     * @param hw FTC hardware map used to look up the analog input
     * @param name configured hardware name of the analog input
     * @return memoized scalar source producing voltage in volts
     */
    public static ScalarSource analogVoltage(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return analogVoltage(hw.get(AnalogInput.class, name));
    }

    // ------------------------------------------------------------------------------------------------
    // Encoder / motor measurements
    // ------------------------------------------------------------------------------------------------

    /**
     * Create a memoized scalar source that reads an FTC motor or encoder position in native ticks.
     *
     * <p>This is the most direct raw readback helper for internal or external encoders exposed through
     * the FTC motor API. The returned value is always in encoder ticks.</p>
     *
     * @param motor FTC motor or encoder-backed device to sample
     * @return memoized scalar source producing encoder position in ticks
     */
    public static ScalarSource motorPositionTicks(DcMotor motor) {
        if (motor == null) {
            throw new IllegalArgumentException("motor is required");
        }
        return ScalarSource.of(motor::getCurrentPosition).memoized();
    }

    /**
     * Create a memoized scalar source that reads position in native ticks and optionally flips the
     * sign to match a caller-chosen logical direction.
     *
     * @param motor     FTC motor or encoder-backed device to sample
     * @param direction logical direction to apply to the reported ticks
     * @return memoized scalar source producing encoder position in ticks, optionally sign-flipped
     */
    public static ScalarSource motorPositionTicks(DcMotor motor, Direction direction) {
        if (direction == null) {
            throw new IllegalArgumentException("direction is required");
        }
        ScalarSource base = motorPositionTicks(motor);
        return direction == Direction.REVERSE ? base.scaled(-1.0) : base;
    }

    /**
     * Create a memoized scalar source that reads position in native ticks from a named FTC motor or
     * external encoder device.
     *
     * @param hw   FTC hardware map used to look up the motor or encoder-backed device
     * @param name configured hardware name of the motor or encoder-backed device
     * @return memoized scalar source producing encoder position in ticks
     */
    public static ScalarSource motorPositionTicks(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return motorPositionTicks(hw.get(DcMotor.class, name));
    }

    /**
     * Create a memoized scalar source that reads position in native ticks from a named FTC motor or
     * external encoder device and optionally flips the sign to match a caller-chosen logical
     * direction.
     *
     * @param hw        FTC hardware map used to look up the motor or encoder-backed device
     * @param name      configured hardware name of the motor or encoder-backed device
     * @param direction logical direction to apply to the reported ticks
     * @return memoized scalar source producing encoder position in ticks, optionally sign-flipped
     */
    public static ScalarSource motorPositionTicks(HardwareMap hw, String name, Direction direction) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return motorPositionTicks(hw.get(DcMotor.class, name), direction);
    }

    /**
     * Create a memoized scalar source that reads FTC motor velocity in native ticks per second.
     *
     * @param motor FTC motor or encoder-backed device that exposes velocity through {@link DcMotorEx}
     * @return memoized scalar source producing velocity in ticks per second
     */
    public static ScalarSource motorVelocityTicksPerSec(DcMotorEx motor) {
        if (motor == null) {
            throw new IllegalArgumentException("motor is required");
        }
        return ScalarSource.of(motor::getVelocity).memoized();
    }

    /**
     * Create a memoized scalar source that reads velocity in native ticks per second and optionally
     * flips the sign to match a caller-chosen logical direction.
     *
     * @param motor     FTC motor or encoder-backed device that exposes velocity through {@link DcMotorEx}
     * @param direction logical direction to apply to the reported velocity
     * @return memoized scalar source producing velocity in ticks per second, optionally sign-flipped
     */
    public static ScalarSource motorVelocityTicksPerSec(DcMotorEx motor, Direction direction) {
        if (direction == null) {
            throw new IllegalArgumentException("direction is required");
        }
        ScalarSource base = motorVelocityTicksPerSec(motor);
        return direction == Direction.REVERSE ? base.scaled(-1.0) : base;
    }

    /**
     * Create a memoized scalar source that reads velocity in native ticks per second from a named FTC
     * motor or encoder-backed device.
     *
     * @param hw   FTC hardware map used to look up the motor or encoder-backed device
     * @param name configured hardware name of the motor or encoder-backed device
     * @return memoized scalar source producing velocity in ticks per second
     */
    public static ScalarSource motorVelocityTicksPerSec(HardwareMap hw, String name) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return motorVelocityTicksPerSec(hw.get(DcMotorEx.class, name));
    }

    /**
     * Create a memoized scalar source that reads velocity in native ticks per second from a named FTC
     * motor or encoder-backed device and optionally flips the sign to match a caller-chosen logical
     * direction.
     *
     * @param hw        FTC hardware map used to look up the motor or encoder-backed device
     * @param name      configured hardware name of the motor or encoder-backed device
     * @param direction logical direction to apply to the reported velocity
     * @return memoized scalar source producing velocity in ticks per second, optionally sign-flipped
     */
    public static ScalarSource motorVelocityTicksPerSec(HardwareMap hw, String name, Direction direction) {
        if (hw == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (name == null) {
            throw new IllegalArgumentException("name is required");
        }
        return motorVelocityTicksPerSec(hw.get(DcMotorEx.class, name), direction);
    }
}
