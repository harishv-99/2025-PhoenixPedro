package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.ftcphoenix.fw.core.color.NormalizedRgba;
import edu.ftcphoenix.fw.core.color.Rgba;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Adapters from FTC SDK sensor devices to Phoenix {@link edu.ftcphoenix.fw.core.source.Source}s.
 *
 * <p>This class mirrors {@link FtcHardware}, but for <b>sensor inputs</b> instead of actuator outputs.
 * The goal is to make sensor reads participate in Phoenix's "one loop, one heartbeat" design:
 * you build sources once, then sample them using the current {@link LoopClock}.</p>
 *
 * <h2>Memoization</h2>
 *
 * <p>All sources returned by this class are <b>memoized per loop</b> by default. This ensures a sensor
 * is sampled at most once per {@link LoopClock#cycle()}, even if multiple subsystems consume the
 * value (directly or through derived signals like hysteresis/debounce).</p>
 */
public final class FtcSensors {

    private FtcSensors() {
        // utility class
    }

    // ------------------------------------------------------------------------------------------------
    // Distance sensors
    // ------------------------------------------------------------------------------------------------

    /**
     * Distance in a caller-selected unit.
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
     * Distance in centimeters.
     */
    public static ScalarSource distanceCm(DistanceSensor sensor) {
        return distance(sensor, DistanceUnit.CM);
    }

    /**
     * Distance in inches.
     */
    public static ScalarSource distanceIn(DistanceSensor sensor) {
        return distance(sensor, DistanceUnit.INCH);
    }

    /**
     * Distance in centimeters from a named device.
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
     * Distance in inches from a named device.
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
     * Raw RGBA channels from an FTC {@link ColorSensor}.
     *
     * <p>The returned value uses {@link Rgba} so robot code does not depend on FTC SDK color types.
     * Channel values are in sensor-native units and may not be limited to 0-255.</p>
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
     * Raw RGBA channels from a named FTC {@link ColorSensor}.
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
     * Normalized RGBA channels from an FTC {@link NormalizedColorSensor}.
     *
     * <p>This is usually the better starting point for color classification because the FTC SDK
     * normalizes the reading into a stable range (typically {@code 0..1}) and supports sensor gain.
     * The alpha channel is commonly useful as an overall brightness / confidence signal.</p>
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
     * Normalized RGBA channels from a named FTC {@link NormalizedColorSensor}.
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
     * True when the touch sensor is pressed.
     */
    public static BooleanSource touchPressed(TouchSensor sensor) {
        if (sensor == null) {
            throw new IllegalArgumentException("sensor is required");
        }
        return BooleanSource.of(sensor::isPressed).memoized();
    }

    /**
     * True when a named {@link TouchSensor} is pressed.
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
        // Be explicit: this is an input adapter.
        ch.setMode(DigitalChannel.Mode.INPUT);
        return ch;
    }

    /**
     * Raw digital input state.
     *
     * <p>FTC {@link DigitalChannel#getState()} returns {@code true} when the pin is HIGH.
     * Some sensors are wired <b>active-low</b> (LOW means "active"), so be deliberate about
     * whether you want {@link #digitalHigh(DigitalChannel)} or {@link #digitalLow(DigitalChannel)}.</p>
     */
    public static BooleanSource digitalHigh(DigitalChannel channel) {
        DigitalChannel ch = requireDigitalInput(channel);
        return BooleanSource.of(ch::getState).memoized();
    }

    /**
     * Raw digital input state, inverted.
     *
     * <p>This returns true when the pin is LOW (active-low sensors).</p>
     */
    public static BooleanSource digitalLow(DigitalChannel channel) {
        DigitalChannel ch = requireDigitalInput(channel);
        return BooleanSource.of(() -> !ch.getState()).memoized();
    }

    /**
     * Raw digital HIGH state from a named {@link DigitalChannel}.
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
     * Raw digital LOW state from a named {@link DigitalChannel}.
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
     * Analog voltage in the sensor's native unit (Volts).
     */
    public static ScalarSource analogVoltage(AnalogInput input) {
        if (input == null) {
            throw new IllegalArgumentException("input is required");
        }
        return ScalarSource.of(input::getVoltage).memoized();
    }

    /**
     * Analog voltage from a named {@link AnalogInput}.
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
}
