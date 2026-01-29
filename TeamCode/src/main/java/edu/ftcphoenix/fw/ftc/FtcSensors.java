package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.ftcphoenix.fw.core.color.Rgba;
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
}
