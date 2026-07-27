package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;

/**
 * FTC-boundary factory for common drivebases.
 *
 * <p>{@link #mecanum(HardwareMap)} is the beginner entry point. A robot with custom wiring or
 * tuning uses the same factory with one complete {@link MecanumConfig}. Both paths return the
 * actual framework-generic {@link MecanumDrivebase}; there is no second FTC runtime wrapper.</p>
 *
 * <pre>{@code
 * MecanumDrivebase drive = FtcDrives.mecanum(hardwareMap);
 * }</pre>
 *
 * <p>Construction resolves the complete motor group before configuring direction and zero-power
 * behavior. It does not write motor power or change run mode. The first drive command performs the
 * coordinated raw-power-mode preflight before any requested wheel power is written.</p>
 */
public final class FtcDrives {

    private FtcDrives() {
        // utility class; no instances
    }

    /** Default front-left motor name. */
    public static final String DEFAULT_FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";

    /** Default front-right motor name. */
    public static final String DEFAULT_FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";

    /** Default back-left motor name. */
    public static final String DEFAULT_BACK_LEFT_MOTOR_NAME = "backLeftMotor";

    /** Default back-right motor name. */
    public static final String DEFAULT_BACK_RIGHT_MOTOR_NAME = "backRightMotor";

    /**
     * Physical FTC wiring for a mecanum drivetrain.
     *
     * <p>This smaller value remains separate from {@link MecanumConfig} because Pedro translation
     * and direction-focused tools also consume the same motor names and directions without owning
     * the direct drivebase's zero-power behavior or mixer tuning.</p>
     */
    public static final class MecanumWiringConfig {

        /** Motor config name for the front-left motor. */
        public String frontLeftName = DEFAULT_FRONT_LEFT_MOTOR_NAME;

        /** Motor config name for the front-right motor. */
        public String frontRightName = DEFAULT_FRONT_RIGHT_MOTOR_NAME;

        /** Motor config name for the back-left motor. */
        public String backLeftName = DEFAULT_BACK_LEFT_MOTOR_NAME;

        /** Motor config name for the back-right motor. */
        public String backRightName = DEFAULT_BACK_RIGHT_MOTOR_NAME;

        /** Logical direction for the front-left motor. */
        public Direction frontLeftDirection = Direction.FORWARD;

        /** Logical direction for the front-right motor. */
        public Direction frontRightDirection = Direction.REVERSE;

        /** Logical direction for the back-left motor. */
        public Direction backLeftDirection = Direction.FORWARD;

        /** Logical direction for the back-right motor. */
        public Direction backRightDirection = Direction.REVERSE;

        private MecanumWiringConfig() {
            // Defaults assigned in field initializers.
        }

        /** Returns a new wiring config populated with Phoenix defaults. */
        public static MecanumWiringConfig defaults() {
            return new MecanumWiringConfig();
        }

        /** Returns an independent copy of this wiring config. */
        public MecanumWiringConfig copy() {
            MecanumWiringConfig copy = new MecanumWiringConfig();
            copy.frontLeftName = frontLeftName;
            copy.frontRightName = frontRightName;
            copy.backLeftName = backLeftName;
            copy.backRightName = backRightName;
            copy.frontLeftDirection = frontLeftDirection;
            copy.frontRightDirection = frontRightDirection;
            copy.backLeftDirection = backLeftDirection;
            copy.backRightDirection = backRightDirection;
            return copy;
        }
    }

    /** Complete FTC construction configuration for a direct mecanum drivebase. */
    public static final class MecanumConfig {

        /** FTC motor names and logical directions. */
        public MecanumWiringConfig wiring = MecanumWiringConfig.defaults();

        /** Whether zero motor power should actively brake instead of float. */
        public boolean enableZeroPowerBrake = true;

        /** Open-loop mixer scaling owned by the resulting drivebase. */
        public MecanumDrivebase.Config drivebase = MecanumDrivebase.Config.defaults();

        private MecanumConfig() {
            // Defaults assigned in field initializers.
        }

        /** Returns a new complete mecanum config populated with Phoenix defaults. */
        public static MecanumConfig defaults() {
            return new MecanumConfig();
        }

        /**
         * Returns an independent deep copy of this config.
         *
         * @throws NullPointerException if either required nested config is {@code null}
         * @throws IllegalArgumentException if a drivebase scale is non-finite or outside
         * [{@code 0.0}, {@code 1.0}]
         */
        public MecanumConfig copy() {
            MecanumConfig copy = new MecanumConfig();
            copy.wiring = Objects.requireNonNull(wiring, "wiring").copy();
            copy.enableZeroPowerBrake = enableZeroPowerBrake;
            copy.drivebase = Objects.requireNonNull(drivebase, "drivebase").copy();
            return copy;
        }
    }

    /**
     * Creates a direct mecanum drivebase with standard wiring, braking, and mixer scaling.
     *
     * @param hardwareMap FTC hardware map used to resolve the four motors
     * @return a new direct mecanum drivebase
     * @throws IllegalArgumentException if {@code hardwareMap} is {@code null}
     */
    public static MecanumDrivebase mecanum(HardwareMap hardwareMap) {
        return mecanum(hardwareMap, MecanumConfig.defaults());
    }

    /**
     * Creates a direct mecanum drivebase from one complete configuration snapshot.
     *
     * <p>Every name and direction is validated before the first hardware lookup. All four motors
     * are then resolved exactly once before direction and BRAKE/FLOAT configuration begins.
     * Construction deliberately leaves power and run mode untouched.</p>
     *
     * @param hardwareMap FTC hardware map used to resolve the four motors
     * @param config complete direct-drive configuration; defensively copied
     * @return a new direct mecanum drivebase
     * @throws IllegalArgumentException if {@code hardwareMap} or {@code config} is {@code null}, or
     * if wiring contains a null/blank name, trim-equivalent duplicate, or null direction, or if a
     * drivebase scale is non-finite or outside [{@code 0.0}, {@code 1.0}]
     * @throws NullPointerException if a required nested config is {@code null}
     */
    public static MecanumDrivebase mecanum(HardwareMap hardwareMap, MecanumConfig config) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap is required");
        }
        if (config == null) {
            throw new IllegalArgumentException("mecanum config is required");
        }

        MecanumConfig snapshot = config.copy();
        MecanumWiringConfig wiring = snapshot.wiring;
        List<PowerOutput> motors = FtcHardware.motorPowerGroup(
                hardwareMap,
                Arrays.asList(
                        wiring.frontLeftName,
                        wiring.frontRightName,
                        wiring.backLeftName,
                        wiring.backRightName),
                Arrays.asList(
                        wiring.frontLeftDirection,
                        wiring.frontRightDirection,
                        wiring.backLeftDirection,
                        wiring.backRightDirection),
                snapshot.enableZeroPowerBrake
                        ? DcMotor.ZeroPowerBehavior.BRAKE
                        : DcMotor.ZeroPowerBehavior.FLOAT);

        return new MecanumDrivebase(
                motors.get(0),
                motors.get(1),
                motors.get(2),
                motors.get(3),
                snapshot.drivebase);
    }
}
