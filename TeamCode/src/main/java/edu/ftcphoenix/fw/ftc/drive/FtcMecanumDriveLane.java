package edu.ftcphoenix.fw.ftc.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.ChassisSpeeds;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.ftc.FtcDrives;

/**
 * FTC-boundary lane owner for a mecanum drivetrain.
 *
 * <p>
 * This class sits one level above the drive primitives. {@link MecanumDrivebase} remains the
 * generic mixer / sink. {@link FtcDrives} remains the FTC helper for creating hardware adapters.
 * {@code FtcMecanumDriveLane} is the stable owner that bundles FTC wiring, brake configuration,
 * drivebase construction, lifecycle, and debug delegation into one reusable lane object that can
 * be reused year to year.
 * </p>
 *
 * <p>
 * The lane deliberately does <em>not</em> choose any operator controls. Stick axes, slow mode,
 * split-driver control, aim assists, and other control semantics belong in robot-owned control
 * objects, not in the framework hardware lane.
 * </p>
 */
public final class FtcMecanumDriveLane implements DriveCommandSink {

    /**
     * Configuration for a mecanum hardware lane.
     *
     * <p>
     * The config groups exactly the pieces that are stable across robots and seasons at the drive
     * hardware/lifecycle layer: wiring, zero-power behavior, and the drivebase's open-loop tuning.
     * Robot-specific control semantics belong elsewhere.
     * </p>
     */
    public static final class Config {

        /**
         * FTC motor names and logical directions for the four mecanum motors.
         */
        public FtcDrives.MecanumWiringConfig wiring = FtcDrives.MecanumWiringConfig.defaults();

        /**
         * If true, the lane configures the drivetrain motors to BRAKE when commanded power is zero.
         */
        public boolean zeroPowerBrake = true;

        /**
         * Open-loop drivebase tuning for the underlying {@link MecanumDrivebase}.
         */
        public MecanumDrivebase.Config drivebase = MecanumDrivebase.Config.defaults();

        private Config() {
            // Defaults assigned in field initializers.
        }

        /**
         * Creates a config populated with Phoenix's default mecanum lane values.
         *
         * @return new mutable config instance
         */
        public static Config defaults() {
            return new Config();
        }

        /**
         * Creates a deep copy of this config.
         *
         * @return copied config whose nested objects can be edited independently
         */
        public Config copy() {
            Config c = new Config();
            c.wiring = this.wiring.copy();
            c.zeroPowerBrake = this.zeroPowerBrake;
            c.drivebase = this.drivebase.copy();
            return c;
        }
    }

    private final Config cfg;
    private final MecanumDrivebase drivebase;

    /**
     * Creates the mecanum lane from one FTC hardware map and one config snapshot.
     *
     * @param hardwareMap FTC hardware map used to create and configure drive hardware
     * @param config lane config; defensively copied for local ownership
     */
    public FtcMecanumDriveLane(HardwareMap hardwareMap, Config config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.cfg = Objects.requireNonNull(config, "config").copy();
        this.drivebase = FtcDrives.mecanum(hardwareMap, this.cfg.wiring, this.cfg.drivebase);
        FtcDrives.setDriveBrake(hardwareMap, this.cfg.wiring, this.cfg.zeroPowerBrake);
    }

    /**
     * Returns a defensive copy of the lane config owned by this drivetrain.
     *
     * @return copied config snapshot describing wiring, brake behavior, and drive tuning
     */
    public Config config() {
        return cfg.copy();
    }

    /**
     * Returns the underlying generic drivebase implementation.
     *
     * <p>
     * Most robot code should talk to the lane through {@link #update(LoopClock)},
     * {@link #drive(DriveSignal)}, and {@link #stop()}. This accessor exists for advanced cases
     * such as reusable helpers that truly need the lower-level type.
     * </p>
     *
     * @return underlying mecanum drivebase owned by this lane
     */
    public MecanumDrivebase drivebase() {
        return drivebase;
    }

    /**
     * Updates per-loop timing on the underlying drivebase.
     *
     * @param clock loop clock whose {@code dtSec} should be used for optional rate limiting
     */
    @Override
    public void update(LoopClock clock) {
        drivebase.update(clock);
    }

    /**
     * Applies a normalized robot-centric drive command for the current loop.
     *
     * @param signal normalized drive command to send to the drivetrain
     */
    @Override
    public void drive(DriveSignal signal) {
        drivebase.drive(signal);
    }

    /**
     * Applies a best-effort physical-velocity intent through the underlying drivebase.
     *
     * @param speeds chassis-speed intent expressed in robot-centric physical units
     */
    public void drive(ChassisSpeeds speeds) {
        drivebase.drive(speeds);
    }

    /**
     * Stops drivetrain output immediately.
     */
    @Override
    public void stop() {
        drivebase.stop();
    }

    /**
     * Dumps the lane's live debug state.
     *
     * @param dbg debug sink to write to; ignored when {@code null}
     * @param prefix key prefix for all entries; may be {@code null} or empty
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "driveLane" : prefix;
        dbg.addData(p + ".zeroPowerBrake", cfg.zeroPowerBrake);
        dbg.addData(p + ".wiring.frontLeftName", cfg.wiring.frontLeftName);
        dbg.addData(p + ".wiring.frontRightName", cfg.wiring.frontRightName);
        dbg.addData(p + ".wiring.backLeftName", cfg.wiring.backLeftName);
        dbg.addData(p + ".wiring.backRightName", cfg.wiring.backRightName);
        drivebase.debugDump(dbg, p + ".drivebase");
    }
}
