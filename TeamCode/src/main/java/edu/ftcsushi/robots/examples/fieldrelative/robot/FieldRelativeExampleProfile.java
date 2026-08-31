package edu.ftcsushi.robots.examples.fieldrelative.robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.localization.FtcImuHeadingEstimator;

/** Data-only configuration for the managed field-relative drive lesson. */
public final class FieldRelativeExampleProfile {

    /** One named practice driver station with independent physical-start and driver-up headings. */
    public static final class Station {
        public final String id;
        public final String label;
        public final double initialRobotFieldHeadingRad;
        public final double controlUpFieldHeadingRad;

        public Station(String id,
                       String label,
                       double initialRobotFieldHeadingRad,
                       double controlUpFieldHeadingRad) {
            if (id == null || id.trim().isEmpty()) {
                throw new IllegalArgumentException("station id must be non-blank");
            }
            if (label == null || label.trim().isEmpty()) {
                throw new IllegalArgumentException("station label must be non-blank");
            }
            if (!Double.isFinite(initialRobotFieldHeadingRad)
                    || !Double.isFinite(controlUpFieldHeadingRad)) {
                throw new IllegalArgumentException("station headings must be finite radians");
            }
            this.id = id.trim();
            this.label = label.trim();
            this.initialRobotFieldHeadingRad = initialRobotFieldHeadingRad;
            this.controlUpFieldHeadingRad = controlUpFieldHeadingRad;
        }
    }

    public FtcDrives.MecanumConfig drive;
    public FtcImuHeadingEstimator.Config imu;
    public GamepadDriveSource.Config manualDrive;
    public List<Station> stations;
    public boolean allowDriveMotion;

    private FieldRelativeExampleProfile() {
    }

    /**
     * Return conservative practice configuration, not claimed BIOBUZZ field geometry.
     * Review hardware and replace the station table before enabling motion.
     */
    public static FieldRelativeExampleProfile current() {
        FieldRelativeExampleProfile profile = new FieldRelativeExampleProfile();
        profile.drive = FtcDrives.MecanumConfig.defaults();
        profile.drive.wiring.frontLeftDirection = Direction.FORWARD;
        profile.drive.wiring.frontRightDirection = Direction.REVERSE;
        profile.drive.wiring.backLeftDirection = Direction.FORWARD;
        profile.drive.wiring.backRightDirection = Direction.REVERSE;
        profile.drive.drivebase.maxAxial = 0.25;
        profile.drive.drivebase.maxLateral = 0.25;
        profile.drive.drivebase.maxOmega = 0.20;

        profile.imu = FtcImuHeadingEstimator.Config.defaults();
        profile.manualDrive = GamepadDriveSource.Config.defaults();
        profile.stations = Collections.unmodifiableList(Arrays.asList(
                new Station("PRACTICE_POS_X", "Practice +X up", 0.0, 0.0),
                new Station("PRACTICE_POS_Y", "Practice +Y up", Math.PI / 2.0, Math.PI / 2.0),
                new Station("PRACTICE_NEG_X", "Practice -X up", Math.PI, Math.PI),
                new Station("PRACTICE_NEG_Y", "Practice -Y up", -Math.PI / 2.0, -Math.PI / 2.0)
        ));
        profile.allowDriveMotion = false;
        return profile;
    }
}
