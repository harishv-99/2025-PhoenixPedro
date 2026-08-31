package edu.ftcsushi.robots.phoenix;

import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcGameTagLayout;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcsushi.robots.phoenix.scoring.PhoenixScoring;
import edu.ftcsushi.robots.phoenix.scoring.PhoenixTargeting;

/**
 * Mutable data-only assembly choices for the current Phoenix robot.
 *
 * <p>The profile groups independently owned configuration sections without validating them or
 * becoming their runtime owner. {@link #current()} returns a fresh complete graph on every call;
 * long-lived owners defensively capture only the active section they retain.</p>
 *
 * <p>Checked-in names, directions, mounts, calibration acknowledgements, and tuning remain physical
 * claims that must be reviewed on Phoenix. A fresh profile or successful software build does not
 * establish their safety or accuracy.</p>
 */
public final class PhoenixProfile {

    /** Complete FTC mecanum construction configuration. */
    public FtcDrives.MecanumConfig drive;

    /** Phoenix's concrete AprilTag backend choice and backend drafts. */
    public PhoenixVisionFactory.Config vision;

    /** Pinpoint and corrected-localization configuration. */
    public FtcOdometryAprilTagLocalizationLane.Config localization;

    /** Fixed AprilTag field facts shared by localization and targeting. */
    public TagLayout fixedAprilTagLayout;

    /** Phoenix TeleOp input shaping and operator tuning. */
    public PhoenixTeleOpControls.Config controls;

    /** Phoenix TeleOp drive-assist policy tuning. */
    public PhoenixDriveAssistService.Config driveAssist;

    /** Scoring hardware, readiness, feed, and controller tuning. */
    public PhoenixScoring.Config scoring;

    /** Human-reviewed calibration acknowledgements. */
    public PhoenixCalibrationConfig calibration;

    /** Alliance target catalog, aiming, and shot-selection tuning. */
    public PhoenixTargeting.Config targeting;

    /** Autonomous timing, route, aiming, and wait policy. */
    public PhoenixAutoConfig auto;

    private PhoenixProfile() {
    }

    /**
     * Returns a fresh complete graph containing Phoenix's checked-in current configuration.
     *
     * <p>Every mutable Config, catalog, and value graph is independently editable. Immutable
     * reviewed field/table collaborators may be shared by their owning factories.</p>
     *
     * @return fresh mutable current Phoenix profile
     */
    public static PhoenixProfile current() {
        PhoenixProfile profile = new PhoenixProfile();
        profile.drive = PhoenixDriveConfiguration.current();
        profile.vision = PhoenixVisionFactory.Config.defaults();
        profile.localization = PhoenixLocalizationConfiguration.current();
        profile.fixedAprilTagLayout = FtcGameTagLayout.currentGameFieldFixed();
        profile.controls = PhoenixTeleOpControls.Config.defaults();
        profile.driveAssist = PhoenixDriveAssistService.Config.defaults();
        profile.scoring = PhoenixScoring.Config.defaults();
        profile.calibration = PhoenixCalibrationConfiguration.current();
        profile.targeting = PhoenixTargeting.Config.defaults();
        profile.auto = PhoenixAutoConfig.defaults();
        return profile;
    }
}
