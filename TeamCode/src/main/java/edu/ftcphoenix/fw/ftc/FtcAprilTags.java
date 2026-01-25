package edu.ftcphoenix.fw.ftc;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

/**
 * Small helpers for building FTC {@link AprilTagLibrary} instances.
 *
 * <p>This is mainly useful for <b>custom printed tags</b> (practice fields, classroom demos, etc.) where you
 * may not have access to an official field layout but you still want the vision pipeline to know the
 * correct tag <b>size</b>.</p>
 */
public final class FtcAprilTags {

    private FtcAprilTags() {
        // utility
    }

    /**
     * Returns the SDK's "current game" AprilTag library.
     */
    public static AprilTagLibrary currentGameLibrary() {
        return AprilTagGameDatabase.getCurrentGameTagLibrary();
    }

    /**
     * Builds a library containing a single custom tag.
     *
     * <p>Use this when you're printing a tag (for example with the Limelight AprilTag generator) and you want
     * pose estimates that account for the printed size.</p>
     *
     * @param id         AprilTag ID
     * @param name       optional name (null is ok)
     * @param sizeInches physical tag size (edge length), in inches
     */
    public static AprilTagLibrary singleTagLibrary(int id, String name, double sizeInches) {
        return new AprilTagLibrary.Builder()
                .addTag(id, name != null ? name : ("Tag " + id), sizeInches, DistanceUnit.INCH)
                .build();
    }

    /**
     * Builds a library that contains the SDK's current-game tags plus one custom tag.
     *
     * <p>This is handy if you want the option to see official field tags <em>and</em> a practice tag, or if you
     * want to override the size for a specific ID.</p>
     *
     * @param id             AprilTag ID
     * @param name           optional name (null is ok)
     * @param sizeInches     physical tag size (edge length), in inches
     * @param allowOverwrite if true, the custom tag may replace an existing tag with the same ID
     */
    public static AprilTagLibrary currentGamePlusSingleTag(int id, String name, double sizeInches, boolean allowOverwrite) {
        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
        if (allowOverwrite) {
            builder.setAllowOverwrite(true);
        }

        builder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        builder.addTag(id, name != null ? name : ("Custom Tag " + id), sizeInches, DistanceUnit.INCH);
        return builder.build();
    }
}
