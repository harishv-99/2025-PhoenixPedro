package edu.ftcsushi.robots.examples.tagalignment;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveOverlayOutput;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Driver mappings plus cached text about the actual held overlay, never a second guidance query. */
final class TagAlignmentControls {
    private final DriveSource drive;
    private boolean held;
    private DriveOverlayOutput lastAlignment;

    TagAlignmentControls(GamepadDevice driver, DriveGuidancePlan plan) {
        this(manual(driver), driver.leftBumper(), plan);
    }

    /** Hardware-neutral test/custom-input seam: all three arguments are borrowed capabilities. */
    TagAlignmentControls(DriveSource manual, BooleanSource hold, DriveGuidancePlan plan) {
        Objects.requireNonNull(manual, "manual");
        Objects.requireNonNull(hold, "hold");
        DriveOverlay alignment = Objects.requireNonNull(plan, "plan").overlay();
        DriveOverlay observed = new DriveOverlay() {
            @Override public void onEnable(LoopClock clock) {
                lastAlignment = null;
                alignment.onEnable(clock);
            }

            @Override public DriveOverlayOutput get(LoopClock clock) {
                DriveOverlayOutput sampled = alignment.get(clock);
                lastAlignment = sampled;
                return sampled;
            }

            @Override public void onDisable(LoopClock clock) {
                lastAlignment = null;
                alignment.onDisable(clock);
            }
        };
        drive = manual.overlayWhen(clock -> {
            held = hold.getAsBoolean(clock);
            return held;
        }, observed, DriveOverlayMask.ALL);
    }

    DriveSource driveSource() { return drive; }

    /** Formats only the last output-phase decision. It does not claim task completion. */
    String status() {
        if (!held) return "Manual control; hold left bumper to align";
        if (lastAlignment == null || lastAlignment.mask.isNone()) {
            return "Tag unavailable: manual control; fresh tag resumes assist while held";
        }
        if (!DriveOverlayMask.ALL.equals(lastAlignment.mask)) {
            return "Partial assist; unavailable axes remain manual; release for full manual control";
        }
        return "Assisting position and heading; release left bumper for manual control";
    }

    private static DriveSource manual(GamepadDevice driver) {
        Objects.requireNonNull(driver, "driver").setAxisDeadband(0.02);
        GamepadDriveSource.Config config = GamepadDriveSource.Config.defaults();
        config.deadband = 0.05;
        config.translateExpo = 1.5;
        config.rotateExpo = 1.5;
        config.translateScale = 1.0;
        config.rotateScale = 1.0;
        return new GamepadDriveSource(driver.leftX(), driver.leftY(), driver.rightX(), config);
    }
}
