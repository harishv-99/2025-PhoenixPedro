package edu.ftcsushi.fw.tools.tester.calibration;

import java.util.function.Supplier;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcsushi.fw.tools.tester.TesterContext;

/** Private, failure-isolated presentation of the calibration owner's already-retained facts. */
final class CalibrationReport {
    private boolean available;
    private long generation;

    /** Replace the old candidate only; optional formatting/transport cannot interrupt control. */
    void publish(TesterContext ctx, String filename, Supplier<String> frozenText) {
        if (ctx == null) return;
        String text;
        try {
            text = frozenText.get();
        } catch (RuntimeException unavailable) {
            clear(ctx);
            return;
        }
        // Formatting has finished before calling an injected transfer implementation.
        long expected = generation + 1;
        clear(ctx);
        if (generation != expected) return;
        try {
            boolean accepted = ctx.downloads.publish(filename, text);
            if (generation == expected) available = accepted;
            else clear(ctx);
        } catch (RuntimeException unavailable) {
            available = false;
        }
    }

    /** Withdraw the local link even when an injected transport cannot clear its storage. */
    void clear(TesterContext ctx) {
        generation++;
        available = false;
        if (ctx == null) return;
        try {
            ctx.downloads.clear();
        } catch (RuntimeException unavailable) {
            // Download cleanup is never a prerequisite for hardware cleanup.
        }
    }

    /** Add at most one row to the existing frame; never commit telemetry. */
    void render(TesterContext ctx) {
        if (!available || ctx == null) return;
        try {
            String url = ctx.downloads.url();
            if (url != null) ctx.telemetry.addData("Frozen calibration report", url);
        } catch (RuntimeException unavailable) {
            // Optional download presentation must not suppress required driver status.
        }
    }

    /** Common evidence limits; processing time is not a claimed sensor acquisition time. */
    static StringBuilder begin(TesterContext ctx, String title) {
        return begin(ctx.clock.nowSec(), ctx.clock.cycle(), title);
    }

    /** Format a processing boundary saved before a cleanup callback could reset its clock. */
    static StringBuilder begin(double processingTimeSec, long cycle, String title) {
        return new StringBuilder(title).append('\n')
                .append("Frozen software evidence; NOT human acceptance or physical validation.\n")
                .append("Processing time sec: ").append(processingTimeSec)
                .append("; loop cycle: ").append(cycle).append('\n')
                .append("Build/configuration revision, generic vs robot-configured selection, deployment,\n")
                .append("independent physical references, human acceptance, configured retest and\n")
                .append("production-owner verification: UNRECORDED; join the external calibration record.\n")
                .append("Robot frame: +X forward, +Y left, +Z up; positive yaw is CCW.\n")
                .append("Pose translations are inches; pose angles are radians.\n");
    }

    /** Print captured configuration, not readback or proof of a particular physical device. */
    static void pinpoint(StringBuilder text, PinpointOdometryPredictor.Config cfg) {
        text.append("Captured Pinpoint configuration (not hardware readback):\n")
                .append("hardwareMapName: ").append(cfg.hardwareMapName).append('\n')
                .append("forwardPodOffsetLeftInches: ").append(cfg.forwardPodOffsetLeftInches).append('\n')
                .append("strafePodOffsetForwardInches: ").append(cfg.strafePodOffsetForwardInches).append('\n')
                .append("encoderResolution: ").append(cfg.encoderResolution).append('\n')
                .append("forwardPodDirection: ").append(cfg.forwardPodDirection).append('\n')
                .append("strafePodDirection: ").append(cfg.strafePodDirection).append('\n')
                .append("yawScalar: ").append(cfg.yawScalar == null ? "factory setting retained" : cfg.yawScalar).append('\n')
                .append("configured quality: ").append(cfg.quality).append('\n');
    }

    /** Preserve full-precision field/body pose coordinates rather than rounded display text. */
    static void pose(StringBuilder text, String label, Pose2d pose) {
        text.append(label).append(" (x in, y in, yaw rad): ");
        if (pose == null) text.append("UNAVAILABLE\n");
        else text.append(pose.xInches).append(", ").append(pose.yInches).append(", ")
                .append(pose.headingRad).append('\n');
    }

    /** Preserve all six existing transform coordinates without computing a new transform. */
    static void pose(StringBuilder text, String label, Pose3d pose) {
        text.append(label).append(" (x/y/z in, yaw/pitch/roll rad): ");
        if (pose == null) text.append("UNAVAILABLE\n");
        else text.append(pose.xInches).append(", ").append(pose.yInches).append(", ")
                .append(pose.zInches).append(", ").append(pose.yawRad).append(", ")
                .append(pose.pitchRad).append(", ").append(pose.rollRad).append('\n');
    }

    /** Freeze only a still-valid clock relationship; never reconstruct an old reset epoch. */
    static void captureAge(StringBuilder text, String label, LoopTimestamp timestamp,
                           TesterContext ctx) {
        double age = timestamp == null ? Double.NaN : timestamp.ageSec(ctx.clock);
        text.append(label).append(" age at report sec: ")
                .append(Double.isFinite(age) ? Double.toString(age) : "UNAVAILABLE").append('\n');
    }
}
