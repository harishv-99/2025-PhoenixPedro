package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

/**
 * Diagnostic description of how a plant's requested target became its applied target.
 *
 * <p>Robot behavior should usually wait on {@link Plant#atTarget()} or
 * {@link Plant#atTarget(double)}. This status is mainly for telemetry, debugging, and explaining
 * why {@code getRequestedTarget()} and {@code getAppliedTarget()} differ.</p>
 */
public final class PlantTargetStatus {
    /**
     * High-level target handling state.
     */
    public enum Kind {
        /**
         * Requested target passed through unchanged.
         */
        ACCEPTED,
        /**
         * Requested target was clamped to the plant's legal static range.
         */
        CLAMPED_TO_RANGE,
        /**
         * Target is being rate-limited by a plant target guard.
         */
        RATE_LIMITED,
        /**
         * Target is being held at the previous applied value by a safety interlock.
         */
        HOLDING_LAST,
        /**
         * Target was replaced by a fallback value by a safety interlock.
         */
        FALLBACK_ACTIVE,
        /**
         * The plant cannot safely apply targets because its reference is not established.
         */
        REFERENCE_NOT_ESTABLISHED,
        /**
         * The final plant target source did not produce a requested target this loop.
         */
        TARGET_UNAVAILABLE,
        /**
         * Plant has been stopped.
         */
        STOPPED
    }

    public static final PlantTargetStatus ACCEPTED = new PlantTargetStatus(Kind.ACCEPTED, "accepted");
    public static final PlantTargetStatus STOPPED = new PlantTargetStatus(Kind.STOPPED, "stopped");

    private final Kind kind;
    private final String message;

    private PlantTargetStatus(Kind kind, String message) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.message = Objects.requireNonNull(message, "message");
    }

    /**
     * Create a status indicating static-range clamping.
     */
    public static PlantTargetStatus clampedToRange(String message) {
        return new PlantTargetStatus(Kind.CLAMPED_TO_RANGE, clean(message, "clamped to plant range"));
    }

    /**
     * Create a status indicating target-rate limiting.
     */
    public static PlantTargetStatus rateLimited(String message) {
        return new PlantTargetStatus(Kind.RATE_LIMITED, clean(message, "rate limited"));
    }

    /**
     * Create a status indicating a hold-last interlock.
     */
    public static PlantTargetStatus holdingLast(String name) {
        return new PlantTargetStatus(Kind.HOLDING_LAST, "holding last target: " + clean(name, "interlock"));
    }

    /**
     * Create a status indicating a fallback target is active.
     */
    public static PlantTargetStatus fallbackActive(String name) {
        return new PlantTargetStatus(Kind.FALLBACK_ACTIVE, "fallback target active: " + clean(name, "interlock"));
    }

    /**
     * Create a status indicating a missing position reference.
     */
    public static PlantTargetStatus referenceNotEstablished(String reason) {
        return new PlantTargetStatus(Kind.REFERENCE_NOT_ESTABLISHED,
                clean(reason, "position reference not established"));
    }

    /**
     * Create a status indicating that the final target source failed to produce a target.
     */
    public static PlantTargetStatus targetUnavailable(String reason) {
        return new PlantTargetStatus(Kind.TARGET_UNAVAILABLE,
                clean(reason, "plant target source unavailable"));
    }

    /**
     * Return the high-level status kind.
     */
    public Kind kind() {
        return kind;
    }

    /**
     * Return a short human-readable message.
     */
    public String message() {
        return message;
    }

    /**
     * True when requested target passed through unchanged.
     */
    public boolean accepted() {
        return kind == Kind.ACCEPTED;
    }

    private static String clean(String text, String fallback) {
        if (text == null || text.trim().isEmpty()) return fallback;
        return text.trim();
    }

    @Override
    public String toString() {
        return kind + ": " + message;
    }
}
