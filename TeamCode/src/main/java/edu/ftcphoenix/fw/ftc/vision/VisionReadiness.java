package edu.ftcphoenix.fw.ftc.vision;

import java.util.Objects;

/**
 * Immutable readiness of one configured vision component.
 *
 * <p>Readiness means that the component can safely provide results for its configured purpose. It
 * does not mean that a target is currently visible. The reason is intended for operator telemetry
 * while a camera opens, a processor is disabled, or a Limelight pipeline settles.</p>
 */
public final class VisionReadiness {

    private static final VisionReadiness READY = new VisionReadiness(true, "ready");

    private final boolean ready;
    private final String reason;

    private VisionReadiness(boolean ready, String reason) {
        this.ready = ready;
        this.reason = reason;
    }

    /**
     * Returns the shared ready value.
     *
     * @return component-ready result
     */
    public static VisionReadiness ready() {
        return READY;
    }

    /**
     * Creates a not-ready value with an actionable operator-facing reason.
     *
     * @param reason concise explanation of what is not ready and, when practical, what to check
     * @return immutable not-ready result
     * @throws NullPointerException     if {@code reason} is null
     * @throws IllegalArgumentException if {@code reason} is blank
     */
    public static VisionReadiness notReady(String reason) {
        String requiredReason = Objects.requireNonNull(reason, "reason").trim();
        if (requiredReason.isEmpty()) {
            throw new IllegalArgumentException("Vision readiness reason must not be blank");
        }
        return new VisionReadiness(false, requiredReason);
    }

    /**
     * Returns whether the configured component can safely provide results.
     *
     * @return {@code true} when ready; independent of whether a target is visible
     */
    public boolean isReady() {
        return ready;
    }

    /**
     * Returns the concise operator-facing readiness reason.
     *
     * @return {@code "ready"} when ready, otherwise an actionable not-ready reason
     */
    public String reason() {
        return reason;
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof VisionReadiness)) {
            return false;
        }
        VisionReadiness that = (VisionReadiness) other;
        return ready == that.ready && reason.equals(that.reason);
    }

    @Override
    public int hashCode() {
        int result = Boolean.valueOf(ready).hashCode();
        result = 31 * result + reason.hashCode();
        return result;
    }

    @Override
    public String toString() {
        return ready ? "READY" : "NOT_READY(" + reason + ")";
    }
}
