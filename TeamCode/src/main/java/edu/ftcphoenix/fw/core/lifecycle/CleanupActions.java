package edu.ftcphoenix.fw.core.lifecycle;

/**
 * Ordered best-effort execution for caller-owned cleanup and compensating actions.
 *
 * <p>The caller remains responsible for choosing eligible actions, putting them in a safe order,
 * and enforcing any exact-once or terminal lifecycle state. This utility owns only the repetitive
 * failure mechanics: every action is attempted after a {@link RuntimeException}, the first failure
 * remains primary, and later failures are attached to it as suppressed exceptions.</p>
 *
 * <p>Suppressed-failure retention follows Java's {@link Throwable#addSuppressed(Throwable)}
 * contract; a custom primary exception that disables suppression cannot retain those failures.</p>
 *
 * <p>This utility catches {@code RuntimeException}, not {@link Error}. An {@code Error} stops the
 * operation immediately and is propagated unchanged.</p>
 */
public final class CleanupActions {

    private CleanupActions() {
        // Static utility class.
    }

    /**
     * Attempts every cleanup action in the supplied order.
     *
     * <p>If one or more actions throw {@code RuntimeException}, the first failure is rethrown after
     * every remaining action has been attempted. Later failures are attached to the first as
     * suppressed exceptions, in action order. A later action that throws the same exception object
     * as the first is not suppressed onto itself.</p>
     *
     * <p>An individual null action is treated as an indexed cleanup failure at that position;
     * later actions are still attempted.</p>
     *
     * @param cleanupActions cleanup or compensating actions in the required safety order
     * @throws NullPointerException if the action array itself is null
     * @throws RuntimeException     after all actions are attempted if any action fails or is null
     */
    public static void attemptAll(Runnable... cleanupActions) {
        if (cleanupActions == null) {
            throw nullActionArrayFailure();
        }

        RuntimeException failure = attemptActions(null, cleanupActions);
        if (failure != null) {
            throw failure;
        }
    }

    /**
     * Attempts every cleanup action while preserving an already-owned primary failure.
     *
     * <p>Every cleanup {@code RuntimeException} is attached directly to
     * {@code primaryFailure} as a suppressed exception, in action order. That same supplied primary
     * object is returned so the caller can explicitly rethrow, wrap, retain, or report it. A cleanup
     * action that throws the primary itself is not suppressed onto itself.</p>
     *
     * <p>A null action array or individual null action is represented as a cleanup failure attached
     * to the primary; any subsequent individual actions are still attempted.</p>
     *
     * @param primaryFailure already-owned failure that cleanup must not replace
     * @param cleanupActions cleanup or compensating actions in the required safety order
     * @return the exact {@code primaryFailure} object, enriched with any cleanup failures
     * @throws NullPointerException if {@code primaryFailure} is null; no cleanup is attempted
     */
    public static RuntimeException attemptAllAfterFailure(
            RuntimeException primaryFailure,
            Runnable... cleanupActions) {
        if (primaryFailure == null) {
            throw new NullPointerException("primaryFailure must not be null");
        }
        if (cleanupActions == null) {
            addSuppressedIfDistinct(primaryFailure, nullActionArrayFailure());
            return primaryFailure;
        }

        return attemptActions(primaryFailure, cleanupActions);
    }

    private static RuntimeException attemptActions(
            RuntimeException primaryFailure,
            Runnable[] cleanupActions) {
        RuntimeException firstFailure = primaryFailure;
        for (int index = 0; index < cleanupActions.length; index++) {
            try {
                Runnable action = cleanupActions[index];
                if (action == null) {
                    throw new NullPointerException(
                            "cleanupActions[" + index + "] must not be null");
                }
                action.run();
            } catch (RuntimeException failure) {
                if (firstFailure == null) {
                    firstFailure = failure;
                } else {
                    addSuppressedIfDistinct(firstFailure, failure);
                }
            }
        }
        return firstFailure;
    }

    private static void addSuppressedIfDistinct(
            RuntimeException primaryFailure,
            RuntimeException cleanupFailure) {
        if (primaryFailure != cleanupFailure) {
            primaryFailure.addSuppressed(cleanupFailure);
        }
    }

    private static NullPointerException nullActionArrayFailure() {
        return new NullPointerException("cleanupActions must not be null");
    }
}
