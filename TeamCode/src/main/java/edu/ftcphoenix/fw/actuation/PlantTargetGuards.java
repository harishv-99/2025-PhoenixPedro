package edu.ftcphoenix.fw.actuation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.control.SlewRateLimiter;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Dynamic plant-level target guards.
 *
 * <p>Target guards are for hardware protection, not ordinary robot behavior policy. Behavior
 * should decide what the robot wants by composing {@code ScalarSource}s, using tools such as
 * {@code BooleanSource.choose(...)}, {@code ScalarOverlayStack}, and
 * {@code ScalarSource.fallbackUnless(...)}. Target guards decide what a specific piece of hardware
 * may safely apply after that behavior target has been sampled.</p>
 *
 * <h2>Common builder usage</h2>
 * <pre>{@code
 * PositionPlant lift = FtcActuators.plant(hardwareMap)
 *     .motor("lift", Direction.FORWARD)
 *     .position()
 *     ...
 *     .targetGuards()
 *         .maxTargetRate(1200.0)
 *         .holdLastTargetUnless("wristClear", wristClear)
 *         .doneTargetGuards()
 *     .targetedByDefaultWritable(0.0)
 *     .build();
 * }</pre>
 */
public final class PlantTargetGuards {

    /**
     * Target-aware safety gate.
     */
    public interface TargetGate {
        /**
         * Return whether {@code candidateTarget} may be applied this loop.
         */
        boolean allowsTarget(double candidateTarget, LoopClock clock);
    }

    /**
     * Result of applying target guards.
     */
    public static final class Result {
        public final double target;
        public final PlantTargetStatus status;

        private Result(double target, PlantTargetStatus status) {
            this.target = target;
            this.status = Objects.requireNonNull(status, "status");
        }
    }

    /**
     * Builder for a guard chain.
     */
    public static final class Builder {
        private final List<GuardRule> rules = new ArrayList<>();
        private Double maxUpPerSec;
        private Double maxDownPerSec;

        /**
         * Limit applied target change symmetrically in units/sec; the rate must be finite and > 0.
         */
        public Builder maxTargetRate(double maxDeltaPerSec) {
            return maxTargetRates(maxDeltaPerSec, maxDeltaPerSec);
        }

        /**
         * Limit applied target change with separate up/down rates in units/sec; each rate must be finite and > 0.
         */
        public Builder maxTargetRates(double maxUpPerSec, double maxDownPerSec) {
            requireRate(maxUpPerSec, "maxUpPerSec");
            requireRate(maxDownPerSec, "maxDownPerSec");
            this.maxUpPerSec = maxUpPerSec;
            this.maxDownPerSec = maxDownPerSec;
            return this;
        }

        /**
         * Hold the previous applied target while {@code allowed} is low.
         */
        public Builder holdLastTargetUnless(String name, BooleanSource allowed) {
            Objects.requireNonNull(allowed, "allowed");
            return holdLastTargetUnless(name, (candidate, clock) -> allowed.getAsBoolean(clock));
        }

        /**
         * Hold the previous applied target while the target-aware gate rejects the candidate.
         */
        public Builder holdLastTargetUnless(String name, TargetGate gate) {
            rules.add(GuardRule.holdLast(cleanName(name), Objects.requireNonNull(gate, "gate")));
            return this;
        }

        /**
         * Replace the candidate with {@code fallbackTarget} while {@code allowed} is low.
         */
        public Builder fallbackTargetUnless(String name, BooleanSource allowed, double fallbackTarget) {
            Objects.requireNonNull(allowed, "allowed");
            return fallbackTargetUnless(name, (candidate, clock) -> allowed.getAsBoolean(clock), fallbackTarget);
        }

        /**
         * Replace the candidate with {@code fallbackTarget} while the target-aware gate rejects it.
         */
        public Builder fallbackTargetUnless(String name, TargetGate gate, double fallbackTarget) {
            if (!Double.isFinite(fallbackTarget)) {
                throw new IllegalArgumentException("fallbackTarget must be finite, got " + fallbackTarget);
            }
            rules.add(GuardRule.fallback(cleanName(name), Objects.requireNonNull(gate, "gate"), fallbackTarget));
            return this;
        }

        /**
         * Build an immutable guard chain.
         */
        public PlantTargetGuards build() {
            SlewRateLimiter limiter = null;
            if (maxUpPerSec != null) limiter = new SlewRateLimiter(maxUpPerSec, maxDownPerSec);
            return new PlantTargetGuards(rules.toArray(new GuardRule[0]), limiter);
        }

        private static void requireRate(double rate, String name) {
            if (!Double.isFinite(rate) || rate <= 0.0) {
                throw new IllegalArgumentException(name + " must be finite and > 0, got " + rate);
            }
        }
    }

    private enum RuleKind {HOLD_LAST, FALLBACK}

    private static final class GuardRule {
        final RuleKind kind;
        final String name;
        final TargetGate gate;
        final double fallbackTarget;
        boolean lastAllowed = true;

        private GuardRule(RuleKind kind, String name, TargetGate gate, double fallbackTarget) {
            this.kind = kind;
            this.name = name;
            this.gate = gate;
            this.fallbackTarget = fallbackTarget;
        }

        static GuardRule holdLast(String name, TargetGate gate) {
            return new GuardRule(RuleKind.HOLD_LAST, name, gate, 0.0);
        }

        static GuardRule fallback(String name, TargetGate gate, double fallbackTarget) {
            return new GuardRule(RuleKind.FALLBACK, name, gate, fallbackTarget);
        }
    }

    private final GuardRule[] rules;
    private final SlewRateLimiter limiter;
    private double lastOut;
    private PlantTargetStatus lastStatus = PlantTargetStatus.STOPPED;
    private boolean hasApplied;

    private PlantTargetGuards(GuardRule[] rules, SlewRateLimiter limiter) {
        this.rules = Objects.requireNonNull(rules, "rules");
        this.limiter = limiter;
    }

    /**
     * Empty guard chain.
     */
    public static PlantTargetGuards none() {
        return new PlantTargetGuards(new GuardRule[0], null);
    }

    /**
     * Start a new guard-chain builder.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Apply the guard chain to a candidate target.
     *
     * @param candidate       candidate after behavior sampling and static range/reference handling
     * @param currentStatus   status before dynamic guards, usually accepted or clamped
     * @param previousApplied previous applied plant target
     * @param clock           current loop clock
     */
    public Result apply(double candidate,
                        PlantTargetStatus currentStatus,
                        double previousApplied,
                        LoopClock clock) {
        PlantTargetStatus status = Objects.requireNonNull(currentStatus, "currentStatus");
        double out = candidate;

        for (GuardRule rule : rules) {
            boolean allowed = rule.gate.allowsTarget(out, clock);
            rule.lastAllowed = allowed;
            if (allowed) continue;
            if (rule.kind == RuleKind.HOLD_LAST) {
                out = previousApplied;
                status = PlantTargetStatus.holdingLast(rule.name);
                break;
            }
            out = rule.fallbackTarget;
            status = PlantTargetStatus.fallbackActive(rule.name);
        }

        if (limiter != null) {
            if (!hasApplied) {
                limiter.reset(previousApplied, clock);
                if (Math.abs(out - previousApplied) > 1e-9) {
                    status = PlantTargetStatus.rateLimited("rate limited from previous applied target");
                }
                out = previousApplied;
            } else {
                double limited = limiter.calculate(out, clock);
                if (limiter.wasLimited()) {
                    status = PlantTargetStatus.rateLimited("rate limited toward target");
                }
                out = limited;
            }
        }

        hasApplied = true;
        lastOut = out;
        lastStatus = status;
        return new Result(out, status);
    }

    /**
     * Reset dynamic state such as rate limiters and remembered guard output.
     */
    public void reset() {
        if (limiter != null) limiter.reset();
        hasApplied = false;
        lastOut = 0.0;
        lastStatus = PlantTargetStatus.STOPPED;
        for (GuardRule rule : rules) rule.lastAllowed = true;
    }

    /**
     * True when no dynamic guard rules or rate limiter are configured.
     */
    public boolean isEmpty() {
        return rules.length == 0 && limiter == null;
    }

    /**
     * Emit guard state for debug telemetry.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "targetGuards" : prefix;
        dbg.addData(p + ".class", "PlantTargetGuards")
                .addData(p + ".ruleCount", rules.length)
                .addData(p + ".lastOut", lastOut)
                .addData(p + ".lastStatus", lastStatus.toString());
        for (int i = 0; i < rules.length; i++) {
            GuardRule rule = rules[i];
            dbg.addData(p + ".rule" + i + ".name", rule.name)
                    .addData(p + ".rule" + i + ".kind", rule.kind.name())
                    .addData(p + ".rule" + i + ".lastAllowed", rule.lastAllowed);
        }
        if (limiter != null) limiter.debugDump(dbg, p + ".limiter");
    }

    private static String cleanName(String name) {
        if (name == null || name.trim().isEmpty()) return "interlock";
        return name.trim();
    }
}
