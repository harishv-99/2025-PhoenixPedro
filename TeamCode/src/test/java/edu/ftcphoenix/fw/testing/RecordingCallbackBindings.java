package edu.ftcphoenix.fw.testing;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.input.binding.CallbackBindings;

/**
 * Test callback surface that records declaration attempts and can inject one registration failure.
 */
public final class RecordingCallbackBindings implements CallbackBindings {

    private final Bindings root = new Bindings();
    private final int failOnAttempt;
    private int registrationAttempts;
    private int successfulRegistrations;

    /** Create an accepting callback surface. */
    public RecordingCallbackBindings() {
        this(0);
    }

    /**
     * Create a callback surface that fails before delegating the selected one-based attempt.
     * Pass zero to accept every declaration.
     */
    public RecordingCallbackBindings(int failOnAttempt) {
        if (failOnAttempt < 0) {
            throw new IllegalArgumentException("failOnAttempt must be zero or positive");
        }
        this.failOnAttempt = failOnAttempt;
    }

    /** Return the real binding root that receives successful declarations. */
    public Bindings root() {
        return root;
    }

    /** Return how many callback declarations were attempted. */
    public int registrationAttempts() {
        return registrationAttempts;
    }

    /** Return how many callback declarations reached the real binding root. */
    public int successfulRegistrations() {
        return successfulRegistrations;
    }

    @Override
    public void onRise(BooleanSource signal, Runnable action) {
        beforeRegistration();
        root.onRise(signal, action);
        successfulRegistrations++;
    }

    @Override
    public void onFall(BooleanSource signal, Runnable action) {
        beforeRegistration();
        root.onFall(signal, action);
        successfulRegistrations++;
    }

    @Override
    public void mirrorOnChange(BooleanSource signal, Consumer<Boolean> consumer) {
        beforeRegistration();
        root.mirrorOnChange(signal, consumer);
        successfulRegistrations++;
    }

    @Override
    public void whileHigh(BooleanSource signal, Runnable action) {
        beforeRegistration();
        root.whileHigh(signal, action);
        successfulRegistrations++;
    }

    @Override
    public void whileLow(BooleanSource signal, Runnable action) {
        beforeRegistration();
        root.whileLow(signal, action);
        successfulRegistrations++;
    }

    @Override
    public void toggleOnRise(BooleanSource signal,
                             Runnable onEnabled,
                             Runnable onDisabled) {
        beforeRegistration();
        root.toggleOnRise(signal, onEnabled, onDisabled);
        successfulRegistrations++;
    }

    @Override
    public void toggleOnRise(BooleanSource signal, Consumer<Boolean> consumer) {
        beforeRegistration();
        root.toggleOnRise(signal, consumer);
        successfulRegistrations++;
    }

    @Override
    public void nudgeOnRise(BooleanSource increaseSignal,
                            BooleanSource decreaseSignal,
                            double step,
                            DoubleConsumer adjuster) {
        beforeRegistration();
        root.nudgeOnRise(increaseSignal, decreaseSignal, step, adjuster);
        successfulRegistrations++;
    }

    @Override
    public void copyEachCycle(ScalarSource source, DoubleConsumer consumer) {
        beforeRegistration();
        root.copyEachCycle(source, consumer);
        successfulRegistrations++;
    }

    private void beforeRegistration() {
        registrationAttempts++;
        if (failOnAttempt != 0 && registrationAttempts == failOnAttempt) {
            throw new IllegalStateException(
                    "injected callback registration failure at attempt " + failOnAttempt);
        }
    }
}
