package edu.ftcphoenix.fw.ftc.ui;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.tools.tester.ui.IntTuner;
import edu.ftcphoenix.fw.tools.tester.ui.ScalarTuner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public final class ContextualUiBindingTest {

    @Test
    public void pickerSelectionDoesNotAlsoEnableNewlyActiveTunerControls() {
        AtomicBoolean selectPressed = new AtomicBoolean();
        AtomicBoolean ready = new AtomicBoolean();
        BooleanSource select = BooleanSource.of(selectPressed::get);

        Bindings bindings = new Bindings();
        SelectionMenu<String> picker = new SelectionMenu<String>()
                .addItem("motor", "motor");
        picker.bind(
                bindings,
                null,
                null,
                select,
                () -> !ready.get(),
                item -> ready.set(true)
        );

        Bindings.ControlContext liveControls = bindings.contextWhen(
                BooleanSource.of(ready::get),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );
        ScalarTuner tuner = new ScalarTuner("Power", -1.0, 1.0, 0.05, 0.2, 0.0);
        IntTuner integerTuner = new IntTuner("Ticks", -100, 100, 1, 10, 0);
        tuner.bind(liveControls, select, null, null, null, null, null);
        integerTuner.bind(liveControls, select, null, null, null, null);

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        update(bindings, clock, 0.02);

        selectPressed.set(true);
        update(bindings, clock, 0.04);
        assertTrue(ready.get());
        assertFalse(tuner.isEnabled());
        assertFalse(integerTuner.isEnabled());

        selectPressed.set(false);
        update(bindings, clock, 0.06);
        assertFalse(tuner.isEnabled());

        selectPressed.set(true);
        update(bindings, clock, 0.08);
        assertTrue(tuner.isEnabled());
        assertTrue(integerTuner.isEnabled());

        // Model BACK to the picker, then choose again. The selection press must not toggle the
        // tuner's retained state, and the reactivated controls must require another release.
        ready.set(false);
        update(bindings, clock, 0.10);
        selectPressed.set(false);
        update(bindings, clock, 0.12);
        selectPressed.set(true);
        update(bindings, clock, 0.14);
        assertTrue(ready.get());
        assertTrue(tuner.isEnabled());
        assertTrue(integerTuner.isEnabled());

        selectPressed.set(false);
        update(bindings, clock, 0.16);
        selectPressed.set(true);
        update(bindings, clock, 0.18);
        assertFalse(tuner.isEnabled());
        assertFalse(integerTuner.isEnabled());
    }

    @Test
    public void navigatorMovesHeldSelectToNextPickerWithoutSelectingIt() {
        AtomicBoolean selectPressed = new AtomicBoolean();
        AtomicInteger firstSelections = new AtomicInteger();
        AtomicInteger secondSelections = new AtomicInteger();
        BooleanSource select = BooleanSource.of(selectPressed::get);

        SelectionMenu<String> second = new SelectionMenu<String>()
                .setTitle("Second")
                .addItem("two", "two")
                .setOnSelect(item -> secondSelections.incrementAndGet());
        MenuNavigator navigator = new MenuNavigator();
        SelectionMenu<String> first = new SelectionMenu<String>()
                .setTitle("First")
                .addItem("one", "one")
                .setOnSelect(item -> {
                    firstSelections.incrementAndGet();
                    navigator.push(second);
                });
        navigator.setRoot(first);

        Bindings bindings = new Bindings();
        Bindings.ControlContext wizardControls = bindings.contextWhen(
                BooleanSource.constant(true),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL
        );
        navigator.bind(
                wizardControls,
                new UiControls(null, null, null, null, select, null, null, null, null)
        );

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        update(bindings, clock, 0.02);

        selectPressed.set(true);
        update(bindings, clock, 0.04);
        assertEquals(1, firstSelections.get());
        assertEquals(0, secondSelections.get());

        update(bindings, clock, 0.06);
        assertEquals(0, secondSelections.get());

        selectPressed.set(false);
        update(bindings, clock, 0.08);
        selectPressed.set(true);
        update(bindings, clock, 0.10);
        assertEquals(1, secondSelections.get());
    }

    private static void update(Bindings bindings, LoopClock clock, double nowSec) {
        clock.update(nowSec);
        bindings.update(clock);
    }
}
