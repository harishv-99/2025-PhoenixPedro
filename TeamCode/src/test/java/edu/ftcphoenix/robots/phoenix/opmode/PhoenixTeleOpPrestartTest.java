package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.robots.phoenix.PhoenixAlliance;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies TeleOp's one editable alliance draft and immutable START freeze. */
public final class PhoenixTeleOpPrestartTest {

    @Test
    public void standaloneTeleOpDefaultsToRedAndPublishesOnlyAfterStartFreeze() {
        PhoenixProfile suppliedProfile = PhoenixProfile.current();
        int redTagId = suppliedProfile.targeting.scoringTagIdFor(PhoenixAlliance.RED);
        PhoenixTeleOpPrestart prestart = new PhoenixTeleOpPrestart(
                suppliedProfile.targeting,
                suppliedProfile.fixedAprilTagLayout,
                new Gamepad(),
                PhoenixAlliance.RED
        );
        LoopClock clock = initializedClock();
        Source<Set<Integer>> eligibleTagIds = prestart.eligibleScoringTagIds();
        RecordingTelemetry telemetry = new RecordingTelemetry();

        assertTrue(expectIllegalState(() -> eligibleTagIds.get(clock))
                .getMessage().contains("not frozen until the FTC START boundary"));
        prestart.update(clock);
        prestart.present(clock, telemetry.proxy());
        assertEquals("Red", telemetry.lastValue("teleop.allianceDraft"));
        assertEquals("entry default", telemetry.lastValue("teleop.allianceDraftSource"));
        assertTrue(telemetry.contains("D-pad: choose"));
        assertFalse(telemetry.contains("A: confirm"));
        assertEquals(
                "READY TO FREEZE AT START",
                telemetry.lastValue("teleop.allianceStartPolicy")
        );
        assertEquals(0, telemetry.updateCalls);

        // Long-lived policy retains only immutable derived selection/readiness facts.
        suppliedProfile.targeting = null;
        suppliedProfile.fixedAprilTagLayout = null;

        assertEquals(RobotProgram.StartDisposition.READY, prestart.freezeForStart());
        assertEquals(PhoenixAlliance.RED, prestart.frozenAlliance());
        assertEquals(Collections.singleton(redTagId), eligibleTagIds.get(clock));
        assertTrue(expectIllegalState(() -> prestart.update(clock))
                .getMessage().contains("already frozen"));
        assertTrue(expectIllegalState(prestart::freezeForStart)
                .getMessage().contains("already frozen"));
    }

    @Test
    public void freshAutoAllianceSeedsVisibleDraftButGamepadOneMayOverrideIt() {
        PhoenixProfile profile = PhoenixProfile.current();
        int redTagId = profile.targeting.scoringTagIdFor(PhoenixAlliance.RED);
        Gamepad driver = new Gamepad();
        PhoenixTeleOpPrestart prestart = new PhoenixTeleOpPrestart(
                profile.targeting,
                profile.fixedAprilTagLayout,
                driver,
                PhoenixAlliance.RED
        );
        LoopClock clock = initializedClock();
        RecordingTelemetry telemetry = new RecordingTelemetry();

        prestart.seedDraftFromAuto(PhoenixAlliance.BLUE);
        prestart.update(clock);
        prestart.present(clock, telemetry.proxy());
        assertEquals("Blue", telemetry.lastValue("teleop.allianceDraft"));
        assertEquals("fresh Auto handoff", telemetry.lastValue("teleop.allianceDraftSource"));
        assertTrue(expectIllegalState(prestart::frozenAlliance)
                .getMessage().contains("not frozen until the FTC START boundary"));

        // BLUE is the second enum row. One real INIT input edge moves the visible draft to RED.
        driver.dpad_up = true;
        clock.update(0.10);
        prestart.update(clock);
        driver.dpad_up = false;
        clock.update(0.20);
        prestart.update(clock);
        prestart.present(clock, telemetry.proxy());
        assertEquals("Red", telemetry.lastValue("teleop.allianceDraft"));
        assertEquals("gamepad 1", telemetry.lastValue("teleop.allianceDraftSource"));

        assertEquals(RobotProgram.StartDisposition.READY, prestart.freezeForStart());
        assertEquals(PhoenixAlliance.RED, prestart.frozenAlliance());
        assertEquals(
                Collections.singleton(redTagId),
                prestart.eligibleScoringTagIds().get(clock)
        );
        assertTrue(expectIllegalState(
                () -> prestart.seedDraftFromAuto(PhoenixAlliance.BLUE)
        ).getMessage().contains("already frozen"));
    }

    @Test
    public void selectedAllianceWithMissingTargetFactBlocksBeforeTargetingStarts() {
        PhoenixProfile profile = PhoenixProfile.current();
        int blueTagId = profile.targeting.scoringTagIdFor(PhoenixAlliance.BLUE);
        profile.targeting.scoringTargets.remove(blueTagId);
        PhoenixTeleOpPrestart prestart = new PhoenixTeleOpPrestart(
                profile.targeting,
                profile.fixedAprilTagLayout,
                new Gamepad(),
                PhoenixAlliance.RED
        );
        LoopClock clock = initializedClock();
        RecordingTelemetry telemetry = new RecordingTelemetry();

        prestart.seedDraftFromAuto(PhoenixAlliance.BLUE);
        prestart.present(clock, telemetry.proxy());
        assertEquals("BLOCKED AT START", telemetry.lastValue("teleop.allianceStartPolicy"));
        assertTrue(telemetry.contains("TeleOp [BLOCKING]"));
        assertTrue(telemetry.contains(Integer.toString(blueTagId)));
        assertEquals(RobotProgram.StartDisposition.BLOCKED, prestart.freezeForStart());

        for (Constructor<?> constructor : PhoenixTeleOpPrestart.class.getDeclaredConstructors()) {
            for (Class<?> parameterType : constructor.getParameterTypes()) {
                assertFalse(
                        "TeleOp prestart must not receive HardwareMap",
                        HardwareMap.class.isAssignableFrom(parameterType)
                );
            }
        }
    }

    private static LoopClock initializedClock() {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        return clock;
    }

    private static IllegalStateException expectIllegalState(Runnable operation) {
        try {
            operation.run();
            fail("Expected IllegalStateException");
            throw new AssertionError("unreachable");
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );
        private final List<String> entries = new ArrayList<String>();
        private int updateCalls;

        private Telemetry proxy() {
            return proxy;
        }

        private String lastValue(String caption) {
            String prefix = caption + "=";
            for (int index = entries.size() - 1; index >= 0; index--) {
                String entry = entries.get(index);
                if (entry.startsWith(prefix)) {
                    return entry.substring(prefix.length());
                }
            }
            fail("Missing telemetry caption " + caption + " in " + entries);
            throw new AssertionError("unreachable");
        }

        private boolean contains(String fragment) {
            for (String entry : entries) {
                if (entry.contains(fragment)) {
                    return true;
                }
            }
            return false;
        }

        @Override
        public Object invoke(Object ignored, Method method, Object[] args) {
            if ("addData".equals(method.getName()) && args != null && args.length >= 2) {
                entries.add(String.valueOf(args[0]) + "=" + String.valueOf(args[1]));
            } else if ("addLine".equals(method.getName())) {
                entries.add(args == null || args.length == 0 ? "" : String.valueOf(args[0]));
            } else if ("update".equals(method.getName())) {
                updateCalls++;
            }
            return defaultValue(method.getReturnType());
        }
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return false;
        }
        if (type == byte.class) {
            return (byte) 0;
        }
        if (type == short.class) {
            return (short) 0;
        }
        if (type == int.class) {
            return 0;
        }
        if (type == long.class) {
            return 0L;
        }
        if (type == float.class) {
            return 0.0f;
        }
        if (type == double.class) {
            return 0.0;
        }
        if (type == char.class) {
            return '\0';
        }
        return null;
    }
}
