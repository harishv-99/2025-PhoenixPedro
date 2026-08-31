package edu.ftcsushi.robots.phoenix.opmode;

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

import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixProfile;
import edu.ftcsushi.robots.phoenix.PhoenixReadiness;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoStrategyId;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix Auto's data-only INIT selection and one immutable START freeze. */
public final class PhoenixAutoPrestartTest {

    @Test
    public void initSelectionFreezesOnceAtStartAndPublishesOnlyTheFrozenAlliance() {
        PhoenixProfile suppliedProfile = PhoenixProfile.current();
        int expectedBlueTagId = suppliedProfile.targeting.blueAllianceScoringTagId;
        PhoenixAutoSpec defaultSpec = integrationSpec(PhoenixAlliance.RED);
        PhoenixAutoSetup setup = PhoenixAutoSetup.fromInitSelection(
                defaultSpec,
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        RecordingTelemetry telemetry = new RecordingTelemetry();
        PrestartHost host = new PrestartHost(setup, suppliedProfile);
        host.telemetry = telemetry.proxy();
        host.gamepad1 = new Gamepad();
        host.gamepad2 = new Gamepad();

        host.runtimeSec = 0.0;
        host.init();
        Source<Set<Integer>> eligibleTagIds = host.prestart.eligibleScoringTagIds();
        LoopClock sampleClock = initializedClock();

        IllegalStateException unavailable = expectIllegalState(
                () -> eligibleTagIds.get(sampleClock)
        );
        assertTrue(unavailable.getMessage().contains("not frozen until the FTC START boundary"));
        assertTrue(expectIllegalState(host.prestart::frozenSpec)
                .getMessage().contains("not frozen until the FTC START boundary"));

        // Exercise the real INIT binding path: move RED -> BLUE, then choose that alliance.
        host.gamepad1.dpad_down = true;
        host.initLoopAt(0.1);
        host.gamepad1.dpad_down = false;
        host.initLoopAt(0.2);
        host.gamepad1.a = true;
        host.initLoopAt(0.3);
        host.gamepad1.a = false;
        host.initLoopAt(0.4);
        assertTrue(telemetry.lastValue("auto.setup").startsWith("Blue /"));

        // Long-lived prestart policy must not observe later mutation of the supplied profile.
        suppliedProfile.calibration = null;
        suppliedProfile.auto = null;
        suppliedProfile.targeting = null;
        suppliedProfile.fixedAprilTagLayout = null;

        host.runtimeSec = 1.0;
        host.start();

        PhoenixAutoSpec frozen = host.prestart.frozenSpec();
        assertEquals(PhoenixAlliance.BLUE, frozen.alliance);
        assertEquals(Collections.singleton(expectedBlueTagId), eligibleTagIds.get(sampleClock));

        IllegalStateException lateUpdate = expectIllegalState(
                () -> host.prestart.update(sampleClock)
        );
        assertTrue(lateUpdate.getMessage().contains("already frozen"));
        assertSame(frozen, host.prestart.frozenSpec());
        assertEquals(Collections.singleton(expectedBlueTagId), eligibleTagIds.get(sampleClock));

        IllegalStateException secondFreeze = expectIllegalState(host.prestart::freezeForStart);
        assertTrue(secondFreeze.getMessage().contains("frozen only once"));
        assertSame(frozen, host.prestart.frozenSpec());
        assertEquals(Collections.singleton(expectedBlueTagId), eligibleTagIds.get(sampleClock));

        host.stop();
    }

    @Test
    public void currentStructuralReadinessBlocksWithActionableDataOnlyPresentation() {
        PhoenixProfile profile = PhoenixProfile.current();
        profile.calibration.pinpointAxesVerified = true;
        profile.calibration.pinpointPodOffsetsCalibrated = true;
        PhoenixAutoSpec spec = PhoenixAutoSpec.audienceSafe(PhoenixAlliance.RED);
        PhoenixAutoPrestart prestart = newPrestart(
                PhoenixAutoSetup.fromFixedSpec(
                        spec,
                        PhoenixReadiness.AutoPurpose.MATCH_AUTO
                ),
                profile,
                new Gamepad(),
                new Gamepad()
        );
        RecordingTelemetry telemetry = new RecordingTelemetry();
        LoopClock clock = initializedClock();

        prestart.present(clock, telemetry.proxy());
        assertEquals("INTEGRATION_ONLY", telemetry.lastValue("auto.routeMaturity"));
        assertEquals("BLOCKED AT START", telemetry.lastValue("auto.startPolicy"));
        assertTrue(telemetry.contains("Auto [BLOCKING]"));
        assertTrue(telemetry.contains("integration-only"));
        assertTrue(telemetry.contains("Implement and validate match geometry"));
        assertEquals(0, telemetry.updateCalls);

        assertEquals(RobotProgram.StartDisposition.BLOCKED, prestart.freezeForStart());
        prestart.present(clock, telemetry.proxy());
        assertEquals("BLOCKED", telemetry.lastValue("auto.startPolicy"));
        assertEquals(0, telemetry.updateCalls);

        for (Constructor<?> constructor : PhoenixAutoPrestart.class.getDeclaredConstructors()) {
            for (Class<?> parameterType : constructor.getParameterTypes()) {
                assertFalse(
                        "Prestart readiness must not receive HardwareMap",
                        HardwareMap.class.isAssignableFrom(parameterType)
                );
            }
        }
    }

    @Test
    public void onlyFixedSetupExposesAnEagerSpec() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixAutoSpec spec = integrationSpec(PhoenixAlliance.RED);
        PhoenixAutoSetup fixedSetup = PhoenixAutoSetup.fromFixedSpec(
                spec,
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        PhoenixAutoSetup selectableSetup = PhoenixAutoSetup.fromInitSelection(
                spec,
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        PhoenixAutoPrestart fixed = newPrestart(
                fixedSetup,
                profile,
                new Gamepad(),
                new Gamepad()
        );
        PhoenixAutoPrestart selectable = newPrestart(
                selectableSetup,
                profile,
                new Gamepad(),
                new Gamepad()
        );

        assertEquals(PhoenixAutoSetup.SelectionMode.FIXED, fixedSetup.selectionMode());
        assertEquals(
                PhoenixAutoSetup.SelectionMode.INIT_SELECTION,
                selectableSetup.selectionMode()
        );
        assertSame(spec, fixedSetup.initialSpec());
        assertSame(spec, fixed.fixedSpecForEagerBuild());
        assertTrue(expectIllegalState(selectable::fixedSpecForEagerBuild)
                .getMessage().contains("Only a fixed Phoenix Auto setup has an eager spec"));
    }

    @Test
    public void selectorUsesOnlyRealChoicesAndShowsAReadOnlyStartSummary() {
        PhoenixProfile profile = PhoenixProfile.current();
        profile.calibration.pinpointAxesVerified = true;
        PhoenixAutoSetup setup = PhoenixAutoSetup.fromInitSelection(
                integrationSpec(PhoenixAlliance.RED),
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        RecordingTelemetry telemetry = new RecordingTelemetry();
        PrestartHost host = new PrestartHost(setup, profile);
        host.telemetry = telemetry.proxy();
        host.gamepad1 = new Gamepad();
        host.gamepad2 = new Gamepad();

        host.init();
        assertFalse(telemetry.contains("X: edit summary"));
        chooseCurrentItem(host, 0.1);
        chooseCurrentItem(host, 0.3);
        int summaryFrameStart = telemetry.entryCount();
        chooseCurrentItem(host, 0.5);

        assertTrue(telemetry.contains("=== Phoenix Auto Selection ==="));
        assertTrue(telemetry.contains("FTC START will freeze this setup."));
        assertTrue(telemetry.containsSince(
                summaryFrameStart,
                "START: freeze selection | X: edit selection"
        ));
        assertFalse(telemetry.containsSince(summaryFrameStart, "Controls: Dpad"));
        assertFalse(telemetry.contains("Partner Plan"));
        assertFalse(telemetry.contains("Confirm Phoenix Auto"));
        assertTrue(expectIllegalState(host.prestart::frozenSpec)
                .getMessage().contains("not frozen until the FTC START boundary"));

        host.runtimeSec = 1.0;
        host.start();
        assertEquals(PhoenixAlliance.RED, host.prestart.frozenSpec().alliance);
        host.stop();
    }

    @Test
    public void wholeModeBlockerRemainsVisibleWithoutDisablingTheValidStrategyRow() {
        PhoenixProfile profile = PhoenixProfile.current();
        profile.calibration.pinpointAxesVerified = false;
        PhoenixAutoSetup setup = PhoenixAutoSetup.fromInitSelection(
                integrationSpec(PhoenixAlliance.RED),
                PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST
        );
        RecordingTelemetry telemetry = new RecordingTelemetry();
        PrestartHost host = new PrestartHost(setup, profile);
        host.telemetry = telemetry.proxy();
        host.gamepad1 = new Gamepad();
        host.gamepad2 = new Gamepad();

        host.init();
        chooseCurrentItem(host, 0.1);
        chooseCurrentItem(host, 0.3);
        chooseCurrentItem(host, 0.5);

        assertTrue(telemetry.contains("=== Phoenix Auto Selection ==="));
        assertTrue(telemetry.contains("Status: [BLOCKED]"));
        assertTrue(telemetry.contains("Pinpoint axis directions"));
        assertTrue(telemetry.contains("START: freeze selection | X: edit selection"));

        host.runtimeSec = 1.0;
        host.start();
        assertEquals(PhoenixAlliance.RED, host.prestart.frozenSpec().alliance);
        host.stop();
    }

    private static PhoenixAutoSpec integrationSpec(PhoenixAlliance alliance) {
        return PhoenixAutoSpec.builder()
                .alliance(alliance)
                .startPosition(PhoenixAutoSpec.StartPosition.AUDIENCE)
                .strategy(PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST)
                .build();
    }

    private static PhoenixAutoPrestart newPrestart(
            PhoenixAutoSetup setup,
            PhoenixProfile profile,
            Gamepad gamepad1,
            Gamepad gamepad2
    ) {
        return new PhoenixAutoPrestart(
                setup,
                profile.calibration,
                profile.targeting,
                profile.fixedAprilTagLayout,
                gamepad1,
                gamepad2
        );
    }

    private static void chooseCurrentItem(PrestartHost host, double pressedAtSec) {
        host.gamepad1.a = true;
        host.initLoopAt(pressedAtSec);
        host.gamepad1.a = false;
        host.initLoopAt(pressedAtSec + 0.1);
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

    private static final class PrestartHost extends FtcRobotOpMode {
        private final PhoenixAutoSetup setup;
        private final PhoenixProfile profile;
        private PhoenixAutoPrestart prestart;
        private double runtimeSec;

        private PrestartHost(PhoenixAutoSetup setup, PhoenixProfile profile) {
            this.setup = setup;
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            prestart = newPrestart(setup, profile, gamepad1, gamepad2);
            program.prestart(prestart);
            program.presenter(prestart::present);
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }

        private void initLoopAt(double nowSec) {
            runtimeSec = nowSec;
            init_loop();
        }
    }

    private static final class RecordingTelemetry implements InvocationHandler {
        private final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                this
        );
        private final List<String> entries = new ArrayList<>();
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

        private int entryCount() {
            return entries.size();
        }

        private boolean containsSince(int startIndex, String fragment) {
            for (int index = startIndex; index < entries.size(); index++) {
                if (entries.get(index).contains(fragment)) {
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
