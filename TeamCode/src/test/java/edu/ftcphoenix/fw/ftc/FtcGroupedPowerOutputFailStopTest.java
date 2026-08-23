package edu.ftcphoenix.fw.ftc;

import org.junit.Test;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.core.hal.PowerOutput;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused failure-contract coverage for the two private grouped power realizations. */
public final class FtcGroupedPowerOutputFailStopTest {

    private enum GroupKind {
        IDENTITY,
        MAPPED
    }

    @Test
    public void cachesStartUnknownAndSuccessfulWritesCommitExactTopLevelTruth() throws Exception {
        Fixture identity = Fixture.create(GroupKind.IDENTITY);
        assertUnknown(identity.group);
        identity.assertAllChildrenUnknown();

        identity.group.setPower(-0.0);

        assertRawDoubleEquals(-0.0, identity.group.getCommandedPower());
        for (ProbeOutput child : identity.children) {
            assertRawDoubleEquals(-0.0, child.getCommandedPower());
        }

        Fixture mapped = Fixture.create(GroupKind.MAPPED);
        assertUnknown(mapped.group);
        mapped.assertAllChildrenUnknown();

        mapped.group.setPower(0.4);

        assertRawDoubleEquals(0.4, mapped.group.getCommandedPower());
        mapped.assertSuccessfulChildCommands(0.4);
    }

    @Test
    public void groupCacheRemainsUnknownUntilEverySuccessfulChildOperationReturns()
            throws Exception {
        for (GroupKind kind : GroupKind.values()) {
            Fixture fixture = Fixture.create(kind);
            for (ProbeOutput child : fixture.children) {
                child.setObserver = () -> assertUnknown(fixture.group);
                child.stopObserver = () -> assertUnknown(fixture.group);
            }

            fixture.group.setPower(0.3);
            assertRawDoubleEquals(0.3, fixture.group.getCommandedPower());

            fixture.group.stop();
            assertRawDoubleEquals(0.0, fixture.group.getCommandedPower());
        }
    }

    @Test
    public void nonFiniteTopRequestStopsAllWithoutRequestedFanout() throws Exception {
        for (GroupKind kind : GroupKind.values()) {
            for (double invalid : new double[]{
                    Double.NaN,
                    Double.NEGATIVE_INFINITY,
                    Double.POSITIVE_INFINITY}) {
                Fixture fixture = Fixture.create(kind);
                fixture.group.setPower(0.2);
                fixture.resetObservations();
                RuntimeException firstCleanup = new IllegalStateException("first cleanup");
                RuntimeException thirdCleanup = new IllegalStateException("third cleanup");
                fixture.children[0].stopFailure = firstCleanup;
                fixture.children[2].stopFailure = thirdCleanup;

                IllegalArgumentException failure = capture(
                        IllegalArgumentException.class,
                        () -> fixture.group.setPower(invalid));

                assertTrue(failure.getMessage().contains("test motor power runtime power"));
                assertTrue(failure.getMessage().contains("must be finite"));
                assertTrue(failure.getMessage().contains(Double.toString(invalid)));
                assertSuppressed(failure, firstCleanup, thirdCleanup);
                assertEquals(Arrays.asList("stop:first", "stop:second", "stop:third"),
                        fixture.events);
                for (ProbeOutput child : fixture.children) {
                    assertEquals(0, child.setAttempts);
                    assertEquals(1, child.stopAttempts);
                }
                assertUnknown(fixture.group);
                assertUnknown(fixture.children[0]);
                assertRawDoubleEquals(0.0, fixture.children[1].getCommandedPower());
                assertUnknown(fixture.children[2]);
            }
        }
    }

    @Test
    public void finiteIdentityRejectionHasNoChildOrCleanupEffect() throws Exception {
        Fixture fixture = Fixture.create(GroupKind.IDENTITY);
        fixture.group.setPower(0.25);
        fixture.resetObservations();

        capture(IllegalStateException.class, () -> fixture.group.setPower(1.01));

        assertTrue(fixture.events.isEmpty());
        fixture.assertNoAttempts();
        assertUnknown(fixture.group);
        for (ProbeOutput child : fixture.children) {
            assertRawDoubleEquals(0.25, child.getCommandedPower());
        }
    }

    @Test
    public void finiteMappedRejectionPrecomputesEveryChildBeforeEffects() throws Exception {
        Fixture fixture = Fixture.create(
                GroupKind.MAPPED,
                new double[]{1.0, 2.0, 1.0},
                new double[]{0.0, 0.0, 0.0});
        fixture.group.setPower(0.25);
        fixture.resetObservations();

        IllegalStateException failure = capture(
                IllegalStateException.class,
                () -> fixture.group.setPower(0.75));

        assertTrue(failure.getMessage().contains("child 2 ('second')"));
        assertTrue(fixture.events.isEmpty());
        fixture.assertNoAttempts();
        assertUnknown(fixture.group);
        assertRawDoubleEquals(0.25, fixture.children[0].getCommandedPower());
        assertRawDoubleEquals(0.5, fixture.children[1].getCommandedPower());
        assertRawDoubleEquals(0.25, fixture.children[2].getCommandedPower());
    }

    @Test
    public void firstMiddleAndLastWriteFailuresAbandonLaterWritesAndStopAllChildren()
            throws Exception {
        for (GroupKind kind : GroupKind.values()) {
            for (int failingIndex = 0; failingIndex < 3; failingIndex++) {
                Fixture fixture = Fixture.create(kind);
                fixture.group.setPower(0.2);
                fixture.resetObservations();
                RuntimeException childFailure =
                        new IllegalStateException("write " + failingIndex + " failed");
                fixture.children[failingIndex].setFailure = childFailure;
                fixture.children[failingIndex].failSetAfterEffect = failingIndex != 0;

                RuntimeException thrown = capture(
                        RuntimeException.class,
                        () -> fixture.group.setPower(0.4));

                assertSame(childFailure, thrown);
                assertEquals(expectedWriteFailureEvents(failingIndex), fixture.events);
                for (int childIndex = 0; childIndex < fixture.children.length; childIndex++) {
                    assertEquals(childIndex <= failingIndex ? 1 : 0,
                            fixture.children[childIndex].setAttempts);
                    boolean effectExpected = childIndex < failingIndex
                            || (childIndex == failingIndex && failingIndex != 0);
                    assertEquals(effectExpected ? 1 : 0,
                            fixture.children[childIndex].setEffects);
                    assertEquals(1, fixture.children[childIndex].stopAttempts);
                    assertRawDoubleEquals(0.0,
                            fixture.children[childIndex].getCommandedPower());
                }
                assertUnknown(fixture.group);
                assertEquals(0, thrown.getSuppressed().length);
            }
        }
    }

    @Test
    public void writeFailureRetainsPrimarySuppressesCleanupAndAllowsSeparateRecovery()
            throws Exception {
        for (GroupKind kind : GroupKind.values()) {
            Fixture fixture = Fixture.create(kind);
            RuntimeException writeFailure = new IllegalStateException("middle write failed");
            RuntimeException firstCleanup = new IllegalStateException("first cleanup failed");
            RuntimeException thirdCleanup = new IllegalStateException("third cleanup failed");
            fixture.children[1].setFailure = writeFailure;
            fixture.children[1].failSetAfterEffect = true;
            fixture.children[0].stopFailure = firstCleanup;
            fixture.children[2].stopFailure = thirdCleanup;

            RuntimeException thrown = capture(
                    RuntimeException.class,
                    () -> fixture.group.setPower(0.4));

            assertSame(writeFailure, thrown);
            assertSuppressed(thrown, firstCleanup, thirdCleanup);
            assertEquals(expectedWriteFailureEvents(1), fixture.events);
            assertUnknown(fixture.group);
            assertUnknown(fixture.children[0]);
            assertRawDoubleEquals(0.0, fixture.children[1].getCommandedPower());
            assertUnknown(fixture.children[2]);

            fixture.children[0].stopFailure = null;
            fixture.children[1].setFailure = null;
            fixture.children[2].stopFailure = null;
            fixture.resetObservations();

            fixture.group.stop();
            fixture.group.stop();

            assertRawDoubleEquals(0.0, fixture.group.getCommandedPower());
            for (ProbeOutput child : fixture.children) {
                assertEquals(0, child.setAttempts);
                assertEquals(2, child.stopAttempts);
                assertRawDoubleEquals(0.0, child.getCommandedPower());
            }

            fixture.resetObservations();
            fixture.group.setPower(-0.3);
            assertRawDoubleEquals(-0.3, fixture.group.getCommandedPower());
            fixture.assertSuccessfulChildCommands(-0.3);
        }
    }

    @Test
    public void firstMiddleAndLastStopFailuresAttemptAllAndRecoverOnLaterStop()
            throws Exception {
        for (GroupKind kind : GroupKind.values()) {
            for (int failingIndex = 0; failingIndex < 3; failingIndex++) {
                Fixture fixture = Fixture.create(kind);
                fixture.group.setPower(0.2);
                fixture.resetObservations();
                RuntimeException stopFailure =
                        new IllegalStateException("stop " + failingIndex + " failed");
                fixture.children[failingIndex].stopFailure = stopFailure;

                RuntimeException thrown = capture(RuntimeException.class, fixture.group::stop);

                assertSame(stopFailure, thrown);
                assertEquals(Arrays.asList("stop:first", "stop:second", "stop:third"),
                        fixture.events);
                assertUnknown(fixture.group);
                for (int childIndex = 0; childIndex < fixture.children.length; childIndex++) {
                    assertEquals(1, fixture.children[childIndex].stopAttempts);
                    if (childIndex == failingIndex) {
                        assertUnknown(fixture.children[childIndex]);
                    } else {
                        assertRawDoubleEquals(0.0,
                                fixture.children[childIndex].getCommandedPower());
                    }
                }

                fixture.children[failingIndex].stopFailure = null;
                fixture.resetObservations();
                fixture.group.stop();

                assertRawDoubleEquals(0.0, fixture.group.getCommandedPower());
                for (ProbeOutput child : fixture.children) {
                    assertEquals(1, child.stopAttempts);
                    assertRawDoubleEquals(0.0, child.getCommandedPower());
                }
            }
        }
    }

    @Test
    public void groupedStopRetainsFirstFailureAndSuppressesLaterFailuresInOrder()
            throws Exception {
        for (GroupKind kind : GroupKind.values()) {
            Fixture fixture = Fixture.create(kind);
            RuntimeException firstFailure = new IllegalStateException("first stop failed");
            RuntimeException secondFailure = new IllegalStateException("second stop failed");
            RuntimeException thirdFailure = new IllegalStateException("third stop failed");
            fixture.children[0].stopFailure = firstFailure;
            fixture.children[1].stopFailure = secondFailure;
            fixture.children[2].stopFailure = thirdFailure;

            RuntimeException thrown = capture(RuntimeException.class, fixture.group::stop);

            assertSame(firstFailure, thrown);
            assertSuppressed(thrown, secondFailure, thirdFailure);
            assertEquals(Arrays.asList("stop:first", "stop:second", "stop:third"),
                    fixture.events);
            assertUnknown(fixture.group);
            fixture.assertAllChildrenUnknown();
        }
    }

    private static List<String> expectedWriteFailureEvents(int failingIndex) {
        List<String> expected = new ArrayList<>();
        String[] names = {"first", "second", "third"};
        for (int index = 0; index <= failingIndex; index++) {
            expected.add("set:" + names[index]);
        }
        for (String name : names) {
            expected.add("stop:" + name);
        }
        return expected;
    }

    private static void assertUnknown(PowerOutput output) {
        assertTrue("Expected an unknown command cache", Double.isNaN(output.getCommandedPower()));
    }

    private static void assertSuppressed(
            RuntimeException failure,
            RuntimeException... expected) {
        Throwable[] actual = failure.getSuppressed();
        assertEquals(expected.length, actual.length);
        for (int index = 0; index < expected.length; index++) {
            assertSame(expected[index], actual[index]);
        }
    }

    private static void assertRawDoubleEquals(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    private static <T extends RuntimeException> T capture(
            Class<T> expectedType,
            Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            assertTrue("Expected " + expectedType.getSimpleName() + ", got " + failure,
                    expectedType.isInstance(failure));
            return expectedType.cast(failure);
        }
        fail("Expected " + expectedType.getSimpleName());
        return null;
    }

    private static final class Fixture {
        private static final double[] DEFAULT_SCALES = {1.0, -0.5, 0.25};
        private static final double[] DEFAULT_BIASES = {0.0, 0.1, -0.2};

        private final GroupKind kind;
        private final double[] scales;
        private final double[] biases;
        private final List<String> events = new ArrayList<>();
        private final ProbeOutput[] children = {
                new ProbeOutput("first", events),
                new ProbeOutput("second", events),
                new ProbeOutput("third", events)
        };
        private final PowerOutput group;

        private Fixture(GroupKind kind, double[] scales, double[] biases) throws Exception {
            this.kind = kind;
            this.scales = scales.clone();
            this.biases = biases.clone();
            List<PowerOutput> outputs = new ArrayList<>();
            outputs.addAll(Arrays.asList(children));
            List<String> names = Arrays.asList("first", "second", "third");
            if (kind == GroupKind.IDENTITY) {
                Class<?> type = Class.forName(
                        FtcActuators.class.getName() + "$IdentityGroupedPowerOutput");
                Constructor<?> constructor = type.getDeclaredConstructor(
                        List.class, List.class, String.class);
                constructor.setAccessible(true);
                group = (PowerOutput) constructor.newInstance(
                        outputs, names, "test motor power");
            } else {
                Class<?> type = Class.forName(
                        FtcActuators.class.getName() + "$GroupedPowerOutput");
                Constructor<?> constructor = type.getDeclaredConstructor(
                        List.class, double[].class, double[].class, List.class, String.class);
                constructor.setAccessible(true);
                group = (PowerOutput) constructor.newInstance(
                        outputs, this.scales, this.biases, names, "test motor power");
            }
        }

        private static Fixture create(GroupKind kind) throws Exception {
            return create(kind, DEFAULT_SCALES, DEFAULT_BIASES);
        }

        private static Fixture create(GroupKind kind, double[] scales, double[] biases)
                throws Exception {
            return new Fixture(kind, scales, biases);
        }

        private void resetObservations() {
            events.clear();
            for (ProbeOutput child : children) {
                child.setAttempts = 0;
                child.setEffects = 0;
                child.stopAttempts = 0;
            }
        }

        private void assertNoAttempts() {
            for (ProbeOutput child : children) {
                assertEquals(0, child.setAttempts);
                assertEquals(0, child.setEffects);
                assertEquals(0, child.stopAttempts);
            }
        }

        private void assertAllChildrenUnknown() {
            for (ProbeOutput child : children) {
                assertUnknown(child);
            }
        }

        private void assertSuccessfulChildCommands(double topLevelPower) {
            for (int index = 0; index < children.length; index++) {
                double expected = kind == GroupKind.IDENTITY
                        ? topLevelPower
                        : scales[index] * topLevelPower + biases[index];
                assertRawDoubleEquals(expected, children[index].getCommandedPower());
            }
        }
    }

    private static final class ProbeOutput implements PowerOutput {
        private final String name;
        private final List<String> events;
        private double last = Double.NaN;
        private int setAttempts;
        private int setEffects;
        private int stopAttempts;
        private RuntimeException setFailure;
        private boolean failSetAfterEffect;
        private RuntimeException stopFailure;
        private Runnable setObserver;
        private Runnable stopObserver;

        private ProbeOutput(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void setPower(double power) {
            last = Double.NaN;
            setAttempts++;
            events.add("set:" + name);
            if (setFailure != null && !failSetAfterEffect) {
                throw setFailure;
            }
            setEffects++;
            if (setFailure != null) {
                throw setFailure;
            }
            if (setObserver != null) {
                setObserver.run();
            }
            last = power;
        }

        @Override
        public double getCommandedPower() {
            return last;
        }

        @Override
        public void stop() {
            last = Double.NaN;
            stopAttempts++;
            events.add("stop:" + name);
            if (stopFailure != null) {
                throw stopFailure;
            }
            if (stopObserver != null) {
                stopObserver.run();
            }
            last = 0.0;
        }
    }
}
