package edu.ftcphoenix.fw.task;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.input.binding.CallbackBindings;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

/** Locks the intentional generic Task construction and cancellation surface. */
public final class TasksApiTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void taskCancellationPolicyIsRequiredAtCompileTime() throws Exception {
        Method cancel = Task.class.getDeclaredMethod("cancel");

        assertTrue(Modifier.isPublic(cancel.getModifiers()));
        assertTrue(Modifier.isAbstract(cancel.getModifiers()));
        assertFalse(cancel.isDefault());

        Method outputCancel = OutputTask.class.getMethod("cancel");
        assertEquals(Task.class, outputCancel.getDeclaringClass());
        assertFalse(outputCancel.isDefault());
    }

    @Test
    public void ordinaryTaskLeavesAndOutputQueueHaveOnlyFactoryConstruction() throws Exception {
        assertTrue(Modifier.isPublic(OutputTaskRunner.class.getModifiers()));
        assertTrue(Modifier.isPublic(RunForSecondsTask.class.getModifiers()));

        assertEquals(0, InstantTask.class.getConstructors().length);
        assertEquals(0, WaitUntilTask.class.getConstructors().length);
        assertEquals(0, OutputForSecondsTask.class.getConstructors().length);
        assertEquals(0, GatedOutputUntilTask.class.getConstructors().length);
        assertEquals(0, OutputTaskRunner.class.getConstructors().length);
        assertEquals(1, RunForSecondsTask.class.getConstructors().length);

        assertFactory("runOnce", Task.class, Runnable.class);
        assertFactory("waitUntil", Task.class, BooleanSource.class);
        assertFactory("outputForSeconds", OutputTask.class,
                String.class, double.class, double.class);
        assertFactory("outputPulse", Tasks.OutputPulseStartStep.class, String.class);
        assertFactory("outputQueue", OutputTaskRunner.class);
        assertFactory("outputQueue", OutputTaskRunner.class, double.class);
    }

    @Test
    public void ambiguousOutputAndLevelTaskBindingPathsRemainAbsent() {
        assertNoPublicMethodNamed(TaskBindings.class, "whileHigh");
        assertNoPublicMethodNamed(TaskBindings.class, "whileLow");
        assertNoPublicMethodNamed(Tasks.class, "gatedOutputUntil");
        assertNoPublicMethodNamed(Tasks.class, "outputUntil");
        assertNoPublicMethodNamed(Tasks.OutputPulseUntilStep.class, "runWindow");

        Set<String> taskBindingMethods = new HashSet<>();
        int publicTaskBindingMethodCount = 0;
        for (Method method : TaskBindings.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && !method.isSynthetic()) {
                publicTaskBindingMethodCount++;
                taskBindingMethods.add(method.getName());
            }
        }
        assertEquals(6, publicTaskBindingMethodCount);
        assertEquals(new HashSet<>(Arrays.asList(
                "of",
                "onRise",
                "onFall",
                "mirrorOnChange",
                "toggleOnRise",
                "nudgeOnRise")), taskBindingMethods);
    }

    @Test
    public void taskBindingsFactoryConsumesOnlyTheCallbackSurfaceAndRunner() throws Exception {
        Method factory = TaskBindings.class.getDeclaredMethod(
                "of",
                CallbackBindings.class,
                TaskRunner.class);

        assertTrue(Modifier.isPublic(factory.getModifiers()));
        assertTrue(Modifier.isStatic(factory.getModifiers()));
        assertEquals(TaskBindings.class, factory.getReturnType());

        int factoryCount = 0;
        for (Method method : TaskBindings.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())
                    && Modifier.isStatic(method.getModifiers())
                    && method.getName().equals("of")) {
                factoryCount++;
            }
        }
        assertEquals(1, factoryCount);
    }

    @Test
    public void sensorEndedPulseUsesSeparateMinimumAndMaximumStages() {
        OutputTaskFactory factory = Tasks.outputPulse("boundedPulse")
                .startImmediately()
                .runOutput(0.75)
                .until(BooleanSource.constant(true))
                .minRunSec(0.05)
                .maxRunSec(0.30)
                .build();
        OutputTask first = factory.create();
        OutputTask second = factory.create();
        assertNotSame(first, second);

        ManualLoopClock manualClock = new ManualLoopClock();
        OutputTaskRunner runner = Tasks.outputQueue(-0.25);
        runner.enqueue(first);

        runner.update(manualClock.clock());
        assertFalse(first.isComplete());
        assertEquals(0.75, runner.output(manualClock.clock()), EPSILON);

        manualClock.nextCycle(0.04);
        runner.update(manualClock.clock());
        assertFalse(first.isComplete());

        manualClock.nextCycle(0.02);
        runner.update(manualClock.clock());
        assertTrue(first.isComplete());
        assertEquals(TaskOutcome.SUCCESS, first.getOutcome());
        assertEquals(-0.25, runner.output(manualClock.clock()), EPSILON);
    }

    @Test
    public void outputQueueLevelHelpersBoundBacklogAndCancelOnOppositeLevel() {
        ManualLoopClock manualClock = new ManualLoopClock();
        boolean[] request = {true};
        BooleanSource requestSource = BooleanSource.of(() -> request[0]);

        List<OutputTask> highTasks = new ArrayList<>();
        OutputTaskRunner highRunner = Tasks.outputQueue(-0.25);
        OutputTaskFactory highFactory = () -> freshLongOutput("high", highTasks);

        highRunner.whileHigh(manualClock.clock(), requestSource, 1, highFactory);
        highRunner.whileHigh(manualClock.clock(), requestSource, 1, highFactory);
        assertEquals(1, highTasks.size());
        assertEquals(1, highRunner.backlogCount());

        highRunner.update(manualClock.clock());
        manualClock.nextCycle(0.02);
        highRunner.whileHigh(manualClock.clock(), requestSource, 1, highFactory);
        assertEquals(1, highTasks.size());
        assertEquals(1, highRunner.backlogCount());

        request[0] = false;
        manualClock.nextCycle(0.02);
        highRunner.whileHigh(manualClock.clock(), requestSource, 1, highFactory);
        assertTrue(highRunner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, highTasks.get(0).getOutcome());
        assertEquals(-0.25, highRunner.output(manualClock.clock()), EPSILON);

        List<OutputTask> lowTasks = new ArrayList<>();
        OutputTaskRunner lowRunner = Tasks.outputQueue(0.10);
        OutputTaskFactory lowFactory = () -> freshLongOutput("low", lowTasks);

        lowRunner.whileLow(manualClock.clock(), requestSource, 1, lowFactory);
        lowRunner.whileLow(manualClock.clock(), requestSource, 1, lowFactory);
        assertEquals(1, lowTasks.size());
        assertEquals(1, lowRunner.backlogCount());
        lowRunner.update(manualClock.clock());

        manualClock.nextCycle(0.02);
        lowRunner.whileLow(manualClock.clock(), requestSource, 1, lowFactory);
        assertEquals(1, lowTasks.size());
        assertEquals(1, lowRunner.backlogCount());

        request[0] = true;
        manualClock.nextCycle(0.02);
        lowRunner.whileLow(manualClock.clock(), requestSource, 1, lowFactory);
        assertTrue(lowRunner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, lowTasks.get(0).getOutcome());
        assertEquals(0.10, lowRunner.output(manualClock.clock()), EPSILON);
    }

    private static OutputTask freshLongOutput(String name, List<OutputTask> created) {
        OutputTask task = Tasks.outputForSeconds(name, 0.75, 10.0);
        created.add(task);
        return task;
    }

    private static void assertFactory(String name,
                                      Class<?> returnType,
                                      Class<?>... parameterTypes) throws Exception {
        Method method = Tasks.class.getMethod(name, parameterTypes);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertTrue(Modifier.isStatic(method.getModifiers()));
        assertEquals(returnType, method.getReturnType());
    }

    private static void assertNoPublicMethodNamed(Class<?> type, String name) {
        for (Method method : type.getMethods()) {
            assertFalse(type.getSimpleName() + " must not expose " + name,
                    method.getName().equals(name));
        }
    }
}
