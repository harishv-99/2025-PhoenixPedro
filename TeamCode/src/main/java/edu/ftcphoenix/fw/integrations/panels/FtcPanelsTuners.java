package edu.ftcphoenix.fw.integrations.panels;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiFunction;
import java.util.function.Function;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;

/**
 * Panels-backed controller-neutral experiments for fresh, exclusive supported Plants.
 *
 * <p>Robot code names the experiment, supplies its finite physical test envelope, and supplies the
 * same canonical fresh-Plant recipe used by production. The workflow derives the actual controller
 * implementation and its fixed tuning topology from that completed Plant; it does not ask robot
 * code to name the motor, regulator, parameter schema, or command target again.</p>
 *
 * <p>The public {@link #draft} map is Panels transport state, not robot configuration. On tester
 * initialization it is replaced atomically with a synchronized map containing exactly the active
 * controller fields and the selected workflow's experiment fields, then the pinned Configurables
 * integration is explicitly refreshed. A captures one quiescent snapshot; merely editing or
 * publishing this map never changes a controller or Plant target.</p>
 */
@Configurable
public final class FtcPanelsTuners {

    private static final FtcPanelsTuners DRAFT_REFRESH_TOKEN = new FtcPanelsTuners();

    /**
     * Active topology-specific Panels draft. Its keys are replaced when the exclusive tester
     * acquires its Plant during INIT.
     */
    @Sorter(sort = 10)
    public static volatile Map<String, Double> draft = emptySynchronizedMap();

    private FtcPanelsTuners() {
    }

    /**
     * Creates one controller-neutral velocity experiment for a fresh feedback Plant.
     *
     * @param testerName human-facing tester title
     * @param allowedTestTargetRange finite bounded permission for one manually selected velocity
     *                               target; it may be signed and may include zero, and the workflow
     *                               never interprets it as an automatic sweep
     * @param plantFactory creates a fresh inactive command-target feedback Plant from the canonical
     *                     production recipe
     */
    public static TeleOpTester velocityControl(
            String testerName,
            ScalarRange allowedTestTargetRange,
            Function<HardwareMap, Plant> plantFactory
    ) {
        return new FtcVelocityControlPanelsTester(
                testerName,
                allowedTestTargetRange,
                plantFactory);
    }

    /**
     * Creates one exact-position endpoint experiment for a Plant whose coordinate is already
     * referenced, or whose claimed controller can safely prepare an assume-current hold.
     *
     * <p>The finite bounded range is the physical experiment and recovery envelope. Its endpoints
     * seed the two editable exact, unwrapped leg targets; it is never interpreted as an automatic
     * sweep. B and automatic timeout sample the current position before that cycle's normal output,
     * then hold it or recover to the nearest envelope boundary.</p>
     *
     * <p>An unreferenced ASSUME_CURRENT Plant establishes that reference from one START-loop sample
     * before its first normal output. A Plant that needs a physical homing/indexing task rejects at
     * that preparation boundary; use the overload with {@code referenceTaskFactory} for it.</p>
     *
     * @param testerName human-facing tester title
     * @param allowedPhysicalTargetRange finite bounded experiment/recovery envelope in Plant units
     * @param plantFactory creates a fresh inactive exact-command position Plant from the canonical
     *                     production recipe
     */
    public static TeleOpTester positionControl(
            String testerName,
            ScalarRange allowedPhysicalTargetRange,
            Function<HardwareMap, PositionPlant> plantFactory
    ) {
        return new FtcPositionControlPanelsTester(
                testerName,
                allowedPhysicalTargetRange,
                plantFactory,
                null);
    }

    /**
     * Creates one exact-position endpoint experiment with an integrated reference Task.
     *
     * <p>The task factory is invoked for each A-authorized reference attempt and receives the same
     * fresh Plant owned by the tester. Cancelling an attempt never reuses that single-use Task.</p>
     *
     * @param testerName human-facing tester title
     * @param allowedPhysicalTargetRange finite bounded experiment/recovery envelope in Plant units
     * @param plantFactory creates a fresh inactive exact-command position Plant from the canonical
     *                     production recipe
     * @param referenceTaskFactory creates one fresh non-blocking reference Task per A-authorized
     *                             attempt, using the tester-owned Plant
     */
    public static TeleOpTester positionControl(
            String testerName,
            ScalarRange allowedPhysicalTargetRange,
            Function<HardwareMap, PositionPlant> plantFactory,
            BiFunction<HardwareMap, PositionPlant, Task> referenceTaskFactory
    ) {
        return new FtcPositionControlPanelsTester(
                testerName,
                allowedPhysicalTargetRange,
                plantFactory,
                Objects.requireNonNull(referenceTaskFactory, "referenceTaskFactory"));
    }

    static void seedActiveDraft(ControlTuningModel.Parameters parameters,
                                Map<String, Double> experimentFields) {
        seedActiveDraft(parameters, experimentFields,
                () -> PanelsConfigurables.INSTANCE.refreshClass(DRAFT_REFRESH_TOKEN));
    }

    /** Package-local seam for proving schema replacement independently of the robot server. */
    static void seedActiveDraft(ControlTuningModel.Parameters parameters,
                                Map<String, Double> experimentFields,
                                Runnable refreshAction) {
        LinkedHashMap<String, Double> seeded = new LinkedHashMap<String, Double>();
        for (Map.Entry<String, Double> entry : parameters.values().entrySet()) {
            seeded.put(controllerKey(entry.getKey()), entry.getValue());
        }
        for (Map.Entry<String, Double> entry : experimentFields.entrySet()) {
            String key = experimentKey(entry.getKey());
            if (seeded.put(key, entry.getValue()) != null) {
                throw new IllegalArgumentException("Duplicate Panels draft field: " + key);
            }
        }
        draft = Collections.synchronizedMap(seeded);
        Objects.requireNonNull(refreshAction, "refreshAction").run();
    }

    static Map<String, Double> readActiveDraft() {
        Map<String, Double> active = draft;
        synchronized (active) {
            return new LinkedHashMap<String, Double>(active);
        }
    }

    static ControlTuningModel.Parameters readControllerFields(
            Map<String, Double> snapshot,
            ControlTuningModel.Parameters schema) {
        LinkedHashMap<String, Double> values = new LinkedHashMap<String, Double>();
        for (String name : schema.values().keySet()) {
            Double value = snapshot.get(controllerKey(name));
            values.put(name, value == null ? Double.NaN : value);
        }
        return new ControlTuningModel.Parameters(values);
    }

    static double readExperimentField(Map<String, Double> snapshot, String name) {
        Double value = snapshot.get(experimentKey(name));
        return value == null ? Double.NaN : value;
    }

    static String controllerKey(String field) {
        return "controller." + field;
    }

    static String experimentKey(String field) {
        return "experiment." + field;
    }

    private static Map<String, Double> emptySynchronizedMap() {
        return Collections.synchronizedMap(new LinkedHashMap<String, Double>());
    }
}
