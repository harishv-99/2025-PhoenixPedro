package edu.ftcphoenix.fw.integrations.panels;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Function;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;

/**
 * Concrete Panels-backed tuning workflows for FTC mechanisms.
 *
 * <p>Robot code selects one complete workflow from this facade. It does not build a controller
 * session, mutable parameter registry, Panels transport, segment recorder, or custom tester state
 * machine. The supplied Plant factory is an explicitly advanced tester seam: it must return a
 * fresh Plant built from the same canonical recipe as production, and the tester becomes that
 * Plant's sole heartbeat and lifecycle owner.</p>
 */
@Configurable
public final class FtcPanelsTuners {

    private static final FtcPanelsTuners DRAFT_REFRESH_TOKEN = new FtcPanelsTuners();

    /** Draft proportional coefficient for the active velocity-PIDF tester. */
    @Sorter(sort = 10)
    public static volatile double kP;

    /** Draft integral coefficient for the active velocity-PIDF tester. */
    @Sorter(sort = 20)
    public static volatile double kI;

    /** Draft derivative coefficient for the active velocity-PIDF tester. */
    @Sorter(sort = 30)
    public static volatile double kD;

    /** Draft feed-forward coefficient for the active velocity-PIDF tester. */
    @Sorter(sort = 40)
    public static volatile double kF;

    /** Draft Plant velocity target in that Plant's units. */
    @Sorter(sort = 50)
    public static volatile double testTarget;

    /** Draft automatic-stop delay in seconds; zero keeps the segment active. */
    @Sorter(sort = 60)
    public static volatile double autoStopAfterSec;

    private FtcPanelsTuners() {
        // utility/configurable facade
    }

    /**
     * Creates one device-managed velocity-PIDF tuning tester.
     *
     * <p>The inactive tester acquires the Plant and FTC controller only from its
     * {@code init(TesterContext)} callback. A changes no hardware until the complete Panels draft
     * has remained unchanged for the best-effort capture interval. A first/cold candidate waits
     * for finite feedback and {@code plant.atTarget(0.0)}. A later candidate is applied as a hot
     * segment without deliberately requesting zero. B requests zero but leaves the session ready
     * for another cold segment. BACK, OpMode stop, disconnect, or a terminal failure stops the
     * Plant and best-effort restores the controller configuration captured at tester init.</p>
     *
     * @param testerName human-facing tester title
     * @param testTargetRange inclusive finite positive range accepted for test targets; its lower
     *                        endpoint seeds the initial Panels draft; it must lie within the
     *                        completed Plant's target range
     * @param plantFactory creates a fresh inactive single-motor FTC device-managed velocity Plant
     *                     with a command target
     * @return an inactive tester ready for an FTC/Panels tester host
     */
    public static TeleOpTester velocityPidf(
            String testerName,
            ScalarRange testTargetRange,
            Function<HardwareMap, Plant> plantFactory
    ) {
        return new FtcVelocityPidfPanelsTester(
                testerName,
                testTargetRange,
                plantFactory);
    }

    static FtcVelocityPidfPanelsTester.Candidate readVelocityDraft() {
        return new FtcVelocityPidfPanelsTester.Candidate(
                kP,
                kI,
                kD,
                kF,
                testTarget,
                autoStopAfterSec);
    }

    static void seedVelocityDraft(FtcVelocityPidfPanelsTester.Gains gains,
                                  double target,
                                  double autoStopAfterSec) {
        kP = gains.kP;
        kI = gains.kI;
        kD = gains.kD;
        kF = gains.kF;
        testTarget = target;
        FtcPanelsTuners.autoStopAfterSec = autoStopAfterSec;
        PanelsConfigurables.INSTANCE.refreshClass(DRAFT_REFRESH_TOKEN);
    }
}
