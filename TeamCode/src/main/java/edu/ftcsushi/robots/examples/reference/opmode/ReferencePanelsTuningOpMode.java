package edu.ftcsushi.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.integrations.panels.FtcPanelsTeleOpTesterOpMode;
import edu.ftcsushi.fw.integrations.panels.FtcPanelsTuners;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;
import edu.ftcsushi.robots.examples.reference.capability.flywheel.ReferenceFlywheelMechanism;

/** Disabled Panels host for learning the exclusive control-tuning workflow safely. */
@TeleOp(name = "FW Reference: Tuning (Panels)", group = "FW Examples")
@Disabled
public final class ReferencePanelsTuningOpMode extends FtcPanelsTeleOpTesterOpMode {

    /** Requires exactly one Panels client so one operator owns every tuning decision. */
    public ReferencePanelsTuningOpMode() {
        super(InputSource.PANELS, PanelsClientRequirement.EXACTLY_ONE);
    }

    /** Creates one tuner that owns a fresh Plant from the production flywheel recipe. */
    @Override
    protected TeleOpTester createTester() {
        ReferenceFlywheelMechanism.Config flywheels =
                ReferenceFlywheelMechanism.Config.defaults();
        return FtcPanelsTuners.velocityControl(
                "Reference Flywheel Velocity Control",
                ScalarRange.bounded(0.0, flywheels.maximumVelocityTicksPerSec),
                hardwareMap -> ReferenceFlywheelMechanism.createPlantForTuning(
                        hardwareMap,
                        flywheels));
    }
}
