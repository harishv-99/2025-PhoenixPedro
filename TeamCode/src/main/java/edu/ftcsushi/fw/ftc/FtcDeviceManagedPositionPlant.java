package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.actuation.PlantTargetStatus;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.PositionPlantSnapshot;
import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;

/** Package-private FTC identity carrier for a single device-managed position Plant. */
final class FtcDeviceManagedPositionPlant implements PositionPlant {

    private final PositionPlant delegate;
    private final DcMotorEx motor;
    private final String motorName;
    private final ScalarRange configuredTargetRange;
    private boolean controllerClaimed;
    private boolean stopped;

    FtcDeviceManagedPositionPlant(PositionPlant delegate,
                                  DcMotorEx motor,
                                  String motorName,
                                  ScalarRange configuredTargetRange) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.motor = Objects.requireNonNull(motor, "motor");
        this.motorName = Objects.requireNonNull(motorName, "motorName");
        this.configuredTargetRange = Objects.requireNonNull(
                configuredTargetRange, "configuredTargetRange");
    }

    DcMotorEx motor() { return motor; }
    String motorName() { return motorName; }
    ScalarRange configuredTargetRange() { return configuredTargetRange; }
    PositionPlant tuningDelegate() { return delegate; }

    synchronized void claimController() {
        if (controllerClaimed) {
            throw new IllegalStateException(
                    "This FTC device-managed position Plant already has a controller-tuning "
                            + "owner; create a fresh Plant for another session");
        }
        if (stopped) {
            throw new IllegalStateException("Claim the FTC device-managed position controller "
                    + "before the Plant is stopped");
        }
        controllerClaimed = true;
    }

    synchronized void requireControllerConfigurationActive(String operation) {
        if (stopped) {
            throw new IllegalStateException(operation + " is unavailable after the bound Plant "
                    + "was stopped; only best-effort restoration remains available");
        }
    }

    @Override public void update(LoopClock clock) { delegate.update(clock); }
    @Override public double getRequestedTarget() { return delegate.getRequestedTarget(); }
    @Override public double getAppliedTarget() { return delegate.getAppliedTarget(); }
    @Override public PlantTargetResolution getTargetResolution() { return delegate.getTargetResolution(); }
    @Override public PlantTargetStatus getTargetStatus() { return delegate.getTargetStatus(); }
    @Override public boolean hasFeedback() { return delegate.hasFeedback(); }
    @Override public double getMeasurement() { return delegate.getMeasurement(); }
    @Override public double getRequestedTargetError() { return delegate.getRequestedTargetError(); }
    @Override public double getAppliedTargetError() { return delegate.getAppliedTargetError(); }
    @Override public boolean atTarget() { return delegate.atTarget(); }
    @Override public boolean atTarget(double target) { return delegate.atTarget(target); }
    @Override public boolean hasCommandTarget() { return delegate.hasCommandTarget(); }
    @Override public ScalarTarget commandTarget() { return delegate.commandTarget(); }
    @Override public PositionPlantSnapshot snapshot() { return delegate.snapshot(); }
    @Override public Periodicity periodicity() { return delegate.periodicity(); }
    @Override public double period() { return delegate.period(); }
    @Override public ScalarRange targetRange() { return delegate.targetRange(); }
    @Override public boolean isReferenced() { return delegate.isReferenced(); }
    @Override public String referenceStatus() { return delegate.referenceStatus(); }
    @Override public void establishReferenceAt(double position) { delegate.establishReferenceAt(position); }
    @Override public void establishReferenceAt(double position, LoopClock clock) {
        delegate.establishReferenceAt(position, clock);
    }
    @Override public boolean supportsCalibrationSearch() { return delegate.supportsCalibrationSearch(); }
    @Override public void beginCalibrationSearch(double power) { delegate.beginCalibrationSearch(power); }
    @Override public void endCalibrationSearch() { delegate.endCalibrationSearch(); }
    @Override
    public void stop() {
        synchronized (this) {
            stopped = true;
        }
        delegate.stop();
    }
    @Override public void debugDump(DebugSink dbg, String prefix) { delegate.debugDump(dbg, prefix); }
}
