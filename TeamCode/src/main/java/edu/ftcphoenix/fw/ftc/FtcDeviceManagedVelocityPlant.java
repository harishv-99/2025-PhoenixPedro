package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetResolution;
import edu.ftcphoenix.fw.actuation.PlantTargetStatus;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Package-private identity carrier for a single FTC device-managed velocity Plant.
 *
 * <p>The core {@link Plant} contract deliberately contains no FTC device metadata. This wrapper
 * keeps that boundary intact while allowing {@link FtcMotorControllers} to derive a configuration
 * handle from the exact motor already selected by {@link FtcActuators}, rather than accepting a
 * second independently supplied hardware name.</p>
 */
final class FtcDeviceManagedVelocityPlant implements Plant {

    private final Plant delegate;
    private final DcMotorEx motor;
    private final String motorName;
    private final ScalarRange targetRange;
    private boolean velocityPidfClaimed;

    FtcDeviceManagedVelocityPlant(Plant delegate,
                                  DcMotorEx motor,
                                  String motorName,
                                  ScalarRange targetRange) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.motor = Objects.requireNonNull(motor, "motor");
        this.motorName = Objects.requireNonNull(motorName, "motorName");
        this.targetRange = Objects.requireNonNull(targetRange, "targetRange");
    }

    DcMotorEx motor() {
        return motor;
    }

    String motorName() {
        return motorName;
    }

    ScalarRange targetRange() {
        return targetRange;
    }

    synchronized void claimVelocityPidf() {
        if (velocityPidfClaimed) {
            throw new IllegalStateException(
                    "This FTC device-managed velocity Plant already has a velocity PIDF "
                            + "configuration owner; create a fresh Plant for another session");
        }
        velocityPidfClaimed = true;
    }

    @Override
    public void update(LoopClock clock) {
        delegate.update(clock);
    }

    @Override
    public double getRequestedTarget() {
        return delegate.getRequestedTarget();
    }

    @Override
    public double getAppliedTarget() {
        return delegate.getAppliedTarget();
    }

    @Override
    public PlantTargetResolution getTargetResolution() {
        return delegate.getTargetResolution();
    }

    @Override
    public PlantTargetStatus getTargetStatus() {
        return delegate.getTargetStatus();
    }

    @Override
    public boolean hasFeedback() {
        return delegate.hasFeedback();
    }

    @Override
    public double getMeasurement() {
        return delegate.getMeasurement();
    }

    @Override
    public double getRequestedTargetError() {
        return delegate.getRequestedTargetError();
    }

    @Override
    public double getAppliedTargetError() {
        return delegate.getAppliedTargetError();
    }

    @Override
    public boolean atTarget() {
        return delegate.atTarget();
    }

    @Override
    public boolean atTarget(double target) {
        return delegate.atTarget(target);
    }

    @Override
    public boolean hasCommandTarget() {
        return delegate.hasCommandTarget();
    }

    @Override
    public ScalarTarget commandTarget() {
        return delegate.commandTarget();
    }

    @Override
    public void stop() {
        delegate.stop();
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        delegate.debugDump(dbg, prefix);
    }
}
