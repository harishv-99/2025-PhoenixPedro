package edu.ftcsushi.fw.testing.ftc;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Verifies the shared example hardware bench's registry and evidence boundaries. */
public final class FtcTestHardwareTest {

    @Test
    public void effectiveNamesAreTrimmedCaseSensitiveAndUniqueAcrossDeviceTypes() {
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe lower = hardware.addMotor("  motor  ");
        FtcTestHardware.MotorProbe upper = hardware.addMotor("Motor");

        assertSame(lower, hardware.motor("motor"));
        assertSame(upper, hardware.motor(" Motor "));
        assertThrows(IllegalArgumentException.class,
                () -> hardware.addDigitalInput(" motor "));
        assertThrows(IllegalArgumentException.class, () -> hardware.addServo("  "));
        assertThrows(IllegalArgumentException.class, () -> hardware.addCrServo(null));
    }

    @Test
    public void lookupFailsClearlyForMissingOrWrongDeviceType() {
        FtcTestHardware hardware = new FtcTestHardware();
        hardware.addMotor("intake");

        IllegalArgumentException wrongType = assertThrows(
                IllegalArgumentException.class,
                () -> hardware.get(Servo.class, "intake"));
        assertTrue(wrongType.getMessage().contains("not a Servo"));

        IllegalArgumentException missing = assertThrows(
                IllegalArgumentException.class,
                () -> hardware.get(DigitalChannel.class, "bottom"));
        assertTrue(missing.getMessage().contains("bottom"));
        assertEquals(2, hardware.lookupCalls());
        assertEquals("bottom", hardware.lastLookupName());
    }

    @Test
    public void motorCommandsNeverManufactureEncoderOrVelocityFeedback() {
        List<String> events = new ArrayList<String>();
        FtcTestHardware hardware = new FtcTestHardware(events);
        FtcTestHardware.MotorProbe probe = hardware.addMotor("lift");
        DcMotorEx motor = hardware.get(DcMotorEx.class, "lift");
        probe.setCurrentPositionTicks(41);
        probe.setMeasuredVelocityTicksPerSec(Double.NaN);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(900);
        motor.setPower(0.25);
        motor.setVelocity(1200.0);

        assertEquals(41, probe.currentPositionTicks());
        assertEquals(41, motor.getCurrentPosition());
        assertTrue(Double.isNaN(motor.getVelocity()));
        assertTrue(Double.isNaN(probe.measuredVelocityTicksPerSec()));
        assertEquals(900, probe.targetPositionTicks());
        assertEquals(1, probe.targetPositionWrites());
        assertEquals(0.25, probe.power(), 0.0);
        assertEquals(1200.0, probe.commandedVelocityTicksPerSec(), 0.0);
        assertEquals(1, probe.powerWrites());
        assertEquals(1, probe.velocityWrites());
        assertEquals(1, probe.velocityReadCalls());
        assertEquals(DcMotorSimple.Direction.REVERSE, probe.direction());
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, probe.zeroPowerBehavior());
        assertEquals("power:lift:0.25", events.get(0));
        assertEquals(1, hardware.totalMotorPowerWrites());
    }

    @Test
    public void unsupportedCallsAndAmbiguousVelocityOverloadsFailFast() {
        FtcTestHardware hardware = new FtcTestHardware();
        hardware.addMotor("flywheel");
        hardware.addServo("release");
        DcMotorEx motor = hardware.get(DcMotorEx.class, "flywheel");
        Servo servo = hardware.get(Servo.class, "release");

        UnsupportedOperationException busy = assertThrows(
                UnsupportedOperationException.class,
                motor::isBusy);
        assertTrue(busy.getMessage().contains("isBusy()"));
        assertTrue(busy.getMessage().contains("explicit input or output evidence"));
        assertThrows(UnsupportedOperationException.class,
                () -> motor.setVelocity(900.0, AngleUnit.DEGREES));
        assertThrows(UnsupportedOperationException.class,
                () -> motor.getVelocity(AngleUnit.DEGREES));
        assertThrows(UnsupportedOperationException.class,
                () -> motor.setTargetPositionTolerance(12));
        assertThrows(UnsupportedOperationException.class,
                motor::getTargetPositionTolerance);
        assertThrows(UnsupportedOperationException.class,
                () -> servo.scaleRange(0.2, 0.8));
    }

    @Test
    public void digitalFeedbackIsIndependentAndServoOutputsAreRecorded() {
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.DigitalProbe input = hardware.addDigitalInput("bottom");
        FtcTestHardware.CrServoProbe transfer = hardware.addCrServo("transfer");
        FtcTestHardware.ServoProbe release = hardware.addServo("release");

        DigitalChannel channel = hardware.get(DigitalChannel.class, "bottom");
        CRServo crServo = hardware.get(CRServo.class, "transfer");
        Servo servo = hardware.get(Servo.class, "release");
        input.setHigh(false);
        crServo.setDirection(DcMotorSimple.Direction.REVERSE);
        crServo.setPower(-0.4);
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0.7);

        assertFalse(channel.getState());
        assertFalse(input.high());
        assertEquals(1, input.stateReadCalls());
        assertEquals(-0.4, transfer.power(), 0.0);
        assertEquals(1, transfer.powerWrites());
        assertEquals(DcMotorSimple.Direction.REVERSE, transfer.direction());
        assertEquals(0.7, release.position(), 0.0);
        assertEquals(1, release.positionWrites());
        assertEquals(Servo.Direction.REVERSE, release.direction());
    }

    @Test
    public void digitalProbeRecordsConfigurationFailureCallbackAndAliases() {
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.DigitalProbe input = hardware.addDigitalInput("first");
        hardware.addDigitalInputAlias("sameDevice", input);
        DigitalChannel first = hardware.get(DigitalChannel.class, "first");
        DigitalChannel alias = hardware.get(DigitalChannel.class, "sameDevice");

        first.setMode(DigitalChannel.Mode.OUTPUT);
        assertEquals(1, input.modeWriteCalls());
        assertSame(first, alias);

        RuntimeException failure = new RuntimeException("injected read failure");
        final int[] callbacks = {0};
        input.beforeNextRead(() -> callbacks[0]++);
        input.setReadFailure(failure);
        assertSame(failure, assertThrows(RuntimeException.class, alias::getState));
        assertEquals(1, callbacks[0]);
        assertEquals(1, input.stateReadCalls());

        input.setReadFailure(null);
        assertTrue(first.getState());
        assertEquals(2, input.stateReadCalls());
    }

}
