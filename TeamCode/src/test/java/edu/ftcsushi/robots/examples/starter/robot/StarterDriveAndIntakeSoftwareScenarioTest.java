package edu.ftcsushi.robots.examples.starter.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcsushi.robots.examples.starter.support.StarterTestHardware;

import static edu.ftcsushi.robots.examples.starter.support.StarterTestHardware.fullTeleOpHardware;
import static edu.ftcsushi.robots.examples.starter.support.StarterTestHardware.prepare;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Managed software proof for the complete Starter drive-and-intake TeleOp composition. */
public final class StarterDriveAndIntakeSoftwareScenarioTest {

    @Test
    public void oneGamepadDrivesContinuouslyAndRequestsIntakeBeforePresentation() {
        // ARRANGE: keep production robot/controls/outputs and managed lifecycle; replace host seams.
        StarterProfile profile = enabledProfile();
        List<String> events = new ArrayList<String>();
        FtcTestHardware hardware = fullTeleOpHardware(profile, events);
        StarterTestHardware.TelemetryProbe telemetry =
                new StarterTestHardware.TelemetryProbe(events);
        Gamepad driver = new Gamepad();
        ManagedTeleOp mode = prepare(
                new ManagedTeleOp(profile), hardware, telemetry, driver);
        mode.init();
        mode.start();
        mode.advanceTo(0.01);
        mode.loop(); // Establish the released button state before testing a rising edge.

        // REQUEST: the same FTC gamepad supplies one button edge and a held forward stick.
        events.clear();
        driver.a = true;
        driver.left_stick_y = -1.0f;
        mode.advanceTo(0.02);
        mode.loop();

        // ASSERT: bindings run before the intake output, drive output, and cached presenter.
        assertEquals(profile.intake.collectPower,
                hardware.motor(profile.intake.motorName).power(), 0.0);
        assertEquals(profile.drive.drivebase.maxAxial,
                hardware.motor(profile.drive.wiring.frontLeftName).power(), 0.0);
        assertEquals(StarterIntake.Mode.COLLECT, telemetry.dataValue("intake.mode"));
        assertInOrder(events,
                "power:" + profile.intake.motorName + ":",
                "power:" + profile.drive.wiring.frontLeftName + ":",
                "telemetry.row:intake.mode",
                "telemetry.commit");

        // HEARTBEAT: drive keeps sampling the held stick; the one intake request stays selected.
        int driveWrites = hardware.motor(profile.drive.wiring.frontLeftName).powerWrites();
        driver.a = false;
        mode.advanceTo(0.04);
        mode.loop();
        assertEquals(driveWrites + 1,
                hardware.motor(profile.drive.wiring.frontLeftName).powerWrites());
        assertEquals(profile.intake.collectPower,
                hardware.motor(profile.intake.motorName).power(), 0.0);

        // HELD SLOW MODE: no edge is needed; every drive sample scales the same stick request.
        double fullForwardPower =
                hardware.motor(profile.drive.wiring.frontLeftName).power();
        driver.right_bumper = true;
        mode.advanceTo(0.06);
        mode.loop();
        assertEquals(fullForwardPower * StarterTeleOpControls.SLOW_TRANSLATE_SCALE,
                hardware.motor(profile.drive.wiring.frontLeftName).power(), 1e-9);

        mode.advanceTo(0.08);
        mode.loop();
        assertEquals(fullForwardPower * StarterTeleOpControls.SLOW_TRANSLATE_SCALE,
                hardware.motor(profile.drive.wiring.frontLeftName).power(), 1e-9);

        // RELEASE: level-based precision mode ends immediately; normal translation returns.
        driver.right_bumper = false;
        mode.advanceTo(0.10);
        mode.loop();
        assertEquals(fullForwardPower,
                hardware.motor(profile.drive.wiring.frontLeftName).power(), 1e-9);

        // TURN: the same held-level decorator independently applies the documented omega scale.
        driver.left_stick_y = 0.0f;
        driver.right_stick_x = -1.0f;
        mode.advanceTo(0.12);
        mode.loop();
        double fullTurnPower =
                hardware.motor(profile.drive.wiring.frontLeftName).power();
        driver.right_bumper = true;
        mode.advanceTo(0.14);
        mode.loop();
        assertEquals(fullTurnPower * StarterTeleOpControls.SLOW_OMEGA_SCALE,
                hardware.motor(profile.drive.wiring.frontLeftName).power(), 1e-9);

        // STOP: terminal cleanup zeros both owners immediately and is idempotent.
        mode.stop();
        assertAllStopped(hardware, profile);
        int writesAfterStop = hardware.totalMotorPowerWrites();
        mode.stop();
        mode.loop();
        assertEquals(writesAfterStop, hardware.totalMotorPowerWrites());
        // NEXT GATE: verify each owner separately before a supervised combined hardware run.
    }

    @Test
    public void duplicateMotorOwnershipFailsBeforeAnyHardwareLookup() {
        // ARRANGE: make the intake name whitespace-equivalent to one drive motor name.
        StarterProfile profile = enabledProfile();
        profile.intake.motorName = "  " + profile.drive.wiring.frontLeftName + "  ";
        FtcTestHardware hardware = new FtcTestHardware();
        ManagedTeleOp mode = prepare(
                new ManagedTeleOp(profile),
                hardware,
                new StarterTestHardware.TelemetryProbe(),
                new Gamepad());

        try {
            mode.init();
            fail("Expected duplicate motor ownership to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("motor ownership collision"));
            assertTrue(expected.getMessage().contains("StarterProfile.intake.motorName"));
            assertTrue(expected.getMessage().contains(
                    "StarterProfile.drive.wiring.frontLeftName"));
        }
        assertEquals(0, hardware.lookupCalls());
    }

    private static StarterProfile enabledProfile() {
        StarterProfile profile = StarterProfile.current();
        profile.allowIntakeMotion = true;
        profile.allowDriveMotion = true;
        return profile;
    }

    private static void assertAllStopped(FtcTestHardware hardware, StarterProfile profile) {
        assertEquals(0.0, hardware.motor(profile.intake.motorName).power(), 0.0);
        assertEquals(0.0, hardware.motor(profile.drive.wiring.frontLeftName).power(), 0.0);
        assertEquals(0.0, hardware.motor(profile.drive.wiring.frontRightName).power(), 0.0);
        assertEquals(0.0, hardware.motor(profile.drive.wiring.backLeftName).power(), 0.0);
        assertEquals(0.0, hardware.motor(profile.drive.wiring.backRightName).power(), 0.0);
    }

    private static void assertInOrder(List<String> events, String... prefixes) {
        int previous = -1;
        for (String prefix : prefixes) {
            int found = eventIndex(events, prefix);
            assertTrue("Missing event " + prefix + " in " + events, found >= 0);
            assertTrue("Events are out of order: " + events, found > previous);
            previous = found;
        }
    }

    private static int eventIndex(List<String> events, String prefix) {
        for (int index = 0; index < events.size(); index++) {
            if (events.get(index).startsWith(prefix)) {
                return index;
            }
        }
        return -1;
    }

    /** Test-only managed host that supplies the reviewed profile synchronously during INIT. */
    private static final class ManagedTeleOp extends FtcRobotOpMode {
        private final StarterProfile profile;
        private double runtimeSec;

        private ManagedTeleOp(StarterProfile profile) {
            this.profile = profile;
        }

        @Override
        protected void configure(RobotProgram program) {
            new StarterRobot(hardwareMap).declareTeleOp(program, profile, gamepad1);
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }

        private void advanceTo(double nowSec) {
            runtimeSec = nowSec;
        }
    }
}
