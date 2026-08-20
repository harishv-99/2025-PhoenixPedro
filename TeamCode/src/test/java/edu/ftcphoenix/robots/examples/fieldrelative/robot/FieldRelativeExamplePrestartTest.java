package edu.ftcphoenix.robots.examples.fieldrelative.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.util.Arrays;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.RobotProgram;

import static org.junit.Assert.assertEquals;

public final class FieldRelativeExamplePrestartTest {
    @Test
    public void freezesSelectedStationWithIndependentHeadings() {
        Gamepad gamepad = new Gamepad();
        FieldRelativeExampleProfile.Station first =
                new FieldRelativeExampleProfile.Station("A", "A", 0.1, 0.2);
        FieldRelativeExampleProfile.Station orthogonal =
                new FieldRelativeExampleProfile.Station("B", "B", -0.7, Math.PI / 2.0);
        FieldRelativeExamplePrestart prestart = new FieldRelativeExamplePrestart(
                Arrays.asList(first, orthogonal),
                gamepad
        );
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        prestart.update(clock); // edge baseline
        gamepad.dpad_down = true;
        clock.update(0.02);
        prestart.update(clock);

        assertEquals(RobotProgram.StartDisposition.READY, prestart.freezeForStart());
        assertEquals(-0.7, prestart.frozenInitialRobotFieldHeadingRad(), 0.0);
        assertEquals(Math.PI / 2.0, prestart.frozenControlUpFieldHeadingRad(), 0.0);
    }
}
