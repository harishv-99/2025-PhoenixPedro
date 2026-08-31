package edu.ftcsushi.robots.phoenix.tester;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Set;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.tools.tester.BaseTeleOpTester;

/**
 * Robot-specific tester for verifying Phoenix's configured drivetrain wiring and directions.
 *
 * <p>This is an integration check for the names and directions already stored in
 * the supplied robot configuration, not a direction-discovery tool. After Driver Station START and one neutral observation,
 * holding exactly one mapped face button commands only that wheel at a fixed positive power.
 * Releasing the button or holding multiple face buttons commands zero to every wheel.</p>
 *
 * <p>INIT is presentation-only: this tester does not construct or update a Plant until Driver Station START.
 * Explicit STOP best-effort stops all constructed Plants.</p>
 */
final class ConfiguredDrivetrainVerificationTester extends BaseTeleOpTester {

    private static final double TEST_POWER = 0.20;

    private Plant plantFL;
    private Plant plantFR;
    private Plant plantBL;
    private Plant plantBR;
    private final FtcDrives.MecanumConfig specifiedDriveConfig;
    private FtcDrives.MecanumConfig driveConfig;
    private boolean started;
    private boolean neutralObserved;
    private int pressedCount;

    /**
     * Creates a tester around one defensive configuration snapshot for focused verification.
     *
     * <p>The robot facade chooses the current drive draft explicitly; this owner never reaches
     * through a process-global profile or offers a hidden default path.</p>
     */
    ConfiguredDrivetrainVerificationTester(FtcDrives.MecanumConfig driveConfig) {
        if (driveConfig == null) {
            throw new IllegalArgumentException("drive config is required");
        }
        specifiedDriveConfig = driveConfig.copy();
    }

    /**
     * Returns the display name shown by the tester framework.
     *
     * @return short user-facing tester name
     */
    @Override
    public String name() {
        return "Configured Drivetrain Verification";
    }

    @Override
    protected void onInit() {
        driveConfig = specifiedDriveConfig.copy();
        started = false;
        neutralObserved = false;
        pressedCount = 0;
    }

    @Override
    protected void onInitLoop(double dtSec) {
        renderInit();
    }

    @Override
    protected void onStart() {
        if (started) {
            throw new IllegalStateException(
                    "Configured drivetrain verification has already started; re-enter the tester for a fresh session.");
        }

        FtcDrives.MecanumWiringConfig wiring = driveConfig.wiring;
        validateAndResolveWiring(wiring);

        plantFL = FtcActuators.plant(ctx.hw)
                .motor(wiring.frontLeftName, wiring.frontLeftDirection)
                .power()
                .targetFromNewCommand(0.0)
                .build();

        plantFR = FtcActuators.plant(ctx.hw)
                .motor(wiring.frontRightName, wiring.frontRightDirection)
                .power()
                .targetFromNewCommand(0.0)
                .build();

        plantBL = FtcActuators.plant(ctx.hw)
                .motor(wiring.backLeftName, wiring.backLeftDirection)
                .power()
                .targetFromNewCommand(0.0)
                .build();

        plantBR = FtcActuators.plant(ctx.hw)
                .motor(wiring.backRightName, wiring.backRightDirection)
                .power()
                .targetFromNewCommand(0.0)
                .build();

        started = true;
        neutralObserved = false;
        pressedCount = 0;
        commandAndUpdate(0.0, 0.0, 0.0, 0.0);
    }

    /** Validate and resolve the complete hardware graph before any Plant can configure a motor. */
    private void validateAndResolveWiring(FtcDrives.MecanumWiringConfig wiring) {
        if (wiring == null) {
            throw new IllegalArgumentException("Phoenix drivetrain wiring is required");
        }

        String[] wheelLabels = {"front-left", "front-right", "back-left", "back-right"};
        String[] names = {
                wiring.frontLeftName,
                wiring.frontRightName,
                wiring.backLeftName,
                wiring.backRightName
        };
        Direction[] directions = {
                wiring.frontLeftDirection,
                wiring.frontRightDirection,
                wiring.backLeftDirection,
                wiring.backRightDirection
        };

        Set<String> uniqueNames = new HashSet<>();
        for (int i = 0; i < names.length; i++) {
            if (names[i] == null || names[i].trim().isEmpty()) {
                throw new IllegalArgumentException(
                        "Phoenix drivetrain " + wheelLabels[i] + " motor name must not be blank");
            }
            names[i] = names[i].trim();
            if (!uniqueNames.add(names[i])) {
                throw new IllegalArgumentException(
                        "Phoenix drivetrain motor names must be unique after trimming; duplicate '"
                                + names[i] + "'");
            }
            if (directions[i] == null) {
                throw new IllegalArgumentException(
                        "Phoenix drivetrain " + wheelLabels[i] + " direction is required");
            }
        }

        IdentityHashMap<DcMotorEx, String> resolvedOwners = new IdentityHashMap<>();
        for (int i = 0; i < names.length; i++) {
            final DcMotorEx resolved;
            try {
                resolved = ctx.hw.get(DcMotorEx.class, names[i]);
            } catch (RuntimeException failure) {
                throw new IllegalArgumentException(
                        "Cannot resolve Phoenix drivetrain " + wheelLabels[i]
                                + " motor '" + names[i] + "' as DcMotorEx",
                        failure);
            }
            String priorOwner = resolvedOwners.put(resolved, wheelLabels[i]);
            if (priorOwner != null) {
                throw new IllegalArgumentException(
                        "Phoenix drivetrain names '" + names[i] + "' (" + wheelLabels[i]
                                + ") and the " + priorOwner
                                + " name resolve to the same motor device");
            }
        }

        // Retain the same normalized names that were validated and resolved above.
        wiring.frontLeftName = names[0];
        wiring.frontRightName = names[1];
        wiring.backLeftName = names[2];
        wiring.backRightName = names[3];
    }

    @Override
    protected void onLoop(double dtSec) {
        boolean x = gamepads.p1().x().getAsBoolean(clock);
        boolean y = gamepads.p1().y().getAsBoolean(clock);
        boolean a = gamepads.p1().a().getAsBoolean(clock);
        boolean b = gamepads.p1().b().getAsBoolean(clock);

        pressedCount = countPressed(x, y, a, b);
        if (!neutralObserved) {
            if (pressedCount == 0) {
                neutralObserved = true;
            }
            commandAndUpdate(0.0, 0.0, 0.0, 0.0);
        } else if (pressedCount == 1) {
            commandAndUpdate(
                    x ? TEST_POWER : 0.0,
                    y ? TEST_POWER : 0.0,
                    a ? TEST_POWER : 0.0,
                    b ? TEST_POWER : 0.0);
        } else {
            commandAndUpdate(0.0, 0.0, 0.0, 0.0);
        }

        renderActive();
    }

    @Override
    protected void onStop() {
        started = false;
        neutralObserved = false;
        pressedCount = 0;
        stopAll();
    }

    private void commandAndUpdate(double fl, double fr, double bl, double br) {
        plantFL.commandTarget().set(fl);
        plantFR.commandTarget().set(fr);
        plantBL.commandTarget().set(bl);
        plantBR.commandTarget().set(br);

        plantFL.update(clock);
        plantFR.update(clock);
        plantBL.update(clock);
        plantBR.update(clock);
    }

    private void renderInit() {
        telemHeader("Configured Drivetrain Verification");
        telemHint("INIT: no drivetrain commands. Raise and secure all wheels, then press Driver Station START.");
        telemHint("This verifies PhoenixDriveConfiguration.current() names/directions; establish raw direction with HW: Actuator Bring-up.");
        telemHint("After Driver Station START, release A/B/X/Y once to arm.");
        telemUpdate();
    }

    private void renderActive() {
        telemHeader("Configured Drivetrain Verification");
        if (!neutralObserved) {
            telemHint("Release A/B/X/Y to arm fresh controls.");
        } else if (pressedCount > 1) {
            telemHint("CONFLICT: multiple buttons held; every wheel is commanded zero.");
        } else {
            telemHint("Hold exactly one: X=FL | Y=FR | A=BL | B=BR. Release commands all zero.");
        }
        ctx.telemetry.addData("FL applied target", plantFL.getAppliedTarget());
        ctx.telemetry.addData("FR applied target", plantFR.getAppliedTarget());
        ctx.telemetry.addData("BL applied target", plantBL.getAppliedTarget());
        ctx.telemetry.addData("BR applied target", plantBR.getAppliedTarget());
        telemUpdate();
    }

    private void stopAll() {
        CleanupActions.attemptAll(
                () -> stopIfPresent(plantFL),
                () -> stopIfPresent(plantFR),
                () -> stopIfPresent(plantBL),
                () -> stopIfPresent(plantBR));
    }

    private static void stopIfPresent(Plant plant) {
        if (plant != null) {
            plant.stop();
        }
    }

    private static int countPressed(boolean x, boolean y, boolean a, boolean b) {
        int count = 0;
        if (x) count++;
        if (y) count++;
        if (a) count++;
        if (b) count++;
        return count;
    }
}
