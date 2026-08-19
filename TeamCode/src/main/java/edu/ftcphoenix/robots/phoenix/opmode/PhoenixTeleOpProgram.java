package edu.ftcphoenix.robots.phoenix.opmode;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.robots.phoenix.PhoenixAlliance;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

/** Internal one-path declaration of the complete managed Phoenix TeleOp program. */
final class PhoenixTeleOpProgram {

    PhoenixTeleOpProgram(PhoenixTeleOp host,
                         RobotProgram program,
                         PhoenixAlliance defaultAlliance) {
        PhoenixTeleOp requiredHost = Objects.requireNonNull(host, "host");
        RobotProgram requiredProgram = Objects.requireNonNull(program, "program");
        PhoenixAlliance requiredDefaultAlliance = Objects.requireNonNull(
                defaultAlliance,
                "defaultAlliance"
        );
        PhoenixProfile profile = PhoenixProfile.current();

        PhoenixTeleOpPrestart prestart = requiredProgram.prestart(
                new PhoenixTeleOpPrestart(
                        profile.targeting,
                        profile.fixedAprilTagLayout,
                        requiredHost.gamepad1,
                        requiredDefaultAlliance
                )
        );
        PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);
        PhoenixRobot robot = new PhoenixRobot(requiredHost.hardwareMap);
        robot.declareTeleOp(
                requiredProgram,
                profile,
                requiredHost.gamepad1,
                requiredHost.gamepad2,
                prestart.eligibleScoringTagIds()
        );

        // Consume only after declareTeleOp has installed localization's pre-START restore seam.
        // A fresh alliance is merely a visible draft seed; PhoenixTeleOpPrestart remains the one
        // START-freeze and target-eligibility authority.
        PhoenixMatchHandoff.RestoreResult restoreResult =
                PhoenixMatchHandoff.restoreForTeleOp(
                        requiredHost,
                        robot,
                        prestart::seedDraftFromAuto
                );

        requiredProgram.presenter(prestart::present);
        requiredProgram.presenter(robot.teleOpPresenter(restoreResult));
    }
}
