package edu.ftcsushi.robots.phoenix.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Objects;
import java.util.Set;

import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.ui.SelectionMenu;
import edu.ftcsushi.fw.ftc.ui.SelectionMenus;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixReadiness;
import edu.ftcsushi.robots.phoenix.scoring.PhoenixTargeting;

/** Owns Phoenix TeleOp's one data-only INIT alliance selection. */
final class PhoenixTeleOpPrestart implements RobotProgram.Prestart {

    private enum DraftOrigin {
        ENTRY_DEFAULT("entry default"),
        AUTO_HANDOFF("fresh Auto handoff"),
        OPERATOR("gamepad 1");

        private final String display;

        DraftOrigin(String display) {
            this.display = display;
        }
    }

    private final PhoenixAlliance defaultAlliance;
    private final EnumMap<PhoenixAlliance, Integer> scoringTagIds =
            new EnumMap<PhoenixAlliance, Integer>(PhoenixAlliance.class);
    private final EnumMap<PhoenixAlliance, PhoenixReadiness.Result> readinessByAlliance =
            new EnumMap<PhoenixAlliance, PhoenixReadiness.Result>(PhoenixAlliance.class);
    private final Bindings bindings = new Bindings();
    private final SelectionMenu<PhoenixAlliance> allianceMenu;

    private PhoenixAlliance draftAlliance;
    private DraftOrigin draftOrigin = DraftOrigin.ENTRY_DEFAULT;
    private PhoenixAlliance frozenAlliance;
    private PhoenixReadiness.Result readiness;
    private RobotProgram.StartDisposition disposition;

    PhoenixTeleOpPrestart(PhoenixTargeting.Config targeting,
                         TagLayout fixedAprilTagLayout,
                         Gamepad gamepad1,
                         PhoenixAlliance defaultAlliance) {
        this.defaultAlliance = Objects.requireNonNull(defaultAlliance, "defaultAlliance");
        for (PhoenixAlliance alliance : PhoenixAlliance.values()) {
            scoringTagIds.put(
                    alliance,
                    targeting == null ? -1 : targeting.scoringTagIdFor(alliance)
            );
            readinessByAlliance.put(
                    alliance,
                    PhoenixReadiness.allianceScoringTarget(
                            alliance,
                            targeting,
                            fixedAprilTagLayout
                    )
            );
        }
        draftAlliance = this.defaultAlliance;
        GamepadDevice driver = new GamepadDevice(
                Objects.requireNonNull(gamepad1, "gamepad1")
        );

        allianceMenu = SelectionMenus.forEnum(
                "TeleOp Alliance",
                PhoenixAlliance.class,
                new SelectionMenus.EnumDisplay<PhoenixAlliance>() {
                    @Override
                    public String id(PhoenixAlliance value) {
                        return value.name();
                    }

                    @Override
                    public String label(PhoenixAlliance value) {
                        return value.label();
                    }

                    @Override
                    public String help(PhoenixAlliance value) {
                        return value.help();
                    }

                    @Override
                    public String tag(PhoenixAlliance value) {
                        return value == PhoenixTeleOpPrestart.this.defaultAlliance
                                ? "DEFAULT"
                                : null;
                    }

                    @Override
                    public boolean enabled(PhoenixAlliance value) {
                        return true;
                    }

                    @Override
                    public String disabledReason(PhoenixAlliance value) {
                        return null;
                    }
                }
        );
        allianceMenu.setHelp(
                "Choose which alliance scoring target TeleOp may select. Gamepad 1 D-pad: "
                        + "choose | START: freeze the highlighted choice."
        );
        allianceMenu.setSelectedId(this.defaultAlliance.name());
        allianceMenu.bind(
                bindings,
                driver.dpadUp(),
                driver.dpadDown(),
                null,
                null
        );
        refreshDraftFromMenu();
        refreshReadiness();
    }

    @Override
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        requireNotFrozen("update");
        PhoenixAlliance beforeUpdate = draftAlliance;
        bindings.update(clock);
        refreshDraftFromMenu();
        if (draftAlliance != beforeUpdate) {
            draftOrigin = DraftOrigin.OPERATOR;
        }
        refreshReadiness();
    }

    @Override
    public RobotProgram.StartDisposition freezeForStart() {
        requireNotFrozen("freeze");
        bindings.clear();
        refreshDraftFromMenu();
        refreshReadiness();
        frozenAlliance = draftAlliance;
        disposition = readiness.isAllowed()
                ? RobotProgram.StartDisposition.READY
                : RobotProgram.StartDisposition.BLOCKED;
        return disposition;
    }

    /** Seed the visible draft from one fresh Auto snapshot without freezing or commanding it. */
    void seedDraftFromAuto(PhoenixAlliance alliance) {
        requireNotFrozen("seed from Auto");
        PhoenixAlliance required = Objects.requireNonNull(alliance, "alliance");
        draftAlliance = required;
        draftOrigin = DraftOrigin.AUTO_HANDOFF;
        if (!allianceMenu.setSelectedId(required.name())) {
            throw new IllegalStateException(
                    "Phoenix TeleOp alliance menu does not contain " + required
            );
        }
        refreshReadiness();
    }

    /** Return the selected alliance's one eligible scoring-tag id after the START freeze. */
    Source<Set<Integer>> eligibleScoringTagIds() {
        return Source.of(clock -> Collections.singleton(
                scoringTagIds.get(frozenAlliance())
        ));
    }

    PhoenixAlliance frozenAlliance() {
        if (frozenAlliance == null) {
            throw new IllegalStateException(
                    "Phoenix TeleOp alliance is not frozen until the FTC START boundary"
            );
        }
        return frozenAlliance;
    }

    /** Add the selector and actionable readiness without committing telemetry. */
    void present(LoopClock clock, Telemetry telemetry) {
        Objects.requireNonNull(clock, "clock");
        if (telemetry == null) {
            return;
        }

        if (frozenAlliance == null) {
            allianceMenu.render(telemetry);
            telemetry.addLine("");
            telemetry.addData("teleop.allianceDraft", draftAlliance.label());
            telemetry.addData("teleop.allianceDraftSource", draftOrigin.display);
            telemetry.addData(
                    "teleop.allianceStartPolicy",
                    readiness.isAllowed() ? "READY TO FREEZE AT START" : "BLOCKED AT START"
            );
        } else {
            telemetry.addData("teleop.alliance", frozenAlliance.label());
            telemetry.addData("teleop.allianceStartPolicy", disposition);
        }

        for (PhoenixReadiness.Issue issue : readiness.issues()) {
            telemetry.addLine(
                    "TeleOp [" + issue.severity() + "] " + issue.message()
                            + " | " + issue.remediation()
            );
        }
    }

    private void refreshDraftFromMenu() {
        draftAlliance = Objects.requireNonNull(
                allianceMenu.selectedValueOrNull(),
                "Phoenix TeleOp alliance menu must contain RED and BLUE"
        );
    }

    private void refreshReadiness() {
        readiness = Objects.requireNonNull(
                readinessByAlliance.get(draftAlliance),
                "Phoenix TeleOp readiness was not captured for " + draftAlliance
        );
    }

    private void requireNotFrozen(String operation) {
        if (frozenAlliance != null) {
            throw new IllegalStateException(
                    "Phoenix TeleOp prestart cannot " + operation
                            + " because its alliance is already frozen"
            );
        }
    }
}
