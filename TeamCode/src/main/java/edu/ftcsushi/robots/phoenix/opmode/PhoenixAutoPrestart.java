package edu.ftcsushi.robots.phoenix.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.ui.MenuNavigator;
import edu.ftcsushi.fw.ftc.ui.SelectionMenu;
import edu.ftcsushi.fw.ftc.ui.SelectionMenus;
import edu.ftcsushi.fw.ftc.ui.SummaryScreen;
import edu.ftcsushi.fw.ftc.ui.UiControls;
import edu.ftcsushi.fw.ftc.input.Gamepads;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixCalibrationConfig;
import edu.ftcsushi.robots.phoenix.PhoenixReadiness;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoStrategyId;
import edu.ftcsushi.robots.phoenix.autonomous.pedro.PhoenixPedroPathFactory;
import edu.ftcsushi.robots.phoenix.scoring.PhoenixTargeting;

/** Owns Phoenix's one data-only INIT selector/readiness policy. */
final class PhoenixAutoPrestart implements RobotProgram.Prestart {

    private final PhoenixAutoSetup setup;
    private final PhoenixAutoSpec.Builder builder;
    private final Bindings bindings;
    private final MenuNavigator navigator;
    private final EnumMap<PhoenixAlliance, Integer> scoringTagIds =
            new EnumMap<PhoenixAlliance, Integer>(PhoenixAlliance.class);
    private final Map<String, PhoenixReadiness.Result> readinessBySelection =
            new HashMap<String, PhoenixReadiness.Result>();

    private PhoenixAutoSpec displayedSpec;
    private PhoenixAutoSpec frozenSpec;
    private PhoenixPedroPathFactory.RouteAvailability routeAvailability;
    private PhoenixReadiness.Result readiness;
    private RobotProgram.StartDisposition disposition;

    PhoenixAutoPrestart(PhoenixAutoSetup setup,
                        PhoenixCalibrationConfig calibration,
                        PhoenixTargeting.Config targeting,
                        TagLayout fixedAprilTagLayout,
                        Gamepad gamepad1,
                        Gamepad gamepad2) {
        this.setup = Objects.requireNonNull(setup, "setup");
        captureReadiness(calibration, targeting, fixedAprilTagLayout);
        PhoenixAutoSpec initial = setup.initialSpec();
        builder = PhoenixAutoSpec.builder()
                .alliance(initial.alliance)
                .startPosition(initial.startPosition)
                .strategy(initial.strategy);

        if (setup.selectionMode() == PhoenixAutoSetup.SelectionMode.INIT_SELECTION) {
            Gamepads gamepads = Gamepads.create(gamepad1, gamepad2);
            bindings = new Bindings();
            UiControls controls = UiControls.gamepad1(gamepads)
                    .withHint("Dpad: select | A: choose | B/BACK: back | Y: home");
            navigator = new MenuNavigator().setControlsHint(controls.hint());
            navigator.bind(bindings, controls);
            navigator.setRoot(allianceScreen());
        } else {
            bindings = null;
            navigator = null;
        }
        refreshPolicy();
    }

    @Override
    public void update(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        if (frozenSpec != null) {
            throw new IllegalStateException("Phoenix Auto prestart policy is already frozen");
        }
        if (bindings != null) {
            bindings.update(clock);
        }
        refreshPolicy();
    }

    @Override
    public RobotProgram.StartDisposition freezeForStart() {
        if (frozenSpec != null) {
            throw new IllegalStateException("Phoenix Auto setup may be frozen only once");
        }
        if (bindings != null) {
            bindings.clear();
        }
        frozenSpec = selectedSpec();
        displayedSpec = frozenSpec;
        routeAvailability = PhoenixPedroPathFactory.routeAvailabilityFor(frozenSpec);
        readiness = readinessFor(frozenSpec);
        disposition = readiness.isAllowed()
                ? RobotProgram.StartDisposition.READY
                : RobotProgram.StartDisposition.BLOCKED;
        return disposition;
    }

    /** Add selection, readiness, and physical-placement facts without committing telemetry. */
    void present(LoopClock clock, Telemetry telemetry) {
        Objects.requireNonNull(clock, "clock");
        if (telemetry == null) {
            return;
        }
        if (navigator != null && frozenSpec == null) {
            navigator.render(telemetry);
            telemetry.addLine("");
        }

        PhoenixAutoSpec spec = displayedSpec;
        telemetry.addData("auto.setup", spec.summary());
        telemetry.addData("auto.purpose", setup.purpose());
        telemetry.addData("auto.routeMaturity", routeAvailability.maturity);
        telemetry.addData("auto.expectedPhysicalStartPedro", formatPose(
                routeAvailability.expectedPedroStartPose
        ));
        if (frozenSpec == null) {
            telemetry.addData("auto.startPolicy", readiness.isAllowed()
                    ? "READY TO FREEZE AT START"
                    : "BLOCKED AT START");
        } else {
            telemetry.addData("auto.startPolicy", disposition);
        }
        for (PhoenixReadiness.Issue issue : readiness.issues()) {
            telemetry.addLine(
                    "Auto [" + issue.severity() + "] " + issue.message()
                            + " | " + issue.remediation()
            );
        }
    }

    PhoenixAutoSpec fixedSpecForEagerBuild() {
        if (setup.selectionMode() != PhoenixAutoSetup.SelectionMode.FIXED) {
            throw new IllegalStateException("Only a fixed Phoenix Auto setup has an eager spec");
        }
        return setup.initialSpec();
    }

    PhoenixAutoSpec frozenSpec() {
        if (frozenSpec == null) {
            throw new IllegalStateException(
                    "Phoenix Auto setup is not frozen until the FTC START boundary"
            );
        }
        return frozenSpec;
    }

    Pose frozenStartingPose() {
        if (frozenSpec == null) {
            throw new IllegalStateException(
                    "Phoenix Auto starting pose is unavailable before FTC START"
            );
        }
        Pose pose = routeAvailability.expectedPedroStartPose;
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    Source<Set<Integer>> eligibleScoringTagIds() {
        return Source.of(clock -> Collections.singleton(selectedAllianceTagId(frozenSpec())));
    }

    private void refreshPolicy() {
        displayedSpec = selectedSpec();
        routeAvailability = PhoenixPedroPathFactory.routeAvailabilityFor(displayedSpec);
        readiness = readinessFor(displayedSpec);
    }

    private PhoenixAutoSpec selectedSpec() {
        if (setup.selectionMode() == PhoenixAutoSetup.SelectionMode.FIXED) {
            return setup.initialSpec();
        }
        return builder.build();
    }

    private SelectionMenu<PhoenixAlliance> allianceScreen() {
        SelectionMenu<PhoenixAlliance> menu = SelectionMenus.forEnum(
                "Alliance",
                PhoenixAlliance.class,
                enumDisplay(
                        value -> value.label(),
                        value -> value.help(),
                        value -> value == builder.alliance()
                )
        );
        menu.setHelp(
                "Step 1: choose the alliance for scoring-AprilTag eligibility and route geometry."
        );
        menu.setOnSelect(item -> {
            builder.alliance(item.value);
            navigator.push(startScreen());
        });
        menu.setSelectedId(builder.alliance().name());
        return menu;
    }

    private SelectionMenu<PhoenixAutoSpec.StartPosition> startScreen() {
        SelectionMenu<PhoenixAutoSpec.StartPosition> menu = SelectionMenus.forEnum(
                "Start Position",
                PhoenixAutoSpec.StartPosition.class,
                enumDisplay(
                        value -> value.label(),
                        value -> value.help(),
                        value -> value == builder.startPosition()
                )
        );
        menu.setHelp("Step 2: choose Phoenix's physical starting location.");
        menu.setOnSelect(item -> {
            builder.startPosition(item.value);
            navigator.push(strategyScreen());
        });
        menu.setSelectedId(builder.startPosition().name());
        return menu;
    }

    private SelectionMenu<PhoenixAutoStrategyId> strategyScreen() {
        SelectionMenu<PhoenixAutoStrategyId> menu = SelectionMenus.forEnum(
                "Strategy",
                PhoenixAutoStrategyId.class,
                new SelectionMenus.EnumDisplay<PhoenixAutoStrategyId>() {
                    @Override
                    public String id(PhoenixAutoStrategyId value) {
                        return value.name();
                    }

                    @Override
                    public String label(PhoenixAutoStrategyId value) {
                        return value.label();
                    }

                    @Override
                    public String help(PhoenixAutoStrategyId value) {
                        return value.help();
                    }

                    @Override
                    public String tag(PhoenixAutoStrategyId value) {
                        if (!strategySelectable(value)) {
                            return "BLOCKED";
                        }
                        return value == builder.strategy() ? "CURRENT" : value.tag();
                    }

                    @Override
                    public boolean enabled(PhoenixAutoStrategyId value) {
                        return strategySelectable(value);
                    }

                    @Override
                    public String disabledReason(PhoenixAutoStrategyId value) {
                        PhoenixAutoSpec spec = specWithStrategy(value);
                        PhoenixReadiness.Result result = PhoenixReadiness.pedroAutoStrategy(
                                spec,
                                setup.purpose()
                        );
                        PhoenixReadiness.Issue blocker = result.firstBlockingIssueOrNull();
                        return blocker == null ? null : blocker.message();
                    }
                }
        );
        menu.setHelp("Step 3: choose the routine; unavailable routes cannot be selected.");
        menu.setOnSelect(item -> {
            builder.strategy(item.value);
            navigator.push(selectionSummary());
        });
        menu.setSelectedId(builder.strategy().name());
        return menu;
    }

    private SummaryScreen selectionSummary() {
        PhoenixAutoSpec preview = builder.build();
        PhoenixReadiness.Result result = readinessFor(preview);
        PhoenixReadiness.Issue blocker = result.firstBlockingIssueOrNull();
        PhoenixPedroPathFactory.RouteAvailability availability =
                PhoenixPedroPathFactory.routeAvailabilityFor(preview);
        return SummaryScreen.builder("Phoenix Auto Selection")
                .help("Review the data-only selection. Hardware was already constructed once.")
                .status(blocker == null ? "READY" : "BLOCKED",
                        blocker == null ? "FTC START will freeze this setup." : blocker.message())
                .row("Alliance", preview.alliance.label())
                .row("Start", preview.startPosition.label())
                .row("Strategy", preview.strategy.label())
                .row("Route maturity", availability.maturity)
                .row("Expected physical start", formatPose(availability.expectedPedroStartPose))
                .warning(blocker == null
                        ? "Verify the physical placement before pressing START."
                        : blocker.remediation())
                .controls("START: freeze selection | X: edit selection")
                .onSecondary(() -> {
                    navigator.setRoot(allianceScreen());
                })
                .consumeBack(true)
                .consumeHome(true)
                .build();
    }

    private boolean strategySelectable(PhoenixAutoStrategyId strategy) {
        return PhoenixReadiness.pedroAutoStrategy(
                specWithStrategy(strategy),
                setup.purpose()
        ).isAllowed();
    }

    private PhoenixAutoSpec specWithStrategy(PhoenixAutoStrategyId strategy) {
        return PhoenixAutoSpec.builder()
                .alliance(builder.alliance())
                .startPosition(builder.startPosition())
                .strategy(strategy)
                .build();
    }

    private int selectedAllianceTagId(PhoenixAutoSpec spec) {
        return scoringTagIds.get(spec.alliance);
    }

    private void captureReadiness(
            PhoenixCalibrationConfig calibration,
            PhoenixTargeting.Config targeting,
            TagLayout fixedAprilTagLayout
    ) {
        for (PhoenixAlliance alliance : PhoenixAlliance.values()) {
            scoringTagIds.put(
                    alliance,
                    targeting == null ? -1 : targeting.scoringTagIdFor(alliance)
            );
            for (PhoenixAutoSpec.StartPosition start : PhoenixAutoSpec.StartPosition.values()) {
                for (PhoenixAutoStrategyId strategy : PhoenixAutoStrategyId.values()) {
                    PhoenixAutoSpec spec = PhoenixAutoSpec.builder()
                            .alliance(alliance)
                            .startPosition(start)
                            .strategy(strategy)
                            .build();
                    readinessBySelection.put(
                            readinessKey(spec),
                            PhoenixReadiness.pedroAuto(
                                    spec,
                                    calibration,
                                    targeting,
                                    fixedAprilTagLayout,
                                    setup.purpose()
                            )
                    );
                }
            }
        }
    }

    private PhoenixReadiness.Result readinessFor(PhoenixAutoSpec spec) {
        return Objects.requireNonNull(
                readinessBySelection.get(readinessKey(spec)),
                "Phoenix Auto readiness was not captured for " + spec.summary()
        );
    }

    private static String readinessKey(PhoenixAutoSpec spec) {
        return spec.alliance.name() + '|' + spec.startPosition.name() + '|'
                + spec.strategy.name();
    }

    private static <E extends Enum<E>> SelectionMenus.EnumDisplay<E> enumDisplay(
            java.util.function.Function<E, String> label,
            java.util.function.Function<E, String> help,
            java.util.function.Predicate<E> current
    ) {
        return new SelectionMenus.EnumDisplay<E>() {
            @Override
            public String id(E value) {
                return value.name();
            }

            @Override
            public String label(E value) {
                return label.apply(value);
            }

            @Override
            public String help(E value) {
                return help.apply(value);
            }

            @Override
            public String tag(E value) {
                return current.test(value) ? "CURRENT" : null;
            }

            @Override
            public boolean enabled(E value) {
                return true;
            }

            @Override
            public String disabledReason(E value) {
                return null;
            }
        };
    }

    private static String formatPose(Pose pose) {
        return String.format(
                "x=%.1f in, y=%.1f in, heading=%.1f deg",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading())
        );
    }
}
