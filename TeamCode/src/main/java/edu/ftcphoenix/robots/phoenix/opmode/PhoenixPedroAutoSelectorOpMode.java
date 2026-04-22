package edu.ftcphoenix.robots.phoenix.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.ui.ConfirmationScreen;
import edu.ftcphoenix.fw.ftc.ui.MenuItem;
import edu.ftcphoenix.fw.ftc.ui.MenuNavigator;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenu;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenus;
import edu.ftcphoenix.fw.ftc.ui.SummaryScreen;
import edu.ftcphoenix.fw.ftc.ui.UiControls;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;

/**
 * Pre-start Phoenix autonomous selector.
 *
 * <p>The selector uses framework telemetry UI helpers to gather a {@link PhoenixAutoSpec} during
 * INIT. Once confirmed, the same Pedro/Phoenix initialization path used by static Auto entries is
 * executed and the selector is replaced by a locked summary screen. The selected values remain
 * robot-owned; the framework UI only handles display, navigation, and button dispatch.</p>
 */
@Autonomous(name = "Phoenix Auto Selector", group = "Phoenix")
public final class PhoenixPedroAutoSelectorOpMode extends PhoenixPedroAutoOpModeBase {

    private final LoopClock uiClock = new LoopClock();
    private final PhoenixAutoSpec.Builder builder = PhoenixAutoSpec.builder();

    private Gamepads gamepads;
    private Bindings bindings;
    private MenuNavigator navigator;
    private PhoenixAutoSpec selectedSpec;

    /**
     * Selector OpMode builds only after confirmation or START.
     */
    @Override
    protected boolean buildRobotInInit() {
        return false;
    }

    /**
     * Current selected spec, using defaults for any step not explicitly changed.
     */
    @Override
    protected PhoenixAutoSpec autoSpec() {
        return selectedSpec != null ? selectedSpec : builder.build();
    }

    /**
     * Create selector UI and bind standard menu controls.
     */
    @Override
    public void init() {
        uiClock.reset(getRuntime());
        gamepads = Gamepads.create(gamepad1, gamepad2);
        bindings = new Bindings();

        UiControls controls = UiControls.gamepad1(gamepads)
                .withHint("Dpad: select | A: choose/confirm | B/BACK: back | Y: home");

        navigator = new MenuNavigator()
                .setControlsHint(controls.hint());
        navigator.bind(bindings, controls);
        navigator.setRoot(allianceScreen());

        renderInitUi();
    }

    /**
     * Update selector bindings and render the current setup screen while waiting for START.
     */
    @Override
    public void init_loop() {
        uiClock.update(getRuntime());
        if (bindings != null) {
            bindings.update(uiClock);
        }
        renderInitUi();
    }

    /**
     * If the operator pressed START without confirming, build the current default/partial spec.
     */
    @Override
    public void start() {
        if (!isAutoInitialized() && initErrorOrNull() == null) {
            selectedSpec = autoSpec();
            initializeRobotForSpec(selectedSpec);
        }
        super.start();
    }

    private void renderInitUi() {
        if (navigator != null) {
            navigator.render(telemetry);
        }

        telemetry.addLine("");
        telemetry.addData("Current", builder.build().summary());
        if (selectedSpec != null) {
            telemetry.addData("Confirmed", selectedSpec.summary());
        }
        if (isAutoInitialized()) {
            telemetry.addLine("Status: [OK] Phoenix Auto initialized. Press START when ready.");
        } else if (initErrorOrNull() != null) {
            telemetry.addLine("Status: [ERROR] " + initErrorOrNull());
        } else {
            telemetry.addLine("Status: Choose values, then confirm before START. START will use the current values.");
        }
        telemetry.update();
    }

    private SelectionMenu<PhoenixAutoSpec.Alliance> allianceScreen() {
        SelectionMenu<PhoenixAutoSpec.Alliance> menu = SelectionMenus.forEnum(
                "Alliance",
                PhoenixAutoSpec.Alliance.class,
                new SelectionMenus.EnumDisplay<PhoenixAutoSpec.Alliance>() {
                    @Override
                    public String id(PhoenixAutoSpec.Alliance value) {
                        return value.name();
                    }

                    @Override
                    public String label(PhoenixAutoSpec.Alliance value) {
                        return value.label();
                    }

                    @Override
                    public String help(PhoenixAutoSpec.Alliance value) {
                        return value.help();
                    }

                    @Override
                    public String tag(PhoenixAutoSpec.Alliance value) {
                        return value == builder.alliance() ? "CURRENT" : null;
                    }

                    @Override
                    public boolean enabled(PhoenixAutoSpec.Alliance value) {
                        return true;
                    }

                    @Override
                    public String disabledReason(PhoenixAutoSpec.Alliance value) {
                        return null;
                    }
                });
        menu.setHelp("Step 1: choose the alliance color for target filtering and path mirroring.");
        menu.setOnSelect(new Consumer<MenuItem<PhoenixAutoSpec.Alliance>>() {
            @Override
            public void accept(MenuItem<PhoenixAutoSpec.Alliance> item) {
                builder.alliance(item.value);
                selectedSpec = null;
                navigator.push(startScreen());
            }
        });
        menu.setSelectedId(builder.alliance().name());
        return menu;
    }

    private SelectionMenu<PhoenixAutoSpec.StartPosition> startScreen() {
        SelectionMenu<PhoenixAutoSpec.StartPosition> menu = SelectionMenus.forEnum(
                "Start Position",
                PhoenixAutoSpec.StartPosition.class,
                new SelectionMenus.EnumDisplay<PhoenixAutoSpec.StartPosition>() {
                    @Override
                    public String id(PhoenixAutoSpec.StartPosition value) {
                        return value.name();
                    }

                    @Override
                    public String label(PhoenixAutoSpec.StartPosition value) {
                        return value.label();
                    }

                    @Override
                    public String help(PhoenixAutoSpec.StartPosition value) {
                        return value.help();
                    }

                    @Override
                    public String tag(PhoenixAutoSpec.StartPosition value) {
                        return value == builder.startPosition() ? "CURRENT" : null;
                    }

                    @Override
                    public boolean enabled(PhoenixAutoSpec.StartPosition value) {
                        return true;
                    }

                    @Override
                    public String disabledReason(PhoenixAutoSpec.StartPosition value) {
                        return null;
                    }
                });
        menu.setHelp("Step 2: choose where Phoenix starts within the selected alliance side.");
        menu.setOnSelect(new Consumer<MenuItem<PhoenixAutoSpec.StartPosition>>() {
            @Override
            public void accept(MenuItem<PhoenixAutoSpec.StartPosition> item) {
                builder.startPosition(item.value);
                selectedSpec = null;
                navigator.push(partnerScreen());
            }
        });
        menu.setSelectedId(builder.startPosition().name());
        return menu;
    }

    private SelectionMenu<PhoenixAutoSpec.PartnerPlan> partnerScreen() {
        SelectionMenu<PhoenixAutoSpec.PartnerPlan> menu = SelectionMenus.forEnum(
                "Partner Plan",
                PhoenixAutoSpec.PartnerPlan.class,
                new SelectionMenus.EnumDisplay<PhoenixAutoSpec.PartnerPlan>() {
                    @Override
                    public String id(PhoenixAutoSpec.PartnerPlan value) {
                        return value.name();
                    }

                    @Override
                    public String label(PhoenixAutoSpec.PartnerPlan value) {
                        return value.label();
                    }

                    @Override
                    public String help(PhoenixAutoSpec.PartnerPlan value) {
                        return value.help();
                    }

                    @Override
                    public String tag(PhoenixAutoSpec.PartnerPlan value) {
                        return value == builder.partnerPlan() ? "CURRENT" : null;
                    }

                    @Override
                    public boolean enabled(PhoenixAutoSpec.PartnerPlan value) {
                        return true;
                    }

                    @Override
                    public String disabledReason(PhoenixAutoSpec.PartnerPlan value) {
                        return null;
                    }
                });
        menu.setHelp("Step 3: choose the partner coordination assumption for strategy/path selection.");
        menu.setOnSelect(new Consumer<MenuItem<PhoenixAutoSpec.PartnerPlan>>() {
            @Override
            public void accept(MenuItem<PhoenixAutoSpec.PartnerPlan> item) {
                builder.partnerPlan(item.value);
                selectedSpec = null;
                navigator.push(strategyScreen());
            }
        });
        menu.setSelectedId(builder.partnerPlan().name());
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
                        return value == builder.strategy() ? "CURRENT" : value.tag();
                    }

                    @Override
                    public boolean enabled(PhoenixAutoStrategyId value) {
                        return true;
                    }

                    @Override
                    public String disabledReason(PhoenixAutoStrategyId value) {
                        return null;
                    }
                });
        menu.setHelp("Step 4: choose the autonomous strategy to build from the selected setup.");
        menu.setOnSelect(new Consumer<MenuItem<PhoenixAutoStrategyId>>() {
            @Override
            public void accept(MenuItem<PhoenixAutoStrategyId> item) {
                builder.strategy(item.value);
                selectedSpec = null;
                navigator.push(confirmScreen());
            }
        });
        menu.setSelectedId(builder.strategy().name());
        return menu;
    }

    private SummaryScreen lockedSummaryScreen(PhoenixAutoSpec spec) {
        return SummaryScreen.builder("Phoenix Auto Locked")
                .status("LOCKED", "Phoenix + Pedro are initialized for this exact spec.")
                .help("Choices are locked after initialization so the visible setup cannot drift away from the queued routine.")
                .row("Alliance", spec.alliance.label())
                .row("Start", spec.startPosition.label())
                .row("Partner", spec.partnerPlan.label())
                .row("Strategy", spec.strategy.label())
                .row("Summary", spec.summary())
                .controls("START: run selected Auto | B/BACK/Y: locked after initialization")
                .consumeBack(true)
                .consumeHome(true)
                .build();
    }

    private ConfirmationScreen confirmScreen() {
        PhoenixAutoSpec preview = builder.build();
        return ConfirmationScreen.builder("Confirm Phoenix Auto")
                .help("Step 5: review the setup. Press A to build Phoenix + Pedro for this spec.")
                .status("READY", "Not initialized until confirmed or START is pressed.")
                .row("Alliance", preview.alliance.label())
                .row("Start", preview.startPosition.label())
                .row("Partner", preview.partnerPlan.label())
                .row("Strategy", preview.strategy.label())
                .warning("Verify these values before START; Driver Station static entries are safer for common match routines.")
                .onConfirm(new Runnable() {
                    @Override
                    public void run() {
                        selectedSpec = builder.build();
                        if (initializeRobotForSpec(selectedSpec)) {
                            navigator.setRoot(lockedSummaryScreen(selectedSpec));
                        }
                    }
                })
                .onReset(new Runnable() {
                    @Override
                    public void run() {
                        selectedSpec = null;
                    }
                })
                .build();
    }
}
