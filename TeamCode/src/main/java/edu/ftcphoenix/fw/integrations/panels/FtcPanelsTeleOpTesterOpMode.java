package edu.ftcphoenix.fw.integrations.panels;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.ftc.FtcTeleOpTesterOpMode;

/**
 * Panels boundary for a Phoenix tester OpMode with mirrored telemetry and one fixed input owner.
 *
 * <p>Both input choices send the tester's row-oriented telemetry to the FTC Driver Station and
 * Panels. {@link InputSource#DRIVER_STATION} accepts only the physical FTC gamepads;
 * {@link InputSource#PANELS} accepts only the two Panels virtual gamepads. Inputs are never merged,
 * and the source selected by the concrete OpMode cannot change while that OpMode is running.</p>
 *
 * <p>The Panels input path reads current vendor gamepad state into two stable FTC {@link Gamepad}
 * objects immediately before tester {@code init}, {@code initLoop}, {@code start}, and {@code loop}
 * callbacks. Losing the last Panels client or failing to read a sample throws. An initial failure
 * prevents tester initialization; a later failure makes {@link FtcTeleOpTesterOpMode} terminally
 * fail-stop its retained tester. Reconnect by stopping and starting a new OpMode instance;
 * reconnecting never silently rearms a terminal instance.</p>
 *
 * <p>This adapter intentionally keeps every {@code com.bylazar} type inside the Panels integration
 * boundary. The joined sink supports the ordinary row operations used by Phoenix testers; it is
 * not a promise that every retained-item, log, action, or removal semantic of an arbitrary FTC
 * {@link Telemetry} implementation is available through Panels.</p>
 */
public abstract class FtcPanelsTeleOpTesterOpMode extends FtcTeleOpTesterOpMode {

    /** The sole owner of tester input for one OpMode instance. */
    public enum InputSource {
        /** Physical Driver Station gamepads; Panels virtual controls are ignored. */
        DRIVER_STATION,

        /** Panels virtual gamepads; physical Driver Station controls are ignored. */
        PANELS
    }

    private final InputSource inputSource;
    private final PanelsBackend panels;

    /**
     * Creates a Panels-capable tester host with one fixed input source.
     *
     * @param inputSource sole input owner for the lifetime of this OpMode
     */
    protected FtcPanelsTeleOpTesterOpMode(InputSource inputSource) {
        this(inputSource, DefaultPanelsBackend.INSTANCE);
    }

    FtcPanelsTeleOpTesterOpMode(InputSource inputSource, PanelsBackend panels) {
        if (inputSource == null) {
            throw new NullPointerException("inputSource");
        }
        if (panels == null) {
            throw new NullPointerException("panels");
        }
        this.inputSource = inputSource;
        this.panels = panels;
    }

    /**
     * Builds the fixed telemetry/input console once during FTC INIT.
     */
    @Override
    protected final TesterConsole createTesterConsole() {
        Telemetry panelsTelemetry = panels.telemetry();
        if (panelsTelemetry == null) {
            throw new IllegalStateException("Panels telemetry adapter returned null");
        }

        Telemetry mirroredTelemetry = new JoinedTelemetry(telemetry, panelsTelemetry);
        if (inputSource == InputSource.DRIVER_STATION) {
            return new FixedInputConsole(mirroredTelemetry, gamepad1, gamepad2);
        }
        return new PanelsInputConsole(mirroredTelemetry, panels);
    }

    interface PanelsBackend {
        Telemetry telemetry();

        int connectedClientCount();

        Gamepad firstGamepadSnapshot();

        Gamepad secondGamepadSnapshot();
    }

    private static final class DefaultPanelsBackend implements PanelsBackend {
        private static final DefaultPanelsBackend INSTANCE = new DefaultPanelsBackend();

        @Override
        public Telemetry telemetry() {
            return PanelsTelemetry.INSTANCE.getFtcTelemetry();
        }

        @Override
        public int connectedClientCount() {
            return Panels.INSTANCE.getClientsCount();
        }

        @Override
        public Gamepad firstGamepadSnapshot() {
            return snapshot(PanelsGamepad.INSTANCE.getFirstManager());
        }

        @Override
        public Gamepad secondGamepadSnapshot() {
            return snapshot(PanelsGamepad.INSTANCE.getSecondManager());
        }

        private static Gamepad snapshot(GamepadManager manager) {
            Gamepad gamepad = new Gamepad();
            gamepad.left_bumper = manager.getL1();
            gamepad.left_trigger = (float) manager.getL2();
            gamepad.right_bumper = manager.getR1();
            gamepad.right_trigger = (float) manager.getR2();

            gamepad.a = manager.getCross();
            gamepad.b = manager.getCircle();
            gamepad.x = manager.getSquare();
            gamepad.y = manager.getTriangle();
            gamepad.dpad_up = manager.getDpadUp();
            gamepad.dpad_down = manager.getDpadDown();
            gamepad.dpad_left = manager.getDpadLeft();
            gamepad.dpad_right = manager.getDpadRight();

            gamepad.left_stick_x = (float) manager.getLeftStickX();
            gamepad.left_stick_y = (float) manager.getLeftStickY();
            gamepad.left_stick_button = manager.getLeftStickPressed();
            gamepad.right_stick_x = (float) manager.getRightStickX();
            gamepad.right_stick_y = (float) manager.getRightStickY();
            gamepad.right_stick_button = manager.getRightStickPressed();

            gamepad.guide = manager.getPs();
            gamepad.start = manager.getOptions();
            gamepad.back = manager.getShare();
            gamepad.touchpad = manager.getTouchpad();
            return gamepad;
        }
    }

    private static class FixedInputConsole implements TesterConsole {
        private final Telemetry telemetry;
        private final Gamepad gamepad1;
        private final Gamepad gamepad2;

        private FixedInputConsole(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
            this.telemetry = telemetry;
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
        }

        @Override
        public final Telemetry telemetry() {
            return telemetry;
        }

        @Override
        public final Gamepad gamepad1() {
            return gamepad1;
        }

        @Override
        public final Gamepad gamepad2() {
            return gamepad2;
        }

        @Override
        public void sampleInputs() {
            // The FTC SDK mutates physical gamepad objects directly.
        }
    }

    private static final class PanelsInputConsole extends FixedInputConsole {
        private final PanelsBackend panels;

        private PanelsInputConsole(Telemetry telemetry, PanelsBackend panels) {
            super(telemetry, new Gamepad(), new Gamepad());
            this.panels = panels;
        }

        @Override
        public void sampleInputs() {
            try {
                requireConnectedClient();
                Gamepad first = panels.firstGamepadSnapshot();
                Gamepad second = panels.secondGamepadSnapshot();
                if (first == null || second == null) {
                    throw new IllegalStateException("Panels returned a null virtual-gamepad snapshot");
                }
                normalizeAliases(first);
                normalizeAliases(second);
                // Reject a disconnect that raced either read rather than accepting stale commands.
                requireConnectedClient();

                gamepad1().copy(first);
                gamepad2().copy(second);
                normalizeAliases(gamepad1());
                normalizeAliases(gamepad2());
            } catch (RuntimeException failure) {
                if (failure instanceof PanelsInputUnavailableException) {
                    throw failure;
                }
                throw new PanelsInputUnavailableException(
                        "Panels tester input failed; stop this OpMode, reconnect Panels, and restart "
                                + "FW: Testers (Panels)",
                        failure);
            }
        }

        private void requireConnectedClient() {
            if (panels.connectedClientCount() <= 0) {
                throw new PanelsInputUnavailableException(
                        "Panels tester input disconnected; reconnect Panels, stop this OpMode, and "
                                + "restart FW: Testers (Panels)");
            }
        }

        private static void normalizeAliases(Gamepad gamepad) {
            boolean a = gamepad.a || gamepad.cross;
            boolean b = gamepad.b || gamepad.circle;
            boolean x = gamepad.x || gamepad.square;
            boolean y = gamepad.y || gamepad.triangle;
            boolean start = gamepad.start || gamepad.options;
            boolean back = gamepad.back || gamepad.share;
            boolean guide = gamepad.guide || gamepad.ps;

            gamepad.a = a;
            gamepad.cross = a;
            gamepad.b = b;
            gamepad.circle = b;
            gamepad.x = x;
            gamepad.square = x;
            gamepad.y = y;
            gamepad.triangle = y;
            gamepad.start = start;
            gamepad.options = start;
            gamepad.back = back;
            gamepad.share = back;
            gamepad.guide = guide;
            gamepad.ps = guide;
        }
    }

    private static final class PanelsInputUnavailableException extends IllegalStateException {
        private PanelsInputUnavailableException(String message) {
            super(message);
        }

        private PanelsInputUnavailableException(String message, RuntimeException cause) {
            super(message, cause);
        }
    }
}
