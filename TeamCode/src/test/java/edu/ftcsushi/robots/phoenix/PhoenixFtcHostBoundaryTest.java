package edu.ftcsushi.robots.phoenix;

import org.junit.Test;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.ftcsushi.fw.architecture.ModernFtcHostBoundaryTest;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Owns the exact reviewed custom-host exceptions inside the Phoenix application bubble. */
public final class PhoenixFtcHostBoundaryTest {

    private static final String APPLICATION_PACKAGE = "edu.ftcsushi.robots.phoenix";

    @Test
    public void applicationOpModesUseManagedHostsOrExactReviewedCustomLifecycles()
            throws IOException {
        Map<String, String> exemptions = customHostExemptions();
        assertEquals("Every retained custom lifecycle must remain an exact reviewed entry",
                23, exemptions.size());

        List<String> violations = ModernFtcHostBoundaryTest.validateScopedProductionHosts(
                Paths.get(System.getProperty("user.dir")),
                Collections.singleton(APPLICATION_PACKAGE),
                exemptions,
                false);

        assertTrue("Phoenix FTC host boundary violations:\n" + joinLines(violations),
                violations.isEmpty());
    }

    private static Map<String, String> customHostExemptions() {
        Map<String, String> exemptions = new LinkedHashMap<String, String>();
        exemptions.put(
                APPLICATION_PACKAGE + ".legacy.Phoenix3",
                "Retained legacy LinearOpMode lifecycle; converting it would rewrite application "
                        + "behavior outside BRAND-02");
        exemptions.put(
                APPLICATION_PACKAGE + ".pedro.PedroTest",
                "Application-owned Pedro native diagnostic retains its vendor Follower lifecycle "
                        + "and is not ordinary Sushi robot code");

        String tunerRationale = "Pedro SelectableOpMode and native tuner callbacks own the vendor "
                + "tool protocol; managed-host conversion would change that diagnostic lifecycle";
        for (String simpleName : new String[]{
                "Tuning",
                "LocalizationTest",
                "ForwardTuner",
                "LateralTuner",
                "TurnTuner",
                "ForwardVelocityTuner",
                "LateralVelocityTuner",
                "ForwardZeroPowerAccelerationTuner",
                "LateralZeroPowerAccelerationTuner",
                "PredictiveBrakingTuner",
                "TranslationalTuner",
                "HeadingTuner",
                "DriveTuner",
                "Line",
                "CentripetalTuner",
                "Triangle",
                "Circle",
                "AnalogMinMaxTuner",
                "SwerveOffsetsTest",
                "SwerveTurnTest",
                "OffsetsTuner"
        }) {
            exemptions.put(
                    APPLICATION_PACKAGE + ".pedro." + simpleName,
                    tunerRationale);
        }
        return Collections.unmodifiableMap(exemptions);
    }

    private static String joinLines(List<String> lines) {
        StringBuilder result = new StringBuilder();
        for (String line : lines) {
            if (result.length() > 0) {
                result.append('\n');
            }
            result.append(" - ").append(line);
        }
        return result.toString();
    }
}
