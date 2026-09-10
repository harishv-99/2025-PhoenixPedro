package edu.ftcsushi.fw.docs;

import org.junit.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Focused authoring safeguards for the optional table lesson, not proof of comprehension. */
public final class CalibrationTableDocumentationTest {
    private static final String AREA = "TeamCode/src/main/java/edu/ftcsushi/fw/docs/testing-calibration/";
    private static final String LESSON = "Interpolate Calibration Measurements.md";

    @Test
    public void firstMeaningPrecedesEachTableConstruction() throws IOException {
        String page = read(AREA + LESSON);
        String beforeOneInput = before(page, "private static final InterpolatingTable1D");
        assertTrue(beforeOneInput.contains("**Interpolation** estimates between stored samples"));
        assertTrue(beforeOneInput.contains("**array** is an ordered list"));
        assertTrue(beforeOneInput.contains("**finite**"));
        assertTrue(beforeOneInput.contains("**immutable** table"));
        assertTrue(beforeOneInput.contains("private"));
        assertTrue(beforeOneInput.contains("static final"));

        String beforeTwoInputs = before(page, "private static final InterpolatingTable2D");
        assertTrue(beforeTwoInputs.contains("**grid** is a rectangular table"));
        assertTrue(beforeTwoInputs.contains("**axis** here is the ordered list"));
        assertTrue(beforeTwoInputs.contains("Rows are always first input, columns second"));
        assertTrue(beforeTwoInputs.contains("**indices**"));
        assertTrue(beforeTwoInputs.contains("alternative to the distance table"));
        assertTrue(beforeTwoInputs.contains("Forward inches (rows) / left inches (columns)"));
    }

    @Test
    public void checkpointKeepsRealLookupAndNamesNumericalAndPhysicalLimits() throws IOException {
        String page = read(AREA + LESSON);
        assertTrue(page.contains("robots/examples/calibration/ShotSpeedCalibration.java"));
        assertTrue(page.contains("robots/examples/calibration/ShotSpeedCalibrationTest.java"));
        assertTrue(page.contains("assertEquals(3200.0, distanceSpeed, 1e-9)"));
        assertTrue(page.contains("assertEquals(3250.0, offsetSpeed, 1e-9)"));
        assertTrue(page.contains("3050"));
        assertTrue(page.contains("3450"));
        for (String label : new String[] {"**Question:**", "**Keep real:**", "**Replace:**",
                "**Observe:**", "**Cannot conclude:**", "**Read the causal chain:**",
                "**Proves:**", "**Does not prove:**", "**Next gate:**"}) {
            assertTrue(label, page.contains(label));
        }
        assertTrue(page.contains("not measured or recommended shooter settings"));
        assertTrue(page.contains("numerical clamp is not range acceptance or a hardware limit"));
        assertTrue(page.contains("Non-finite inputs still return `NaN`"));
        assertTrue(page.contains("A stale but"));
        assertTrue(page.contains("no Driver Station program to run"));
        assertTrue(page.contains("Reading the predictions completes this lesson"));
        assertTrue(page.contains("your adapted code works"));
    }

    @Test
    public void discoveryStaysOptionalAndTuningUsesOneSortedGrammar() throws IOException {
        assertTrue(read(AREA + "README.md").contains(LESSON));
        String tuning = read(AREA + "Control Tuning Workflow.md");
        assertTrue(tuning.contains(LESSON));
        assertTrue(tuning.contains("InterpolatingTable1D.ofSorted("));
        assertFalse(tuning.contains("ofSortedPairs"));
        assertFalse(tuning.contains("ofUnsorted"));

        String navigation = read("zensical.toml");
        String target = "docs/testing-calibration/" + LESSON;
        int index = navigation.indexOf(target);
        assertTrue(index > navigation.indexOf("{ \"Test & Tune\" = ["));
        assertTrue(index < navigation.indexOf("{ \"Advanced\" = ["));
        assertTrue("One canonical navigation home", navigation.indexOf(target, index + 1) == -1);
    }

    private static String before(String text, String boundary) {
        int index = text.indexOf(boundary);
        assertTrue("Missing example boundary " + boundary, index >= 0);
        return text.substring(0, index);
    }

    private static String read(String relative) throws IOException {
        Path root = Paths.get("").toAbsolutePath();
        while (root != null && !Files.exists(root.resolve("settings.gradle"))) root = root.getParent();
        if (root == null) throw new IOException("Run from the repository or its TeamCode directory");
        return new String(Files.readAllBytes(root.resolve(relative)), StandardCharsets.UTF_8);
    }
}
