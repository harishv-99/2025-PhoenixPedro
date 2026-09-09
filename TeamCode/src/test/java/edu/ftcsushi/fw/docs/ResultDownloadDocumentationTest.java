package edu.ftcsushi.fw.docs;

import org.junit.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Maintainer checks for result discovery and bounded follow-up intake, not reading comprehension. */
public final class ResultDownloadDocumentationTest {
    private static final String DOCS = "TeamCode/src/main/java/edu/ftcsushi/fw/docs/";

    @Test
    public void optionalRunbookExplainsEvidenceBeforeReplayAndKeepsPhysicalStop() throws IOException {
        String lesson = read(DOCS + "testing-calibration/Download and Inspect Experiment Results.md");
        assertTrue(lesson.indexOf("A **report**") < lesson.indexOf("A **recording**"));
        assertTrue(lesson.contains("never delay STOP"));
        assertTrue(lesson.contains("not connect another Panels control client"));
        assertTrue(lesson.contains("acquisition"));
        assertTrue(lesson.contains("UNRECORDED"));
        assertTrue(lesson.contains("INCOMPLETE"));
        assertTrue(lesson.contains("COMPLETE_MATCH"));
        assertTrue(lesson.contains("accTitle:"));
        assertTrue(lesson.contains("accDescr:"));
        String build = read("TeamCode/build.gradle");
        assertTrue(build.contains("tasks.register('replayControlExperiment', JavaExec)"));
        assertTrue(lesson.contains(":TeamCode:replayControlExperiment"));
    }

    @Test
    public void owningRunbooksLinkOneDownloadHomeAndBringUpNoLongerTeachesLogcat() throws IOException {
        String[] pages = {"testing-calibration/Using the Tester Console.md",
                "testing-calibration/Actuator Bring-up.md",
                "testing-calibration/Robot Calibration Tutorials.md",
                "testing-calibration/Control Tuning Workflow.md", "examples/Subsystem Experiments.md"};
        for (String page : pages) {
            assertTrue(page, read(DOCS + page).contains("Download and Inspect Experiment Results.md"));
        }
        assertFalse(read(DOCS + "testing-calibration/Actuator Bring-up.md").contains("Logcat"));
        assertFalse(read("TeamCode/src/main/java/edu/ftcsushi/fw/tools/tester/ActuatorBringUpTester.java")
                .contains("RobotLog"));
    }

    @Test
    public void diagnosticFollowupsAreUniqueProposalsAndDoNotReactivateSourceFiltering() throws IOException {
        String tracker = read("FRAMEWORK_IMPROVEMENT_TRACKER.md");
        Matcher rows = Pattern.compile("(?m)^\\| (\\d+) \\| ([A-Z]+-\\d+) \\| ([^\\r\\n]+)$")
                .matcher(tracker.substring(tracker.indexOf("## Recommended implementation order"),
                        tracker.indexOf("### Diagnostic follow-up intake")));
        Set<String> ids = new HashSet<String>();
        int expectedOrder = 1;
        while (rows.find()) {
            assertEquals(expectedOrder++, Integer.parseInt(rows.group(1)));
            assertTrue("Duplicate queue ID " + rows.group(2), ids.add(rows.group(2)));
        }
        for (int index = 2; index <= 7; index++) {
            String id = "DIAG-0" + index;
            assertTrue(ids.contains(id));
            assertTrue(Pattern.compile("(?m)^\\| \\d+ \\| " + id + " \\| [^|]+ \\| Proposed \\|")
                    .matcher(tracker).find());
            assertEquals(1, tracker.split("### " + id + " -", -1).length - 1);
        }
        assertTrue(Pattern.compile("(?m)^\\| \\d+ \\| SOURCE-03 \\| [^|]+ \\| Deferred \\|")
                .matcher(tracker).find());
    }

    private static String read(String relative) throws IOException {
        Path root = Paths.get("").toAbsolutePath();
        while (root != null && !Files.exists(root.resolve("settings.gradle"))) root = root.getParent();
        if (root == null) throw new IOException("Run from the repository or its TeamCode directory");
        return new String(Files.readAllBytes(root.resolve(relative)), StandardCharsets.UTF_8);
    }
}
