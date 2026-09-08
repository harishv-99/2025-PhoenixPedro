package edu.ftcsushi.fw.docs;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.DirectoryStream;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the current Sushi documentation information architecture without network access. */
public final class DocumentationLinksTest {

    private static final String FRAMEWORK_DOCS_PATH =
            "TeamCode/src/main/java/edu/ftcsushi/fw";
    private static final String PUBLISHED_API_ROOT =
            "https://harishv-99.github.io/2025-PhoenixPedro/api/";
    private static final String MAINTAINED_REPOSITORY_ROOT =
            "https://github.com/harishv-99/2025-PhoenixPedro/";
    private static final String FENCE =
            String.valueOf((char) 96) + (char) 96 + (char) 96;
    private static final int PUBLISHED_SHELL_COMMAND_PAIR_COUNT = 28;

    private static final List<String> GUIDE_AREAS = Arrays.asList(
            "Get Started",
            "Learn",
            "Build",
            "Test & Tune",
            "Advanced",
            "Reference");

    private static final List<String> GET_STARTED_NAV_TARGETS = Arrays.asList(
            "README.md",
            "docs/getting-started/Framework Overview.md",
            "docs/getting-started/First Software Tour.md",
            "docs/getting-started/Build and Run.md",
            "docs/README.md");

    private static final List<String> BUILD_MARKDOWN_FILES = Arrays.asList(
            "Combine Drive and Intake.md",
            "Continuous Intake.md",
            "First Autonomous.md",
            "First Drive.md",
            "First Pedro Auto.md",
            "Move a Referenced Lift.md",
            "Named Claw.md",
            "README.md",
            "Read a Switch.md",
            "Referenced Lift.md",
            "Run One Timed Auto.md",
            "Single Flywheel Velocity.md");

    private static final List<String> BUILD_RECIPE_FILES = Arrays.asList(
            "Read a Switch.md",
            "First Drive.md",
            "Continuous Intake.md",
            "Named Claw.md",
            "Referenced Lift.md",
            "Move a Referenced Lift.md",
            "Single Flywheel Velocity.md",
            "Combine Drive and Intake.md",
            "Run One Timed Auto.md",
            "First Autonomous.md",
            "First Pedro Auto.md");

    private static final Map<String, String> BUILD_RECIPE_TEST_SELECTORS =
            buildRecipeTestSelectors();
    private static final Map<String, Integer> PUBLISHED_SHELL_PAIRS_BY_PAGE =
            publishedShellPairsByPage();

    private static final List<String> BUILD_NAV_TARGETS = Arrays.asList(
            "docs/build/README.md",
            "docs/build/Read a Switch.md",
            "docs/build/Continuous Intake.md",
            "docs/build/Named Claw.md",
            "docs/build/First Drive.md",
            "docs/build/Combine Drive and Intake.md",
            "docs/build/Run One Timed Auto.md",
            "docs/build/Referenced Lift.md",
            "docs/build/Move a Referenced Lift.md",
            "docs/build/First Autonomous.md",
            "docs/build/Single Flywheel Velocity.md");

    private static final List<String> TEST_AND_TUNE_NAV_TARGETS = Arrays.asList(
            "docs/testing-calibration/README.md",
            "docs/examples/Hardware-free Reference Scenarios.md",
            "docs/testing-calibration/Using the Tester Console.md",
            "docs/testing-calibration/Actuator Bring-up.md",
            "docs/testing-calibration/Robot Calibration Tutorials.md",
            "docs/testing-calibration/Control Tuning Workflow.md",
            "docs/testing-calibration/How to test a Sushi component.md",
            "docs/testing-calibration/Add Calibration Testers to Your Robot.md",
            "docs/testing-calibration/Add Vision to Your Calibration Suite.md",
            "docs/testing-calibration/Enable Powered Calibration.md",
            "docs/testing-calibration/Guided Calibration Walkthroughs.md",
            "docs/troubleshooting/README.md",
            "docs/troubleshooting/Common Problems.md");

    private static final List<String> REFERENCE_CATEGORY_FILES = Arrays.asList(
            "Actuation Plants and control.md",
            "Drive geometry and spatial reasoning.md",
            "FTC adapters testing and tuning.md",
            "Integrations and extension seams.md",
            "Program and lifecycle.md",
            "Sensing localization and vision.md",
            "Tasks outcomes and coordination.md",
            "Values sources and bindings.md");

    private static final List<String> REFERENCE_MARKDOWN_FILES = Arrays.asList(
            "Actuation Plants and control.md",
            "Drive geometry and spatial reasoning.md",
            "FTC adapters testing and tuning.md",
            "Glossary.md",
            "Integrations and extension seams.md",
            "Program and lifecycle.md",
            "README.md",
            "Sensing localization and vision.md",
            "Sushi Cheat Sheet.md",
            "Tasks outcomes and coordination.md",
            "Values sources and bindings.md");

    private static final List<String> REFERENCE_NAV_TARGETS = Arrays.asList(
            "docs/reference/README.md",
            "docs/reference/Program and lifecycle.md",
            "docs/reference/Values sources and bindings.md",
            "docs/reference/Actuation Plants and control.md",
            "docs/reference/Tasks outcomes and coordination.md",
            "docs/reference/Drive geometry and spatial reasoning.md",
            "docs/reference/Sensing localization and vision.md",
            "docs/reference/FTC adapters testing and tuning.md",
            "docs/reference/Integrations and extension seams.md",
            "docs/reference/Sushi Cheat Sheet.md",
            "docs/reference/Glossary.md",
            PUBLISHED_API_ROOT);

    private static final List<String> JAVADOC_GROUPS = Arrays.asList(
            "Program & lifecycle",
            "Values, sources & bindings",
            "Actuation, Plants & control",
            "Tasks, outcomes & coordination",
            "Drive, geometry & spatial reasoning",
            "Sensing, localization & vision",
            "FTC adapters, testing & tuning",
            "Integrations & extension seams",
            "Maintained robot examples");

    private static final List<String> DELETED_EXAMPLE_SYMBOLS = Arrays.asList(
            "BasicDriveAuto",
            "BasicDriveControls",
            "BasicDriveProfile",
            "BasicDriveStopOwner",
            "BasicHardwareOwnership",
            "BasicRobotAuto",
            "BasicRobotAutoRoutines",
            "BasicRobotTeleOp",
            "BasicRobotScenarioTest",
            "BasicPedroAutoPaths",
            "BasicPedroAutoRoutine",
            "BasicPedroAutoMechanism",
            "BasicPedroAutoExample",
            "BasicPedroAutoRobot",
            "BasicPedroProfile",
            "BasicPedroAutoMechanismTest",
            "BasicPedroAutoRoutineTest",
            "BasicPedroMechanismTestFactory",
            "BasicPedroAutoExampleTest",
            "BasicPedroAutoConfigurationTest",
            "BasicPedroAutoRobotTest",
            "BasicPedroProfileAndApiTest",
            "ReferenceAutoRoutines",
            "ReferenceLift",
            "ReferenceLiftMechanism",
            "ReferenceAuto",
            "ReferenceTeleOp",
            "ReferenceCapabilities",
            "ReferenceProfile",
            "ReferenceRobot",
            "ReferenceTeleOpControls",
            "ReferenceRobotTesters",
            "ReferenceLiftMechanismTest",
            "ReferenceLiftSoftwareScenarioTest",
            "ReferenceAutoRoutinesTest",
            "ReferenceRobotTest",
            "ReferenceTeleOpControlsTest");

    private static final Pattern JAVA_FENCE = Pattern.compile(
            "(?m)^\\x60\\x60\\x60java(?:[ \\t]+hl_lines=\"[0-9]+(?: [0-9]+)*\")?"
                    + "[ \\t]*$");
    private static final Pattern SOURCE_EXCERPT = Pattern.compile(
            "(?m)^<!-- source-excerpt: ([^>]+) -->\\r?\\n"
                    + "\\x60\\x60\\x60java(?:[ \\t]+hl_lines=\"[0-9]+(?: [0-9]+)*\")?"
                    + "[ \\t]*\\r?\\n"
                    + "([\\s\\S]*?)\\r?\\n\\x60\\x60\\x60[ \\t]*$");
    private static final Pattern COMPLETE_SOURCE = Pattern.compile(
            "\\[Complete source:[^]]+]\\(<"
                    + Pattern.quote(MAINTAINED_REPOSITORY_ROOT)
                    + "(?:blob|tree)/master/([^>]+)>\\)");
    private static final Pattern CALLOUT_START = Pattern.compile(
            "^!!![ \\t]+(info|warning|danger|success|tip)"
                    + "[ \\t]+\"([^\"]+)\"[ \\t]*$");
    private static final Pattern ANY_CALLOUT_START = Pattern.compile(
            "^(!!!|\\?\\?\\?\\+?)[ \\t]+([^ \\t]+)"
                    + "(?:[ \\t]+\"([^\"]*)\")?[ \\t]*$");
    private static final Pattern HEADING_ATTRIBUTE_ID = Pattern.compile(
            "(?:^|\\s)\\{[^}]*#([A-Za-z0-9][A-Za-z0-9_-]*)[^}]*}[ \\t]*$");
    private static final Pattern HIGHLIGHTED_JAVA_FENCE = Pattern.compile(
            "^\\x60\\x60\\x60java[ \\t]+hl_lines=\"([0-9]+(?: [0-9]+)*)\"[ \\t]*$");
    private static final Pattern CONCEPT_CALLOUT_START = Pattern.compile(
            "(?m)^!!![ \\t]+info[ \\t]+\"New concept: [^\"]+\"[ \\t]*$");

    @Rule
    public final TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void maintainedRepositoryMarkdownHasValidLocalLinksAnchorsAndFences()
            throws IOException {
        Path repositoryRoot = repositoryRoot();

        assertNoFailures(MarkdownIntegrity.validateRepository(repositoryRoot));
    }

    @Test
    public void everyPublishedShellCommandIsAnEquivalentWindowsMacOsTabPair()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        Path docsRoot = configuredDocsRoot(repositoryRoot, config);
        String theme = sectionBetween(config, "[project.theme]", "[project.validation]");
        List<Path> pages = new ArrayList<Path>();
        collectMarkdownFiles(docsRoot, pages);
        List<String> failures = new ArrayList<String>();
        Set<String> visitedPages = new LinkedHashSet<String>();
        int pairCount = 0;

        assertEquals("content.tabs.link must be enabled exactly once",
                1, Collections.frequency(
                        activeTomlStringArrayEntries(theme, "features"),
                        "content.tabs.link"));
        assertEquals("content.tabs.link must appear only in the theme feature list",
                1, literalCount(config, "\"content.tabs.link\""));
        for (Path page : pages) {
            String relative = repositoryRelativePath(docsRoot, page);
            visitedPages.add(relative);
            ShellCommandTabs.Validation validation =
                    ShellCommandTabs.validate(relative, readUtf8(page));
            pairCount += validation.pairs.size();
            failures.addAll(validation.failures);
            validatePublishedShellPairInventory(
                    relative, validation.pairs.size(), failures);
        }
        for (String expectedPage : PUBLISHED_SHELL_PAIRS_BY_PAGE.keySet()) {
            if (!visitedPages.contains(expectedPage)) {
                failures.add(expectedPage
                        + ": approved shell-command inventory page is missing");
            }
        }

        assertEquals("Published shell-command tab-pair inventory changed",
                PUBLISHED_SHELL_COMMAND_PAIR_COUNT, pairCount);
        assertTrue("Published shell-command tab failures:\n" + joinLines(failures),
                failures.isEmpty());
    }

    @Test
    public void themeFeatureInventoryIgnoresCommentedSpoofs() {
        String commentedOnly = "features = [\n"
                + "  # \"content.tabs.link\",\n"
                + "  \"navigation.tabs\",\n"
                + "]\n";
        assertFalse("A commented feature must not count as active",
                activeTomlStringArrayEntries(commentedOnly, "features")
                        .contains("content.tabs.link"));

        String multilineSpoof = "features = [\n"
                + "  \"\"\"\n"
                + "  \"content.tabs.link\"\n"
                + "  \"\"\"\n"
                + "]\n";
        assertFalse("String contents must not count as an active array entry",
                activeTomlStringArrayEntries(multilineSpoof, "features")
                        .contains("content.tabs.link"));

        String active = "features = [\n"
                + "  # \"content.tabs.link\",\n"
                + "  \"content.tabs.link\", # synchronize platform tabs\n"
                + "]\n";
        assertEquals(Collections.singletonList("content.tabs.link"),
                activeTomlStringArrayEntries(active, "features"));
    }

    @Test
    public void publishedShellPairInventoryIsExactPerPage() {
        int approvedPairs = 0;
        for (Integer count : PUBLISHED_SHELL_PAIRS_BY_PAGE.values()) {
            approvedPairs += count;
        }
        assertEquals("The approved inventory must cover exactly 20 published pages",
                20, PUBLISHED_SHELL_PAIRS_BY_PAGE.size());
        assertEquals("The per-page inventory must account for every approved pair",
                PUBLISHED_SHELL_COMMAND_PAIR_COUNT, approvedPairs);

        List<String> failures = new ArrayList<String>();
        validatePublishedShellPairInventory(
                "docs/build/First Drive.md", 0, failures);
        validatePublishedShellPairInventory(
                "docs/unexpected/New Commands.md", 1, failures);

        assertFailureContains(failures,
                "docs/build/First Drive.md: expected 1 shell-command pair, found 0");
        assertFailureContains(failures,
                "docs/unexpected/New Commands.md: unexpected shell-command pair");
    }

    @Test
    public void shellCommandTabParserNormalizesOnlyPlatformSpellings() {
        String markdown = "````markdown\n"
                + "```powershell\nnot a published shell fence\n```\n"
                + "````\n\n"
                + "=== \"Windows\"\n\n"
                + "    ```powershell\n"
                + "    .\\gradlew.bat --console=plain `\n"
                + "      :TeamCode:testDebugUnitTest --tests example.Scenario\n"
                + "    python -m venv build/docs-venv\n"
                + "    .\\build\\docs-venv\\Scripts\\python.exe -m zensical build --strict\n"
                + "    ```\n\n"
                + "=== \"macOS\"\n\n"
                + "    ```bash\n"
                + "    ./gradlew --console=plain " + '\\' + "\n"
                + "      :TeamCode:testDebugUnitTest --tests example.Scenario\n"
                + "    python3 -m venv build/docs-venv\n"
                + "    ./build/docs-venv/bin/python -m zensical build --strict\n"
                + "    ```\n";

        ShellCommandTabs.Validation validation =
                ShellCommandTabs.validate("Guide.md", markdown);

        assertTrue("Valid shell tabs failed:\n" + joinLines(validation.failures),
                validation.failures.isEmpty());
        assertEquals(1, validation.pairs.size());
        assertEquals("./gradlew --console=plain :TeamCode:testDebugUnitTest "
                        + "--tests example.Scenario\n"
                        + "python3 -m venv build/docs-venv\n"
                        + "./build/docs-venv/bin/python -m zensical build --strict",
                validation.pairs.get(0).normalizedWindows);
        assertEquals("Platform normalization must retain argument order and spelling",
                validation.pairs.get(0).normalizedWindows,
                validation.pairs.get(0).normalizedMacOs);
        assertTrue("The exact nonblank test selector must be discoverable in both tabs",
                validation.hasEquivalentGradleTestSelector("example.Scenario"));
        assertFalse("A different or blank selector must not satisfy a Build recipe",
                validation.hasEquivalentGradleTestSelector(""));
        assertFalse("A prefix of the real selector must not satisfy a Build recipe",
                validation.hasEquivalentGradleTestSelector("example"));
    }

    @Test
    public void shellCommandTabParserReportsMalformedStructureActionably() {
        ShellCommandTabs.Validation standalone = ShellCommandTabs.validate(
                "Standalone.md",
                "```powershell\n.\\gradlew.bat help\n```\n");
        assertFailureContains(standalone.failures, "standalone published shell fence");
        assertFailureContains(standalone.failures,
                "labels/order must be exactly Windows then macOS");

        String wrongOrderAndFences = "=== \"macOS\"\n\n"
                + "```powershell\n"
                + ".\\gradlew.bat help\n"
                + "```\n\n"
                + "=== \"Windows\"\n\n"
                + "    ```zsh\n"
                + "    ./gradlew help\n"
                + "    ```\n";
        ShellCommandTabs.Validation malformed =
                ShellCommandTabs.validate("Malformed.md", wrongOrderAndFences);
        assertFailureContains(malformed.failures,
                "expected exact tab label `=== \"Windows\"`");
        assertFailureContains(malformed.failures,
                "must open exactly as four spaces plus ```powershell");
        assertFailureContains(malformed.failures,
                "expected exact tab label `=== \"macOS\"`");
        assertFailureContains(malformed.failures,
                "must open exactly as four spaces plus ```bash");
    }

    @Test
    public void shellCommandTabParserRejectsPlatformLeaksAndArgumentDrift() {
        String leakedTokens = "=== \"Windows\"\n\n"
                + "    ```powershell\n"
                + "    ./gradlew verify " + '\\' + "\n"
                + "      python3 ./build/docs-venv/bin/python\n"
                + "    ```\n\n"
                + "=== \"macOS\"\n\n"
                + "    ```bash\n"
                + "    .\\gradlew.bat verify `\n"
                + "      python .\\build\\docs-venv\\Scripts\\python.exe\n"
                + "    ```\n";
        ShellCommandTabs.Validation leaked =
                ShellCommandTabs.validate("Leaks.md", leakedTokens);
        assertFailureContains(leaked.failures, "POSIX-only token `./gradlew`");
        assertFailureContains(leaked.failures, "POSIX continuation `\\`");
        assertFailureContains(leaked.failures, "POSIX-only token `python3`");
        assertFailureContains(leaked.failures, "POSIX-only token `bin/python`");
        assertFailureContains(leaked.failures, "Windows-only token `.\\gradlew.bat`");
        assertFailureContains(leaked.failures, "PowerShell continuation ```");
        assertFailureContains(leaked.failures, "Windows-only token `python`");
        assertFailureContains(leaked.failures, "Windows-only token `Scripts/python.exe`");

        String drift = "=== \"Windows\"\n\n"
                + "    ```powershell\n"
                + "    .\\gradlew.bat --console=plain verify --tests example.First\n"
                + "    ```\n\n"
                + "=== \"macOS\"\n\n"
                + "    ```bash\n"
                + "    ./gradlew --console=plain verify --tests example.Second\n"
                + "    ```\n";
        ShellCommandTabs.Validation drifted =
                ShellCommandTabs.validate("Drift.md", drift);
        assertFailureContains(drifted.failures, "normalized commands differ");
        assertFailureContains(drifted.failures, "example.First");
        assertFailureContains(drifted.failures, "example.Second");

        String platformWordsAsArguments = "=== \"Windows\"\n\n"
                + "    ```powershell\n"
                + "    python -m example --literal python\n"
                + "    ```\n\n"
                + "=== \"macOS\"\n\n"
                + "    ```bash\n"
                + "    python3 -m example --literal python3\n"
                + "    ```\n";
        ShellCommandTabs.Validation preservedArguments =
                ShellCommandTabs.validate("Arguments.md", platformWordsAsArguments);
        assertFailureContains(preservedArguments.failures, "normalized commands differ");
        assertFailureContains(preservedArguments.failures, "--literal python`");
        assertFailureContains(preservedArguments.failures, "--literal python3`");
    }

    @Test
    public void shellCommandTabParserFindsContainerAndAliasFreeFences() {
        String markdown = "> ```bash\n"
                + "> echo quoted\n"
                + "> ```\n\n"
                + "- ```shell-session\n"
                + "  $ echo listed\n"
                + "  ```\n\n"
                + "1. ```fish\n"
                + "   echo ordered\n"
                + "   ```\n\n"
                + "> ```{ .zsh }\n"
                + "> echo attributed\n"
                + "> ```\n";

        ShellCommandTabs.Validation validation =
                ShellCommandTabs.validate("Containers.md", markdown);

        assertFailureContains(validation.failures,
                "standalone published shell fence `bash`");
        assertFailureContains(validation.failures,
                "standalone published shell fence `shell-session`");
        assertFailureContains(validation.failures,
                "standalone published shell fence `fish`");
        assertFailureContains(validation.failures,
                "standalone published shell fence `zsh`");

        ShellCommandTabs.Validation escapedContainer = ShellCommandTabs.validate(
                "ContainerBoundary.md",
                "> ```markdown\n"
                        + "> the blockquoted fence is never closed\n\n"
                        + "```bash\n"
                        + "echo top-level command\n"
                        + "```\n");
        assertFailureContains(escapedContainer.failures,
                "standalone published shell fence `bash`");

        ShellCommandTabs.Validation escapedListContainer = ShellCommandTabs.validate(
                "ListContainerBoundary.md",
                "- ```markdown\n"
                        + "  the list-contained fence is never closed\n\n"
                        + "```bash\n"
                        + "echo top-level command\n"
                        + "```\n");
        assertFailureContains(escapedListContainer.failures,
                "standalone published shell fence `bash`");
    }

    @Test
    public void shellCommandTabsRequireExecutableCommands() {
        ShellCommandTabs.Validation validation = ShellCommandTabs.validate(
                "Comments.md",
                shellTabPair("# Windows explanation only", "# macOS explanation only"));

        assertFailureContains(validation.failures,
                "Windows shell fence must contain an executable command, not only comments");
        assertFailureContains(validation.failures,
                "macOS shell fence must contain an executable command, not only comments");
        assertTrue("A comment-only tab set must not count as a complete pair",
                validation.pairs.isEmpty());

        ShellCommandTabs.Validation blockComment = ShellCommandTabs.validate(
                "BlockComments.md",
                shellTabPair("<# Windows explanation\ncontinued #>",
                        "# macOS explanation only"));
        assertFailureContains(blockComment.failures,
                "Windows shell fence must contain an executable command, not only comments");

        ShellCommandTabs.Validation lineComment = ShellCommandTabs.validate(
                "LineComment.md",
                shellTabPair(
                        "# <# explanation\npython -c \"print('<# quoted #>')\"",
                        "# <# explanation\npython3 -c \"print('<# quoted #>')\""));
        assertTrue("A line-comment marker must not open a later block comment:\n"
                        + joinLines(lineComment.failures),
                lineComment.failures.isEmpty());
    }

    @Test
    public void shellCommandTabsRejectMalformedContinuations() {
        String trailingWhitespace = shellTabPair(
                ".\\gradlew.bat verify `  \n  :TeamCode:testDebugUnitTest",
                "./gradlew verify \\ \t\n  :TeamCode:testDebugUnitTest");
        ShellCommandTabs.Validation trailing =
                ShellCommandTabs.validate("Trailing.md", trailingWhitespace);
        assertFailureContains(trailing.failures,
                "Windows continuation marker must be the final character");
        assertFailureContains(trailing.failures,
                "macOS continuation marker must be the final character");

        String blankGap = shellTabPair(
                ".\\gradlew.bat verify `\n\n  :TeamCode:testDebugUnitTest",
                "./gradlew verify \\\n\n  :TeamCode:testDebugUnitTest");
        ShellCommandTabs.Validation gapped =
                ShellCommandTabs.validate("Gap.md", blankGap);
        assertFailureContains(gapped.failures,
                "Windows continuation cannot skip a blank line");
        assertFailureContains(gapped.failures,
                "macOS continuation cannot skip a blank line");

        ShellCommandTabs.Validation escapedBackticks = ShellCommandTabs.validate(
                "EscapedBackticks.md",
                shellTabPair("python example ``", "python3 example ``"));
        assertTrue("An even backtick run is literal, not a continuation:\n"
                        + joinLines(escapedBackticks.failures),
                escapedBackticks.failures.isEmpty());

        ShellCommandTabs.Validation escapedBackslashes = ShellCommandTabs.validate(
                "EscapedBackslashes.md",
                shellTabPair("python example \\\\", "python3 example \\\\"));
        assertTrue("An even backslash run is literal, not a continuation:\n"
                        + joinLines(escapedBackslashes.failures),
                escapedBackslashes.failures.isEmpty());
    }

    @Test
    public void shellCommandTabsReportAnUnclosedMacOsFenceDirectly() {
        String markdown = "=== \"Windows\"\n\n"
                + "    ```powershell\n"
                + "    .\\gradlew.bat help\n"
                + "    ```\n\n"
                + "=== \"macOS\"\n\n"
                + "    ```bash\n"
                + "    ./gradlew help\n";

        ShellCommandTabs.Validation validation =
                ShellCommandTabs.validate("UnclosedMac.md", markdown);

        assertFailureContains(validation.failures, "the macOS shell fence is unclosed");
        assertTrue("An unclosed macOS block must not count as a complete pair",
                validation.pairs.isEmpty());
    }

    @Test
    public void shellCommandTabsRequireQuotedGradleWildcardSelectors() {
        ShellCommandTabs.Validation quoted = ShellCommandTabs.validate(
                "Quoted.md",
                shellTabPair(
                        ".\\gradlew.bat test --tests 'example.*'",
                        "./gradlew test --tests 'example.*'"));
        assertTrue("Quoted wildcard selectors must remain valid:\n"
                        + joinLines(quoted.failures),
                quoted.failures.isEmpty());

        ShellCommandTabs.Validation unquoted = ShellCommandTabs.validate(
                "Unquoted.md",
                shellTabPair(
                        ".\\gradlew.bat test --tests example.*",
                        "./gradlew test --tests example.*"));
        assertFailureContains(unquoted.failures,
                "Windows Gradle --tests wildcard selector must be quoted");
        assertFailureContains(unquoted.failures,
                "macOS Gradle --tests wildcard selector must be quoted");

        ShellCommandTabs.Validation bracketGlob = ShellCommandTabs.validate(
                "BracketGlob.md",
                shellTabPair(
                        ".\\gradlew.bat test --tests example.[AB]",
                        "./gradlew test --tests example.[AB]"));
        assertFailureContains(bracketGlob.failures,
                "Windows Gradle --tests wildcard selector must be quoted");
        assertFailureContains(bracketGlob.failures,
                "macOS Gradle --tests wildcard selector must be quoted");

        ShellCommandTabs.Validation commentedExact = ShellCommandTabs.validate(
                "CommentedExact.md",
                shellTabPair(
                        ".\\gradlew.bat test # --tests example.Scenario",
                        "./gradlew test # --tests example.Scenario"));
        assertFalse("A selector in an inline comment must not satisfy a Build recipe",
                commentedExact.hasEquivalentGradleTestSelector("example.Scenario"));

        ShellCommandTabs.Validation commentedWildcard = ShellCommandTabs.validate(
                "CommentedWildcard.md",
                shellTabPair(
                        ".\\gradlew.bat test # --tests example.*",
                        "./gradlew test # --tests example.*"));
        assertTrue("A wildcard in an inline comment must not trigger validation:\n"
                        + joinLines(commentedWildcard.failures),
                commentedWildcard.failures.isEmpty());

        ShellCommandTabs.Validation quotedHash = ShellCommandTabs.validate(
                "QuotedHash.md",
                shellTabPair(
                        ".\\gradlew.bat test --tests 'example.#Scenario'",
                        "./gradlew test --tests 'example.#Scenario'"));
        assertTrue("A quoted hash remains part of the selector",
                quotedHash.hasEquivalentGradleTestSelector("example.#Scenario"));

        ShellCommandTabs.Validation missing = ShellCommandTabs.validate(
                "MissingSelector.md",
                shellTabPair(
                        ".\\gradlew.bat test --tests",
                        "./gradlew test --tests"));
        assertFailureContains(missing.failures,
                "Windows Gradle --tests must be followed by a nonblank selector");
        assertFailureContains(missing.failures,
                "macOS Gradle --tests must be followed by a nonblank selector");
        assertFalse(missing.hasEquivalentGradleTestSelector("example.Scenario"));
    }

    @Test
    public void currentGuidesDoNotTeachRemovedCalibrationCommandHandoffApis()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        List<Path> currentGuides = new ArrayList<Path>();
        collectMarkdownFiles(configuredDocsRoot(repositoryRoot, config), currentGuides);

        for (Path guide : currentGuides) {
            String text = readUtf8(guide);
            assertTrue(guide + " still teaches removed SearchAfterStep",
                    !text.contains("SearchAfterStep"));
            assertTrue(guide + " still teaches removed resumeTargeting()",
                    !text.contains("resumeTargeting"));
            assertTrue(guide + " still teaches removed holdAfterReference(...)",
                    !text.contains("holdAfterReference"));
        }
    }

    @Test
    public void everyMarkedSourceExcerptIsExactContiguousJava() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path docsRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        List<Path> pages = new ArrayList<Path>();
        collectMarkdownFiles(docsRoot, pages);
        List<String> failures = new ArrayList<String>();

        for (Path page : pages) {
            String markdown = readUtf8(page);
            int markers = literalCount(markdown, "<!-- source-excerpt:");
            Matcher excerpts = SOURCE_EXCERPT.matcher(markdown);
            int matches = 0;
            while (excerpts.find()) {
                matches++;
                String sourcePath = excerpts.group(1).trim();
                String snippet = normalizeExcerpt(excerpts.group(2));
                Path source = repositoryRoot.resolve(sourcePath).toAbsolutePath().normalize();
                if (!source.startsWith(repositoryRoot.toAbsolutePath().normalize())
                        || !Files.isRegularFile(source)
                        || !source.getFileName().toString().endsWith(".java")) {
                    failures.add(repositoryRelativePath(repositoryRoot, page)
                            + ": invalid source excerpt path " + sourcePath);
                } else if (snippet.isEmpty()
                        || !containsDedentedBlock(readUtf8(source), snippet)) {
                    failures.add(repositoryRelativePath(repositoryRoot, page)
                            + ": excerpt is not exact contiguous source from " + sourcePath);
                }
            }
            if (matches != markers) {
                failures.add(repositoryRelativePath(repositoryRoot, page)
                        + ": every source-excerpt marker must immediately label one Java fence; "
                        + "markers=" + markers + ", matches=" + matches);
            }
        }

        assertTrue("Source excerpt provenance failures: " + failures, failures.isEmpty());
    }

    @Test
    public void siteNavigationHasExactlySixGoalAreasWithTabsAndPruning()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        Path docsRoot = configuredDocsRoot(repositoryRoot, config);

        assertEquals("Top-level documentation areas changed", GUIDE_AREAS, topLevelAreas(config));
        assertEquals("navigation.tabs must be enabled exactly once",
                1, literalCount(config, "\"navigation.tabs\""));
        assertEquals("navigation.prune must be enabled exactly once",
                1, literalCount(config, "\"navigation.prune\""));
        assertTrue("The site must use focused section navigation",
                config.contains("\"navigation.sections\""));

        Matcher entries = Pattern.compile("=\\s*\"([^\"]+\\.md)\"").matcher(config);
        List<String> missing = new ArrayList<String>();
        while (entries.find()) {
            String target = entries.group(1);
            Path resolved = docsRoot.resolve(target).toAbsolutePath().normalize();
            if (!resolved.startsWith(docsRoot)) {
                missing.add(target + " (escapes docs_dir)");
            } else if (!Files.isRegularFile(resolved)) {
                missing.add(target + " (missing)");
            }
        }
        assertTrue("Missing documentation navigation targets: " + missing, missing.isEmpty());

        String frameworkHome = readUtf8(docsRoot.resolve("README.md"));
        assertTrue("The framework doorway must use a visual card grid",
                frameworkHome.contains("<div class=\"grid cards\" markdown>"));
        for (String area : GUIDE_AREAS) {
            assertEquals("The framework doorway must show one card for " + area,
                    1, literalCount(frameworkHome, "**" + area + "**"));
        }
    }

    @Test
    public void getStartedHasOneFtcLoopFirstSoftwareRoute() throws IOException {
        Path repositoryRoot = repositoryRoot();
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        Path docsRoot = configuredDocsRoot(repositoryRoot, config);

        assertEquals("Get Started navigation inventory or order changed",
                GET_STARTED_NAV_TARGETS, navTargets(navAreaBlock(config, "Get Started")));

        String welcome = readUtf8(docsRoot.resolve("README.md"));
        String guideMap = readUtf8(docsRoot.resolve("docs/README.md"));
        List<String> failures = new ArrayList<String>();
        requireOrdered(welcome, "Welcome", failures,
                "while (opModeIsActive())",
                "loop()",
                "Complete the Get Started path",
                "(<docs/getting-started/Framework Overview.md>)",
                "(<docs/build/README.md>)");
        requireOrdered(guideMap, "Guide map", failures,
                "(<getting-started/Framework Overview.md>)",
                "(<getting-started/First Software Tour.md>)",
                "(<getting-started/Build and Run.md>)",
                "## Choose by outcome");
        String mapProse = guideMap.replaceAll("\\s+", " ").toLowerCase(Locale.ROOT);
        assertTrue("The Guide map must let readers learn before optional setup and hardware",
                mapProse.contains("reading")
                        && mapProse.contains("optional")
                        && guideMap.contains("(<build/Read a Switch.md>)")
                        && guideMap.contains("(<build/Combine Drive and Intake.md>)")
                        && guideMap.contains("(<build/Run One Timed Auto.md>)"));
        assertTrue("Get Started route failures: " + failures, failures.isEmpty());
    }

    @Test
    public void searchableMarkdownHasOneAllowedAreaTagAndRedirectsHaveNone()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        Path docsRoot = configuredDocsRoot(repositoryRoot, config);
        List<Path> pages = new ArrayList<Path>();
        collectMarkdownFiles(docsRoot, pages);
        List<String> failures = new ArrayList<String>();

        Map<String, String> navAreaByPage = new LinkedHashMap<String, String>();
        for (String area : GUIDE_AREAS) {
            for (String target : navTargets(navAreaBlock(config, area))) {
                if (!target.endsWith(".md")) {
                    continue;
                }
                String prior = navAreaByPage.put(target, area);
                if (prior != null) {
                    failures.add(target + ": appears in both " + prior + " and " + area);
                }
            }
        }

        Set<String> allowed = new LinkedHashSet<String>(GUIDE_AREAS);
        Map<String, Integer> areaCounts = new LinkedHashMap<String, Integer>();
        for (String area : GUIDE_AREAS) {
            areaCounts.put(area, 0);
        }
        Set<String> excluded = new LinkedHashSet<String>();

        for (Path page : pages) {
            PageMetadata metadata = pageMetadata(page);
            String relative = repositoryRelativePath(docsRoot, page);
            String navArea = navAreaByPage.get(relative);
            if (!metadata.hasFrontMatter) {
                failures.add(relative + ": missing YAML front matter");
                continue;
            }
            if (metadata.searchExcluded) {
                excluded.add(relative);
                if (!metadata.tags.isEmpty()) {
                    failures.add(relative + ": search-excluded page must not declare an area tag");
                }
                if (navArea != null) {
                    failures.add(relative + ": search-excluded page must not remain in navigation");
                }
                continue;
            }
            if (metadata.tags.size() != 1) {
                failures.add(relative + ": searchable page needs exactly one area tag, found "
                        + metadata.tags);
                continue;
            }
            String tag = metadata.tags.get(0);
            if (!allowed.contains(tag)) {
                failures.add(relative + ": unsupported area tag " + tag);
                continue;
            }
            areaCounts.put(tag, areaCounts.get(tag) + 1);
            if (navArea == null) {
                failures.add(relative + ": searchable page is absent from navigation");
            } else if (!tag.equals(navArea)) {
                failures.add(relative + ": area tag " + tag
                        + " disagrees with navigation area " + navArea);
            }
        }

        assertEquals("Only the two superseded course URLs should be search-excluded",
                new LinkedHashSet<String>(Arrays.asList(
                        "docs/getting-started/Basic Mechanisms Robot.md",
                        "docs/getting-started/First Pedro Auto.md")),
                excluded);
        for (Map.Entry<String, Integer> count : areaCounts.entrySet()) {
            if (count.getValue() == 0) {
                failures.add("No searchable page uses area tag " + count.getKey());
            }
        }
        assertTrue("Area-tag failures: " + failures, failures.isEmpty());
    }

    @Test
    public void globalGuideSearchUsesTheBuiltInIndexAndSeparatesApiSearch()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        Path docsRoot = configuredDocsRoot(repositoryRoot, config);
        String override = readUtf8(repositoryRoot.resolve("overrides/main.html"));
        String requirements = readUtf8(repositoryRoot.resolve("requirements-docs.txt"));
        String frameworkHome = readUtf8(docsRoot.resolve("README.md"));
        String docsHome = readUtf8(docsRoot.resolve("docs/README.md"));
        String referenceHome = readUtf8(docsRoot.resolve("docs/reference/README.md"));

        assertTrue("The theme must retain Zensical's built-in search UI",
                config.contains("custom_dir = \"overrides\"")
                        && config.contains("\"search.highlight\"")
                        && requirements.contains("zensical==0.0.51")
                        && override.contains("{% extends \"base.html\" %}")
                        && override.contains("{{ super() }}")
                        && override.contains("data-md-component=\"search\""));
        assertTrue("The scope helper must integrate with the pinned open ShadowRoot search",
                !override.contains("data-md-component=\"search-query\"")
                        && override.contains("element.shadowRoot")
                        && override.contains("input[role=\"combobox\"]")
                        && override.contains("discoverSearchRoots(document.documentElement)")
                        && override.contains("new MutationObserver")
                        && override.contains("controls.insertAdjacentElement(\"afterend\", help)"));
        assertTrue("The search input must state its global guide scope accessibly",
                override.contains("const searchLabel = \"Search all guides\"")
                        && override.contains(
                                "headerLabel.setAttribute(\"aria-label\", searchLabel)")
                        && override.contains(
                                "headerLabel.setAttribute(\"title\", searchLabel)")
                        && override.contains(
                                "search.setAttribute(\"aria-label\", searchLabel)")
                        && override.contains(
                                "trigger.setAttribute(\"aria-label\", searchLabel)")
                        && override.contains("input.placeholder = searchLabel")
                        && override.contains("input.setAttribute(\"aria-label\", searchLabel)")
                        && override.contains("input.setAttribute(\"aria-describedby\"")
                        && override.contains(
                                "Searches all six guide areas. Filter by area. "
                                        + "For exact classes and methods, use API search."));
        assertTrue("Closed search controls must leave the tab and accessibility trees",
                override.contains("root.host.setAttribute(\"aria-hidden\", \"true\")")
                        && override.contains("root.host.removeAttribute(\"aria-hidden\")")
                        && override.contains("savedTabIndexes.set(control")
                        && override.contains("control.tabIndex = -1")
                        && override.contains("savedTabIndexes.delete(control)"));
        assertTrue("Existing area filters must be valid keyboard controls with toggle state",
                override.contains("list.setAttribute(\"role\", \"group\")")
                        && override.contains("item.setAttribute(\"role\", \"button\")")
                        && override.contains("item.setAttribute(\"aria-pressed\"")
                        && override.contains("filterBaseClasses.get(root)")
                        && override.contains("item.tabIndex = isAvailableToKeyboard")
                        && override.contains("item.addEventListener(\"keydown\"")
                        && override.contains("event.key === \"Enter\"")
                        && override.contains("event.key === \" \"")
                        && override.contains("event.stopPropagation()")
                        && override.contains("item.click()"));
        assertTrue("The helper must not implement a second search engine",
                !override.contains("fetch(")
                        && !override.contains("XMLHttpRequest")
                        && !override.contains("search_index")
                        && !override.toLowerCase(Locale.ROOT).contains("lunr"));

        assertTrue("The visual doorway must say search remains global across tabs",
                frameworkHome.contains("**Search all guides** searches all six areas")
                        && frameworkHome.contains("even while one tab is open")
                        && frameworkHome.contains("area tags")
                        && frameworkHome.contains(PUBLISHED_API_ROOT));
        assertTrue("The exhaustive hub must distinguish guide and API searches",
                docsHome.contains("The site search is global, not limited to the selected tab")
                        && docsHome.contains("one area tag")
                        && docsHome.contains("exact classes, members, signatures, or overloads")
                        && docsHome.contains(PUBLISHED_API_ROOT));
        assertTrue("Reference must repeat the guide/API distinction",
                referenceHome.contains("**Search all guides** searches every area")
                        && referenceHome.contains("Javadoc search is separate")
                        && referenceHome.contains("Java types and members"));
        assertTrue("Navigation must expose API member search separately",
                config.contains("{ \"API: search types and members\" = \""
                        + PUBLISHED_API_ROOT + "\" }"));
    }

    @Test
    public void buildAreaHasCumulativeKnowledgeAndFocusedIndependentFixtures()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path docsRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        Path buildRoot = docsRoot.resolve("docs/build");
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));

        assertEquals("Build Markdown inventory changed",
                BUILD_MARKDOWN_FILES, markdownFileNames(buildRoot));
        assertEquals("Build navigation inventory or order changed",
                BUILD_NAV_TARGETS, navTargets(navAreaBlock(config, "Build")));

        String index = readUtf8(buildRoot.resolve("README.md"));
        for (String recipe : BUILD_RECIPE_FILES) {
            assertTrue("Build index does not link " + recipe,
                    index.contains("(<" + recipe + ">)"));
        }
        List<String> routeFailures = new ArrayList<String>();
        requireOrdered(index, "Build README", routeFailures,
                "(<Read a Switch.md>)",
                "(<Continuous Intake.md>)",
                "(<Named Claw.md>)",
                "(<First Drive.md>)",
                "(<Combine Drive and Intake.md>)",
                "(<Run One Timed Auto.md>)",
                "(<Referenced Lift.md>)",
                "(<Move a Referenced Lift.md>)",
                "(<First Autonomous.md>)",
                "(<Single Flywheel Velocity.md>)");
        String indexProse = index.replaceAll("\\s+", " ");
        assertTrue("Build index must expose cumulative knowledge with independent fixtures: "
                        + routeFailures,
                indexProse.contains("knowledge is cumulative")
                        && indexProse.contains("hardware fixtures are intentionally independent")
                        && indexProse.toLowerCase(Locale.ROOT).contains("read the lessons in order")
                        && routeFailures.isEmpty());
        assertTrue("Drive must remain an independent outcome path",
                index.contains("Drive is independent")
                        && index.contains("(<First Drive.md>)"));
        assertTrue("Build must expose the first TeleOp and Auto integration steps",
                index.contains("(<Combine Drive and Intake.md>)")
                        && index.contains("(<Run One Timed Auto.md>)")
                        && index.contains("## Where each concept first appears")
                        && index.contains("continuous gamepad axes")
                        && index.contains("synchronous button meaning")
                        && index.contains("one behavior over time in Auto"));
        String optionalFeedback = sectionBetween(index,
                "## Add feedback when your robot needs it",
                "## Optional: author the slice in your robot");
        assertContainsAll("Feedback must remain a continuation after basic TeleOp and Auto",
                optionalFeedback,
                "optional continuation", "(<Referenced Lift.md>)", "(<Move a Referenced Lift.md>)",
                "(<First Autonomous.md>)", "(<Single Flywheel Velocity.md>)");
        assertTrue("Pedro must be discoverable as an Advanced integration, not a required Build",
                navTargets(navAreaBlock(config, "Advanced"))
                        .contains("docs/build/First Pedro Auto.md")
                        && !BUILD_NAV_TARGETS.contains("docs/build/First Pedro Auto.md"));

        String setup = readUtf8(docsRoot.resolve("docs/getting-started/Build and Run.md"));
        String oldCourseRedirect = readUtf8(docsRoot.resolve(
                "docs/getting-started/Basic Mechanisms Robot.md"));
        String drive = readUtf8(buildRoot.resolve("First Drive.md"));
        String setupProse = setup.replaceAll("\\s+", " ");
        assertTrue("Optional setup must return to the reading tour before hardware selection",
                setup.contains("(<Framework Overview.md>)")
                        && setup.contains("(<First Software Tour.md>)")
                        && setupProse.contains("optional for readers")
                        && setupProse.contains("teaching OpModes remain disabled")
                        && setupProse.contains("choose the next Build outcome")
                        && oldCourseRedirect.contains("(<../build/README.md>)")
                        && drive.contains("Continuous Intake.md")
                        && !setup.contains("select only the mechanism"));
    }

    @Test
    public void beginnerCourseSeparatesReadingRunningAndAuthoringInTheReadersRobot()
            throws IOException {
        Path docsRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH).resolve("docs");
        String index = readUtf8(docsRoot.resolve("build/README.md"));
        String setup = readUtf8(docsRoot.resolve("getting-started/Build and Run.md"));
        String tour = readUtf8(docsRoot.resolve("getting-started/First Software Tour.md"));
        String modes = sectionBetween(index,
                "## Choose how to follow a lesson", "## Understand one part");
        String authoring = sectionBetween(index,
                "## Optional: author the slice in your robot",
                "## Where each concept first appears");

        assertContainsAll("The complete course must be available by reading", index,
                "Reading is a complete learning path",
                "no installation, code changes, test run, or matching hardware is required");
        assertContainsAll("Reading and executing a checkpoint must remain distinct", modes,
                "predict", "observation", "reading checkpoint", "optionally",
                "reading an expected result does not claim that you ran it",
                "physical check", "separate");
        assertContainsAll("Optional authoring must name independent main and test package homes",
                authoring,
                "TeamCode/src/main/java/edu/ftcsushi/robots/myrobot/",
                "TeamCode/src/test/java/edu/ftcsushi/robots/myrobot/",
                "edu.ftcsushi.robots.myrobot", "package-private controls",
                "test-only", "never in robot main", "construction calls", "Gradle selector",
                "wrong expected value should fail", "does not verify your owner");
        assertContainsAll("Software setup must be optional for readers", setup,
                "optional for readers", "First Software Tour.md");
        assertContainsAll("The tour must offer optional execution without making it completion",
                tour, "Optional: run the examples", "predict", "expected behavior",
                "running the tests is not a graduation requirement");
    }

    @Test
    public void switchLessonOwnsTheCompleteObservationPathAndIntakeControlsStayFocused()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path frameworkRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        Path buildRoot = frameworkRoot.resolve("docs/build");
        String sensor = readUtf8(buildRoot.resolve("Read a Switch.md"));
        String service = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/"
                        + "BasicSwitchService.java"));
        String host = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicsensing/"
                        + "BasicSwitchTeleOp.java"));
        String robot = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/"
                        + "StarterRobot.java"));
        String focusedControls = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/"
                        + "StarterIntakeControls.java"));
        String intake = readUtf8(buildRoot.resolve("Continuous Intake.md"));

        assertContainsAll("Switch lesson must expose its active input, filter, and lifecycle graph",
                sensor,
                "config.switchName = \"lessonSwitch\"",
                "config.pressedDebounceSec = 0.02",
                "config.releasedDebounceSec = 0.02",
                "FtcSensors.digitalLow(map, switchName)",
                "rawPressedSource.debouncedOnOff(pressedDelay, releasedDelay)",
                "NOT_OBSERVED = new Status(false, false, false)",
                "program.service(", "program.presenter(", "limitSwitch.status()",
                "BasicSwitchService.java", "BasicSwitchTeleOp.java",
                "BasicSwitchSoftwareScenarioTest.java", "BasicSwitchTestRig.java");
        assertContainsAll("Switch evidence must distinguish sampling, status, and physical truth",
                sensor,
                "Construction does not sample", "immutable", "does not cause two hardware reads",
                "START's zero elapsed interval", "STOP occurs before START",
                "INIT", "observed", "false", "unknown", "does not discover the wiring",
                "not on a new thread", "commits telemetry once");
        assertTrue("Switch status lookup must remain a cached observation",
                service.contains("return status;")
                        && host.contains("BasicSwitchService.Status status = limitSwitch.status();")
                        && !host.contains("getAsBoolean(")
                        && !host.contains("telemetry.update()"));
        assertTrue("The focused switch fixture must own no actuator or drive output",
                !service.contains("FtcActuators") && !host.contains("program.output(")
                        && !host.contains("program.drive(") && host.contains("@Disabled"));

        String focusedDeclaration = sectionBetween(robot,
                "public void declareIntakeTeleOp(", "public StarterIntake declareAuto(");
        assertTrue("Focused intake must use its shared button owner without constructing drive",
                focusedDeclaration.contains("new StarterIntakeControls(")
                        && !focusedDeclaration.contains("StarterTeleOpControls")
                        && !focusedDeclaration.contains("GamepadDriveSource")
                        && !focusedControls.contains("fw.drive.")
                        && intake.contains("robot/StarterIntakeControls.java")
                        && !intake.contains("robot/StarterTeleOpControls.java"));
    }

    @Test
    public void everyBuildRecipeUsesTheSourceBackedEvidenceAnatomy() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path buildRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH).resolve("docs/build");
        List<String> failures = new ArrayList<String>();

        assertEquals("Build recipe test-selector inventory changed",
                new LinkedHashSet<String>(BUILD_RECIPE_FILES),
                BUILD_RECIPE_TEST_SELECTORS.keySet());

        for (String fileName : BUILD_RECIPE_FILES) {
            Path page = buildRoot.resolve(fileName);
            String markdown = readUtf8(page);

            requireExactlyOnce(markdown, "**Outcome:**", fileName, failures);
            Matcher prerequisites = Pattern.compile(
                    "(?m)^\\*\\*(?:Prerequisites|(?:Optional[^*]*\\. )?"
                            + "Knowledge before this page):\\*\\*")
                    .matcher(markdown);
            String prerequisitesLabel = prerequisites.find() ? prerequisites.group() : "";
            if (prerequisitesLabel.isEmpty() || prerequisites.find()) {
                failures.add(fileName + ": declare exactly one knowledge-prerequisite block");
            }
            requireExactlyOnce(markdown, "## Critical production idea", fileName, failures);
            requireExactlyOnce(markdown, "## Files in this checkpoint", fileName, failures);
            requireExactlyOnce(markdown, "## Software checkpoint:", fileName, failures);
            requireExactlyOnce(markdown, "## Isolated hardware gate", fileName, failures);
            requireExactlyOnce(markdown, "**Next gate:**", fileName, failures);
            requireOrdered(markdown, fileName, failures,
                    "**Outcome:**",
                    prerequisitesLabel,
                    "## Critical production idea",
                    "## Files in this checkpoint",
                    "## Software checkpoint:",
                    "## Isolated hardware gate",
                    "**Next gate:**");

            requireExactlyOnce(markdown, "- **Question:**", fileName, failures);
            requireExactlyOnce(markdown, "- **Keep real:**", fileName, failures);
            requireExactlyOnce(markdown, "- **Replace:**", fileName, failures);
            requireExactlyOnce(markdown, "- **Observe:**", fileName, failures);
            requireExactlyOnce(markdown, "- **Cannot conclude:**", fileName, failures);
            requireOrdered(markdown, fileName, failures,
                    "- **Question:**",
                    "- **Keep real:**",
                    "- **Replace:**",
                    "- **Observe:**",
                    "- **Cannot conclude:**");

            requireExactlyOnce(markdown, "**Read the causal chain:**", fileName, failures);
            requireExactlyOnce(markdown, "**Proves:**", fileName, failures);
            requireExactlyOnce(markdown, "**Does not prove:**", fileName, failures);
            requireExactlyOnce(markdown, "**Expected observations:**", fileName, failures);
            requireExactlyOnce(markdown, "**Reading checkpoint:**", fileName, failures);
            requireOrdered(markdown, fileName, failures,
                    "**Read the causal chain:**",
                    "**Proves:**",
                    "**Does not prove:**",
                    "**Next gate:**");
            requireOrdered(markdown, fileName, failures,
                    "**Expected observations:**",
                    "**Reading checkpoint:**",
                    "## Isolated hardware gate");

            boolean hasMainManifest = Pattern.compile(
                    "(?m)^\\*\\*Main(?: added here)?:\\*\\*$")
                    .matcher(markdown).find();
            boolean hasTestManifest = Pattern.compile("(?m)^\\*\\*Test:\\*\\*$")
                    .matcher(markdown).find();
            if (!hasMainManifest || !hasTestManifest) {
                failures.add(fileName + ": missing exact Main/Test checkpoint manifest");
            }
            ShellCommandTabs.Validation shellTabs =
                    ShellCommandTabs.validate(fileName, markdown);
            String expectedSelector = BUILD_RECIPE_TEST_SELECTORS.get(fileName);
            if (expectedSelector == null
                    || !shellTabs.hasEquivalentGradleTestSelector(expectedSelector)) {
                failures.add(fileName + ": missing equivalent Windows/macOS focused Gradle "
                        + "scenario command pair with exact nonblank --tests selector "
                        + expectedSelector);
            }
            if (!markdown.contains(PUBLISHED_API_ROOT)) {
                failures.add(fileName + ": missing generated API link");
            }
            if (markdown.contains("source-file:")
                    || markdown.contains("annotated-source")
                    || markdown.contains("teaching-shape")) {
                failures.add(fileName + ": contains a full, annotated, or invented Java source");
            }

            validateBuildNotice(markdown, fileName, failures);

            validateBuildSources(repositoryRoot, fileName, markdown, failures);
        }

        assertTrue("Build recipe contract failures: " + failures, failures.isEmpty());
    }

    @Test
    public void actuatorLessonsTeachTheCompleteFirstPathThenOnlyNewDecisions()
            throws IOException {
        Path buildRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH).resolve("docs/build");
        String intake = readUtf8(buildRoot.resolve("Continuous Intake.md"));
        List<String> failures = new ArrayList<String>();

        assertTrue("The first actuator lesson must build on observation without requiring hardware",
                intake.contains("**Knowledge before this page:**")
                        && intake.contains("(<Read a Switch.md>)")
                        && intake.contains("complete first actuator lesson")
                        && intake.contains("without installing software or owning a motor"));
        requireOrdered(intake, "Continuous Intake.md", failures,
                "### 1. Keep physical answers in data-only configuration",
                "### 2. Map each name forward once",
                "### 3. Build the one final hardware writer",
                "### 4. Separate requests, cached status, update, and stop",
                "### 5. Declare the output owner once",
                "### 6. Give buttons semantic meaning",
                "### 7. Put the complete slice under the managed host");
        for (String required : Arrays.asList(
                "enum Mode",
                "profile.intake.motorName",
                "profile.allowIntakeMotion = false",
                "SemanticScalarCommand.forEnum",
                "FtcActuators.plant(",
                ".motor(motorName, direction)",
                ".power()",
                ".targetExactlyFrom(modeCommand)",
                "modeCommand.set(",
                "modeCommand.snapshot(plant.snapshot())",
                "plant.update(clock)",
                "plant.stop()",
                "program.output(",
                "program.callbackBindings()",
                "FtcRobotOpMode",
                "does **not** command motion")) {
            if (!intake.contains(required)) {
                failures.add("Continuous Intake.md: missing first-path teaching for " + required);
            }
        }

        Map<String, String> cumulative = new LinkedHashMap<String, String>();
        cumulative.put("Named Claw.md", "Continuous Intake.md");
        cumulative.put("Referenced Lift.md", "Read a Switch.md");
        cumulative.put("Move a Referenced Lift.md", "Referenced Lift.md");
        cumulative.put("Single Flywheel Velocity.md", "Move a Referenced Lift.md");
        for (Map.Entry<String, String> lesson : cumulative.entrySet()) {
            String markdown = readUtf8(buildRoot.resolve(lesson.getKey()));
            requireExactlyOnce(markdown, "**Builds on:**", lesson.getKey(), failures);
            requireExactlyOnce(markdown, "**New here:**", lesson.getKey(), failures);
            if (!markdown.contains("(<" + lesson.getValue() + ">)")) {
                failures.add(lesson.getKey() + ": missing immediate prerequisite link to "
                        + lesson.getValue());
            }
        }

        String reference = readUtf8(buildRoot.resolve("Referenced Lift.md"));
        String move = readUtf8(buildRoot.resolve("Move a Referenced Lift.md"));
        String claw = readUtf8(buildRoot.resolve("Named Claw.md"));
        String referenceProse = reference.replaceAll("\\s+", " ");
        assertTrue("Claw lesson must make the fixture button meanings explicit",
                claw.contains("driver.a()")
                        && claw.contains("BasicClaw.State.CLOSED")
                        && claw.contains("driver.y()")
                        && claw.contains("BasicClaw.State.HALF")
                        && claw.contains("driver.b()")
                        && claw.contains("BasicClaw.State.OPEN"));
        assertTrue("Reference lesson must hand off to the separate move outcome",
                reference.contains("(<Move a Referenced Lift.md>)")
                        && reference.contains("FtcSensors.digitalLow(")
                        && reference.contains(".debouncedOnOff(0.02, 0.02)")
                        && reference.contains("BasicLift.Height.STOWED")
                        && referenceProse.contains("press STOP, de-energize the robot")
                        && referenceProse.contains(
                                "The next OpMode run correctly starts unreferenced")
                        && referenceProse.contains("never touch or reconnect the linkage"));
        assertTrue("Move lesson must prove the advertised feedback-aware Task",
                move.contains("BasicLiftMoveSoftwareScenarioTest.java")
                        && move.contains(".untilReachedBy(lift)")
                        && move.contains(".leaveRequestOnCancel()")
                        && move.contains("TaskOutcome.SUCCESS")
                        && move.contains("a new OpMode run starts unreferenced")
                        && move.contains("driver.dpadLeft()")
                        && move.contains("driver.x()"));

        String velocity = readUtf8(buildRoot.resolve("Single Flywheel Velocity.md"));
        assertTrue("Velocity lesson must teach the valid numeric-command exception",
                velocity.contains("velocity itself is the public capability intent")
                        && velocity.contains("flywheel.commandTarget()")
                        && velocity.contains("ScalarTasks.set(")
                        && velocity.contains(".cancelTo(STOPPED_VELOCITY_TICKS_PER_SEC)")
                        && velocity.contains("assertFalse(reachVelocity.isComplete())")
                        && velocity.contains("setMeasuredVelocityTicksPerSec(300.0)")
                        && velocity.contains("assertEquals(TaskOutcome.SUCCESS")
                        && velocity.contains("Feedback-aware Tasks are exercised")
                        && velocity.contains("TeleOp button queue"));
        assertTrue("Progressive actuator lesson failures: " + failures, failures.isEmpty());
    }

    @Test
    public void firstDriveTeachesTheCompleteContinuousProductionPath() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path frameworkRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        String drive = readUtf8(frameworkRoot.resolve("docs/build/First Drive.md"));
        String gamepad = readUtf8(frameworkRoot.resolve("ftc/input/GamepadDevice.java"));
        List<String> failures = new ArrayList<String>();

        requireOrdered(drive, "First Drive.md", failures,
                "### 1. Adapt the FTC gamepad during INIT",
                "### 2. Give the three axes robot-frame meanings",
                "### 3. Review one complete mecanum configuration",
                "### 4. Declare a continuously sampled drive path");
        for (String required : Arrays.asList(
                "new GamepadDevice(gamepad1)",
                "requiredDriver.setAxisDeadband(0.02)",
                "requiredDriver.leftX()",
                "requiredDriver.leftY()",
                "requiredDriver.rightX()",
                "driveSourceConfig.deadband = 0.05",
                "driveSourceConfig.translateExpo = 1.5",
                "driveSourceConfig.rotateExpo = 1.5",
                "driveSourceConfig.translateScale = 1.0",
                "driveSourceConfig.rotateScale = 1.0",
                "frontLeftName",
                "frontRightDirection",
                "maxAxial = 0.25",
                "maxLateral = 0.25",
                "maxOmega = 0.20",
                "program.drive(",
                "FirstDriveSoftwareScenarioTest",
                "assertWheelPowers",
                "mode.stop()")) {
            if (!drive.contains(required)) {
                failures.add("First Drive.md: missing reconstruction teaching for " + required);
            }
        }
        requireOrdered(drive, "First Drive.md", failures,
                "!!! info \"New concept: Stick shaping\"",
                "A **deadband** is a small centered stick range",
                "```java hl_lines=\"3 5 6 7 8 9 10\"",
                "requiredDriver.setAxisDeadband(0.02)",
                "driveSourceConfig.rotateScale = 1.0",
                "| Stage | Exact values | Effect |",
                "Controller correction",
                "Driver shaping",
                "Drive caps");
        assertEquals("First Drive canonical concept count", 1,
                matcherCount(CONCEPT_CALLOUT_START.matcher(drive)));
        assertTrue("First Drive must warn about construction-time neutral calibration",
                drive.replaceAll("\\s+", " ").contains("before pressing INIT"));
        String firstPass = sectionBetween(drive,
                "## First pass: current values every loop",
                "## Full build: reconstruct the production path");
        assertTrue("First Drive first pass must teach current values without event machinery",
                firstPass.contains("every active FTC loop")
                        && firstPass.contains("sampled continuously")
                        && !firstPass.contains("CallbackBindings")
                        && !firstPass.contains("TaskBindings"));
        assertTrue("Gamepad buttons must remain truthful held-level sources",
                gamepad.contains("Buttons ({@link BooleanSource}) that report the current held state")
                        && gamepad.contains("Button sources do not detect edges themselves")
                        && gamepad.contains("CallbackBindings")
                        && !gamepad.contains("a {@link BooleanSource} with edge detection"));
        assertTrue("First Drive reconstruction failures: " + failures, failures.isEmpty());
    }

    @Test
    public void buildSpineTeachesManagedIntegrationBeforeTaskComposition() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path buildRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH).resolve("docs/build");
        String teleOp = readUtf8(buildRoot.resolve("Combine Drive and Intake.md"));
        String timedAuto = readUtf8(buildRoot.resolve("Run One Timed Auto.md"));
        String composedAuto = readUtf8(buildRoot.resolve("First Autonomous.md"));
        List<String> failures = new ArrayList<String>();

        for (String required : Arrays.asList(
                "new StarterRobot(hardwareMap).declareTeleOp(",
                "requireDistinctMotorOwners(activeProfile)",
                "controls.bind(program.callbackBindings(), intake)",
                "program.drive(",
                "scaledWhen(requiredDriver.rightBumper()",
                "StarterTeleOpControls.SLOW_TRANSLATE_SCALE",
                "StarterTeleOpControls.SLOW_OMEGA_SCALE",
                "mode.advanceTo(0.08)",
                "driver.right_bumper = false",
                "Clock → Bindings → Tasks → intake output → drive output → Presenters",
                "StarterDriveAndIntakeSoftwareScenarioTest",
                "held-bumper drive continuously")) {
            if (!teleOp.contains(required)) {
                failures.add("Combine Drive and Intake.md: missing integration teaching for "
                        + required);
            }
        }
        assertTrue("The integration lesson must explain pre-lookup ownership rejection",
                teleOp.replaceAll("\\s+", " ")
                        .contains("rejects an exact duplicate before any hardware lookup"));
        assertTrue("The integration lesson must explain total managed STOP",
                teleOp.replaceAll("\\s+", " ")
                        .contains("STOP immediately zeros all five motors"));

        requireOrdered(timedAuto, "Run One Timed Auto.md", failures,
                "program.rootTask(oneTimedCollect(intake))",
                ".forSeconds(durationSec)",
                ".then(Mode.STOPPED)",
                "## Software checkpoint: time begins at START");
        for (String required : Arrays.asList(
                "StarterTimedAutoSoftwareScenarioTest",
                "TaskOutcome.SUCCESS",
                "observes `CANCELLED`",
                "fresh single-use",
                "10.75")) {
            if (!timedAuto.contains(required)) {
                failures.add("Run One Timed Auto.md: missing root-Task teaching for " + required);
            }
        }

        requireOrdered(composedAuto, "First Autonomous.md", failures,
                "(<Run One Timed Auto.md>)",
                "BasicAutoRoutines.liftOnly(lift)",
                "program.rootTask(auto)",
                "### Optional capstone: add the proven claw");
        for (String required : Arrays.asList(
                "Tasks.sequence(",
                "Tasks.parallelDeadline",
                "leaveRequestOnCancel()",
                "persistent request",
                "TaskOutcome.TIMEOUT",
                "TaskOutcome.CANCELLED",
                "suppress")) {
            if (!composedAuto.contains(required)) {
                failures.add("First Autonomous.md: missing composition teaching for " + required);
            }
        }
        assertTrue("Managed Build-spine failures: " + failures, failures.isEmpty());
    }

    @Test
    public void independentActuatorFixturesExposeTheirActiveProfilesAndManagedHosts()
            throws IOException {
        Path buildRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH).resolve("docs/build");
        Map<String, List<String>> requiredByPage = new LinkedHashMap<String, List<String>>();
        requiredByPage.put("Named Claw.md", Arrays.asList(
                "### Put this focused fixture into your robot",
                "BasicClawProfile.current()",
                "closedNativePosition = 0.25",
                "openNativePosition = 0.70",
                "BasicClaw.Status",
                "program.output(",
                "program.callbackBindings()",
                "program.presenter("));
        requiredByPage.put("Referenced Lift.md", Arrays.asList(
                "### Put this focused fixture into your robot",
                "BasicLiftProfile.current()",
                "ticksPerIn = 100.0",
                "BasicLiftHomeControls",
                "BasicLiftHomeTeleOp",
                "program.taskBindings()",
                "program.presenter(",
                "Height.STOWED",
                "TaskOutcome.TIMEOUT"));
        requiredByPage.put("Move a Referenced Lift.md", Arrays.asList(
                "### Put this focused fixture into your robot",
                "BasicLiftTeleOp",
                "program.callbackBindings()",
                "program.taskBindings()",
                "program.presenter(",
                "requestedPositionIn()",
                "measuredPositionIn()",
                "TaskOutcome.CANCELLED",
                "persistent request"));
        requiredByPage.put("Single Flywheel Velocity.md", Arrays.asList(
                "### Put this focused fixture into your robot",
                "BasicFlywheelProfile.current()",
                "candidateVelocityTicksPerSec = 250.0",
                "allowFlywheelMotion = false",
                "requireCandidateInConfiguredRange",
                "BasicFlywheelTeleOp",
                "program.presenter(",
                "Control Tuning Workflow"));

        List<String> failures = new ArrayList<String>();
        for (Map.Entry<String, List<String>> entry : requiredByPage.entrySet()) {
            String markdown = readUtf8(buildRoot.resolve(entry.getKey()));
            for (String required : entry.getValue()) {
                if (!markdown.contains(required)) {
                    failures.add(entry.getKey() + ": missing focused-fixture teaching for "
                            + required);
                }
            }
        }
        assertTrue("Focused actuator fixture failures: " + failures, failures.isEmpty());
    }

    @Test
    public void pedroBuildLessonReportsTheExactRetainedSoftwareAttempt() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path frameworkRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        String page = readUtf8(frameworkRoot.resolve("docs/build/First Pedro Auto.md"));
        String source = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/basic/"
                        + "BasicPedroAuto.java"));
        String integration = readUtf8(frameworkRoot.resolve("integrations/pedro/README.md"));
        String design = readUtf8(frameworkRoot.resolve(
                "docs/design/Recommended Robot Design.md"));

        String pageProse = page.replaceAll("\\s+", " ");
        assertTrue("Pedro outcome must stay software-only while physical motion is blocked",
                pageProse.contains("verify its classified software outcome")
                        && page.contains("blocked software-boundary checkpoint")
                        && page.contains("advanced Pedro integration guide")
                        && page.contains("## Isolated hardware gate — currently blocked")
                        && pageProse.contains("no physical motion is authorized"));
        assertTrue("Pedro telemetry must observe the exact retained RouteTask",
                page.contains("routeTask.getRouteStatus()")
                        && source.contains("routeTask.getRouteStatus()")
                        && !source.contains("getLatestRouteStatus()"));
        for (String explanation : Arrays.asList(page, integration, design, source)) {
            String normalizedExplanation = explanation.replaceAll("\\s+", " ");
            assertTrue("Pedro power-limit explanation must distinguish overwrite from reset",
                    normalizedExplanation.contains("globalMaxPower")
                            && normalizedExplanation.contains("does not reset")
                            && normalizedExplanation.contains("overwrit")
                            && !normalizedExplanation.contains("resets the Follower")
                            && !normalizedExplanation.contains("restores the Follower"));
        }
    }

    @Test
    public void plantChooserRoutesOutcomesWithoutBecomingAnApiCatalog() throws IOException {
        Path frameworkRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH);
        String chooser = readUtf8(frameworkRoot.resolve(
                "docs/getting-started/learn-sushi/Plants and Hardware.md"));

        for (String destination : Arrays.asList(
                "../../build/Continuous Intake.md",
                "../../build/Named Claw.md",
                "../../build/Referenced Lift.md",
                "../../build/Move a Referenced Lift.md",
                "../../build/Single Flywheel Velocity.md",
                "../../advanced/Paired Flywheel Velocity.md",
                "../../advanced/Periodic Turret Position.md")) {
            assertTrue("Plant chooser is missing outcome route " + destination,
                    chooser.contains("(<" + destination + ">)"));
        }
        assertTrue("Plant chooser must stay a decision guide rather than duplicate reference",
                chooser.contains("**Learning mode:** Decision guide")
                        && chooser.contains("## Start with the outcome")
                        && chooser.contains("## The contract shared by every row")
                        && chooser.contains("## Choose semantic or numeric intent")
                        && !chooser.contains("source-excerpt:")
                        && !chooser.contains(FENCE + "java"));
    }

    @Test
    public void currentLearningRoutesDoNotPointBackToTheSupersededBasicCourse()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path frameworkRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        assertTrue("Root README must route directly to the current Build area",
                !readUtf8(repositoryRoot.resolve("README.md"))
                        .contains("Basic Mechanisms Robot.md"));

        List<Path> pages = new ArrayList<Path>();
        collectMarkdownFiles(frameworkRoot, pages);
        List<String> failures = new ArrayList<String>();
        for (Path page : pages) {
            if (page.getFileName().toString().equals("Basic Mechanisms Robot.md")) {
                continue;
            }
            if (readUtf8(page).contains("Basic Mechanisms Robot.md")) {
                failures.add(repositoryRelativePath(repositoryRoot, page));
            }
        }
        assertTrue("Current guides still link the superseded Basic course: " + failures,
                failures.isEmpty());
    }

    @Test
    public void testingGuideDefinesTheFiveLevelLadderAndExplanationGrammar()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path docsRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH).resolve("docs");
        String testing = readUtf8(docsRoot.resolve(
                "testing-calibration/How to test a Sushi component.md"));
        String testingHome = readUtf8(docsRoot.resolve("testing-calibration/README.md"));

        assertTrue("Testing philosophy must name owner, outside world, heartbeat, and evidence",
                testing.contains("Test the owner of one question")
                        && testing.contains("replace only the world outside that owner")
                        && testing.contains("preserve the")
                        && testing.contains("production heartbeat")
                        && testing.contains("claim only the evidence actually observed"));

        String[] levels = {
            "| 1. Semantic intent |",
            "| 2. Software-device scenario |",
            "| 3. Supplied managed slice |",
            "| 4. Maintainer regression |",
            "| 5. Physical bring-up, calibration, or experiment |"
        };
        for (String level : levels) {
            assertEquals("Evidence ladder row changed: " + level,
                    1, literalCount(testing, level));
        }
        assertEquals("Evidence ladder must contain exactly five numbered levels",
                5, matcherCount(Pattern.compile("(?m)^\\| [1-5]\\. ").matcher(testing)));

        String[] preamble = {
            "- **Question:**",
            "- **Keep real:**",
            "- **Replace:**",
            "- **Observe:**",
            "- **Cannot conclude:**"
        };
        for (String label : preamble) {
            assertEquals("Testing preamble label changed: " + label,
                    1, literalCount(testing, label));
        }

        String[] causalLabels = {
            "// ARRANGE:",
            "// REQUEST:",
            "// BEFORE HEARTBEAT:",
            "// HEARTBEAT:",
            "// INJECT EVIDENCE:",
            "// ASSERT:",
            "// NEXT GATE:"
        };
        for (String label : causalLabels) {
            assertEquals("Causal Java label changed: " + label,
                    1, literalCount(testing, label));
        }

        String[] postEvidence = {
            "- **Read the causal chain:**",
            "- **Proves:**",
            "- **Does not prove:**",
            "- **Next gate:**"
        };
        for (String label : postEvidence) {
            assertEquals("Post-evidence label changed: " + label,
                    1, literalCount(testing, label));
        }

        assertTrue("Student-facing complexity budget is missing",
                testing.contains("no more than two")
                        && testing.contains("35 executable lines")
                        && testing.contains("100–120 physical lines"));
        assertTrue("Maintainer-only test techniques must remain clearly supplied evidence",
                testing.contains("reflection-based")
                        && testing.contains("dynamic proxies")
                        && testing.contains("bytecode or annotation checks")
                        && testing.contains("not a")
                        && testing.contains("template a beginner must reverse engineer"));
        int firstExperiment = testingHome.indexOf(
                "[Hardware-free Reference Scenarios](<../examples/Hardware-free Reference Scenarios.md>)");
        int console = testingHome.indexOf(
                "[Using the tester console](<Using the Tester Console.md>)");
        assertTrue("Test & Tune home must route from software evidence to the console",
                firstExperiment >= 0 && console > firstExperiment);
        assertTrue("Test & Tune home must retain the deeper testing philosophy",
                testingHome.contains(
                        "[Testing philosophy](<How to test a Sushi component.md>)"));
    }

    @Test
    public void testAndTuneHasOneExecutableSourceOptionalOperationalSpine()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path frameworkRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        Path docsRoot = frameworkRoot.resolve("docs");
        Path testAndTune = docsRoot.resolve("testing-calibration");
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        String home = readUtf8(testAndTune.resolve("README.md"));
        String console = readUtf8(testAndTune.resolve("Using the Tester Console.md"));
        String scenarios = readUtf8(docsRoot.resolve(
                "examples/Hardware-free Reference Scenarios.md"));
        String actuator = readUtf8(testAndTune.resolve("Actuator Bring-up.md"));
        String tuning = readUtf8(testAndTune.resolve("Control Tuning Workflow.md"));
        String calibration = readUtf8(testAndTune.resolve("Robot Calibration Tutorials.md"));
        String guided = readUtf8(testAndTune.resolve("Guided Calibration Walkthroughs.md"));
        String actuatorSource = readUtf8(frameworkRoot.resolve(
                "tools/tester/ActuatorBringUpTester.java"));
        String velocityTunerSource = readUtf8(frameworkRoot.resolve(
                "integrations/panels/FtcVelocityControlPanelsTester.java"));
        String controlTuningAdaptersSource = readUtf8(frameworkRoot.resolve(
                "integrations/panels/ControlTuningAdapters.java"));
        String panelsHostSource = readUtf8(frameworkRoot.resolve(
                "integrations/panels/FtcPanelsTeleOpTesterOpMode.java"));
        String standardTestersSource = readUtf8(frameworkRoot.resolve(
                "tools/tester/StandardTesters.java"));
        String walkthroughBuilderSource = readUtf8(frameworkRoot.resolve(
                "tools/tester/calibration/CalibrationWalkthroughBuilder.java"));
        String referenceFlywheelSource = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/"
                        + "flywheel/ReferenceFlywheelMechanism.java"));
        String referenceTuningHostSource = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/"
                        + "ReferencePanelsTuningOpMode.java"));

        assertEquals("Test & Tune navigation inventory or order changed",
                TEST_AND_TUNE_NAV_TARGETS,
                navTargets(navAreaBlock(config, "Test & Tune")));
        assertTrue("Test & Tune home must remain a short chooser",
                proseWordCount(home) <= 450);
        assertContainsAll("Test & Tune outcome order", home,
                "start without hardware", "using the tester console",
                "choose one physical question", "generic tester does not save",
                "build and run", "ftc robot configuration");
        assertContainsAll("Exact Test & Tune navigation labels", config,
                "\"First software experiment\" = \"docs/examples/Hardware-free Reference Scenarios.md\"",
                "\"Using the tester console\" = \"docs/testing-calibration/Using the Tester Console.md\"",
                "\"Actuator bring-up\" = \"docs/testing-calibration/Actuator Bring-up.md\"",
                "\"Robot calibration\" = \"docs/testing-calibration/Robot Calibration Tutorials.md\"",
                "\"Control tuning\" = \"docs/testing-calibration/Control Tuning Workflow.md\"");

        assertTrue("Tester-console first contact exceeds its operational budget",
                proseWordCount(console) <= 850);
        assertContainsAll("Fixed tester-console ownership", console,
                "fw: testers (driver station)", "physical ftc gamepads only",
                "fw: testers (panels)", "panels virtual gamepads only",
                "fixed and mutually exclusive", "inputs are never merged",
                "telemetry appears");
        assertContainsAll("Panels connection and recovery", console,
                "before pressing init", "192.168.49.1:8001", "192.168.43.1:8001",
                "telemetry", "combined gamepad", "use exactly one panels client",
                "last panels client disconnects", "another client remains connected",
                "terminally fail-stops", "does not rearm", "fresh",
                "driver station stop is the emergency stop");
        assertContainsAll("Exact tester home", console,
                "framework tester home", "hw: actuator bring-up",
                "framework: calibration & localization",
                "advanced: hardware diagnostics", "dpad up/down", "a", "back");
        int actuatorMenu = console.indexOf("1. **HW: Actuator Bring-up**");
        int calibrationMenu = console.indexOf(
                "2. **Framework: Calibration & Localization**");
        int advancedMenu = console.indexOf("3. **Advanced: Hardware Diagnostics**");
        assertTrue("Tester-console menu labels must stay in implementation order",
                actuatorMenu >= 0 && calibrationMenu > actuatorMenu
                        && advancedMenu > calibrationMenu);
        String[] panelMappings = {
            "| Cross | A |", "| Circle | B |", "| Square | X |",
            "| Triangle | Y |", "| Options | START |", "| Share | BACK |"
        };
        for (String mapping : panelMappings) {
            assertTrue("Missing Panels-to-tester mapping " + mapping,
                    console.contains(mapping));
        }
        assertContainsAll("Panels implementation mappings", panelsHostSource,
                "gamepad.a = manager.getcross()", "gamepad.b = manager.getcircle()",
                "gamepad.x = manager.getsquare()", "gamepad.y = manager.gettriangle()",
                "gamepad.start = manager.getoptions()", "gamepad.back = manager.getshare()",
                "this(inputsource, panelsclientrequirement.at_least_one)",
                "connectedclientcount >= 1",
                "panels tester input disconnected");
        assertContainsAll("Standard tester implementation menu", standardTestersSource,
                "framework tester home", "hw: actuator bring-up",
                "framework: calibration & localization",
                "advanced: hardware diagnostics");
        int sourceActuatorMenu = standardTestersSource.indexOf("\"HW: Actuator Bring-up\"");
        int sourceCalibrationMenu = standardTestersSource.indexOf(
                "\"Framework: Calibration & Localization\"");
        int sourceAdvancedMenu = standardTestersSource.indexOf(
                "\"Advanced: Hardware Diagnostics\"");
        assertTrue("Standard tester menu source order changed",
                sourceActuatorMenu >= 0 && sourceCalibrationMenu > sourceActuatorMenu
                        && sourceAdvancedMenu > sourceCalibrationMenu);

        int starterStart = scenarios.indexOf(
                "## Start here: ask for intake, then advance one heartbeat");
        int scenarioChooser = scenarios.indexOf("## Choose the next scenario by question");
        assertTrue("Starter experiment must precede the scenario chooser",
                starterStart >= 0 && scenarioChooser > starterStart);
        int starterPrerequisite = scenarios.indexOf(
                "[Build and Run](<../getting-started/Build and Run.md>)");
        assertTrue("Direct scenario navigation must retain its setup prerequisite",
                starterPrerequisite >= 0 && starterPrerequisite < starterStart);
        String starter = scenarios.substring(starterStart, scenarioChooser);
        assertTrue("Starter experiment exceeds its first-use prose budget",
                proseWordCount(starter) <= 650);
        assertContainsAll("Executable Starter experiment", starter,
                "startermechanismlessontest", "--tests",
                "question:", "keep real:", "replace:", "observe:",
                "cannot conclude:", "// arrange:", "// request:",
                "// heartbeat:", "read the causal chain:", "proves:",
                "does not prove:", "next gate:", "using the tester console");
        assertContainsAll("Starter write evidence stays exact", starter,
                "zero motor writes before the update",
                "cached and recorded power after the update");

        int actuatorFirstStart = actuator.indexOf("## Before starting the OpMode");
        int actuatorMotorStart = actuator.indexOf("## DC motor: direction");
        assertTrue("Actuator first contact exceeds its operational budget",
                actuatorFirstStart >= 0 && actuatorMotorStart > actuatorFirstStart
                        && proseWordCount(actuator.substring(
                        actuatorFirstStart, actuatorMotorStart)) <= 700);
        assertContainsAll("Actuator active defaults", actuator,
                "within `0.05..0.30`", "in `0.05` steps",
                "within `0.01..0.25`", "in `0.01` steps",
                "at most `0.005`", "two fresh a presses", "logical command `0.5`",
                "sushiactuatorbringup");
        assertContainsAll("Actuator implementation defaults", actuatorSource,
                "power_min = 0.05", "power_max = 0.30", "power_step = 0.05",
                "servo_jog_rate_initial_per_sec = 0.05",
                "servo_jog_rate_min_per_sec = 0.01",
                "servo_jog_rate_max_per_sec = 0.25",
                "servo_jog_rate_step_per_sec = 0.01",
                "servo_max_step_per_cycle = 0.005",
                "servo_bootstrap_command = 0.5",
                "requirepreparedanddisarmed(\"change direction\")",
                "requirepreparedanddisarmed(\"capture an endpoint\")");
        assertContainsAll("Actuator preparation and cleanup boundary", actuator,
                "after a has prepared the device and while disarmed",
                "best-effort attempts zero", "physical state is uncertain",
                "wait for the linkage to become visibly stationary",
                "the captured number proves which command was submitted");

        int tuningFirstStart = tuning.indexOf("## First experiment: one velocity segment");
        int tuningArchitectureStart = tuning.indexOf(
                "## Choose the workflow that matches production");
        assertTrue("Velocity first experiment exceeds its operational budget",
                tuningFirstStart >= 0 && tuningArchitectureStart > tuningFirstStart
                        && proseWordCount(tuning.substring(
                        tuningFirstStart, tuningArchitectureStart)) <= 800);
        assertContainsAll("Velocity first experiment", tuning,
                "first experiment: one velocity segment", "target", "measurement",
                "error", "controller gains", "segment", "acceptance rule",
                "flywheelleft", "flywheelright", "5000.0", "100.0",
                "target `0`", "autostopaftersec = 5.0", "update all",
                "own smaller reviewed session range",
                "written run card", "does not display it before motion",
                "fw reference: tuning (panels)", "@disabled",
                "capturing", "cold start wait", "zero wait", "may appear",
                "a wait can be skipped", "current segment",
                "aggregate", "member.1.*", "member.2.*",
                "divergent_initial_readbacks", "do not press a",
                "individual pre-apply values are not shown",
                "five seconds from accepted segment start",
                "best-effort attempts plant stop", "b requests zero",
                "driver station stop");
        assertContainsAll("Velocity tuner active defaults", velocityTunerSource,
                "default_auto_stop_after_sec = 5.0",
                "initialtesttarget = this.testtargetrange.clamp(0.0)",
                "field_target", "field_auto_stop_sec",
                "capturing: waiting for the complete active draft",
                "cold start wait: finite feedback and plant attarget(0.0) required");
        assertContainsAll("Grouped velocity evidence stays visible", controlTuningAdaptersSource,
                "divergent_initial_readbacks", "first_ordered_member_only",
                "member.nativeMeasurement()", "member.nativeError()",
                "member.nativeTolerance()", "member.withinMappedPlantTolerance()");
        assertContainsAll("Reference velocity example defaults", referenceFlywheelSource,
                "leftmotorname = \"flywheelleft\"",
                "rightmotorname = \"flywheelright\"",
                "maximumvelocitytickspersec = 5000.0",
                "velocitytolerancetickspersec = 100.0");
        assertContainsAll("Reference tuning host identity and input policy",
                referenceTuningHostSource,
                "@teleop(name = \"fw reference: tuning (panels)\"",
                "@disabled", "panelsclientrequirement.exactly_one",
                "scalarrange.bounded(0.0, flywheels.maximumvelocitytickspersec)");
        assertContainsAll("Position tuning remains advanced", tuning,
                "no maintained robot/example position-tuning opmode",
                "advanced adaptation reference", "reference policy", "hold behavior");

        int calibrationStart = calibration.indexOf("## Choose your stage");
        int calibrationDetails = calibration.indexOf("## Before you start");
        assertTrue("Calibration first contact must be a bounded stage chooser",
                calibrationStart >= 0 && calibrationDetails > calibrationStart
                        && proseWordCount(calibration.substring(
                        calibrationStart, calibrationDetails)) <= 650);
        assertContainsAll("Generic-versus-configured calibration boundary", calibration,
                "probe and record", "rebuild and verify",
                "source access is optional for stage 1 only", "independent fact probes",
                "project supplies a fresh configured verifier",
                "does not generate a robot-specific verifier",
                "reconstructs a fresh tester `config` from framework defaults",
                "identity camera mount", "do not persist or propagate results",
                "cannot verify production configuration",
                "record -> rebuild -> fresh robot-configured tester -> verify",
                "menu help says “verify apriltag detections and the field pose solve.”",
                "messages name missing prerequisites");
        assertContainsAll("Exact standard calibration entries", calibration,
                "calib: camera mount (webcam)", "calib: camera mount (limelight)",
                "loc: apriltag localization (webcam)",
                "loc: apriltag localization (limelight)",
                "calib: pinpoint axis check", "calib: pinpoint pod offsets",
                "loc: pinpoint + field corrections (webcam)",
                "loc: pinpoint + field corrections (limelight)");
        assertContainsAll("Advanced calibration routes stay optional", calibration,
                "optional advanced route", "high-resolution external encoder velocity comparison",
                "optional advanced: powered and vision-assisted pod offsets",
                "optional ekf comparison", "does **not** register an ekf entry",
                "optional advanced: guided suite construction");
        String cameraCalibration = sectionBetween(calibration,
                "## Camera mount", "## AprilTag-only localization check");
        assertContainsAll("Camera mount teaches evidence and calibration boundaries", cameraCalibration,
                "**extrinsics**", "**intrinsics**", "**shooter/intake alignment**",
                "known robot z/pitch/roll stay zero", "a **batch**", "a camera **frame**",
                "not the previous screen's preview", "repeated or older capture timestamps",
                "wins", "**strictly after**", "driver station start", "gamepad start",
                "**historical**", "no copy/paste average is printed",
                "| action / image | accepted count | why |");
        String axisCalibration = sectionBetween(calibration,
                "## Pinpoint axis directions", "## Pinpoint pod offsets");
        assertContainsAll("Axis directions explain the active sign experiment", axisCalibration,
                "reported sign", "not commands to drive", "end value minus its start value",
                "magnitude (size ignoring sign)",
                "minTranslationInches = 6.0", "minRotationDeg = 20.0",
                "before constructing", "copies the settings at construction",
                "without changing its facing", "check rotation last", "press x again",
                "current loop cycle", "moving only along the other axis does not count");
        assertContainsAll("Axis recommendations are relative to the captured configuration",
                axisCalibration,
                "| `FORWARD` | Positive | Keep `FORWARD` |",
                "| `FORWARD` | Negative | Change to `REVERSED` |",
                "| `REVERSED` | Positive | Keep `REVERSED` |",
                "| `REVERSED` | Negative | Change to `FORWARD` |",
                "cfg.pinpoint.forwardPodDirection", "cfg.pinpoint.strafePodDirection",
                "last completed samples", "historical results", "no new direction recommendation",
                "does not apply or save", "fresh robot-configured tester",
                "cannot tell which way a person actually pushed", "sensor that measures turning",
                "yawScalar = null", "factory calibration", "must stay positive");
        assertContainsAll("Generic pod-offset solve is gated by its active defaults", calibration,
                "can be the rookie manual path only when",
                "`gobilda_4_bar_pod` resolution", "both `forward`",
                "actively applies those reconstructed defaults",
                "do not accept a generic offset solve",
                "fresh robot-configured calibrator");
        String podCalibration = sectionBetween(calibration,
                "## Pinpoint pod offsets", "## Pinpoint plus field corrections");
        assertContainsAll("Pod geometry explains what the sample and recommendation mean", podCalibration,
                "reference point", "point whose position pinpoint reports",
                "incorrect offsets", "does not physically move or recenter",
                "real start-to-end movement", "cannot distinguish it from incorrect pod geometry",
                "absolute replacement offsets", "not adjustments to add",
                "difference from the configured values", "not the offsets themselves");
        String assistedCalibration = sectionBetween(calibration,
                "## Optional advanced: powered and vision-assisted pod offsets",
                "## Optional EKF comparison");
        assertContainsAll("Assisted calibration explains matched-time evidence", assistedCalibration,
                "capture time", "delivery time", "raw odometry", "same capture time",
                "exact", "interpolated", "robot-at-start axes",
                "search-only", "strictly advancing capture timestamps",
                "three reads of one frame still count as one",
                "one matched pair", "missing or unmatched assisted end",
                "no recommendation", "fresh explicitly unassisted attempt",
                "cfg.autoComputeAfterAutoSample = false", "final a must select a matched end",
                "accTitle:", "accDescr:", "**text version:**", "software evidence only");
        assertContainsAll("Assisted history has visible independent bounds", assistedCalibration,
                "config.assistOdometryHistory", "PlanarPoseHistory.Config.defaults()",
                "`retentionSec` | `0.50 s`", "`maxSamples` | `128`",
                "`maxInterpolationGapSec` | `0.10 s`",
                "`maxInterpolationTranslationInches` | `12.0 in`",
                "`maxInterpolationYawRad` | `Math.PI / 2.0`",
                "before constructing", "without a vision factory", "draft is ignored",
                "cfg.aprilTags.maxDetectionAgeSec", "does not create missing odometry history");
        assertContainsAll("Generic calibration implementation boundary", standardTestersSource,
                "calib: camera mount (webcam)", "calib: camera mount (limelight)",
                "loc: apriltag localization (webcam)",
                "loc: apriltag localization (limelight)",
                "pinpointaxisdirectiontester.config cfg = pinpointaxisdirectiontester.config.defaults()",
                "pinpointpodoffsetcalibrator.config cfg = pinpointpodoffsetcalibrator.config.defaults()",
                "cfg.cameraMount = CameraMountConfig.identity()",
                "GlobalEstimatorMode.FUSION");
        assertFalse("The standard tester registry must not imply a generic EKF entry",
                standardTestersSource.contains("GlobalEstimatorMode.EKF"));

        assertContainsAll("Guided calibration stays an honest advanced mapping", guided,
                "architecture reference", "does not save calibration results",
                "guidedWalkthrough(profile)", "supplier<teleoptester>",
                "pinpointAxesVerified", "pinpointOffsetsVerified", "no tag authorizes powered motion",
                "fresh inactive tester", "not another mount calibrator", "ownership checklist",
                "record the result, edit the profile, rebuild");
        assertFalse("Guided calibration must use the maintained assembly rather than a stub",
                guided.contains("teaching-shape"));
        assertFalse("Guided calibration must not retain fictional RobotCalibration calls",
                guided.contains("RobotCalibration::"));
        assertContainsAll("Guided calibration example uses the real builder signature",
                walkthroughBuilderSource, "public int addStep(String label",
                "Supplier<CalibrationStatus> status",
                "Supplier<TeleOpTester> testerFactory");
    }

    /** Maintainer contract checks, not evidence that a robot passed calibration. */
    @Test
    public void calibrationAcceptanceExtendsOneSharedLabCard() throws IOException {
        Path docsRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH).resolve("docs");
        String calibration = readUtf8(docsRoot.resolve(
                "testing-calibration/Robot Calibration Tutorials.md"));
        String record = sectionBetween(calibration,
                "## Keep one calibration record", "## Actuator direction and safe endpoints");
        assertEquals("Calibration has one canonical extension, not several competing forms",
                1, literalCount(calibration, "## Keep one calibration record"));
        assertContainsAll("Calibration extends the shared card instead of duplicating it", record,
                "Subsystem Experiments.md#copyable-lab-card-and-results-sheet",
                "same record", "question, safety plan", "trial table", "acceptance criterion",
                "not the advanced flywheel example", "not applicable", "no camera calibration data",
                "outside the Robot Controller", "testers do not save it");
        assertFalse("The extension must not duplicate the lab card's safety form",
                record.contains("Emergency STOP owner:"));
        assertFalse("The extension must not duplicate the lab card's complete trial table",
                record.contains("| Trial | Starting condition | Command |"));
        assertContainsAll("Calibration facts identify the exact candidate-to-production handoff", record,
                "**candidate**", "**revision**", "**Independent validation**", "uncommitted edits",
                "Device identity", "actual OpMode", "generic or configured",
                "reference point", "coordinate frame and units",
                "Current configured value -> exact candidate assignment",
                "fitting trial IDs", "held-out validation trial IDs", "Canonical source file + field",
                "Reviewed candidate trial", "Rebuilt and deployed revision", "deployment confirmation",
                "Fresh configured tester", "captured values checked", "Production-owner check",
                "same configuration confirmed", "Decision scope", "still-unverified uses");
        assertContainsAll("Calibration acceptance remains a scoped human evidence decision", record,
                "Before trials", "no physical pass threshold", "only the reviewed trial",
                "not ordinary robot use", "fresh configured suite", "separate production-owner check",
                "**Accept / Revise and repeat / Reject**", "source edit", "configured verifier",
                "independent reference", "**blocked**", "do not mark acceptance",
                "Keep rejected results", "retune", "new independent checks", "needs retest",
                "hardware, pod setting, mount, layout, image size, target model, or configuration",
                "Preserve the old record", "affected robot-owned acknowledgement",
                "blocked camera check does not invalidate a verified encoder sign",
                "never certifies the whole robot");
        String axisRecord = sectionBetween(calibration,
                "### Example record: axis directions", "## Pinpoint pod offsets");
        assertContainsAll("Direction record checks the instructed motion against the captured sign",
                axisRecord, "physical hand motion", "captured encoder setting", "measured delta",
                "`REVERSED`", "negative X delta", "change to `FORWARD`", "fresh configured tester",
                "left and CCW", "No real run", "does not prove distance accuracy");
        String podRecord = sectionBetween(calibration,
                "### Example record: manual pod offsets", "## Pinpoint plus field corrections");
        assertContainsAll("Manual pod record preserves geometry and independent placement evidence",
                podRecord, "pod resolution/directions", "fixed robot reference point", "floor mark",
                "physical recentering", "heading change", "current offsets", "absolute replacement",
                "rebuilt configured trial", "not measured acceptance", "independently checked",
                "zero software is not recentering", "failed trials", "production-owner verification");
        for (String companion : new String[]{"examples/Subsystem Experiments.md",
                "testing-calibration/Add Calibration Testers to Your Robot.md",
                "testing-calibration/Add Vision to Your Calibration Suite.md",
                "testing-calibration/Guided Calibration Walkthroughs.md"}) {
            assertTrue(companion + " must route to the same calibration record",
                    readUtf8(docsRoot.resolve(companion)).contains(
                            "Robot Calibration Tutorials.md#keep-one-calibration-record"));
        }
    }

    /** Reads the published numbers; no duplicate expected-coordinate fixture certifies the table. */
    @Test
    public void cameraValidationSeparatesHeldOutEvidenceAndChecksIllustrativeErrors()
            throws IOException {
        String calibration = readUtf8(repositoryRoot().resolve(FRAMEWORK_DOCS_PATH)
                .resolve("docs/testing-calibration/Robot Calibration Tutorials.md"));
        String camera = sectionBetween(calibration,
                "### Compare camera estimates with measured locations",
                "## AprilTag-only localization check");
        assertContainsAll("Optional camera validation distinguishes fitting from independent checks",
                camera, "Optional camera depth", "**Fitting**", "**Held-out**",
                "same", "#keep-one-calibration-record", "not another form or tester",
                "mount-fitting batch separately", "image size", "intrinsics", "pipeline",
                "tag IDs/size/layout", "candidate mount/revision", "Freeze that candidate",
                "rebuild/deploy", "configured AprilTag-localization tester",
                "independently measured stationary placements", "field X/Y/yaw", "fieldToRobot",
                "Pose age", "inches and degrees", "radians", "new validation placements");
        assertContainsAll("Existing displays must not be promoted into independent camera evidence",
                camera, "**residual**", "captured-average mount", "Avg residual", "Range check",
                "not independently measured field errors", "sample count does not certify",
                "distinct fresh images", "repeated button presses");
        assertContainsAll("Projection needs an actual configured source and separate tool evidence",
                camera, "**Projection**", "no maintained generic floor-object projection-validation tester",
                "configured observation/display source", "**blocked**", "until supplied",
                "AprilTag screens do not verify ball locations", "target point and assumed height",
                "image capture time", "robot pose at that capture time", "same frame",
                "localization error can contribute", "does not isolate a camera fault",
                "shooter/intake", "separate tool check", "pickup or shot");
        assertContainsAll("The numeric illustration has visible assumptions and no physical pass claim",
                camera, "invented arithmetic", "not a physical run or pass", "`M1`", "`F1`",
                "none of `V1`", "independently known field pose `(0, 0, 0)`",
                "field +X is forward and +Y is left", "`2 in`", "not a recommended height",
                "neither configuration values nor permission", "subtract measured from estimated",
                "dX = estimated X - measured X", "sqrt(dX*dX + dY*dY)", "systematic offset",
                "cannot identify its cause", "heading errors", "do not mix mount, robot-pose");
        assertIllustrativeCameraValidationTable(camera);
    }

    /** These damaged published rows prove the arithmetic check cannot pass by skipping bad data. */
    @Test
    public void cameraValidationArithmeticRejectsMissingMalformedAndInconsistentRows()
            throws IOException {
        String calibration = readUtf8(repositoryRoot().resolve(FRAMEWORK_DOCS_PATH)
                .resolve("docs/testing-calibration/Robot Calibration Tutorials.md"));
        String camera = sectionBetween(calibration,
                "### Compare camera estimates with measured locations",
                "## AprilTag-only localization check");
        String firstRow = null;
        for (String line : camera.split("\\R")) {
            if (line.startsWith("| V1 |")) firstRow = line;
        }
        assertTrue("The illustrative V1 row must exist before testing damaged variants", firstRow != null);
        String[] cells = firstRow.split("\\|", -1);
        String[] invalid = {
                camera.replace(firstRow, ""),
                camera.replace(firstRow, firstRow + "\n" + firstRow),
                camera.replace(firstRow, firstRow + "\n| X4 | malformed |"),
                camera.replace(firstRow, firstRow + "\n" + firstRow.replace("V1", "X4")),
                camera.replace(firstRow, "| V1 | incomplete |"),
                camera.replace(firstRow, firstRow.replace(cells[3], " (24) ")),
                camera.replace(firstRow, firstRow.replace(cells[3], " (NaN, 12) ")),
                camera.replace(firstRow, firstRow.replace(cells[3], " (Infinity, 12) ")),
                camera.replace(firstRow, firstRow.replace(cells[5], " (0, 0) ")),
                camera.replace(firstRow, firstRow.replace(cells[6], " -1 "))
        };
        for (int i = 0; i < invalid.length; i++) {
            boolean rejected = false;
            try {
                assertIllustrativeCameraValidationTable(invalid[i]);
            } catch (AssertionError expected) {
                rejected = true;
            }
            assertTrue("Damaged camera table variant " + i + " must be rejected", rejected);
        }
    }

    @Test
    public void calibrationIntegrationUsesIndependentSourceBackedLessons() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path docsRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH).resolve("docs");
        Path calibrationRoot = docsRoot.resolve("testing-calibration");
        String basicFile = "Add Calibration Testers to Your Robot.md";
        String[] lessonFiles = {
                basicFile,
                "Add Vision to Your Calibration Suite.md",
                "Enable Powered Calibration.md"
        };
        String examplePath = "TeamCode/src/main/java/edu/ftcsushi/robots/examples/calibration/";
        for (String lesson : lessonFiles) {
            String markdown = readUtf8(calibrationRoot.resolve(lesson));
            assertContainsAll(lesson + " has a declared learning purpose", markdown,
                    "test & tune", "learning mode", "before", "rebuild");
            assertTrue(lesson + " must use the independent compiling example",
                    markdown.contains("<!-- source-excerpt: " + examplePath));
            assertTrue(lesson + " needs exact API lookup", markdown.contains(PUBLISHED_API_ROOT));
            assertTrue(lesson + " needs a separately labeled complete-source lookup",
                    markdown.toLowerCase(Locale.ROOT).contains("complete source"));
            assertFalse(lesson + " must not teach an invented assembly fragment",
                    markdown.contains("teaching-shape"));
            assertFalse(lesson + " must not rely on a production application",
                    Pattern.compile("edu\\.ftcsushi\\.robots\\.(?!examples\\.)")
                            .matcher(markdown).find());
            assertEquals(lesson + " must give every Java excerpt checked-in provenance",
                    literalCount(markdown, FENCE + "java"),
                    literalCount(markdown, "<!-- source-excerpt:"));
        }

        String basic = readUtf8(calibrationRoot.resolve(basicFile));
        assertContainsAll("The basic integration names all three maintained roles", basic,
                "CalibrationRobotProfile", "CalibrationTesters", "CalibrationTestersOpMode",
                "FtcTeleOpTesterOpMode", "FW Example: Calibration Testers", "@Disabled");
        assertContainsAll("The basic integration explains fresh ownership before optional depth", basic,
                "a **profile**", "a **factory**", "lambda", "supplier<teleoptester>",
                "does not start another thread", "fresh inactive owner",
                "do not keep a second calibration-only copy", "supported devices",
                "accTitle:", "accDescr:", "**text version:**", "FTC STOP");
        String vision = readUtf8(calibrationRoot.resolve(lessonFiles[1]));
        assertContainsAll("Optional vision keeps hardware choice and evidence explicit", vision,
                "a **backend**", "a **pose**", "a **frame**", "an **apriltag**",
                "Function<String, AprilTagCameraFactory>", "CameraMountConfig.identity()",
                "cameraMountAccepted", "currentGameFieldFixed()", "APRILTAG_POSE", "FUSION",
                "maxDetectionAgeSec = 0.35", "0.25 s", "receipt staleness",
                "estimated capture age", "not two cutoffs on the same age",
                "fresh configured", "physical accuracy", "chosen robot reference point",
                "z/pitch/roll remain zero", "image/lens", "extrinsics", "intrinsics");
        String powered = readUtf8(calibrationRoot.resolve(lessonFiles[2]));
        assertContainsAll("Optional powered integration cannot imply passive or accepted hardware",
                powered, "poweredMotionReviewed", "cameraMountAccepted", "before constructing",
                "enableAutoTagSearchAtStart = false", "enableAutoTagSearchAtEnd = false",
                "never silently falls back", "Disabling tag searches does **not**",
                "Y still requests", "right stick", "left stick", "both searches are off",
                "autoComputeAfterAutoSample = true", "final A", "cooperatively",
                "watchdog", "FTC STOP", "a menu status must never authorize motion");
        assertTrue("The basic integration must explain optional local verification",
                ShellCommandTabs.validate(basicFile, basic).hasEquivalentGradleTestSelector(
                        "edu.ftcsushi.robots.examples.calibration.*"));
        for (String doorway : new String[]{"README.md", "testing-calibration/README.md",
                "testing-calibration/Robot Calibration Tutorials.md",
                "testing-calibration/Guided Calibration Walkthroughs.md",
                "examples/Subsystem Experiments.md"}) {
            assertTrue(doorway + " must route configured-tester authors to the canonical lesson",
                    readUtf8(docsRoot.resolve(doorway)).contains(basicFile));
        }
        for (String type : new String[]{"CalibrationRobotProfile", "CalibrationTesters",
                "CalibrationTestersOpMode"}) {
            String source = readUtf8(repositoryRoot.resolve(examplePath + type + ".java"));
            assertFalse(type + " must remain independent of production applications",
                    Pattern.compile("edu\\.ftcsushi\\.robots\\.(?!examples\\.)")
                            .matcher(source).find());
        }
    }

    @Test
    public void referenceHasEightCategoriesAndMatchingJavadocGroups() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path docsRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        Path referenceRoot = docsRoot.resolve("docs/reference");
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        String referenceHome = readUtf8(referenceRoot.resolve("README.md"));
        String build = readUtf8(repositoryRoot.resolve("TeamCode/build.gradle"));

        assertEquals("Reference Markdown inventory changed",
                REFERENCE_MARKDOWN_FILES, markdownFileNames(referenceRoot));
        assertEquals("Reference navigation inventory or order changed",
                REFERENCE_NAV_TARGETS, navTargets(navAreaBlock(config, "Reference")));

        for (String category : REFERENCE_CATEGORY_FILES) {
            String categoryText = readUtf8(referenceRoot.resolve(category));
            assertTrue("Reference index does not link " + category,
                    referenceHome.contains("(<" + category + ">)"));
            assertTrue(category + " must start from ordinary or explicit entry points",
                    categoryText.contains("## Ordinary entry points")
                            || categoryText.contains("## Entry points"));
            assertTrue(category + " must link exact generated API documentation",
                    categoryText.contains(PUBLISHED_API_ROOT));
            assertTrue(category + " must preserve one concise truth reminder",
                    categoryText.contains("## Remember"));
        }

        List<String> groupNames = new ArrayList<String>();
        Matcher groups = Pattern.compile("options\\.group\\('([^']+)'").matcher(build);
        while (groups.find()) {
            groupNames.add(groups.group(1));
        }
        assertEquals("Javadoc categories must parallel eight Reference areas plus examples",
                JAVADOC_GROUPS, groupNames);
        assertTrue("Strict Javadocs must cover framework and maintained examples",
                build.contains("tasks.register('sushiJavadocs', Javadoc)")
                        && build.contains("include('edu/ftcsushi/fw/**/*.java')")
                        && build.contains("include('edu/ftcsushi/robots/examples/**/*.java')")
                        && build.contains("options.addBooleanOption('Werror', true)"));
    }

    @Test
    public void supersededCourseUrlsAreSmallSearchExcludedRedirects() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path docsRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH).resolve("docs");
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        Map<String, String> redirects = new LinkedHashMap<String, String>();
        redirects.put("getting-started/Basic Mechanisms Robot.md",
                "../build/README.md");
        redirects.put("getting-started/First Pedro Auto.md",
                "../build/First Pedro Auto.md");

        for (Map.Entry<String, String> redirect : redirects.entrySet()) {
            Path page = docsRoot.resolve(redirect.getKey());
            String markdown = readUtf8(page);
            PageMetadata metadata = pageMetadata(page);

            assertTrue(redirect.getKey() + " must be search-excluded",
                    metadata.hasFrontMatter && metadata.searchExcluded);
            assertTrue(redirect.getKey() + " must have no area tag", metadata.tags.isEmpty());
            assertTrue(redirect.getKey() + " is no longer a small redirect",
                    Files.readAllLines(page, StandardCharsets.UTF_8).size() <= 15
                            && proseWordCount(page) <= 80);
            assertTrue(redirect.getKey() + " must point to its current Build destination",
                    markdown.contains("(<" + redirect.getValue() + ">)"));
            assertTrue(redirect.getKey() + " must not carry old course content",
                    !markdown.contains("source-excerpt:")
                            && !markdown.contains("source-file:")
                            && !markdown.contains(FENCE + "java")
                            && !markdown.contains("**Learning mode:**"));
            assertTrue(redirect.getKey() + " must not remain in navigation",
                    !config.contains(redirect.getKey()));
        }
    }

    @Test
    public void currentGuidesDoNotNameDeletedExampleAuthorities() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path docsRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        List<Path> pages = new ArrayList<Path>();
        collectMarkdownFiles(docsRoot, pages);
        List<String> failures = new ArrayList<String>();

        for (Path page : pages) {
            String markdown = readUtf8(page);
            for (String symbol : DELETED_EXAMPLE_SYMBOLS) {
                if (Pattern.compile("\\b" + Pattern.quote(symbol) + "\\b")
                        .matcher(markdown).find()) {
                    failures.add(repositoryRelativePath(repositoryRoot, page)
                            + ": names deleted example " + symbol);
                }
            }
        }
        assertTrue("Deleted example references remain: " + failures, failures.isEmpty());
    }

    @Test
    public void sushiFrameworkIdentityAndDocumentationBoundaryStayExplicit() throws IOException {
        Path repositoryRoot = repositoryRoot();
        String rootReadme = readUtf8(repositoryRoot.resolve("README.md"));
        String namespaceReadme = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/README.md"));
        Path frameworkRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        String frameworkHome = readUtf8(frameworkRoot.resolve("README.md"));
        String docsHome = readUtf8(frameworkRoot.resolve("docs/README.md"));
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        String build = readUtf8(repositoryRoot.resolve("TeamCode/build.gradle"));
        List<String> doorwayFailures = new ArrayList<String>();

        assertTrue("Root README must identify Sushi as the framework",
                rootReadme.startsWith("# Sushi framework"));
        assertTrue("Namespace README must identify Sushi as the framework",
                namespaceReadme.startsWith("# Sushi framework"));
        assertTrue("Framework doorway must identify Sushi",
                frameworkHome.contains("# Welcome to Sushi"));
        assertTrue("Documentation hub must identify Sushi",
                docsHome.contains("# Guide map")
                        && docsHome.contains("Sushi"));
        requireOrdered(rootReadme, "Repository README", doorwayFailures,
                "Framework Overview.md>)",
                "First Software Tour.md>)",
                "docs/README.md>)",
                "docs/build/README.md>)");
        requireOrdered(namespaceReadme, "Namespace README", doorwayFailures,
                "Framework Overview.md>)",
                "First Software Tour.md>)",
                "fw/docs/README.md>)",
                "fw/docs/build/README.md>)");
        for (String doorway : Arrays.asList(rootReadme, namespaceReadme)) {
            assertTrue("Repository doorways must offer setup as an optional activity",
                    doorway.contains("Build and Run.md>)")
                            && doorway.toLowerCase(Locale.ROOT).contains("optional"));
        }
        assertTrue("Repository doorways disagree with the Get Started route: "
                + doorwayFailures, doorwayFailures.isEmpty());
        assertTrue("Documentation site must use the Sushi identity and framework docs root",
                config.contains("site_name = \"Sushi Framework\"")
                        && configuredDocsRoot(repositoryRoot, config).equals(frameworkRoot)
                        && !config.contains("= \"fw/")
                        && !config.contains("ftcphoenix"));
        assertTrue("Strict API documentation must use the Sushi task and title",
                build.contains("tasks.register('sushiJavadocs', Javadoc)")
                        && build.contains("Sushi Framework API")
                        && !build.contains("phoenixJavadocs"));
        assertTrue("The six current area homes must exist",
                Files.isRegularFile(frameworkRoot.resolve("docs/getting-started/Build and Run.md"))
                        && Files.isRegularFile(frameworkRoot.resolve(
                                "docs/getting-started/First Software Tour.md"))
                        && Files.isRegularFile(frameworkRoot.resolve(
                                "docs/getting-started/Beginner's Guide.md"))
                        && Files.isRegularFile(frameworkRoot.resolve("docs/build/README.md"))
                        && Files.isRegularFile(frameworkRoot.resolve(
                                "docs/testing-calibration/README.md"))
                        && Files.isRegularFile(frameworkRoot.resolve("docs/advanced/README.md"))
                        && Files.isRegularFile(frameworkRoot.resolve("docs/reference/README.md")));
        assertTrue("Former framework namespace roots must not remain",
                !Files.exists(repositoryRoot.resolve("TeamCode/src/main/java/edu/ftcphoenix"))
                        && !Files.exists(repositoryRoot.resolve(
                                "TeamCode/src/test/java/edu/ftcphoenix")));

        assertNoStaleFrameworkBranding(
                repositoryRoot,
                frameworkRoot,
                Arrays.asList(
                        repositoryRoot.resolve("README.md"),
                        repositoryRoot.resolve(
                                "TeamCode/src/main/java/edu/ftcsushi/README.md"),
                        repositoryRoot.resolve("AGENTS.md"),
                        repositoryRoot.resolve(
                                ".agents/skills/execute-framework-improvements/SKILL.md"),
                        repositoryRoot.resolve(
                                ".agents/skills/execute-framework-improvements/agents/openai.yaml"),
                        repositoryRoot.resolve(".github/workflows/docs-site.yml"),
                        repositoryRoot.resolve("TeamCode/build.gradle"),
                        repositoryRoot.resolve("requirements-docs.txt"),
                        repositoryRoot.resolve("zensical.toml")));

        assertNoProductionApplicationReferences(
                repositoryRoot,
                frameworkRoot,
                Arrays.asList(
                        repositoryRoot.resolve(
                                "TeamCode/src/main/java/edu/ftcsushi/README.md"),
                        repositoryRoot.resolve("TeamCode/build.gradle")));
    }

    @Test
    public void currentTrackerGuidanceDoesNotDependOnTheProductionApplication()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        List<String> lines = Files.readAllLines(
                repositoryRoot.resolve("FRAMEWORK_IMPROVEMENT_TRACKER.md"),
                StandardCharsets.UTF_8);
        Pattern applicationReference = Pattern.compile("(?i)phoenix");
        Pattern nonTerminalQueueRow = Pattern.compile(
                "^\\| \\d+ \\|.*\\| (?:Proposed|Researching|Ready|In progress|Verifying|Deferred)"
                        + " \\|.*$");
        Pattern completedQueueRow = Pattern.compile(
                "^\\| \\d+ \\|.*\\| Done \\|.*$");
        List<String> failures = new ArrayList<String>();
        boolean inspectBlock = false;

        for (int index = 0; index < lines.size(); index++) {
            String line = lines.get(index);
            if (line.startsWith("## Design authority and goal")
                    || line.startsWith("### AUDIT-01 - ")) {
                inspectBlock = true;
            } else if (line.startsWith("## External competition capability benchmark")) {
                inspectBlock = false;
            }

            if ((inspectBlock || nonTerminalQueueRow.matcher(line).matches())
                    && !completedQueueRow.matcher(line).matches()) {
                String semanticText = line.replace("2025-PhoenixPedro", "");
                Matcher match = applicationReference.matcher(semanticText);
                if (match.find()) {
                    failures.add("FRAMEWORK_IMPROVEMENT_TRACKER.md:" + (index + 1)
                            + ": " + match.group());
                }
            }
        }

        assertTrue("Production application references remain in current tracker guidance: "
                + failures, failures.isEmpty());
    }

    @Test
    public void taskGuidesTeachOutcomeAwareCompositionAndExplicitRepair() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        Path frameworkRoot = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw");
        Path docsRoot = frameworkRoot.resolve("docs");

        String principles = readUtf8(frameworkRoot.resolve("Framework Principles.md"));
        String tasks = readUtf8(docsRoot.resolve("design/Tasks & Macros Quickstart.md"));
        String beginner = readUtf8(
                docsRoot.resolve("getting-started/learn-sushi/Tasks and Autonomous.md"));
        String adaptive = readUtf8(
                docsRoot.resolve("examples/Timestamped Adaptive Collection.md"));
        String cheatSheet = readUtf8(docsRoot.resolve("reference/Sushi Cheat Sheet.md"));

        assertTrue("Framework Principles must make exact-success sequence the ordinary policy",
                principles.contains("`sequence(...)` starts a later child")
                        && principles.contains("only after exact `SUCCESS`"));
        assertTrue("Tasks guide must distinguish eager construction from gated start",
                tasks.contains("The child Tasks are constructed when this graph is built")
                        && tasks.contains("Only exact\n`SUCCESS` starts the next child"));
        assertTrue("Tasks guide must teach explicit natural-outcome repair without finally",
                tasks.contains("Use `sequenceOnCompletion(...)` only when")
                        && tasks.contains("It is not Java\n`finally`"));
        assertTrue("Tasks guide must retain truthful wait-all and fail-closed branch policy",
                tasks.contains("mixed non-success kinds report `UNKNOWN`")
                        && tasks.contains("`CANCELLED` and `UNKNOWN`\nstart neither branch"));
        assertTrue("Beginner Task lesson must teach the safe default before exceptional repair",
                beginner.replaceAll("\\s+", " ")
                        .contains("`Tasks.sequence(...)` starts its next child only after exact `SUCCESS`")
                        && beginner.indexOf("sequenceOnCompletion(...)")
                                > beginner.indexOf("`Tasks.sequence(...)`"));
        assertTrue("Adaptive park takeover must use explicit completion continuation",
                adaptive.contains(
                        "program.rootTask(Tasks.sequenceOnCompletion(boundedPrePark, park));")
                        && adaptive.contains("This continuation is not Java"));
        assertTrue("Cheat sheet must keep both sequence choices visible",
                cheatSheet.contains("`Tasks.sequence(...)` is the ordinary prerequisite chain")
                        && cheatSheet.contains("`Tasks.sequenceOnCompletion(...)` is only for"));
    }

    @Test
    public void firstContactDefinesLoopAndDeferredWorkBeforeThePicture() throws IOException {
        Path frameworkRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH);
        String overview = readUtf8(frameworkRoot.resolve(
                "docs/getting-started/Framework Overview.md"));
        String principles = readUtf8(frameworkRoot.resolve("Framework Principles.md"));
        String maintainers = readUtf8(frameworkRoot.resolve(
                "docs/maintainers/Maintainer Notes.md"));
        String controls = readUtf8(frameworkRoot.resolve(
                "docs/getting-started/learn-sushi/Controls and Intent.md"));
        String plants = readUtf8(frameworkRoot.resolve(
                "docs/getting-started/learn-sushi/Plants and Hardware.md"));
        String normalized = overview.replaceAll("\\s+", " ");
        List<String> failures = new ArrayList<String>();

        requireOrdered(overview, "Framework Overview.md", failures,
                "while (opModeIsActive())",
                "loop()",
                "gamepad1.a",
                "driver.a()",
                "Calling a method runs it now.",
                "Registration saves the function; it does not run it.",
                "A Task is a bookmark for unfinished work.",
                "```mermaid",
                "`FtcRobotOpMode`",
                "`RobotProgram`",
                "configure(RobotProgram program)");
        assertContainsAll("Canonical first-use concept anchors", overview,
                "{ #source }", "New concept: Source",
                "{ #saved-callback }", "New concept: saved callback and lambda",
                "{ #task }", "New concept: Task",
                "learn-sushi/Plants and Hardware.md#plant");
        assertContainsAll("Later lessons must link to the canonical first-use explanations",
                controls,
                "{ #intent }", "New concept: intent",
                "Framework Overview.md#source",
                "Framework Overview.md#saved-callback",
                "Tasks and Autonomous.md");
        assertContainsAll("The Plant chooser must define Plant once and reuse prior concepts",
                plants,
                "{ #plant }", "New concept: Plant",
                "Controls and Intent.md#intent",
                "Framework Overview.md#task");
        assertEquals("Framework Overview canonical concept count", 3,
                matcherCount(CONCEPT_CALLOUT_START.matcher(overview)));
        assertEquals("Controls and Intent canonical concept count", 1,
                matcherCount(CONCEPT_CALLOUT_START.matcher(controls)));
        assertEquals("Plants and Hardware canonical concept count", 1,
                matcherCount(CONCEPT_CALLOUT_START.matcher(plants)));
        assertTrue("First contact must explain same-loop deferred execution without threads",
                normalized.contains("When a future active loop detects the rise, it invokes the "
                        + "callback synchronously during that loop; Sushi does not create a thread."));

        String diagram = sectionBetween(overview, "```mermaid", "```");
        String plainDiagram = diagram.toLowerCase(Locale.ROOT);
        for (String action : Arrays.asList(
                "create robot parts",
                "save button rules",
                "refresh sensor observations",
                "check saved rules",
                "advance ongoing actions",
                "update robot parts",
                "show telemetry",
                "cancel ongoing actions",
                "stop hardware owners")) {
            assertTrue("First-contact diagram is missing plain action " + action,
                    plainDiagram.contains(action));
        }
        String activeDiagramNode = sectionBetween(plainDiagram, "a[\"active", "s[\"stop");
        requireOrdered(activeDiagramNode, "First-contact active loop", failures,
                "refresh sensor observations", "check saved rules",
                "advance ongoing actions", "update robot parts", "show telemetry");
        for (String unexplained : Arrays.asList(
                "bindings", "tasks", "services", "presenters", "plants")) {
            assertTrue("First-contact diagram leaks framework noun " + unexplained,
                    !plainDiagram.contains(unexplained));
        }

        for (String advanced : Arrays.asList(
                "Prestart",
                "BLOCKED",
                "Services",
                "StartDisposition",
                "RuntimeException",
                "sequenceOnCompletion",
                "parallelDeadline")) {
            assertTrue("First contact leaks advanced lifecycle/composition term " + advanced,
                    !overview.contains(advanced));
        }
        String principlesProse = principles.replaceAll("\\s+", " ");
        assertTrue("Framework Principles must preserve the novice/deferred-execution contract",
                principlesProse.contains("must not assume lambdas, callback registration")
                        && principlesProse.contains(
                                "Before the first API that saves code for later")
                        && principlesProse.contains(
                                "First-contact diagrams use those plain actions"));
        assertTrue("Maintainer guidance must preserve the canonical Get Started teaching order",
                maintainers.contains("Keep one canonical Get Started order")
                        && maintainers.contains("registration does not run the function")
                        && maintainers.contains("same-loop synchronous invocation"));
        assertTrue("First-contact teaching-order failures: " + failures, failures.isEmpty());
    }

    @Test
    public void frameworkPrinciplesDefineTheDocumentationQualityRubric() throws IOException {
        Path frameworkRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH);
        String principles = readUtf8(frameworkRoot.resolve("Framework Principles.md"));
        String principle = sectionBetween(
                principles,
                "## 6. One current, proven story",
                "## Design checklist");
        String rubric = sectionBetween(
                principle,
                "### Documentation review criteria",
                "### Implications");
        String implications = sectionBetween(
                principle,
                "### Implications",
                "[`Maintainer Notes`]");
        String maintainerBoundary = sectionBetween(
                principle,
                "[`Maintainer Notes`]",
                "Good documentation is an implication of simplicity");
        String checklist = sectionBetween(
                principles,
                "## Design checklist",
                "## Where exact contracts live");

        assertContainsAll("Documentation-criteria applicability", rubric,
                "apply each criterion to the artifact or feature it names",
                "need not contain code, a checkpoint, production wiring, or a visual",
                "unless its stated purpose requires one", "reconstruction applies to lessons",
                "explanation applies to displayed code and tests",
                "checkpoint or experiment to support a claim");
        assertContainsAll("Audience and outcome criterion",
                markdownTableRow(rubric, "Audience and outcome"),
                "primary reader", "purpose", "observable result or question", "prerequisites",
                "completion evidence", "next choice", "hub or reference page", "lookup scope");
        assertContainsAll("Learning order criterion",
                markdownTableRow(rubric, "Learning order"),
                "familiar robot actions", "prior concepts", "before", "required", "optional",
                "advanced");
        assertContainsAll("Reconstructability criterion",
                markdownTableRow(rubric, "Reconstructability"),
                "lesson", "production path", "declared prerequisites", "configuration",
                "ownership", "wiring", "heartbeat", "stop decisions", "mechanical gaps");
        assertContainsAll("Explanation criterion",
                markdownTableRow(rubric, "Explanation"),
                "every displayed code fragment or test", "what happens", "when", "who owns", "why",
                "reverse-engineer");
        assertContainsAll("Evidence criterion",
                markdownTableRow(rubric, "Evidence"),
                "evidentiary checkpoint", "question", "observation",
                "supported and unsupported conclusions", "next gate", "substitutes an outside boundary",
                "retained production behavior", "each replacement");
        assertContainsAll("Truth and safety criterion",
                markdownTableRow(rubric, "Truth and safety"),
                "software evidence", "physical claims", "controlled hardware evidence", "blocked");
        assertContainsAll("Discovery criterion",
                markdownTableRow(rubric, "Discovery"),
                "goal-oriented headings", "canonical home", "framework taxonomy", "narrative",
                "api", "complete-source lookup");
        assertContainsAll("Accessibility criterion",
                markdownTableRow(rubric, "Accessibility"),
                "essential meaning", "text", "labeled", "text equivalent");
        assertContainsAll("Current authority criterion",
                markdownTableRow(rubric, "Current authority"),
                "narrative", "api contracts", "maintained examples", "tests",
                "reported limitations", "same supported present state");

        assertContainsAll("Progressive-disclosure rule", implications,
                "progressive disclosure", "must not assume lambdas", "plain action description");
        assertContainsAll("Truthful-simplification rule", implications,
                "conceptual comparison", "behaviorally truthful", "maintained production path",
                "unsafe default", "competing architecture");
        assertContainsAll("Focused-hardware rule", implications,
                "knowledge may accumulate", "hardware fixtures remain focused",
                "unrelated mechanism");
        assertContainsAll("Reconstruction rule", implications,
                "reconstruction test", "production configuration", "managed heartbeat",
                "stop path", "small mechanical details");
        assertContainsAll("Explained-test rule", implications,
                "causal experiment", "reverse-engineer", "student-facing test",
                "replaces only the outside world", "what it cannot prove", "next gate");
        assertContainsAll("Evidence-boundary rule", implications,
                "software and hardware evidence", "does not prove wiring",
                "conservative supervised conditions", "explicit stop plan", "blocked");
        assertContainsAll("Discovery rule", implications,
                "reader's goal or question", "narrative search", "javadocs own exact type/member",
                "complete-source links");
        assertContainsAll("Accessible-visual rule", implications,
                "visuals supplement", "accessible label", "text equivalent", "only carrier");

        assertContainsAll("Maintainer-mechanics boundary", maintainerBoundary,
                "(<docs/maintainers/maintainer notes.md>)", "mechanical authoring contract",
                "details implement this rubric", "do not create a second set");
        assertContainsAll("Design-checklist proof question", checklist,
                "intended reader", "declared prerequisites", "reconstruct",
                "evidence, its limits", "next gate", "tell that same story");
    }

    @Test
    public void visualTeachingGrammarIsSparseSemanticAndAccessible() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path frameworkRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH);
        String principles = readUtf8(frameworkRoot.resolve("Framework Principles.md"));
        String maintainers = readUtf8(frameworkRoot.resolve(
                "docs/maintainers/Maintainer Notes.md"));
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));

        assertContainsAll("Point-of-use documentation principle", principles,
                "plain robot language", "active code and values", "searching another section",
                "infer them from `defaults()`", "first contact short", "later level");
        assertContainsAll("Accessible visual-emphasis principle", principles,
                "small, consistent visual vocabulary", "labels state why", "normal reading order",
                "color", "interaction", "only carrier");
        assertContainsAll("Maintainer visual vocabulary", maintainers,
                "new concept: <term>", "warning: <problem>", "danger: <hazard>",
                "checkpoint: <observation>", "tip: <shortcut>", "roughly 80 words",
                "one in an h2 section", "at most three concept callouts", "one on a build page",
                "summarizes or replaces", "hl_lines", "no more than ten", "supplemental");
        assertTrue("The native renderer must keep admonitions and code highlighting enabled",
                config.contains("admonition = {}")
                        && config.contains("pymdownx.highlight.line_spans = \"__span\"")
                        && config.contains("pymdownx.highlight.pygments_lang_class = true"));

        List<Path> pages = new ArrayList<Path>();
        collectMarkdownFiles(frameworkRoot, pages);
        List<String> failures = new ArrayList<String>();
        int calloutCount = 0;
        int highlightedFenceCount = 0;
        for (Path page : pages) {
            PageMetadata metadata = pageMetadata(page);
            String markdown = readUtf8(page);
            calloutCount += validateVisualCallouts(
                    repositoryRoot, page, markdown, metadata.tags, failures);
            highlightedFenceCount += validateHighlightedJavaFences(
                    repositoryRoot, page, markdown, failures);
        }
        assertTrue("The approved visual grammar must be exercised by canonical concept boxes",
                calloutCount >= 6);
        assertTrue("The point-of-use teaching grammar must exercise code-line highlighting",
                highlightedFenceCount > 0);
        assertTrue("Visual-teaching grammar failures: " + failures, failures.isEmpty());
    }

    @Test
    public void visualTeachingGrammarHonorsFencesAndValidatesHighlightLines() {
        Path repositoryRoot = temporaryFolder.getRoot().toPath();
        Path page = repositoryRoot.resolve("Guide.md");
        String markdown = "````text\n"
                + "!!! danger \"not rendered\"\n"
                + "```\n"
                + "````\n\n"
                + "## Real section\n\n"
                + "!!! info \"New concept: visible\"\n\n"
                + "    This visible definition is short and textual.\n";
        List<String> failures = new ArrayList<String>();

        assertEquals(1, validateVisualCallouts(
                repositoryRoot,
                page,
                markdown,
                Collections.singletonList("Learn"),
                failures));
        assertTrue("Fence-aware callout failures: " + failures, failures.isEmpty());
        assertEquals("A literal nested fence must not count as rendered highlighting", 0,
                validateHighlightedJavaFences(repositoryRoot, page, markdown, failures));

        String invalidHighlight = "```java hl_lines=\"0 2 2 4 999999999999999999999\"\n"
                + "one\n"
                + "two\n"
                + "three\n"
                + "```\n";
        failures.clear();
        assertEquals(1, validateHighlightedJavaFences(
                repositoryRoot, page, invalidHighlight, failures));
        assertFailureContains(failures, "highlighted line 0 is outside");
        assertFailureContains(failures, "highlighted line 2 is duplicated");
        assertFailureContains(failures, "highlighted line 4 is outside");
        assertFailureContains(failures, "highlighted line number is too large");

        failures.clear();
        String emptyHighlight = "```java hl_lines=\"   \"\n"
                + "one\n"
                + "```\n";
        assertEquals(0, validateHighlightedJavaFences(
                repositoryRoot, page, emptyHighlight, failures));
        assertFailureContains(failures, "hl_lines must be a space-separated list");
    }

    @Test
    public void firstContactDiagramIsExplicitlyConfiguredAndAccessible() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        Matcher extensionSection = Pattern.compile(
                "(?ms)^\\[project\\.markdown_extensions][ \\t]*\\r?\\n"
                        + "(.*?)(?=^\\[[^\\r\\n]+][ \\t]*\\r?$|\\z)")
                .matcher(config);
        assertTrue("Zensical must explicitly configure Markdown extensions",
                extensionSection.find());
        String extensions = extensionSection.group(1);
        Set<String> activeExtensionLines = new HashSet<String>();
        for (String line : extensions.split("\\r?\\n")) {
            String trimmed = line.trim();
            if (!trimmed.isEmpty() && !trimmed.startsWith("#")) {
                activeExtensionLines.add(trimmed);
            }
        }
        String[] requiredExtensions = {
            "abbr = {}",
            "admonition = {}",
            "attr_list = {}",
            "def_list = {}",
            "footnotes = {}",
            "md_in_html = {}",
            "toc.permalink = true",
            "pymdownx.arithmatex.generic = true",
            "pymdownx.betterem = {}",
            "pymdownx.caret = {}",
            "pymdownx.details = {}",
            "pymdownx.emoji.emoji_generator = \"zensical.extensions.emoji.to_svg\"",
            "pymdownx.emoji.emoji_index = \"zensical.extensions.emoji.twemoji\"",
            "pymdownx.highlight.anchor_linenums = true",
            "pymdownx.highlight.line_spans = \"__span\"",
            "pymdownx.highlight.pygments_lang_class = true",
            "pymdownx.inlinehilite = {}",
            "pymdownx.keys = {}",
            "pymdownx.magiclink = {}",
            "pymdownx.mark = {}",
            "pymdownx.smartsymbols = {}",
            "pymdownx.superfences.custom_fences = [",
            "{ name = \"mermaid\", class = \"mermaid\", "
                    + "format = \"pymdownx.superfences.fence_code_format\" },",
            "pymdownx.tabbed.alternate_style = true",
            "pymdownx.tabbed.combine_header_slug = true",
            "pymdownx.tasklist.custom_checkbox = true",
            "pymdownx.tilde = {}"
        };
        for (String requiredExtension : requiredExtensions) {
            assertTrue(
                    "Missing explicit Zensical default/Mermaid extension: " + requiredExtension,
                    activeExtensionLines.contains(requiredExtension));
        }

        Path overviewPath = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/"
                        + "Framework Overview.md");
        String overview = readUtf8(overviewPath);
        Matcher diagrams = Pattern.compile("```mermaid\\s*([\\s\\S]*?)```").matcher(overview);
        int diagramCount = 0;
        while (diagrams.find()) {
            diagramCount++;
            String diagram = diagrams.group(1);
            assertTrue("Every Mermaid diagram needs accTitle metadata",
                    Pattern.compile("(?m)^[ \\t]*accTitle:[ \\t]*\\S+")
                            .matcher(diagram).find());
            assertTrue("Every Mermaid diagram needs accDescr metadata",
                    Pattern.compile("(?m)^[ \\t]*accDescr:[ \\t]*\\S+")
                            .matcher(diagram).find());

            String afterDiagram = overview.substring(diagrams.end()).replace("\r\n", "\n");
            assertTrue("Every Mermaid diagram needs an adjacent plain-text fallback",
                    afterDiagram.startsWith("\n**Text version:**\n"));
            int nextHeading = afterDiagram.indexOf("\n## ", 1);
            String fallback = nextHeading < 0
                    ? afterDiagram
                    : afterDiagram.substring(0, nextHeading);
            assertTrue("The adjacent plain-text fallback must not be empty",
                    fallback.trim().length() > "**Text version:**".length());
        }
        assertTrue("The first-contact page needs at least one Mermaid diagram", diagramCount > 0);
    }

    @Test
    public void firstContactPagesStayWithinProgressiveDisclosureBudgets()
            throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path learningRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH)
                .resolve("docs/getting-started");
        Path overview = learningRoot.resolve("Framework Overview.md");
        Path hub = learningRoot.resolve("Beginner's Guide.md");
        Path topics = learningRoot.resolve("learn-sushi");

        assertTrue("First-contact overview exceeds 900 prose words",
                proseWordCount(overview) <= 900);
        assertTrue("First-contact overview exceeds three Java excerpts",
                javaFenceCount(overview) <= 3);
        assertTrue("First-contact overview exceeds 30 displayed Java lines",
                displayedJavaLineCount(overview) <= 30);
        assertTrue("Topic router exceeds 450 prose words", proseWordCount(hub) <= 450);

        int topicWords = 0;
        for (String topic : Arrays.asList(
                "Robot Roles.md",
                "Controls and Intent.md",
                "Plants and Hardware.md",
                "Tasks and Autonomous.md",
                "Evidence and Experiments.md",
                "From Requirement to Robot.md")) {
            int words = proseWordCount(topics.resolve(topic));
            assertTrue(topic + " exceeds the per-topic progressive-disclosure budget: " + words,
                    words <= 1300);
            topicWords += words;
        }
        assertTrue("Six Learn pages exceed 5,400 prose words: " + topicWords,
                topicWords <= 5400);
    }

    @Test
    public void firstSoftwareTourKeepsThreeExecutionShapesSmallAndInOrder()
            throws IOException {
        Path docsRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH).resolve("docs");
        String tour = readUtf8(docsRoot.resolve("getting-started/First Software Tour.md"));
        String sensor = readUtf8(docsRoot.resolve("build/Read a Switch.md"));
        String intake = readUtf8(docsRoot.resolve("build/Continuous Intake.md"));
        String timedAuto = readUtf8(docsRoot.resolve("build/Run One Timed Auto.md"));
        List<String> failures = new ArrayList<String>();

        assertTrue("First-pass headings must rely on one generated heading ID, not duplicate it",
                !sensor.contains("<a id=\"first-pass-")
                        && !intake.contains("<a id=\"first-pass-")
                        && !timedAuto.contains("<a id=\"first-pass-"));

        requireOrdered(tour, "First Software Tour.md", failures,
                "Read a Switch.md#first-pass-observations-every-loop",
                "Continuous Intake.md#first-pass-run-a-function-once-per-press",
                "Run One Timed Auto.md#first-pass-work-that-continues-across-loops",
                "## Completion check");
        String tourProse = tour.replaceAll("\\s+", " ");
        assertTrue("The first tour must support reading without running or installing anything",
                tourProse.contains("Reading is a complete path")
                        && tourProse.contains("no installation, code changes, test run, or robot")
                        && tourProse.contains("running the tests is not a graduation requirement")
                        && tour.contains("Software success does not grant permission to enable motion")
                        && tour.contains("(<../README.md>)")
                        && proseWordCount(tour) <= 700);
        assertEquals("Every tour stop must let a reader predict an observable result", 3,
                literalCount(tour, "**Predict:**"));
        assertEquals("Every tour prediction needs an explained expected result", 3,
                literalCount(tour, "**Expected behavior:**"));

        Map<String, String> firstPasses = new LinkedHashMap<String, String>();
        firstPasses.put("Read a Switch.md", sectionBetween(sensor,
                "## First pass: observations every loop",
                "## Critical production idea"));
        firstPasses.put("Continuous Intake.md", sectionBetween(intake,
                "## First pass: run a function once per press",
                "## Full build: reconstruct the production path"));
        firstPasses.put("Run One Timed Auto.md", sectionBetween(timedAuto,
                "## First pass: work that continues across loops",
                "## Full build: reconstruct the production path"));
        for (Map.Entry<String, String> entry : firstPasses.entrySet()) {
            assertTrue(entry.getKey() + " first pass exceeds a five-minute prose budget",
                    proseWordCount(entry.getValue()) <= 450);
            assertTrue(entry.getKey() + " first pass exceeds two short Java excerpts",
                    javaFenceCount(entry.getValue()) <= 2);
            assertTrue(entry.getKey() + " first pass leaks advanced Task composition",
                    !entry.getValue().contains("sequenceOnCompletion")
                            && !entry.getValue().contains("parallelDeadline")
                            && !entry.getValue().contains("branchOnOutcome"));
        }

        String sensorFirst = firstPasses.get("Read a Switch.md");
        String sensorFirstProse = sensorFirst.replaceAll("\\s+", " ");
        assertTrue("Switch first pass must distinguish a sampled fact from cached presentation",
                sensorFirst.contains("rawPressedSource.getAsBoolean(clock)")
                        && sensorFirst.contains("pressedSource.getAsBoolean(clock)")
                        && sensorFirst.contains("status = new Status(true, rawPressed, pressed)")
                        && sensorFirstProse.contains("does not read the switch again")
                        && sensorFirstProse.contains("observed=false")
                        && sensorFirstProse.contains("unknown")
                        && !sensorFirst.contains("CallbackBindings")
                        && !sensorFirst.contains("TaskBindings"));

        String intakeFirst = firstPasses.get("Continuous Intake.md");
        String intakeFirstProse = intakeFirst.replaceAll("\\s+", " ");
        assertTrue("Intake first pass must explain registration, synchronous execution, and hold",
                intakeFirst.contains("driver.a()")
                        && intakeFirst.contains("() ->")
                        && intakeFirst.contains("does **not** execute")
                        && intakeFirstProse.contains("same loop")
                        && intakeFirstProse.contains("does not start a thread")
                        && intakeFirst.contains("Hold A")
                        && intakeFirst.contains("Press A again")
                        && intakeFirstProse.contains("`COLLECT` persists until X selects `STOPPED`"));

        String timedFirst = firstPasses.get("Run One Timed Auto.md");
        String timedFirstProse = timedFirst.replaceAll("\\s+", " ");
        assertTrue("Timed Auto first pass must teach a fresh cancellable loop bookmark",
                timedFirstProse.contains("bookmark for unfinished work")
                        && timedFirstProse.contains("FTC START starts the saved Task")
                        && timedFirstProse.contains("without `sleep()`")
                        && timedFirstProse.contains("single-use")
                        && timedFirstProse.contains("cancels the active Task first"));
        assertTrue("First software tour ordering failures: " + failures, failures.isEmpty());
    }

    @Test
    public void introductoryApiDocsExplainSetupNowAndExecutionLater() throws IOException {
        Path frameworkRoot = repositoryRoot().resolve(FRAMEWORK_DOCS_PATH);
        String host = readUtf8(frameworkRoot.resolve("ftc/FtcRobotOpMode.java"));
        String program = readUtf8(frameworkRoot.resolve("ftc/RobotProgram.java"));
        String gamepad = readUtf8(frameworkRoot.resolve("ftc/input/GamepadDevice.java"));
        String callbacks = readUtf8(
                frameworkRoot.resolve("input/binding/CallbackBindings.java"));
        String taskBindings = readUtf8(frameworkRoot.resolve("task/TaskBindings.java"));
        String hostProse = javadocBefore(
                host,
                "public abstract class FtcRobotOpMode");
        String programProse = javadocBefore(
                program,
                "public final class RobotProgram");
        String gamepadProse = javadocBefore(
                gamepad,
                "public BooleanSource a()");
        String callbackProse = javadocBefore(
                callbacks,
                "public interface CallbackBindings");
        String taskBindingProse = javadocBefore(
                taskBindings,
                "public final class TaskBindings");

        assertTrue("FtcRobotOpMode must explain the one-time beginner setup bridge",
                hostProse.contains("build its checklist once during FTC INIT")
                        && hostProse.contains(
                                "uses that checklist for the rest of the FTC lifecycle"));
        assertTrue("RobotProgram must distinguish declaration from execution",
                programProse.contains("saves a piece for later; it does not run that piece's "
                                + "later behavior")
                        && programProse.contains("immediate setup work documented by that method")
                        && programProse.contains("invokes the saved pieces in lifecycle order"));
        assertTrue("GamepadDevice.a() must describe a reusable current-state reader",
                gamepadProse.contains("reusable reader of the A button's current state")
                        && gamepadProse.contains("not a one-time boolean snapshot"));
        assertTrue("CallbackBindings must explain same-loop synchronous deferred execution",
                callbackProse.contains("registration does not invoke the callback")
                        && callbackProse.contains("or create a thread")
                        && callbackProse.contains(
                                "invokes the saved callback synchronously in that loop"));
        assertTrue("TaskBindings must distinguish registration, construction, and execution",
                taskBindingProse.contains("Registration only saves a Task factory")
                        && taskBindingProse.contains("neither creates nor runs a Task")
                        && taskBindingProse.contains("does not create a thread")
                        && taskBindingProse.contains("fresh single-use Task"));
    }

    /** File newline spelling is portable; content, relative indentation, and required prose are not optional. */
    @Test
    public void utf8ReadsNormalizeOnlyLineEndingsWithoutWeakeningSourceOrNoticeChecks()
            throws IOException {
        Path root = temporaryFolder.getRoot().toPath();
        String markdown = "# Example\n\nNotice:\n- Watch the result.\n\n## Files in this checkpoint\n";
        String source = "class Example {\n    void run() {\n"
                + "        emit(\"café\\r\\n\");  \n    }\n}\n";
        String snippet = normalizeExcerpt("    void run() {\n"
                + "        emit(\"café\\r\\n\");  \n    }");
        String[] endings = {"\n", "\r\n", "\r"};
        for (int i = 0; i < endings.length; i++) {
            String guideFile = "Guide" + i + ".md";
            String sourceFile = "Example" + i + ".java";
            write(root, guideFile, markdown.replace("\n", endings[i]));
            write(root, sourceFile, source.replace("\n", endings[i]));
            String actualGuide = readUtf8(root.resolve(guideFile));
            String actualSource = readUtf8(root.resolve(sourceFile));
            assertEquals("Only line terminators may change in Markdown", markdown, actualGuide);
            assertEquals("Preserve UTF-8, indentation, trailing spaces, and escaped source text",
                    source, actualSource);
            assertTrue("The same contiguous source excerpt must work for each newline spelling",
                    containsDedentedBlock(actualSource, snippet));
            assertFalse("A different source call must still fail exact excerpt comparison",
                    containsDedentedBlock(actualSource, snippet.replace("emit(", "other(")));
            assertFalse("Changed relative indentation must still fail exact excerpt comparison",
                    containsDedentedBlock(actualSource, snippet.replace("    emit(", "   emit(")));
            List<String> failures = new ArrayList<>();
            validateBuildNotice(actualGuide, guideFile, failures);
            assertTrue("A present Notice is valid for every newline spelling: " + failures,
                    failures.isEmpty());
            write(root, guideFile, markdown.replace("Notice:\n", "")
                    .replace("\n", endings[i]));
            validateBuildNotice(readUtf8(root.resolve(guideFile)), guideFile, failures);
            assertEquals("Newline normalization must not invent a missing Notice",
                    Collections.singletonList(guideFile + ": missing bounded Notice section"), failures);
        }
    }

    @Test
    public void validatesAuthoredBuildAreaWhileSkippingGeneratedBuildOutput()
            throws IOException {
        Path root = temporaryFolder.getRoot().toPath();
        write(root, "docs/build/Guide.md", "# Authored build guide\n");
        write(root, "README.md", "[guide](docs/build/Guide.md)\n");
        write(root, "build/Generated.md", "[ignored](Missing.md)\n");

        assertNoFailures(MarkdownIntegrity.validateRepository(root));
    }

    @Test
    public void acceptsLiteralAndEncodedSpacesAndDuplicateHeadingAnchors() throws IOException {
        Path root = temporaryFolder.getRoot().toPath();
        write(root, "Docs/Guide File.md",
                "# Hello, World!\n\n"
                        + "# Repeat\n\n# Repeat\n\n# Repeat-1\n\n"
                        + "## Saved function { #saved-callback }\n\n"
                        + "## Numbered concept { #51-webcam-raw-apriltag-correction }\n\n"
                        + "Setext heading\n--------------\n");
        write(root, "README.md",
                "[literal](<Docs/Guide File.md#hello-world>)\n"
                        + "[encoded](Docs/Guide%20File.md#repeat-1)\n"
                        + "[collision](<Docs/Guide File.md#repeat-1-1>)\n"
                        + "[explicit](<Docs/Guide File.md#saved-callback>)\n"
                        + "[numbered](<Docs/Guide File.md#51-webcam-raw-apriltag-correction>)\n"
                        + "[setext](<Docs/Guide File.md#setext-heading>)\n");

        assertNoFailures(MarkdownIntegrity.validateRepository(root));
    }

    @Test
    public void rejectsDuplicateExplicitHeadingIds() throws IOException {
        Path root = temporaryFolder.getRoot().toPath();
        write(root, "Guide.md",
                "# First { #stable }\n\n"
                        + "## Second { #stable }\n");
        write(root, "README.md", "[stable](Guide.md#stable)\n");

        assertFailureContains(
                MarkdownIntegrity.validateRepository(root),
                "duplicate explicit heading id");
    }

    @Test
    public void reportsWrongCaseMissingPathAndMissingCrossFileAnchor() throws IOException {
        Path root = temporaryFolder.getRoot().toPath();
        write(root, "Docs/Guide File.md", "# Present heading\n");
        write(root, "README.md",
                "[case](<docs/Guide File.md>)\n"
                        + "[missing](Docs/Missing.md)\n"
                        + "[anchor](<Docs/Guide File.md#absent-heading>)\n");

        List<String> failures = MarkdownIntegrity.validateRepository(root);
        assertFailureContains(failures, "path case differs");
        assertFailureContains(failures, "target does not exist");
        assertFailureContains(failures, "heading fragment does not exist");
    }

    @Test
    public void reportsRepositoryEscapeAndUnclosedFence() throws IOException {
        Path root = temporaryFolder.getRoot().toPath();
        write(root, "README.md",
                "[escape](../outside.md)\n"
                        + "[absolute](C\\:/outside.md)\n\n"
                        + "```java\nignored();\n");

        List<String> failures = MarkdownIntegrity.validateRepository(root);
        assertFailureContains(failures, "target escapes repository root");
        assertFailureContains(failures, "absolute local path is not portable");
        assertFailureContains(failures, "unclosed fenced code block");
        assertFailureStartsWith(
                failures,
                "README.md:1: ../outside.md — target escapes repository root");
    }

    @Test
    public void ignoresLinkSyntaxInsideInlineAndFencedCode() throws IOException {
        Path root = temporaryFolder.getRoot().toPath();
        write(root, "Guide.md", "# Guide\n");
        write(root, "Guide(1).md", "# Escaped punctuation\n");
        write(root, "DocsGuide.md", "# Must not receive a collapsed backslash path\n");
        write(root, "README.md",
                "`[inline](Missing.md)`\n\n"
                        + "~~~markdown\n[fenced](AlsoMissing.md)\n~~~\n\n"
                        + "\\[escaped-label](NotALink.md)\n"
                        + "[real](Guide.md#guide)\n"
                        + "[escaped](Guide\\(1\\).md#escaped-punctuation)\n");

        assertNoFailures(MarkdownIntegrity.validateRepository(root));

        write(root, "BadLabel.md", "[show \\[bracket](Missing.md)\n");
        assertFailureStartsWith(
                MarkdownIntegrity.validateRepository(root),
                "BadLabel.md:1: Missing.md — target does not exist");

        write(root, "BadInline.md", "stray ` [broken](Missing.md)\n");
        assertFailureContains(
                MarkdownIntegrity.validateRepository(root),
                "target does not exist");

        write(root, "Bad.md", "[backslash](Docs\\Guide.md)\n");
        assertFailureContains(
                MarkdownIntegrity.validateRepository(root),
                "local path must use '/' separators");
    }

    private static Path repositoryRoot() {
        return MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
    }

    private static void write(Path root, String relativePath, String contents)
            throws IOException {
        Path path = root.resolve(relativePath);
        Files.createDirectories(path.getParent() == null ? root : path.getParent());
        Files.write(path, contents.getBytes(StandardCharsets.UTF_8));
    }

    private static String readUtf8(Path path) throws IOException {
        // Git may check the same authored file out with LF or CRLF. Source-excerpt comparison
        // already uses this newline contract; retain every other character exactly.
        return new String(Files.readAllBytes(path), StandardCharsets.UTF_8)
                .replace("\r\n", "\n").replace('\r', '\n');
    }

    private static String shellTabPair(String windowsCommands, String macOsCommands) {
        return "=== \"Windows\"\n\n"
                + "    ```powershell\n"
                + indentShellTabBody(windowsCommands)
                + "    ```\n\n"
                + "=== \"macOS\"\n\n"
                + "    ```bash\n"
                + indentShellTabBody(macOsCommands)
                + "    ```\n";
    }

    private static String indentShellTabBody(String commands) {
        StringBuilder indented = new StringBuilder();
        String normalized = commands.replace("\r\n", "\n").replace('\r', '\n');
        for (String line : normalized.split("\n", -1)) {
            indented.append("    ").append(line).append('\n');
        }
        return indented.toString();
    }

    private static Map<String, String> buildRecipeTestSelectors() {
        Map<String, String> selectors = new LinkedHashMap<String, String>();
        selectors.put("Read a Switch.md",
                "edu.ftcsushi.robots.examples.basicsensing.BasicSwitchSoftwareScenarioTest");
        selectors.put("First Drive.md",
                "edu.ftcsushi.robots.examples.firstdrive.FirstDriveSoftwareScenarioTest");
        selectors.put("Continuous Intake.md",
                "edu.ftcsushi.robots.examples.starter.robot.StarterMechanismLessonTest");
        selectors.put("Named Claw.md",
                "edu.ftcsushi.robots.examples.basicmechanisms.BasicClawSoftwareScenarioTest");
        selectors.put("Referenced Lift.md",
                "edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftSoftwareScenarioTest");
        selectors.put("Move a Referenced Lift.md",
                "edu.ftcsushi.robots.examples.basicmechanisms.BasicLiftMoveSoftwareScenarioTest");
        selectors.put("Single Flywheel Velocity.md",
                "edu.ftcsushi.robots.examples.basicflywheel.BasicFlywheelSoftwareScenarioTest");
        selectors.put("Combine Drive and Intake.md",
                "edu.ftcsushi.robots.examples.starter.robot."
                        + "StarterDriveAndIntakeSoftwareScenarioTest");
        selectors.put("Run One Timed Auto.md",
                "edu.ftcsushi.robots.examples.starter.opmode."
                        + "StarterTimedAutoSoftwareScenarioTest");
        selectors.put("First Autonomous.md",
                "edu.ftcsushi.robots.examples.basicmechanisms.BasicAutoSoftwareScenarioTest");
        selectors.put("First Pedro Auto.md",
                "edu.ftcsushi.robots.examples.pedro.basic.BasicPedroRouteSoftwareScenarioTest");
        return Collections.unmodifiableMap(selectors);
    }

    private static Map<String, Integer> publishedShellPairsByPage() {
        Map<String, Integer> counts = new LinkedHashMap<String, Integer>();
        counts.put("docs/build/Combine Drive and Intake.md", 1);
        counts.put("docs/build/Continuous Intake.md", 1);
        counts.put("docs/build/First Autonomous.md", 1);
        counts.put("docs/build/First Drive.md", 1);
        counts.put("docs/build/First Pedro Auto.md", 1);
        counts.put("docs/build/Move a Referenced Lift.md", 1);
        counts.put("docs/build/Named Claw.md", 1);
        counts.put("docs/build/Read a Switch.md", 1);
        counts.put("docs/build/Referenced Lift.md", 1);
        counts.put("docs/build/Run One Timed Auto.md", 1);
        counts.put("docs/build/Single Flywheel Velocity.md", 1);
        counts.put("docs/examples/Field-relative Drive.md", 1);
        counts.put("docs/examples/Hardware-free Reference Scenarios.md", 2);
        counts.put("docs/examples/Subsystem Experiments.md", 1);
        counts.put("docs/getting-started/Build and Run.md", 3);
        counts.put("docs/getting-started/First Software Tour.md", 3);
        counts.put("docs/maintainers/Maintainer Notes.md", 4);
        counts.put("docs/testing-calibration/Control Tuning Workflow.md", 1);
        counts.put("docs/testing-calibration/Add Calibration Testers to Your Robot.md", 1);
        counts.put("docs/troubleshooting/Common Problems.md", 1);
        return Collections.unmodifiableMap(counts);
    }

    private static void validatePublishedShellPairInventory(String page,
                                                            int found,
                                                            List<String> failures) {
        Integer expected = PUBLISHED_SHELL_PAIRS_BY_PAGE.get(page);
        if (expected == null) {
            if (found > 0) {
                failures.add(page + ": unexpected shell-command pair count " + found
                        + "; add no new published command page without updating the approved "
                        + "inventory");
            }
            return;
        }
        if (found != expected) {
            failures.add(page + ": expected " + expected + " shell-command "
                    + (expected == 1 ? "pair" : "pairs") + ", found " + found);
        }
    }

    private static List<String> activeTomlStringArrayEntries(String section, String key) {
        String[] lines = section.replace("\r\n", "\n").replace('\r', '\n')
                .split("\n", -1);
        Pattern assignment = Pattern.compile(
                "^" + Pattern.quote(key) + "[ \\t]*=[ \\t]*\\[[ \\t]*$");
        Pattern entry = Pattern.compile("^\"([^\"]+)\"[ \\t]*,?[ \\t]*$");
        List<String> entries = new ArrayList<String>();
        boolean insideArray = false;
        for (String line : lines) {
            String active = stripTomlComment(line).trim();
            if (!insideArray) {
                if (assignment.matcher(active).matches()) {
                    insideArray = true;
                }
                continue;
            }
            if ("]".equals(active)) {
                return entries;
            }
            if (active.isEmpty()) {
                continue;
            }
            Matcher item = entry.matcher(active);
            if (item.matches()) {
                entries.add(item.group(1));
            } else {
                return Collections.emptyList();
            }
        }
        return Collections.emptyList();
    }

    private static String stripTomlComment(String line) {
        boolean insideSingleQuote = false;
        boolean insideDoubleQuote = false;
        boolean escaped = false;
        for (int index = 0; index < line.length(); index++) {
            char character = line.charAt(index);
            if (insideDoubleQuote && character == '\\' && !escaped) {
                escaped = true;
                continue;
            }
            if (character == '"' && !insideSingleQuote && !escaped) {
                insideDoubleQuote = !insideDoubleQuote;
            } else if (character == '\'' && !insideDoubleQuote) {
                insideSingleQuote = !insideSingleQuote;
            } else if (character == '#' && !insideSingleQuote && !insideDoubleQuote) {
                return line.substring(0, index);
            }
            escaped = false;
        }
        return line;
    }

    private static String javadocBefore(String javaSource, String declaration) {
        int declarationStart = javaSource.indexOf(declaration);
        if (declarationStart < 0) {
            throw new AssertionError("Missing Java declaration: " + declaration);
        }

        int commentStart = javaSource.lastIndexOf("/**", declarationStart);
        int commentEnd = commentStart < 0 ? -1 : javaSource.indexOf("*/", commentStart);
        if (commentStart < 0 || commentEnd < 0 || commentEnd > declarationStart) {
            throw new AssertionError("Missing Javadoc immediately before: " + declaration);
        }

        String between = javaSource.substring(commentEnd + 2, declarationStart).trim();
        if (!between.isEmpty()) {
            throw new AssertionError("Javadoc is not attached to: " + declaration);
        }

        return javaSource.substring(commentStart, commentEnd + 2)
                .replaceAll("(?m)^\\s*/?\\*+/? ?", " ")
                .replaceAll("\\s+", " ")
                .trim();
    }

    private static void collectMarkdownFiles(Path root, final List<Path> files)
            throws IOException {
        Files.walkFileTree(root, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attributes) {
                if (attributes.isRegularFile()
                        && file.getFileName().toString().endsWith(".md")) {
                    files.add(file);
                }
                return FileVisitResult.CONTINUE;
            }
        });
        Collections.sort(files);
    }

    private static List<String> markdownFileNames(Path directory) throws IOException {
        List<String> names = new ArrayList<String>();
        try (DirectoryStream<Path> files = Files.newDirectoryStream(directory, "*.md")) {
            for (Path file : files) {
                names.add(file.getFileName().toString());
            }
        }
        Collections.sort(names);
        return names;
    }

    private static List<String> topLevelAreas(String config) {
        List<String> areas = new ArrayList<String>();
        Matcher groups = Pattern.compile("(?m)^  \\{ \"([^\"]+)\" = \\[$")
                .matcher(config);
        while (groups.find()) {
            areas.add(groups.group(1));
        }
        return areas;
    }

    private static String navAreaBlock(String config, String area) {
        String marker = "{ \"" + area + "\" = [";
        int markerStart = config.indexOf(marker);
        assertTrue("Missing navigation area " + area, markerStart >= 0);
        assertEquals("Navigation area must occur exactly once: " + area,
                markerStart, config.lastIndexOf(marker));

        int listStart = config.indexOf('[', markerStart);
        int listEnd = matchingBracket(config, listStart);
        assertTrue("Unclosed navigation area " + area, listEnd > listStart);
        return config.substring(listStart + 1, listEnd);
    }

    private static int matchingBracket(String text, int opening) {
        int depth = 0;
        boolean quoted = false;
        boolean escaped = false;
        for (int index = opening; index < text.length(); index++) {
            char character = text.charAt(index);
            if (quoted) {
                if (escaped) {
                    escaped = false;
                } else if (character == '\\') {
                    escaped = true;
                } else if (character == '"') {
                    quoted = false;
                }
                continue;
            }
            if (character == '"') {
                quoted = true;
            } else if (character == '[') {
                depth++;
            } else if (character == ']') {
                depth--;
                if (depth == 0) {
                    return index;
                }
            }
        }
        return -1;
    }

    private static List<String> navTargets(String areaBlock) {
        List<String> targets = new ArrayList<String>();
        Matcher entries = Pattern.compile("=\\s*\"([^\"]+)\"").matcher(areaBlock);
        while (entries.find()) {
            targets.add(entries.group(1));
        }
        return targets;
    }

    private static PageMetadata pageMetadata(Path page) throws IOException {
        List<String> lines = Files.readAllLines(page, StandardCharsets.UTF_8);
        if (lines.isEmpty() || !"---".equals(lines.get(0).trim())) {
            return new PageMetadata(false, false, Collections.<String>emptyList());
        }

        int end = -1;
        for (int index = 1; index < lines.size(); index++) {
            if ("---".equals(lines.get(index).trim())) {
                end = index;
                break;
            }
        }
        if (end < 0) {
            return new PageMetadata(false, false, Collections.<String>emptyList());
        }

        boolean excluded = false;
        String section = "";
        List<String> tags = new ArrayList<String>();
        for (int index = 1; index < end; index++) {
            String line = lines.get(index);
            String trimmed = line.trim();
            if (!line.isEmpty() && !Character.isWhitespace(line.charAt(0))) {
                section = trimmed.endsWith(":")
                        ? trimmed.substring(0, trimmed.length() - 1)
                        : "";
                continue;
            }
            if ("tags".equals(section) && trimmed.startsWith("- ")) {
                tags.add(trimmed.substring(2).trim());
            } else if ("search".equals(section) && "exclude: true".equals(trimmed)) {
                excluded = true;
            }
        }
        return new PageMetadata(true, excluded, tags);
    }

    private static void requireExactlyOnce(String markdown,
                                           String required,
                                           String page,
                                           List<String> failures) {
        int count = literalCount(markdown, required);
        if (count != 1) {
            failures.add(page + ": expected one " + required + ", found " + count);
        }
    }

    private static void requireOrdered(String markdown,
                                       String page,
                                       List<String> failures,
                                       String... required) {
        int previous = -1;
        for (String token : required) {
            int found = markdown.indexOf(token);
            if (found < 0) {
                failures.add(page + ": missing ordered teaching element " + token);
                return;
            }
            if (found <= previous) {
                failures.add(page + ": teaching elements are out of order at " + token);
                return;
            }
            previous = found;
        }
    }

    private static String sectionBetween(String markdown, String start, String end) {
        int sectionStart = markdown.indexOf(start);
        assertTrue("Missing section start " + start, sectionStart >= 0);
        int sectionEnd = markdown.indexOf(end, sectionStart + start.length());
        assertTrue("Missing section end " + end, sectionEnd > sectionStart);
        return markdown.substring(sectionStart, sectionEnd);
    }

    /** Checks only the six-column, three-placement teaching table, not arbitrary Markdown. */
    private static void assertIllustrativeCameraValidationTable(String camera) {
        String header = "| Check | Distance / image position | Measured field point (in) | "
                + "Estimated field point (in) | Error (dX, dY) (in) | Position error (in) |";
        String[] lines = camera.split("\\R");
        int headerIndex = -1;
        for (int i = 0; i < lines.length; i++) {
            if (lines[i].trim().equals(header)) {
                assertEquals("The named camera-validation table must appear once", -1, headerIndex);
                headerIndex = i;
            }
        }
        assertTrue("Camera-validation table must name its coordinates and error units", headerIndex >= 0);
        assertTrue("Camera-validation table needs a separator after its header", headerIndex + 1 < lines.length);
        assertTrue("Camera-validation table needs exactly six separator cells",
                lines[headerIndex + 1].trim().matches("\\|(?:\\s*:?-{3,}:?\\s*\\|){6}"));
        Set<String> checks = new LinkedHashSet<>();
        Set<String> placements = new LinkedHashSet<>();
        Set<Double> measuredDistances = new HashSet<>();
        Set<List<Double>> errors = new HashSet<>();
        for (int i = headerIndex + 2; i < lines.length && !lines[i].trim().isEmpty(); i++) {
            String line = lines[i];
            assertTrue("Every camera-validation table body line must be a row: " + line,
                    line.trim().startsWith("|") && line.trim().endsWith("|"));
            String[] cells = line.trim().split("\\|", -1);
            assertEquals("Camera-validation row needs exactly six cells: " + line, 8, cells.length);
            String check = cells[1].trim();
            assertTrue("Unexpected camera-validation check " + check,
                    Arrays.asList("V1", "V2", "V3").contains(check));
            assertTrue("Duplicate camera-validation check " + check, checks.add(check));
            placements.add(cells[2].trim());
            double[] measured = calibrationCoordinatePair(check + " measured", cells[3]);
            double[] estimated = calibrationCoordinatePair(check + " estimated", cells[4]);
            double[] publishedError = calibrationCoordinatePair(check + " error", cells[5]);
            double magnitude = finiteCalibrationNumber(check + " position error", cells[6]);
            double dx = estimated[0] - measured[0];
            double dy = estimated[1] - measured[1];
            assertTrue(check + " must have finite derived errors",
                    Double.isFinite(dx) && Double.isFinite(dy) && Double.isFinite(Math.hypot(dx, dy)));
            assertEquals(check + " dX must be estimated minus measured", dx, publishedError[0], 1e-9);
            assertEquals(check + " dY must be estimated minus measured", dy, publishedError[1], 1e-9);
            assertEquals(check + " position error must be the straight-line separation",
                    Math.hypot(dx, dy), magnitude, 1e-9);
            assertTrue(check + " must support the prose's +X/+Y systematic-offset example",
                    dx > 0.0 && dy > 0.0);
            errors.add(Arrays.asList(dx, dy));
            measuredDistances.add(Math.hypot(measured[0], measured[1]));
        }
        assertEquals("All three held-out checks must be present exactly once",
                new LinkedHashSet<>(Arrays.asList("V1", "V2", "V3")), checks);
        assertEquals("The illustration must cover three distance/image-position cases",
                new LinkedHashSet<>(Arrays.asList("Near / left", "Middle / center", "Far / right")),
                placements);
        assertEquals("The independently known origin must have three distinct measured distances",
                3, measuredDistances.size());
        assertEquals("The explanation describes the same coordinate offset in all three checks",
                1, errors.size());
    }

    private static double[] calibrationCoordinatePair(String description, String cell) {
        Matcher pair = Pattern.compile("\\(\\s*([^,()]+)\\s*,\\s*([^,()]+)\\s*\\)")
                .matcher(cell.trim());
        assertTrue(description + " must be a numeric (X, Y) pair: " + cell, pair.matches());
        return new double[]{finiteCalibrationNumber(description + " X", pair.group(1)),
                finiteCalibrationNumber(description + " Y", pair.group(2))};
    }

    private static double finiteCalibrationNumber(String description, String text) {
        double value;
        try {
            value = Double.parseDouble(text.trim());
        } catch (NumberFormatException invalid) {
            throw new AssertionError(description + " must be numeric: " + text, invalid);
        }
        assertTrue(description + " must be finite", Double.isFinite(value));
        return value;
    }

    /** Shared by the full Build contract and its line-ending regression. */
    private static void validateBuildNotice(String markdown, String fileName, List<String> failures) {
        int noticeStart = markdown.indexOf("\nNotice:\n");
        int filesStart = markdown.indexOf("## Files in this checkpoint");
        if (noticeStart < 0 || filesStart <= noticeStart) {
            failures.add(fileName + ": missing bounded Notice section");
        } else {
            String notice = markdown.substring(noticeStart, filesStart);
            int observations = matcherCount(Pattern.compile("(?m)^- ").matcher(notice));
            if (observations < 1 || observations > 3) {
                failures.add(fileName + ": Notice must contain one to three observations, found "
                        + observations);
            }
        }
    }

    private static String markdownTableRow(String markdown, String criterion) {
        String prefix = "| **" + criterion + "** |";
        for (String line : markdown.split("\\R")) {
            if (line.startsWith(prefix)) {
                return line;
            }
        }
        fail("Missing documentation-criteria row " + criterion);
        return "";
    }

    private static void assertContainsAll(String contract, String source, String... required) {
        String normalized = source.replaceAll("\\s+", " ").toLowerCase(Locale.ROOT);
        for (String token : required) {
            assertTrue(contract + " is missing " + token,
                    normalized.contains(token.toLowerCase(Locale.ROOT)));
        }
    }

    private static int validateVisualCallouts(Path repositoryRoot,
                                              Path page,
                                              String markdown,
                                              List<String> areas,
                                              List<String> failures) {
        String relative = repositoryRelativePath(repositoryRoot, page);
        String[] lines = markdown.replace("\r\n", "\n").replace('\r', '\n')
                .split("\n", -1);
        MarkdownIntegrity.Fence openFence = null;
        int callouts = 0;
        int conceptCallouts = 0;
        int calloutsInSection = 0;

        for (int index = 0; index < lines.length; index++) {
            String line = lines[index];
            String trimmed = line.trim();
            if (openFence != null) {
                if (MarkdownIntegrity.isFenceClose(line, openFence)) {
                    openFence = null;
                }
                continue;
            }
            MarkdownIntegrity.Fence opening = MarkdownIntegrity.fenceOpening(line, index + 1);
            if (opening != null) {
                openFence = opening;
                continue;
            }
            if (trimmed.startsWith("## ") && !trimmed.startsWith("### ")) {
                calloutsInSection = 0;
                continue;
            }

            Matcher any = ANY_CALLOUT_START.matcher(trimmed);
            if (!any.matches()) {
                continue;
            }

            callouts++;
            calloutsInSection++;
            if (calloutsInSection > 1) {
                failures.add(relative + ":" + (index + 1)
                        + ": more than one callout in one H2 section");
            }

            String marker = any.group(1);
            String title = any.group(3);
            if (marker.startsWith("???")) {
                if (title == null || !title.startsWith("Optional: ")) {
                    failures.add(relative + ":" + (index + 1)
                            + ": collapsible material must be explicitly titled Optional");
                }
            } else {
                Matcher approved = CALLOUT_START.matcher(trimmed);
                if (!approved.matches()) {
                    failures.add(relative + ":" + (index + 1)
                            + ": callout must use one approved type and a visible semantic title");
                } else {
                    String type = approved.group(1);
                    title = approved.group(2);
                    String expectedPrefix;
                    if ("info".equals(type)) {
                        expectedPrefix = "New concept: ";
                        conceptCallouts++;
                    } else if ("warning".equals(type)) {
                        expectedPrefix = "Warning: ";
                    } else if ("danger".equals(type)) {
                        expectedPrefix = "Danger: ";
                    } else if ("success".equals(type)) {
                        expectedPrefix = "Checkpoint: ";
                    } else {
                        expectedPrefix = "Tip: ";
                    }
                    if (!title.startsWith(expectedPrefix)
                            || title.length() == expectedPrefix.length()) {
                        failures.add(relative + ":" + (index + 1)
                                + ": " + type + " callout title must start with "
                                + expectedPrefix);
                    }
                }
            }

            int bodyEnd = index + 1;
            StringBuilder body = new StringBuilder();
            int paragraphs = 0;
            boolean paragraphOpen = false;
            while (bodyEnd < lines.length) {
                String bodyLine = lines[bodyEnd];
                if (bodyLine.trim().isEmpty()) {
                    if (paragraphOpen) {
                        paragraphs++;
                        paragraphOpen = false;
                    }
                    body.append('\n');
                    bodyEnd++;
                    continue;
                }
                if (!(bodyLine.startsWith("    ") || bodyLine.startsWith("\t"))) {
                    break;
                }
                paragraphOpen = true;
                body.append(bodyLine.trim()).append('\n');
                bodyEnd++;
            }
            if (paragraphOpen) {
                paragraphs++;
            }
            if (body.toString().trim().isEmpty()) {
                failures.add(relative + ":" + (index + 1) + ": callout body is empty");
            }
            if (proseWordCount(body.toString()) > 80) {
                failures.add(relative + ":" + (index + 1)
                        + ": callout exceeds 80 words");
            }
            if (paragraphs > 2) {
                failures.add(relative + ":" + (index + 1)
                        + ": callout exceeds two paragraphs");
            }

            int next = bodyEnd;
            while (next < lines.length && lines[next].trim().isEmpty()) {
                next++;
            }
            if (next < lines.length
                    && ANY_CALLOUT_START.matcher(lines[next].trim()).matches()) {
                failures.add(relative + ":" + (index + 1)
                        + ": callouts must not be adjacent");
            }
            index = bodyEnd - 1;
        }

        if ((areas.contains("Get Started") || areas.contains("Learn"))
                && conceptCallouts > 3) {
            failures.add(relative + ": first-contact/Learn page has " + conceptCallouts
                    + " concept callouts; maximum is 3");
        } else if (areas.contains("Build") && conceptCallouts > 1) {
            failures.add(relative + ": Build page has " + conceptCallouts
                    + " concept callouts; maximum is 1");
        }
        return callouts;
    }

    private static int validateHighlightedJavaFences(Path repositoryRoot,
                                                     Path page,
                                                     String markdown,
                                                     List<String> failures) {
        String relative = repositoryRelativePath(repositoryRoot, page);
        String[] lines = markdown.replace("\r\n", "\n").replace('\r', '\n')
                .split("\n", -1);
        int highlightedFences = 0;
        MarkdownIntegrity.Fence enclosingFence = null;
        for (int index = 0; index < lines.length; index++) {
            String line = lines[index];
            if (enclosingFence != null) {
                if (MarkdownIntegrity.isFenceClose(line, enclosingFence)) {
                    enclosingFence = null;
                }
                continue;
            }

            MarkdownIntegrity.Fence fenceOpening =
                    MarkdownIntegrity.fenceOpening(line, index + 1);
            if (fenceOpening == null) {
                continue;
            }
            Matcher opening = HIGHLIGHTED_JAVA_FENCE.matcher(line.trim());
            if (!opening.matches()) {
                if (line.trim().startsWith(FENCE + "java")
                        && line.contains("hl_lines=")) {
                    failures.add(relative + ":" + (index + 1)
                            + ": hl_lines must be a space-separated list of positive integers");
                }
                enclosingFence = fenceOpening;
                continue;
            }
            highlightedFences++;
            int closing = index + 1;
            while (closing < lines.length
                    && !MarkdownIntegrity.isFenceClose(lines[closing], fenceOpening)) {
                closing++;
            }
            if (closing == lines.length) {
                failures.add(relative + ":" + (index + 1)
                        + ": highlighted Java fence is not closed");
                continue;
            }

            int displayedLines = closing - index - 1;
            Set<Integer> highlighted = new LinkedHashSet<Integer>();
            for (String token : opening.group(1).trim().split(" +")) {
                int lineNumber;
                try {
                    lineNumber = Integer.parseInt(token);
                } catch (NumberFormatException invalidNumber) {
                    failures.add(relative + ":" + (index + 1)
                            + ": highlighted line number is too large: " + token);
                    continue;
                }
                if (lineNumber < 1 || lineNumber > displayedLines) {
                    failures.add(relative + ":" + (index + 1)
                            + ": highlighted line " + lineNumber
                            + " is outside this " + displayedLines + "-line excerpt");
                } else if (!highlighted.add(lineNumber)) {
                    failures.add(relative + ":" + (index + 1)
                            + ": highlighted line " + lineNumber + " is duplicated");
                }
            }
            if (highlighted.size() > 10) {
                failures.add(relative + ":" + (index + 1)
                        + ": highlight more than ten lines only by splitting the teaching excerpt");
            }
            index = closing;
        }
        return highlightedFences;
    }

    private static void validateBuildSources(Path repositoryRoot,
                                             String pageName,
                                             String markdown,
                                             List<String> failures) throws IOException {
        Set<String> completeSources = new LinkedHashSet<String>();
        boolean linksMain = false;
        boolean linksTest = false;
        int manifestStart = markdown.indexOf("## Files in this checkpoint");
        int manifestEnd = markdown.indexOf("## Software checkpoint:", manifestStart);
        String manifest = manifestStart >= 0 && manifestEnd > manifestStart
                ? markdown.substring(manifestStart, manifestEnd)
                : "";
        Matcher links = COMPLETE_SOURCE.matcher(markdown);
        while (links.find()) {
            String sourcePath = links.group(1);
            completeSources.add(sourcePath);
            linksMain |= sourcePath.startsWith("TeamCode/src/main/java/");
            linksTest |= sourcePath.startsWith("TeamCode/src/test/java/");
            if (!Files.exists(repositoryRoot.resolve(sourcePath))) {
                failures.add(pageName + ": complete source link is missing " + sourcePath);
            }
        }
        if (completeSources.size() < 2 || !linksMain || !linksTest) {
            failures.add(pageName + ": complete-source manifest needs main and test authorities");
        }

        Matcher excerpts = SOURCE_EXCERPT.matcher(markdown);
        int excerptCount = 0;
        boolean excerptsMain = false;
        boolean excerptsTest = false;
        while (excerpts.find()) {
            excerptCount++;
            String sourcePath = excerpts.group(1).trim();
            String snippet = normalizeExcerpt(excerpts.group(2));
            int lines = snippet.isEmpty() ? 0 : snippet.split("\\n", -1).length;
            if (lines < 1 || lines > 12) {
                failures.add(pageName + ": source excerpt must contain 1–12 lines, found "
                        + lines + " for " + sourcePath);
            }
            Path source = repositoryRoot.resolve(sourcePath).toAbsolutePath().normalize();
            if (!source.startsWith(repositoryRoot.toAbsolutePath().normalize())
                    || !Files.isRegularFile(source)
                    || !source.getFileName().toString().endsWith(".java")) {
                failures.add(pageName + ": invalid source excerpt path " + sourcePath);
            } else if (!containsDedentedBlock(readUtf8(source), snippet)) {
                failures.add(pageName + ": excerpt is not exact contiguous source from "
                        + sourcePath);
            }
            if (!manifest.contains(source.getFileName().toString())) {
                failures.add(pageName + ": checkpoint manifest does not name excerpt authority "
                        + source.getFileName());
            }
            excerptsMain |= sourcePath.startsWith("TeamCode/src/main/java/");
            excerptsTest |= sourcePath.startsWith("TeamCode/src/test/java/");
        }

        int javaFences = matcherCount(JAVA_FENCE.matcher(markdown));
        if (excerptCount < 2 || javaFences != excerptCount || !excerptsMain || !excerptsTest) {
            failures.add(pageName + ": expected exact main and test source excerpts; "
                    + "excerpts=" + excerptCount + ", Java fences=" + javaFences);
        }
    }

    private static int literalCount(String text, String literal) {
        int count = 0;
        int from = 0;
        while (true) {
            int found = text.indexOf(literal, from);
            if (found < 0) {
                return count;
            }
            count++;
            from = found + literal.length();
        }
    }

    private static int matcherCount(Matcher matcher) {
        int count = 0;
        while (matcher.find()) {
            count++;
        }
        return count;
    }

    private static String normalizeExcerpt(String text) {
        String[] lines = text.replace("\r\n", "\n").replace('\r', '\n').split("\n", -1);
        int first = 0;
        while (first < lines.length && lines[first].trim().isEmpty()) {
            first++;
        }
        int last = lines.length;
        while (last > first && lines[last - 1].trim().isEmpty()) {
            last--;
        }
        int commonIndent = Integer.MAX_VALUE;
        for (int index = first; index < last; index++) {
            if (!lines[index].trim().isEmpty()) {
                commonIndent = Math.min(commonIndent, leadingSpaces(lines[index]));
            }
        }
        if (commonIndent == Integer.MAX_VALUE) {
            commonIndent = 0;
        }
        StringBuilder normalized = new StringBuilder();
        for (int index = first; index < last; index++) {
            if (normalized.length() > 0) {
                normalized.append('\n');
            }
            String line = lines[index];
            int remove = Math.min(commonIndent, leadingSpaces(line));
            normalized.append(line.substring(remove));
        }
        return normalized.toString();
    }

    private static int leadingSpaces(String line) {
        int count = 0;
        while (count < line.length() && line.charAt(count) == ' ') {
            count++;
        }
        return count;
    }

    private static boolean containsDedentedBlock(String source, String normalizedSnippet) {
        if (normalizedSnippet.isEmpty()) {
            return false;
        }
        String[] sourceLines = source.replace("\r\n", "\n").replace('\r', '\n')
                .split("\n", -1);
        int snippetLineCount = normalizedSnippet.split("\n", -1).length;
        for (int start = 0; start + snippetLineCount <= sourceLines.length; start++) {
            StringBuilder candidate = new StringBuilder();
            for (int index = 0; index < snippetLineCount; index++) {
                if (index > 0) {
                    candidate.append('\n');
                }
                candidate.append(sourceLines[start + index]);
            }
            if (normalizeExcerpt(candidate.toString()).equals(normalizedSnippet)) {
                return true;
            }
        }
        return false;
    }

    private static Path configuredDocsRoot(Path repositoryRoot, String config) {
        Matcher assignment = Pattern.compile(
                "(?m)^[ \\t]*docs_dir[ \\t]*=[ \\t]*\"([^\"]+)\"[ \\t]*$")
                .matcher(config);
        assertTrue("zensical.toml must declare one active docs_dir", assignment.find());
        String configuredPath = assignment.group(1);
        assertTrue("zensical.toml must not declare multiple active docs_dir values",
                !assignment.find());
        return repositoryRoot.resolve(configuredPath).toAbsolutePath().normalize();
    }

    private static void assertNoStaleFrameworkBranding(
            final Path repositoryRoot,
            Path authoredSourceRoot,
            List<Path> supportingFiles) throws IOException {
        final Pattern staleBranding = Pattern.compile(
                "(?i)\\bPhoenix framework\\b|edu\\.ftcphoenix|learn-phoenix|phoenixJavadocs");
        final List<String> failures = new ArrayList<String>();

        for (Path supportingFile : supportingFiles) {
            collectStaleFrameworkBranding(
                    repositoryRoot,
                    supportingFile,
                    staleBranding,
                    failures);
        }
        Files.walkFileTree(authoredSourceRoot, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attributes)
                    throws IOException {
                collectStaleFrameworkBranding(
                        repositoryRoot,
                        file,
                        staleBranding,
                        failures);
                return FileVisitResult.CONTINUE;
            }
        });

        assertTrue("Stale framework branding remains: " + failures, failures.isEmpty());
    }

    private static void collectStaleFrameworkBranding(
            Path repositoryRoot,
            Path file,
            Pattern staleBranding,
            List<String> failures) throws IOException {
        int lineNumber = 0;
        for (String line : Files.readAllLines(file, StandardCharsets.UTF_8)) {
            lineNumber++;
            Matcher match = staleBranding.matcher(line);
            if (match.find()) {
                failures.add(repositoryRelativePath(repositoryRoot, file) + ":" + lineNumber
                        + ": " + match.group());
            }
        }
    }

    private static void assertNoProductionApplicationReferences(
            final Path repositoryRoot,
            Path frameworkSourceRoot,
            List<Path> supportingFiles) throws IOException {
        final Pattern applicationReference = Pattern.compile("(?i)phoenix");
        final List<String> failures = new ArrayList<String>();

        for (Path supportingFile : supportingFiles) {
            collectProductionApplicationReferences(
                    repositoryRoot,
                    supportingFile,
                    applicationReference,
                    failures);
        }
        collectProductionApplicationReferencesUnder(
                repositoryRoot,
                frameworkSourceRoot,
                applicationReference,
                failures);
        collectProductionApplicationReferencesUnder(
                repositoryRoot,
                repositoryRoot.resolve(".agents"),
                applicationReference,
                failures);
        collectProductionApplicationReferencesUnder(
                repositoryRoot,
                repositoryRoot.resolve(".github"),
                applicationReference,
                failures);
        try (DirectoryStream<Path> rootEntries = Files.newDirectoryStream(repositoryRoot)) {
            for (Path entry : rootEntries) {
                if (Files.isRegularFile(entry)
                        && !entry.getFileName().toString().equals(
                                "FRAMEWORK_IMPROVEMENT_TRACKER.md")
                        && isMaintainedSharedText(entry)) {
                    collectProductionApplicationReferences(
                            repositoryRoot,
                            entry,
                            applicationReference,
                            failures);
                }
            }
        }

        assertTrue("Production application references remain in shared Sushi surfaces: "
                + failures, failures.isEmpty());
    }

    private static void collectProductionApplicationReferencesUnder(
            final Path repositoryRoot,
            Path root,
            final Pattern applicationReference,
            final List<String> failures) throws IOException {
        if (!Files.exists(root)) {
            return;
        }
        if (Files.isRegularFile(root)) {
            collectProductionApplicationReferences(
                    repositoryRoot,
                    root,
                    applicationReference,
                    failures);
            return;
        }
        Files.walkFileTree(root, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attributes)
                    throws IOException {
                if (isMaintainedSharedText(file)) {
                    collectProductionApplicationReferences(
                            repositoryRoot,
                            file,
                            applicationReference,
                            failures);
                }
                return FileVisitResult.CONTINUE;
            }
        });
    }

    private static boolean isMaintainedSharedText(Path file) {
        String fileName = file.getFileName().toString().toLowerCase(Locale.ROOT);
        return fileName.endsWith(".java")
                || fileName.endsWith(".md")
                || fileName.endsWith(".toml")
                || fileName.endsWith(".gradle")
                || fileName.endsWith(".properties")
                || fileName.endsWith(".txt")
                || fileName.endsWith(".yml")
                || fileName.endsWith(".yaml")
                || fileName.endsWith(".json")
                || fileName.endsWith(".xml");
    }

    private static void collectProductionApplicationReferences(
            Path repositoryRoot,
            Path file,
            Pattern applicationReference,
            List<String> failures) throws IOException {
        int lineNumber = 0;
        for (String line : Files.readAllLines(file, StandardCharsets.UTF_8)) {
            lineNumber++;
            String semanticText = line.replace("2025-PhoenixPedro", "");
            Matcher match = applicationReference.matcher(semanticText);
            if (match.find()) {
                failures.add(repositoryRelativePath(repositoryRoot, file) + ":" + lineNumber
                        + ": " + match.group());
            }
        }
    }

    private static String repositoryRelativePath(Path repositoryRoot, Path file) {
        return repositoryRoot.toAbsolutePath().normalize()
                .relativize(file.toAbsolutePath().normalize())
                .toString()
                .replace('\\', '/');
    }

    private static int proseWordCount(Path path) throws IOException {
        return proseWordCount(readUtf8(path));
    }

    private static int proseWordCount(String markdown) {
        boolean insideFence = false;
        StringBuilder prose = new StringBuilder();
        for (String line : markdown.replace("\r\n", "\n").split("\n", -1)) {
            String trimmed = line.trim();
            if (trimmed.startsWith(FENCE) || trimmed.startsWith("~~~")) {
                insideFence = !insideFence;
            } else if (!insideFence) {
                prose.append(' ').append(trimmed);
            }
        }
        String text = prose.toString().trim();
        return text.isEmpty() ? 0 : text.split("\\s+").length;
    }

    private static int javaFenceCount(Path path) throws IOException {
        return javaFenceCount(readUtf8(path));
    }

    private static int javaFenceCount(String markdown) {
        return matcherCount(JAVA_FENCE.matcher(markdown));
    }

    private static int displayedJavaLineCount(Path path) throws IOException {
        boolean insideJava = false;
        int collapsedDetailsDepth = 0;
        int count = 0;
        for (String line : Files.readAllLines(path, StandardCharsets.UTF_8)) {
            String trimmed = line.trim();
            if (trimmed.equalsIgnoreCase("<details>")) {
                collapsedDetailsDepth++;
            } else if (trimmed.equalsIgnoreCase("</details>")
                    && collapsedDetailsDepth > 0) {
                collapsedDetailsDepth--;
            }
            if (!insideJava && JAVA_FENCE.matcher(trimmed).matches()) {
                insideJava = true;
            } else if (insideJava && trimmed.equals(FENCE)) {
                insideJava = false;
            } else if (insideJava && collapsedDetailsDepth == 0 && !trimmed.isEmpty()) {
                count++;
            }
        }
        return count;
    }

    private static void assertNoFailures(List<String> failures) {
        assertTrue("Markdown integrity failures:\n" + joinLines(failures), failures.isEmpty());
    }

    private static void assertFailureContains(List<String> failures, String expectedText) {
        for (String failure : failures) {
            if (failure.contains(expectedText)) {
                return;
            }
        }
        fail("Expected a failure containing '" + expectedText + "' but got:\n"
                + joinLines(failures));
    }

    private static void assertFailureStartsWith(List<String> failures, String expectedText) {
        for (String failure : failures) {
            if (failure.startsWith(expectedText)) {
                return;
            }
        }
        fail("Expected a failure starting with '" + expectedText + "' but got:\n"
                + joinLines(failures));
    }

    private static String joinLines(List<String> lines) {
        StringBuilder joined = new StringBuilder();
        for (String line : lines) {
            if (joined.length() > 0) {
                joined.append('\n');
            }
            joined.append(line);
        }
        return joined.toString();
    }

    private static final class ShellCommandTabs {
        private static final String WINDOWS_LABEL = "=== \"Windows\"";
        private static final String MAC_OS_LABEL = "=== \"macOS\"";
        private static final String WINDOWS_FENCE = "    ```powershell";
        private static final String MAC_OS_FENCE = "    ```bash";
        private static final String TAB_FENCE_CLOSE = "    ```";

        private static final Set<String> SHELL_LANGUAGES = new HashSet<String>(Arrays.asList(
                "bash",
                "bat",
                "batch",
                "cmd",
                "console",
                "fish",
                "powershell",
                "ps1",
                "pwsh",
                "sh",
                "shell",
                "shell-session",
                "zsh"));

        private static final Pattern WINDOWS_GRADLE = Pattern.compile(
                "(?i)(?<!\\S)\\.\\\\gradlew\\.bat(?=\\s|$)");
        private static final Pattern MAC_OS_GRADLE = Pattern.compile(
                "(?<!\\S)\\./gradlew(?=\\s|$)");
        private static final Pattern WINDOWS_VENV_PYTHON = Pattern.compile(
                "(?i)(?<!\\S)(\\S*?)[\\\\/]Scripts[\\\\/]python\\.exe(?=\\s|$)");
        private static final Pattern MAC_OS_VENV_PYTHON = Pattern.compile(
                "(?<!\\S)\\S*/bin/python(?=\\s|$)");
        private static final Pattern WINDOWS_PYTHON = Pattern.compile(
                "(?<!\\S)python(?=\\s|$)");
        private static final Pattern MAC_OS_PYTHON = Pattern.compile(
                "(?<!\\S)python3(?=\\s|$)");
        private static final Pattern GRADLE_TESTS_OPTION = Pattern.compile(
                "(?<!\\S)--tests(?=\\s|=|$)");

        private ShellCommandTabs() {
        }

        private static Validation validate(String page, String markdown) {
            String[] lines = markdown.replace("\r\n", "\n").replace('\r', '\n')
                    .split("\n", -1);
            List<FenceBlock> blocks = fenceBlocks(lines);
            Map<Integer, FenceBlock> blockByOpeningLine =
                    new HashMap<Integer, FenceBlock>();
            for (FenceBlock block : blocks) {
                blockByOpeningLine.put(block.openingLine, block);
            }

            List<String> failures = new ArrayList<String>();
            List<Pair> pairs = new ArrayList<Pair>();
            Set<FenceBlock> claimed = new HashSet<FenceBlock>();
            for (FenceBlock windowsBlock : blocks) {
                if (claimed.contains(windowsBlock)
                        || !("powershell".equals(windowsBlock.language)
                        || lineEquals(lines, windowsBlock.openingLine - 2, WINDOWS_LABEL))) {
                    continue;
                }
                claimed.add(windowsBlock);
                boolean exactPair = true;

                if (!lineEquals(lines, windowsBlock.openingLine - 2, WINDOWS_LABEL)) {
                    failures.add(location(page, windowsBlock.openingLine)
                            + "standalone published shell fence: labels/order must be exactly "
                            + "Windows then macOS; expected exact tab label `" + WINDOWS_LABEL
                            + "` two lines before the PowerShell fence");
                    exactPair = false;
                }
                if (!lineEquals(lines, windowsBlock.openingLine - 1, "")) {
                    failures.add(location(page, windowsBlock.openingLine)
                            + "the Windows label must be followed by one blank line before its "
                            + "fence");
                    exactPair = false;
                }
                if (!WINDOWS_FENCE.equals(lines[windowsBlock.openingLine])) {
                    failures.add(location(page, windowsBlock.openingLine)
                            + "the Windows tab must open exactly as four spaces plus "
                            + "```powershell; found `"
                            + visibleLine(lines[windowsBlock.openingLine]) + "`");
                    exactPair = false;
                }
                exactPair &= validateBlock(
                        page, lines, windowsBlock, "Windows", failures);

                if (windowsBlock.closingLine < 0) {
                    failures.add(location(page, windowsBlock.openingLine)
                            + "the Windows shell fence is unclosed, so the macOS companion "
                            + "cannot be parsed");
                    continue;
                }
                if (!lineEquals(lines, windowsBlock.closingLine + 1, "")) {
                    failures.add(location(page, windowsBlock.closingLine)
                            + "the Windows fence must be followed by one blank line before the "
                            + "macOS label");
                    exactPair = false;
                }
                if (!lineEquals(lines, windowsBlock.closingLine + 2, MAC_OS_LABEL)) {
                    failures.add(location(page, windowsBlock.closingLine + 2)
                            + "labels/order must be exactly Windows then macOS; expected exact "
                            + "tab label `" + MAC_OS_LABEL + "`, found `"
                            + visibleLine(lineAt(lines, windowsBlock.closingLine + 2)) + "`");
                    exactPair = false;
                }
                if (!lineEquals(lines, windowsBlock.closingLine + 3, "")) {
                    failures.add(location(page, windowsBlock.closingLine + 3)
                            + "the macOS label must be followed by one blank line before its "
                            + "fence");
                    exactPair = false;
                }

                int expectedMacOsOpening = windowsBlock.closingLine + 4;
                FenceBlock macOsBlock = blockByOpeningLine.get(expectedMacOsOpening);
                if (macOsBlock == null) {
                    failures.add(location(page, expectedMacOsOpening)
                            + "the macOS tab must contain exactly one four-space-indented "
                            + "```bash fence immediately after its label");
                    continue;
                }
                claimed.add(macOsBlock);
                if (!MAC_OS_FENCE.equals(lines[macOsBlock.openingLine])) {
                    failures.add(location(page, macOsBlock.openingLine)
                            + "the macOS tab must open exactly as four spaces plus ```bash; "
                            + "found `" + visibleLine(lines[macOsBlock.openingLine]) + "`");
                    exactPair = false;
                }
                exactPair &= validateBlock(
                        page, lines, macOsBlock, "macOS", failures);
                if (macOsBlock.closingLine < 0) {
                    failures.add(location(page, macOsBlock.openingLine)
                            + "the macOS shell fence is unclosed; close it with four spaces "
                            + "plus ```");
                    continue;
                }

                if (!exactPair) {
                    continue;
                }
                List<String> windowsBody = bodyLines(lines, windowsBlock);
                List<String> macOsBody = bodyLines(lines, macOsBlock);
                validateContinuations(
                        page, windowsBlock.openingLine, windowsBody, true, failures);
                validateContinuations(
                        page, macOsBlock.openingLine, macOsBody, false, failures);
                validatePlatformTokens(
                        page, windowsBlock.openingLine, windowsBody, true, failures);
                validatePlatformTokens(
                        page, macOsBlock.openingLine, macOsBody, false, failures);
                String normalizedWindows = normalizeCommands(
                        page, windowsBlock.openingLine, windowsBody, true, failures);
                String normalizedMacOs = normalizeCommands(
                        page, macOsBlock.openingLine, macOsBody, false, failures);
                validateGradleTestSelectors(
                        page, windowsBlock.openingLine, normalizedWindows, "Windows", failures);
                validateGradleTestSelectors(
                        page, macOsBlock.openingLine, normalizedMacOs, "macOS", failures);
                Pair pair = new Pair(normalizedWindows, normalizedMacOs);
                pairs.add(pair);
                if (!normalizedWindows.equals(normalizedMacOs)) {
                    failures.add(location(page, windowsBlock.openingLine)
                            + "normalized commands differ; Windows=`"
                            + oneLine(normalizedWindows) + "`, macOS=`"
                            + oneLine(normalizedMacOs) + "`. Keep every other argument and "
                            + "command in the same order");
                }
            }

            for (FenceBlock block : blocks) {
                if (SHELL_LANGUAGES.contains(block.language) && !claimed.contains(block)) {
                    failures.add(location(page, block.openingLine)
                            + "standalone published shell fence `" + block.language
                            + "`; wrap every command in one exact Windows then macOS tab pair");
                }
            }
            return new Validation(pairs, failures);
        }

        private static boolean validateBlock(String page,
                                             String[] lines,
                                             FenceBlock block,
                                             String platform,
                                             List<String> failures) {
            boolean valid = true;
            if (block.closingLine < 0) {
                return false;
            }
            if (!TAB_FENCE_CLOSE.equals(lines[block.closingLine])) {
                failures.add(location(page, block.closingLine)
                        + "the " + platform + " tab fence must close as four spaces plus ```; "
                        + "found `" + visibleLine(lines[block.closingLine]) + "`");
                valid = false;
            }
            boolean hasExecutableCommand = false;
            boolean[] insidePowerShellBlockComment = new boolean[1];
            for (int line = block.openingLine + 1; line < block.closingLine; line++) {
                if (lines[line].isEmpty()) {
                    continue;
                }
                if (!lines[line].startsWith("    ")) {
                    failures.add(location(page, line)
                            + "every " + platform + " command line must remain indented by at "
                            + "least four spaces inside its tab");
                    valid = false;
                } else {
                    String command = lines[line].substring(4).trim();
                    if ("Windows".equals(platform)) {
                        command = powerShellExecutableText(
                                command, insidePowerShellBlockComment).trim();
                    }
                    if (!command.isEmpty() && !command.startsWith("#")) {
                        hasExecutableCommand = true;
                    }
                }
            }
            if (!hasExecutableCommand) {
                failures.add(location(page, block.openingLine)
                        + "the " + platform + " shell fence must contain an executable "
                        + "command, not only comments");
                valid = false;
            }
            return valid;
        }

        private static String powerShellExecutableText(String line,
                                                       boolean[] insideBlockComment) {
            StringBuilder executable = new StringBuilder();
            int index = 0;
            char quote = 0;
            while (index < line.length()) {
                if (insideBlockComment[0]) {
                    int commentEnd = line.indexOf("#>", index);
                    if (commentEnd < 0) {
                        break;
                    }
                    insideBlockComment[0] = false;
                    index = commentEnd + 2;
                    continue;
                }

                char character = line.charAt(index);
                if (character == '`' && quote != '\'') {
                    executable.append(character);
                    if (index + 1 < line.length()) {
                        executable.append(line.charAt(index + 1));
                        index += 2;
                    } else {
                        index++;
                    }
                } else if (quote != 0) {
                    executable.append(character);
                    if (character == quote) {
                        if (index + 1 < line.length()
                                && line.charAt(index + 1) == quote) {
                            executable.append(quote);
                            index++;
                        } else {
                            quote = 0;
                        }
                    }
                    index++;
                } else if (character == '\'' || character == '"') {
                    quote = character;
                    executable.append(character);
                    index++;
                } else if (character == '#') {
                    break;
                } else if (character == '<' && index + 1 < line.length()
                        && line.charAt(index + 1) == '#') {
                    insideBlockComment[0] = true;
                    index += 2;
                } else {
                    executable.append(character);
                    index++;
                }
            }
            return executable.toString();
        }

        private static void validatePlatformTokens(String page,
                                                   int openingLine,
                                                   List<String> body,
                                                   boolean windows,
                                                   List<String> failures) {
            for (int index = 0; index < body.size(); index++) {
                String line = body.get(index).trim();
                int sourceLine = openingLine + index + 1;
                if (windows) {
                    rejectToken(page, sourceLine, line, MAC_OS_GRADLE,
                            "POSIX-only token `./gradlew`", failures);
                    rejectToken(page, sourceLine, line, MAC_OS_PYTHON,
                            "POSIX-only token `python3`", failures);
                    rejectToken(page, sourceLine, line, MAC_OS_VENV_PYTHON,
                            "POSIX-only token `bin/python`", failures);
                    if (continuationMarkerIndex(line, '\\') >= 0) {
                        failures.add(location(page, sourceLine)
                                + "Windows command contains POSIX continuation `\\`; use the "
                                + "PowerShell backtick continuation");
                    }
                } else {
                    rejectToken(page, sourceLine, line, WINDOWS_GRADLE,
                            "Windows-only token `.\\gradlew.bat`", failures);
                    rejectToken(page, sourceLine, line, WINDOWS_PYTHON,
                            "Windows-only token `python`", failures);
                    rejectToken(page, sourceLine, line, WINDOWS_VENV_PYTHON,
                            "Windows-only token `Scripts/python.exe`", failures);
                    if (continuationMarkerIndex(line, '`') >= 0) {
                        failures.add(location(page, sourceLine)
                                + "macOS command contains Windows-only PowerShell continuation "
                                + "```; use the POSIX `\\` continuation");
                    }
                }
            }
        }

        private static void validateContinuations(String page,
                                                  int openingLine,
                                                  List<String> body,
                                                  boolean windows,
                                                  List<String> failures) {
            char continuation = windows ? '`' : '\\';
            String platform = windows ? "Windows" : "macOS";
            for (int index = 0; index < body.size(); index++) {
                String line = body.get(index);
                int marker = continuationMarkerIndex(line, continuation);
                if (marker < 0) {
                    continue;
                }
                int sourceLine = openingLine + index + 1;
                if (marker != line.length() - 1) {
                    failures.add(location(page, sourceLine)
                            + platform + " continuation marker must be the final character; "
                            + "remove trailing whitespace");
                }
                if (index + 1 < body.size() && body.get(index + 1).trim().isEmpty()) {
                    failures.add(location(page, sourceLine)
                            + platform + " continuation cannot skip a blank line");
                }
            }
        }

        private static int continuationMarkerIndex(String line, char marker) {
            int end = line.length() - 1;
            while (end >= 0 && Character.isWhitespace(line.charAt(end))) {
                end--;
            }
            if (end < 0 || line.charAt(end) != marker) {
                return -1;
            }
            int start = end;
            while (start > 0 && line.charAt(start - 1) == marker) {
                start--;
            }
            return (end - start + 1) % 2 == 1 ? end : -1;
        }

        private static void rejectToken(String page,
                                        int lineNumber,
                                        String line,
                                        Pattern pattern,
                                        String description,
                                        List<String> failures) {
            if (pattern.matcher(line).find()) {
                failures.add(location(page, lineNumber)
                        + (description.startsWith("Windows") ? "macOS" : "Windows")
                        + " command contains " + description);
            }
        }

        private static String normalizeCommands(String page,
                                                int openingLine,
                                                List<String> body,
                                                boolean windows,
                                                List<String> failures) {
            char continuation = windows ? '`' : '\\';
            List<String> commands = new ArrayList<String>();
            StringBuilder command = new StringBuilder();
            boolean awaitingContinuation = false;
            for (String physicalLine : body) {
                String line = physicalLine.trim();
                if (line.isEmpty()) {
                    continue;
                }
                int marker = continuationMarkerIndex(line, continuation);
                boolean continues = marker >= 0;
                if (continues) {
                    line = line.substring(0, marker).trim();
                }
                if (command.length() > 0 && !line.isEmpty()) {
                    command.append(' ');
                }
                command.append(line);
                awaitingContinuation = continues;
                if (!continues) {
                    commands.add(normalizePlatformTokens(command.toString()));
                    command.setLength(0);
                }
            }
            if (command.length() > 0) {
                commands.add(normalizePlatformTokens(command.toString()));
            }
            if (awaitingContinuation) {
                failures.add(location(page, openingLine)
                        + (windows ? "Windows" : "macOS")
                        + " command ends with a continuation but has no following command line");
            }
            return joinLines(commands);
        }

        private static String normalizePlatformTokens(String command) {
            int executableEnd = 0;
            while (executableEnd < command.length()
                    && !Character.isWhitespace(command.charAt(executableEnd))) {
                executableEnd++;
            }
            String executable = command.substring(0, executableEnd);
            String normalizedExecutable = executable;
            if (WINDOWS_GRADLE.matcher(executable).matches()) {
                normalizedExecutable = "./gradlew";
            } else if (WINDOWS_PYTHON.matcher(executable).matches()) {
                normalizedExecutable = "python3";
            } else if (WINDOWS_VENV_PYTHON.matcher(executable).matches()) {
                normalizedExecutable = canonicalizeWindowsVenvPython(executable);
            }
            return normalizedExecutable + command.substring(executableEnd);
        }

        private static void validateGradleTestSelectors(String page,
                                                        int openingLine,
                                                        String commands,
                                                        String platform,
                                                        List<String> failures) {
            for (String command : commands.split("\\n", -1)) {
                for (GradleTestSelector selector : gradleTestSelectors(
                        command, "Windows".equals(platform))) {
                    if (selector.value == null || selector.value.trim().isEmpty()) {
                        failures.add(location(page, openingLine)
                                + platform + " Gradle --tests must be followed by a nonblank "
                                + "selector");
                    } else if (containsWildcard(selector.value) && !selector.quoted) {
                        failures.add(location(page, openingLine)
                                + platform + " Gradle --tests wildcard selector must be quoted; "
                                + "wrap it in single quotes so zsh does not expand it");
                    }
                }
            }
        }

        private static List<GradleTestSelector> gradleTestSelectors(String command,
                                                                    boolean windows) {
            List<GradleTestSelector> selectors = new ArrayList<GradleTestSelector>();
            command = shellCodeBeforeInlineComment(command, windows);
            if (!(command.equals("./gradlew") || command.startsWith("./gradlew "))) {
                return selectors;
            }
            Matcher option = GRADLE_TESTS_OPTION.matcher(command);
            while (option.find()) {
                int start = option.end();
                if (start < command.length() && command.charAt(start) == '=') {
                    start++;
                } else {
                    while (start < command.length()
                            && Character.isWhitespace(command.charAt(start))) {
                        start++;
                    }
                }
                if (start >= command.length() || command.charAt(start) == '-') {
                    selectors.add(new GradleTestSelector(null, false));
                    continue;
                }

                int end = start;
                char quote = 0;
                while (end < command.length()) {
                    char character = command.charAt(end);
                    if (quote == 0 && Character.isWhitespace(character)) {
                        break;
                    }
                    if (character == '\'' || character == '"') {
                        if (quote == 0) {
                            quote = character;
                        } else if (quote == character) {
                            quote = 0;
                        }
                    }
                    end++;
                }
                String token = command.substring(start, end);
                boolean quoted = token.length() >= 2
                        && ((token.charAt(0) == '\''
                        && token.charAt(token.length() - 1) == '\'')
                        || (token.charAt(0) == '"'
                        && token.charAt(token.length() - 1) == '"'));
                String value = quoted
                        ? token.substring(1, token.length() - 1)
                        : token;
                selectors.add(new GradleTestSelector(value, quoted));
            }
            return selectors;
        }

        private static String shellCodeBeforeInlineComment(String command,
                                                           boolean windows) {
            char quote = 0;
            char escape = windows ? '`' : '\\';
            for (int index = 0; index < command.length(); index++) {
                char character = command.charAt(index);
                if (character == escape && quote != '\'') {
                    index++;
                } else if (quote != 0) {
                    if (character == quote) {
                        if (index + 1 < command.length()
                                && command.charAt(index + 1) == quote) {
                            index++;
                        } else {
                            quote = 0;
                        }
                    }
                } else if (character == '\'' || character == '"') {
                    quote = character;
                } else if (character == '#') {
                    return command.substring(0, index).trim();
                }
            }
            return command;
        }

        private static boolean containsWildcard(String value) {
            return value.indexOf('*') >= 0
                    || value.indexOf('?') >= 0
                    || value.indexOf('[') >= 0;
        }

        private static String canonicalizeWindowsVenvPython(String command) {
            Matcher matcher = WINDOWS_VENV_PYTHON.matcher(command);
            StringBuffer normalized = new StringBuffer();
            while (matcher.find()) {
                String prefix = matcher.group(1).replace('\\', '/');
                matcher.appendReplacement(normalized,
                        Matcher.quoteReplacement(prefix + "/bin/python"));
            }
            matcher.appendTail(normalized);
            return normalized.toString();
        }

        private static List<String> bodyLines(String[] lines, FenceBlock block) {
            List<String> body = new ArrayList<String>();
            for (int line = block.openingLine + 1; line < block.closingLine; line++) {
                body.add(lines[line].isEmpty() ? "" : lines[line].substring(4));
            }
            return body;
        }

        private static List<FenceBlock> fenceBlocks(String[] lines) {
            List<FenceBlock> blocks = new ArrayList<FenceBlock>();
            for (int line = 0; line < lines.length; line++) {
                FenceBlock opening = fenceOpening(lines[line], line);
                if (opening == null) {
                    continue;
                }
                int closing = line + 1;
                boolean lowerContainerFence = false;
                while (closing < lines.length
                        && !isFenceClose(lines[closing], opening)) {
                    FenceBlock laterOpening = fenceOpening(lines[closing], closing);
                    if (laterOpening != null
                            && (laterOpening.blockquoteDepth < opening.blockquoteDepth
                            || (laterOpening.blockquoteDepth == opening.blockquoteDepth
                            && opening.listContentIndent >= 0
                            && laterOpening.markerIndent < opening.listContentIndent))) {
                        lowerContainerFence = true;
                        break;
                    }
                    closing++;
                }
                opening.closingLine = closing < lines.length && !lowerContainerFence
                        ? closing
                        : -1;
                blocks.add(opening);
                if (opening.closingLine < 0) {
                    if (lowerContainerFence) {
                        line = closing - 1;
                        continue;
                    }
                    break;
                }
                line = opening.closingLine;
            }
            return blocks;
        }

        private static FenceBlock fenceOpening(String line, int lineNumber) {
            FencePrefix prefix = fencePrefix(line);
            int start = prefix.markerStart;
            if (start < 0) {
                return null;
            }
            char marker = line.charAt(start);
            if (marker != '`' && marker != '~') {
                return null;
            }
            int markerLength = markerRunLength(line, start, marker);
            if (markerLength < 3) {
                return null;
            }
            String info = line.substring(start + markerLength).trim();
            return new FenceBlock(
                    lineNumber, marker, markerLength, fenceLanguage(info),
                    prefix.blockquoteDepth, prefix.markerStart,
                    prefix.hasListMarker ? prefix.markerStart : -1);
        }

        private static boolean isFenceClose(String line, FenceBlock opening) {
            FencePrefix prefix = fencePrefix(line);
            int start = prefix.markerStart;
            if (start < 0 || prefix.blockquoteDepth != opening.blockquoteDepth
                    || (opening.listContentIndent >= 0
                    && start < opening.listContentIndent)
                    || line.charAt(start) != opening.marker) {
                return false;
            }
            int length = markerRunLength(line, start, opening.marker);
            return length >= opening.markerLength
                    && line.substring(start + length).trim().isEmpty();
        }

        private static int markerRunLength(String line, int start, char marker) {
            int end = start;
            while (end < line.length() && line.charAt(end) == marker) {
                end++;
            }
            return end - start;
        }

        private static FencePrefix fencePrefix(String value) {
            int index = skipWhitespace(value, 0);
            int blockquoteDepth = 0;
            boolean hasListMarker = false;
            boolean foundContainer = true;
            while (index < value.length() && foundContainer) {
                foundContainer = false;
                if (value.charAt(index) == '>') {
                    blockquoteDepth++;
                    index = skipWhitespace(value, index + 1);
                    foundContainer = true;
                    continue;
                }
                int listEnd = listMarkerEnd(value, index);
                if (listEnd >= 0) {
                    hasListMarker = true;
                    index = skipWhitespace(value, listEnd);
                    foundContainer = true;
                }
            }
            return new FencePrefix(
                    index < value.length() ? index : -1,
                    blockquoteDepth, hasListMarker);
        }

        private static int listMarkerEnd(String value, int start) {
            char first = value.charAt(start);
            if ((first == '-' || first == '+' || first == '*')
                    && start + 1 < value.length()
                    && Character.isWhitespace(value.charAt(start + 1))) {
                return start + 1;
            }
            int end = start;
            while (end < value.length() && Character.isDigit(value.charAt(end))) {
                end++;
            }
            if (end == start || end + 1 >= value.length()
                    || (value.charAt(end) != '.' && value.charAt(end) != ')')
                    || !Character.isWhitespace(value.charAt(end + 1))) {
                return -1;
            }
            return end + 1;
        }

        private static int skipWhitespace(String value, int start) {
            int index = start;
            while (index < value.length()
                    && (value.charAt(index) == ' ' || value.charAt(index) == '\t')) {
                index++;
            }
            return index;
        }

        private static String fenceLanguage(String info) {
            if (info.isEmpty()) {
                return "";
            }
            String attribute = info;
            if (attribute.startsWith("{") && attribute.endsWith("}")) {
                attribute = attribute.substring(1, attribute.length() - 1).trim();
            }
            String token = attribute.split("[ \\t]", 2)[0].toLowerCase(Locale.ROOT);
            if (token.startsWith(".")) {
                token = token.substring(1);
            }
            if (token.endsWith("}")) {
                token = token.substring(0, token.length() - 1);
            }
            return token;
        }

        private static boolean lineEquals(String[] lines, int index, String expected) {
            return index >= 0 && index < lines.length && expected.equals(lines[index]);
        }

        private static String lineAt(String[] lines, int index) {
            return index >= 0 && index < lines.length ? lines[index] : "<end of file>";
        }

        private static String visibleLine(String line) {
            return line.isEmpty() ? "<blank>" : line.replace("\t", "\\t");
        }

        private static String location(String page, int zeroBasedLine) {
            return page + ":" + Math.max(1, zeroBasedLine + 1) + ": ";
        }

        private static String oneLine(String commands) {
            return commands.replace("\n", " | ");
        }

        private static final class Validation {
            private final List<Pair> pairs;
            private final List<String> failures;

            private Validation(List<Pair> pairs, List<String> failures) {
                this.pairs = pairs;
                this.failures = failures;
            }

            private boolean hasEquivalentGradleTestSelector(String expectedSelector) {
                if (!failures.isEmpty() || expectedSelector == null
                        || expectedSelector.trim().isEmpty()) {
                    return false;
                }
                for (Pair pair : pairs) {
                    if (!pair.normalizedWindows.equals(pair.normalizedMacOs)) {
                        continue;
                    }
                    for (String command : pair.normalizedWindows.split("\\n", -1)) {
                        for (GradleTestSelector selector : gradleTestSelectors(command, true)) {
                            if (expectedSelector.equals(selector.value)) {
                                return true;
                            }
                        }
                    }
                }
                return false;
            }
        }

        private static final class GradleTestSelector {
            private final String value;
            private final boolean quoted;

            private GradleTestSelector(String value, boolean quoted) {
                this.value = value;
                this.quoted = quoted;
            }
        }

        private static final class Pair {
            private final String normalizedWindows;
            private final String normalizedMacOs;

            private Pair(String normalizedWindows, String normalizedMacOs) {
                this.normalizedWindows = normalizedWindows;
                this.normalizedMacOs = normalizedMacOs;
            }
        }

        private static final class FenceBlock {
            private final int openingLine;
            private final char marker;
            private final int markerLength;
            private final String language;
            private final int blockquoteDepth;
            private final int markerIndent;
            private final int listContentIndent;
            private int closingLine;

            private FenceBlock(int openingLine,
                               char marker,
                               int markerLength,
                               String language,
                               int blockquoteDepth,
                               int markerIndent,
                               int listContentIndent) {
                this.openingLine = openingLine;
                this.marker = marker;
                this.markerLength = markerLength;
                this.language = language;
                this.blockquoteDepth = blockquoteDepth;
                this.markerIndent = markerIndent;
                this.listContentIndent = listContentIndent;
                this.closingLine = -1;
            }
        }

        private static final class FencePrefix {
            private final int markerStart;
            private final int blockquoteDepth;
            private final boolean hasListMarker;

            private FencePrefix(int markerStart,
                                int blockquoteDepth,
                                boolean hasListMarker) {
                this.markerStart = markerStart;
                this.blockquoteDepth = blockquoteDepth;
                this.hasListMarker = hasListMarker;
            }
        }
    }

    private static final class PageMetadata {
        private final boolean hasFrontMatter;
        private final boolean searchExcluded;
        private final List<String> tags;

        private PageMetadata(boolean hasFrontMatter,
                             boolean searchExcluded,
                             List<String> tags) {
            this.hasFrontMatter = hasFrontMatter;
            this.searchExcluded = searchExcluded;
            this.tags = new ArrayList<String>(tags);
        }
    }

    static final class MarkdownIntegrity {

        private static final Pattern URI_SCHEME =
                Pattern.compile("^[A-Za-z][A-Za-z0-9+.-]*:");
        private static final Pattern WINDOWS_ABSOLUTE =
                Pattern.compile("^[A-Za-z]:[/\\\\].*");
        private static final Pattern MARKDOWN_LINK_IN_HEADING =
                Pattern.compile("!?\\[([^]]*)]\\([^)]*\\)");
        private static final Pattern HTML_TAG = Pattern.compile("<[^>]+>");

        private static final Set<String> IGNORED_DIRECTORY_NAMES = new HashSet<>(Arrays.asList(
                ".git",
                ".gradle",
                ".idea",
                "generated",
                "vendor"
        ));

        private static final Set<String> COPIED_FTC_MARKDOWN = new HashSet<>(Arrays.asList(
                "FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/"
                        + "external/samples/readme.md",
                "FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/"
                        + "external/samples/sample_conventions.md",
                "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md",
                "TeamCode/src/main/res/raw/readme.md"
        ));

        private MarkdownIntegrity() {
        }

        static Path findRepositoryRoot(Path startingPath) {
            Path current = startingPath.toAbsolutePath().normalize();
            while (current != null) {
                if (Files.isRegularFile(current.resolve("settings.gradle"))) {
                    return current;
                }
                current = current.getParent();
            }
            throw new IllegalStateException(
                    "Could not find repository root containing settings.gradle from "
                            + startingPath.toAbsolutePath());
        }

        static List<String> validateRepository(Path repositoryRoot) throws IOException {
            Path root = repositoryRoot.toAbsolutePath().normalize();
            Inventory inventory = Inventory.collect(root);
            Scanner scanner = new Scanner(root, inventory.canonicalPaths);
            for (Path source : inventory.markdownSources) {
                scanner.scan(source);
            }
            return scanner.failures();
        }

        private static final class Inventory {
            private final CanonicalPaths canonicalPaths = new CanonicalPaths();
            private final List<Path> markdownSources = new ArrayList<>();

            static Inventory collect(final Path root) throws IOException {
                final Inventory inventory = new Inventory();
                inventory.canonicalPaths.add(root, root);

                Files.walkFileTree(root, new SimpleFileVisitor<Path>() {
                    @Override
                    public FileVisitResult preVisitDirectory(
                            Path directory,
                            BasicFileAttributes attributes) {
                        if (!directory.equals(root) && isIgnoredDirectory(root, directory)) {
                            return FileVisitResult.SKIP_SUBTREE;
                        }
                        inventory.canonicalPaths.add(root, directory);
                        return FileVisitResult.CONTINUE;
                    }

                    @Override
                    public FileVisitResult visitFile(Path file, BasicFileAttributes attributes) {
                        if (attributes.isSymbolicLink()) {
                            return FileVisitResult.CONTINUE;
                        }
                        inventory.canonicalPaths.add(root, file);
                        String relative = relativePath(root, file);
                        if (isMarkdown(file) && !COPIED_FTC_MARKDOWN.contains(relative)) {
                            inventory.markdownSources.add(file);
                        }
                        return FileVisitResult.CONTINUE;
                    }
                });

                Collections.sort(inventory.markdownSources);
                return inventory;
            }

            private static boolean isIgnoredDirectory(Path root, Path directory) {
                Path name = directory.getFileName();
                if (name == null) {
                    return false;
                }
                String value = name.toString();
                if ("build".equals(value)) {
                    Path parent = directory.getParent();
                    return root.equals(parent)
                            || (parent != null
                            && (Files.isRegularFile(parent.resolve("build.gradle"))
                            || Files.isRegularFile(parent.resolve("build.gradle.kts"))));
                }
                return IGNORED_DIRECTORY_NAMES.contains(value);
            }
        }

        private static final class CanonicalPaths {
            private final Map<String, Path> exact = new LinkedHashMap<>();
            private final Map<String, String> byLowerCase = new HashMap<>();

            void add(Path root, Path path) {
                String relative = relativePath(root, path);
                exact.put(relative, path);
                String lower = relative.toLowerCase(Locale.ROOT);
                if (!byLowerCase.containsKey(lower)) {
                    byLowerCase.put(lower, relative);
                }
            }

            Path exactPath(String relative) {
                return exact.get(relative);
            }

            String caseInsensitiveMatch(String relative) {
                return byLowerCase.get(relative.toLowerCase(Locale.ROOT));
            }
        }

        private static final class Scanner {
            private final Path root;
            private final CanonicalPaths canonicalPaths;
            private final List<String> failures = new ArrayList<>();
            private final Map<String, Set<String>> anchorCache = new HashMap<>();

            Scanner(Path root, CanonicalPaths canonicalPaths) {
                this.root = root;
                this.canonicalPaths = canonicalPaths;
            }

            List<String> failures() {
                return Collections.unmodifiableList(failures);
            }

            void scan(Path source) throws IOException {
                List<String> lines = Files.readAllLines(source, StandardCharsets.UTF_8);
                Fence openFence = null;

                for (int index = 0; index < lines.size(); index++) {
                    String line = lines.get(index);
                    int lineNumber = index + 1;

                    if (openFence != null) {
                        if (isFenceClose(line, openFence)) {
                            openFence = null;
                        }
                        continue;
                    }

                    Fence opening = fenceOpening(line, lineNumber);
                    if (opening != null) {
                        openFence = opening;
                        continue;
                    }

                    String visible = maskInlineCode(line);
                    for (String target : inlineLinkTargets(visible)) {
                        validateTarget(source, lineNumber, target);
                    }
                }

                if (openFence != null) {
                    addFailure(
                            source,
                            openFence.openingLine,
                            repeat(openFence.marker, openFence.length),
                            "unclosed fenced code block");
                }
            }

            private void validateTarget(Path source, int lineNumber, String rawTarget)
                    throws IOException {
                String trimmedTarget = rawTarget.trim();
                if (trimmedTarget.isEmpty()) {
                    return;
                }
                if (WINDOWS_ABSOLUTE.matcher(trimmedTarget).matches()) {
                    addFailure(
                            source,
                            lineNumber,
                            rawTarget,
                            "absolute local path is not portable");
                    return;
                }

                String target = unescapeMarkdown(trimmedTarget);
                if (WINDOWS_ABSOLUTE.matcher(target).matches()) {
                    addFailure(
                            source,
                            lineNumber,
                            rawTarget,
                            "absolute local path is not portable");
                    return;
                }
                if (target.toLowerCase(Locale.ROOT).startsWith("file:")) {
                    addFailure(source, lineNumber, rawTarget, "file URI is not a repository link");
                    return;
                }
                if (isRemote(target)) {
                    return;
                }

                int hash = target.indexOf('#');
                String pathPart = hash >= 0 ? target.substring(0, hash) : target;
                String fragmentPart = hash >= 0 ? target.substring(hash + 1) : "";
                int query = pathPart.indexOf('?');
                if (query >= 0) {
                    pathPart = pathPart.substring(0, query);
                }

                final String decodedPath;
                final String decodedFragment;
                try {
                    decodedPath = percentDecode(pathPart);
                    decodedFragment = percentDecode(fragmentPart);
                } catch (IllegalArgumentException malformedEncoding) {
                    addFailure(source, lineNumber, rawTarget, "invalid percent encoding");
                    return;
                }

                if (decodedPath.startsWith("/")
                        || decodedPath.startsWith("\\")
                        || WINDOWS_ABSOLUTE.matcher(decodedPath).matches()) {
                    addFailure(source, lineNumber, rawTarget, "absolute local path is not portable");
                    return;
                }
                if (decodedPath.indexOf('\\') >= 0) {
                    addFailure(source, lineNumber, rawTarget, "local path must use '/' separators");
                    return;
                }

                final Path resolved;
                try {
                    resolved = decodedPath.isEmpty()
                            ? source.toAbsolutePath().normalize()
                            : source.getParent().resolve(decodedPath).toAbsolutePath().normalize();
                } catch (InvalidPathException invalidPath) {
                    addFailure(source, lineNumber, rawTarget, "invalid local path");
                    return;
                }

                if (!resolved.startsWith(root)) {
                    addFailure(source, lineNumber, rawTarget, "target escapes repository root");
                    return;
                }

                String relative = relativePath(root, resolved);
                Path canonical = canonicalPaths.exactPath(relative);
                if (canonical == null) {
                    String caseMatch = canonicalPaths.caseInsensitiveMatch(relative);
                    if (caseMatch != null) {
                        addFailure(
                                source,
                                lineNumber,
                                rawTarget,
                                "path case differs; expected " + caseMatch);
                    } else {
                        addFailure(source, lineNumber, rawTarget, "target does not exist");
                    }
                    return;
                }

                if (!decodedFragment.isEmpty() && isMarkdown(canonical)) {
                    Set<String> anchors = anchorsFor(canonical);
                    if (!anchors.contains(decodedFragment)) {
                        addFailure(
                                source,
                                lineNumber,
                                rawTarget,
                                "heading fragment does not exist: #" + decodedFragment);
                    }
                }
            }

            private Set<String> anchorsFor(Path markdown) throws IOException {
                String relative = relativePath(root, markdown);
                Set<String> cached = anchorCache.get(relative);
                if (cached != null) {
                    return cached;
                }

                Set<String> anchors = new LinkedHashSet<>();
                List<String> lines = Files.readAllLines(markdown, StandardCharsets.UTF_8);
                Fence openFence = null;
                String setextCandidate = null;
                int firstContentLine = frontMatterEnd(lines);

                for (int index = firstContentLine; index < lines.size(); index++) {
                    String line = lines.get(index);
                    if (openFence != null) {
                        if (isFenceClose(line, openFence)) {
                            openFence = null;
                        }
                        setextCandidate = null;
                        continue;
                    }
                    Fence opening = fenceOpening(line, index + 1);
                    if (opening != null) {
                        openFence = opening;
                        setextCandidate = null;
                        continue;
                    }

                    String heading = atxHeading(line);
                    if (heading != null) {
                        Matcher explicitId = HEADING_ATTRIBUTE_ID.matcher(heading);
                        if (explicitId.find()) {
                            String id = explicitId.group(1);
                            if (!anchors.add(id)) {
                                addFailure(markdown, index + 1, "#" + id,
                                        "duplicate explicit heading id");
                            }
                        } else {
                            addUniqueAnchor(anchors, githubHeadingSlug(heading));
                        }
                        setextCandidate = null;
                        continue;
                    }
                    if (isSetextUnderline(line) && setextCandidate != null) {
                        addUniqueAnchor(anchors, githubHeadingSlug(setextCandidate));
                        setextCandidate = null;
                    } else if (line.trim().isEmpty()) {
                        setextCandidate = null;
                    } else {
                        setextCandidate = line.trim();
                    }
                }

                anchorCache.put(relative, anchors);
                return anchors;
            }

            private void addFailure(Path source, int line, String target, String reason) {
                failures.add(relativePath(root, source) + ":" + line + ": " + target
                        + " — " + reason);
            }
        }

        private static List<String> inlineLinkTargets(String line) {
            List<String> targets = new ArrayList<>();
            int from = 0;
            while (from < line.length()) {
                int opening = line.indexOf("](", from);
                if (opening < 0) {
                    break;
                }
                if (matchingLabelOpen(line, opening) < 0) {
                    from = opening + 2;
                    continue;
                }
                int contentStart = opening + 2;
                int closing = matchingLinkClose(line, contentStart);
                if (closing < 0) {
                    break;
                }
                String destination = destinationPart(line.substring(contentStart, closing));
                if (destination != null) {
                    targets.add(destination);
                }
                from = closing + 1;
            }
            return targets;
        }

        private static int matchingLinkClose(String line, int contentStart) {
            int nestedParentheses = 0;
            boolean escaped = false;
            boolean inAngleDestination = false;
            for (int index = contentStart; index < line.length(); index++) {
                char character = line.charAt(index);
                if (escaped) {
                    escaped = false;
                    continue;
                }
                if (character == '\\') {
                    escaped = true;
                    continue;
                }
                if (character == '<' && nestedParentheses == 0) {
                    inAngleDestination = true;
                    continue;
                }
                if (character == '>' && inAngleDestination) {
                    inAngleDestination = false;
                    continue;
                }
                if (inAngleDestination) {
                    continue;
                }
                if (character == '(') {
                    nestedParentheses++;
                } else if (character == ')') {
                    if (nestedParentheses == 0) {
                        return index;
                    }
                    nestedParentheses--;
                }
            }
            return -1;
        }

        private static String destinationPart(String linkContents) {
            String trimmed = linkContents.trim();
            if (trimmed.isEmpty()) {
                return null;
            }
            if (trimmed.charAt(0) == '<') {
                int end = trimmed.indexOf('>');
                return end > 0 ? trimmed.substring(1, end) : null;
            }

            int nestedParentheses = 0;
            boolean escaped = false;
            for (int index = 0; index < trimmed.length(); index++) {
                char character = trimmed.charAt(index);
                if (escaped) {
                    escaped = false;
                    continue;
                }
                if (character == '\\') {
                    escaped = true;
                } else if (character == '(') {
                    nestedParentheses++;
                } else if (character == ')' && nestedParentheses > 0) {
                    nestedParentheses--;
                } else if (Character.isWhitespace(character) && nestedParentheses == 0) {
                    return trimmed.substring(0, index);
                }
            }
            return trimmed;
        }

        private static String maskInlineCode(String line) {
            StringBuilder masked = new StringBuilder(line);
            int searchFrom = 0;
            while (searchFrom < line.length()) {
                int opening = line.indexOf('`', searchFrom);
                if (opening < 0) {
                    break;
                }
                int runLength = markerRunLength(line, opening, '`');
                int closing = exactBacktickRun(line, opening + runLength, runLength);
                if (closing < 0) {
                    searchFrom = opening + runLength;
                    continue;
                }
                int maskThrough = closing + runLength;
                for (int index = opening; index < maskThrough; index++) {
                    masked.setCharAt(index, ' ');
                }
                searchFrom = maskThrough;
            }
            return masked.toString();
        }

        private static int exactBacktickRun(String line, int from, int requiredLength) {
            int searchFrom = from;
            while (searchFrom < line.length()) {
                int candidate = line.indexOf('`', searchFrom);
                if (candidate < 0) {
                    return -1;
                }
                int length = markerRunLength(line, candidate, '`');
                if (length == requiredLength) {
                    return candidate;
                }
                searchFrom = candidate + length;
            }
            return -1;
        }

        private static boolean isBackslashEscaped(String value, int index) {
            int backslashes = 0;
            for (int cursor = index - 1; cursor >= 0 && value.charAt(cursor) == '\\'; cursor--) {
                backslashes++;
            }
            return backslashes % 2 != 0;
        }

        private static int matchingLabelOpen(String line, int labelClose) {
            int nestedLabels = 0;
            for (int index = labelClose - 1; index >= 0; index--) {
                char character = line.charAt(index);
                if (isBackslashEscaped(line, index)) {
                    continue;
                }
                if (character == ']') {
                    nestedLabels++;
                } else if (character == '[') {
                    if (nestedLabels == 0) {
                        return index;
                    }
                    nestedLabels--;
                }
            }
            return -1;
        }

        private static Fence fenceOpening(String line, int lineNumber) {
            int start = firstNonSpace(line);
            if (start < 0 || start > 3) {
                return null;
            }
            char marker = line.charAt(start);
            if (marker != '`' && marker != '~') {
                return null;
            }
            int length = markerRunLength(line, start, marker);
            return length >= 3 ? new Fence(marker, length, lineNumber) : null;
        }

        private static boolean isFenceClose(String line, Fence fence) {
            int start = firstNonSpace(line);
            if (start < 0 || start > 3 || line.charAt(start) != fence.marker) {
                return false;
            }
            int length = markerRunLength(line, start, fence.marker);
            return length >= fence.length && line.substring(start + length).trim().isEmpty();
        }

        private static int firstNonSpace(String value) {
            for (int index = 0; index < value.length(); index++) {
                if (value.charAt(index) != ' ') {
                    return index;
                }
            }
            return -1;
        }

        private static int markerRunLength(String value, int start, char marker) {
            int index = start;
            while (index < value.length() && value.charAt(index) == marker) {
                index++;
            }
            return index - start;
        }

        private static String atxHeading(String line) {
            int start = firstNonSpace(line);
            if (start < 0 || start > 3 || line.charAt(start) != '#') {
                return null;
            }
            int hashes = markerRunLength(line, start, '#');
            if (hashes > 6) {
                return null;
            }
            int textStart = start + hashes;
            if (textStart < line.length() && !Character.isWhitespace(line.charAt(textStart))) {
                return null;
            }
            String heading = line.substring(textStart).trim();
            heading = heading.replaceFirst("\\s+#+\\s*$", "").trim();
            return heading;
        }

        private static int frontMatterEnd(List<String> lines) {
            if (lines.isEmpty() || !lines.get(0).trim().equals("---")) {
                return 0;
            }
            for (int index = 1; index < lines.size(); index++) {
                if (lines.get(index).trim().equals("---")) {
                    return index + 1;
                }
            }
            return 0;
        }

        private static boolean isSetextUnderline(String line) {
            int start = firstNonSpace(line);
            if (start < 0 || start > 3) {
                return false;
            }
            String marker = line.substring(start).trim();
            if (marker.isEmpty()) {
                return false;
            }
            char expected = marker.charAt(0);
            if (expected != '=' && expected != '-') {
                return false;
            }
            for (int index = 1; index < marker.length(); index++) {
                if (marker.charAt(index) != expected) {
                    return false;
                }
            }
            return true;
        }

        private static void addUniqueAnchor(Set<String> anchors, String baseSlug) {
            String candidate = baseSlug;
            int suffix = 0;
            while (anchors.contains(candidate)) {
                suffix++;
                candidate = baseSlug + "-" + suffix;
            }
            anchors.add(candidate);
        }

        private static String githubHeadingSlug(String heading) {
            Matcher links = MARKDOWN_LINK_IN_HEADING.matcher(heading);
            String withoutLinks = links.replaceAll("$1").replace("`", "");
            withoutLinks = HTML_TAG.matcher(withoutLinks).replaceAll("");
            String lower = withoutLinks.toLowerCase(Locale.ROOT);
            StringBuilder slug = new StringBuilder();
            for (int offset = 0; offset < lower.length(); ) {
                int codePoint = lower.codePointAt(offset);
                offset += Character.charCount(codePoint);
                if (Character.isLetterOrDigit(codePoint) || codePoint == '-' || codePoint == '_') {
                    slug.appendCodePoint(codePoint);
                } else if (Character.isWhitespace(codePoint)) {
                    slug.append('-');
                }
            }
            return slug.toString();
        }

        private static boolean isRemote(String target) {
            return target.startsWith("//") || URI_SCHEME.matcher(target).find();
        }

        private static boolean isMarkdown(Path path) {
            Path fileName = path.getFileName();
            return fileName != null
                    && fileName.toString().toLowerCase(Locale.ROOT).endsWith(".md");
        }

        private static String percentDecode(String value) {
            try {
                return URLDecoder.decode(value.replace("+", "%2B"), "UTF-8");
            } catch (java.io.UnsupportedEncodingException impossible) {
                throw new AssertionError(impossible);
            }
        }

        private static String unescapeMarkdown(String value) {
            StringBuilder unescaped = new StringBuilder();
            for (int index = 0; index < value.length(); index++) {
                char character = value.charAt(index);
                if (character == '\\' && index + 1 < value.length()) {
                    char next = value.charAt(index + 1);
                    if (isAsciiPunctuation(next)) {
                        unescaped.append(next);
                        index++;
                        continue;
                    }
                }
                unescaped.append(character);
            }
            return unescaped.toString();
        }

        private static boolean isAsciiPunctuation(char character) {
            return (character >= '!' && character <= '/')
                    || (character >= ':' && character <= '@')
                    || (character >= '[' && character <= '`')
                    || (character >= '{' && character <= '~');
        }

        private static String relativePath(Path root, Path path) {
            return root.relativize(path.toAbsolutePath().normalize()).toString().replace('\\', '/');
        }

        private static String repeat(char character, int count) {
            StringBuilder repeated = new StringBuilder(count);
            for (int index = 0; index < count; index++) {
                repeated.append(character);
            }
            return repeated.toString();
        }

        private static final class Fence {
            private final char marker;
            private final int length;
            private final int openingLine;

            private Fence(char marker, int length, int openingLine) {
                this.marker = marker;
                this.length = length;
                this.openingLine = openingLine;
            }
        }
    }
}
