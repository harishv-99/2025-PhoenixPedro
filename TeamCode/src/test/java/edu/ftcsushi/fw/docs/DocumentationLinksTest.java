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

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies deterministic repository-local Markdown navigation without network access. */
public final class DocumentationLinksTest {

    @Rule
    public final TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void maintainedRepositoryMarkdownHasValidLocalLinksAnchorsAndFences()
            throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));

        assertNoFailures(MarkdownIntegrity.validateRepository(repositoryRoot));
    }

    @Test
    public void documentationSiteNavigationTargetsExistingMarkdown() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        String config = new String(
                Files.readAllBytes(repositoryRoot.resolve("zensical.toml")),
                StandardCharsets.UTF_8);
        Path docsRoot = configuredDocsRoot(repositoryRoot, config);
        Matcher entries = Pattern.compile("=\\s*\\\"([^\\\"]+\\.md)\\\"").matcher(config);
        List<String> failures = new ArrayList<String>();
        while (entries.find()) {
            String target = entries.group(1);
            Path resolved = docsRoot.resolve(target).toAbsolutePath().normalize();
            if (!resolved.startsWith(docsRoot.toAbsolutePath().normalize())) {
                failures.add(target + " (escapes docs_dir)");
            } else if (!Files.isRegularFile(resolved)) {
                failures.add(target + " (missing)");
            }
        }
        assertTrue("Missing documentation navigation targets: " + failures, failures.isEmpty());
    }

    @Test
    public void sushiFrameworkIdentityAndDocumentationBoundaryStayExplicit() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        String rootReadme = readUtf8(repositoryRoot.resolve("README.md"));
        String namespaceReadme = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/README.md"));
        String docsHome = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/README.md"));
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        String build = readUtf8(repositoryRoot.resolve("TeamCode/build.gradle"));

        assertTrue("Root README must identify Sushi as the framework",
                rootReadme.startsWith("# Sushi framework"));
        assertTrue("Namespace README must identify Sushi as the framework",
                namespaceReadme.startsWith("# Sushi framework"));
        assertTrue("Documentation home must identify Sushi",
                docsHome.startsWith("# Sushi documentation"));
        Path configuredDocsRoot = configuredDocsRoot(repositoryRoot, config);
        assertTrue("Documentation site must use the Sushi identity",
                config.contains("site_name = \"Sushi Framework\"")
                        && config.contains("\"Learn Sushi topics\"")
                        && configuredDocsRoot.equals(repositoryRoot.resolve(
                                "TeamCode/src/main/java/edu/ftcsushi/fw").toAbsolutePath()
                                .normalize())
                        && config.contains("{ \"Home\" = \"docs/README.md\" }")
                        && !config.contains("= \"fw/")
                        && !config.contains("ftcphoenix"));
        assertTrue("Strict API documentation must use the Sushi task and title",
                build.contains("tasks.register('sushiJavadocs', Javadoc)")
                        && build.contains("Sushi Framework API")
                        && !build.contains("phoenixJavadocs"));
        assertTrue("Renamed beginner and reference pages must exist",
                Files.isRegularFile(repositoryRoot.resolve(
                        "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/"
                                + "First Sushi Robot Code.md"))
                        && Files.isDirectory(repositoryRoot.resolve(
                                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/"
                                        + "learn-sushi"))
                        && Files.isRegularFile(repositoryRoot.resolve(
                                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/reference/"
                                        + "Sushi Cheat Sheet.md")));
        assertTrue("Former framework page paths must not remain",
                !Files.exists(repositoryRoot.resolve(
                        "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/"
                                + "First Phoenix Robot Code.md"))
                        && !Files.exists(repositoryRoot.resolve(
                                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/"
                                        + "learn-phoenix"))
                        && !Files.exists(repositoryRoot.resolve(
                                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/reference/"
                                        + "Phoenix Cheat Sheet.md")));
        assertTrue("Former Java namespace roots must not remain",
                !Files.exists(repositoryRoot.resolve(
                        "TeamCode/src/main/java/edu/ftcphoenix"))
                        && !Files.exists(repositoryRoot.resolve(
                                "TeamCode/src/test/java/edu/ftcphoenix")));

        assertNoStaleFrameworkBranding(
                repositoryRoot,
                repositoryRoot.resolve("TeamCode/src/main/java/edu/ftcsushi/fw"),
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
                repositoryRoot.resolve("TeamCode/src/main/java/edu/ftcsushi/fw"),
                Arrays.asList(
                        repositoryRoot.resolve(
                                "TeamCode/src/main/java/edu/ftcsushi/README.md"),
                        repositoryRoot.resolve("TeamCode/build.gradle")));
    }

    @Test
    public void currentTrackerGuidanceDoesNotDependOnTheProductionApplication()
            throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
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
    public void learningNavigationAndCompatibilityPagesKeepOneCurrentPath() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        String config = readUtf8(repositoryRoot.resolve("zensical.toml"));
        Matcher startGroup = Pattern.compile(
                "\\{\\s*\"Start here\"\\s*=\\s*\\[([^]]+)]\\s*},",
                Pattern.DOTALL).matcher(config);
        assertTrue("Missing Start here navigation group", startGroup.find());
        assertNavigationEntries(
                "Start here",
                startGroup.group(1),
                Arrays.asList(
                        "Sushi in one picture",
                        "Set up and verify the project",
                        "Write your first Sushi robot code",
                        "Test a mechanism without hardware",
                        "Build a robot step by step",
                        "Choose a Sushi topic",
                        "Documentation home",
                        "Getting started"),
                Arrays.asList(
                        "docs/getting-started/Framework Overview.md",
                        "docs/getting-started/Build and Run.md",
                        "docs/getting-started/First Sushi Robot Code.md",
                        "docs/getting-started/Test a Mechanism Without Hardware.md",
                        "docs/getting-started/Build a Robot Step by Step.md",
                        "docs/getting-started/Beginner's Guide.md",
                        "docs/README.md",
                        "docs/getting-started/README.md"));

        Matcher learningGroup = Pattern.compile(
                "\\{\\s*\"Learn Sushi topics\"\\s*=\\s*\\[([^]]+)]\\s*},",
                Pattern.DOTALL).matcher(config);
        assertTrue("Missing Learn Sushi topics navigation group", learningGroup.find());

        assertNavigationEntries(
                "Learn Sushi topics",
                learningGroup.group(1),
                Arrays.asList(
                        "Robot roles",
                        "Controls and intent",
                        "Plants and hardware",
                        "Tasks and autonomous",
                        "Evidence and experiments",
                        "From requirement to robot",
                        "Role paths"),
                Arrays.asList(
                        "docs/getting-started/learn-sushi/Robot Roles.md",
                        "docs/getting-started/learn-sushi/Controls and Intent.md",
                        "docs/getting-started/learn-sushi/Plants and Hardware.md",
                        "docs/getting-started/learn-sushi/Tasks and Autonomous.md",
                        "docs/getting-started/learn-sushi/Evidence and Experiments.md",
                        "docs/getting-started/learn-sushi/From Requirement to Robot.md",
                        "docs/getting-started/learn-sushi/Role Paths.md"));

        Matcher examplesGroup = Pattern.compile(
                "\\{\\s*\"Examples\"\\s*=\\s*\\[([^]]+)]\\s*},",
                Pattern.DOTALL).matcher(config);
        assertTrue("Missing Examples navigation group", examplesGroup.find());
        assertNavigationEntries(
                "Examples",
                examplesGroup.group(1),
                Arrays.asList(
                        "Examples home",
                        "Modern starter robot",
                        "Hardware-free reference scenarios",
                        "Framework components",
                        "Field-relative drive",
                        "Pedro autonomous reference",
                        "Subsystem experiments",
                        "BIOBUZZ capability map"),
                Arrays.asList(
                        "docs/examples/README.md",
                        "docs/examples/Modern Starter Robot.md",
                        "docs/examples/Hardware-free Reference Scenarios.md",
                        "docs/examples/Framework Components Through Examples.md",
                        "docs/examples/Field-relative Drive.md",
                        "docs/examples/Pedro Autonomous Reference.md",
                        "docs/examples/Subsystem Experiments.md",
                        "docs/examples/BIOBUZZ Capability Map.md"));
        assertTrue("Primary navigation still exposes the old build-along labels",
                !config.contains("Your first mechanism")
                        && !config.contains("Your first TeleOp")
                        && !config.contains("Your first Task and Auto"));

        assertCompatibilityPage(repositoryRoot,
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/First Mechanism.md",
                "learn-sushi/Plants and Hardware.md",
                "../testing-calibration/Actuator Bring-up.md");
        assertCompatibilityPage(repositoryRoot,
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/First TeleOp.md",
                "learn-sushi/Controls and Intent.md",
                "../testing-calibration/Robot Calibration Tutorials.md");
        assertCompatibilityPage(repositoryRoot,
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/First Task and Auto.md",
                "learn-sushi/Tasks and Autonomous.md",
                "../testing-calibration/Robot Calibration Tutorials.md");
    }

    @Test
    public void buildSeasonAnchorKeepsTheEvidenceBasedStudentWorkflow() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        Path anchorPath = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/"
                        + "Build a Robot Step by Step.md");
        assertTrue("Build-season anchor is missing", Files.isRegularFile(anchorPath));

        String anchor = readUtf8(anchorPath);
        String[] requiredStageHeadings = {
            "## 1. Define behavior and success criteria",
            "## 2. Establish the compiling robot skeleton",
            "## 3. Build one subsystem software slice",
            "## 4. Bring up hardware in isolation",
            "## 5. Run a bounded subsystem experiment",
            "## 6. Integrate continuously and grow TeleOp",
            "## 7. Qualify routes and autonomous policy separately",
            "## 8. Rehearse the complete robot"
        };
        int priorStage = -1;
        for (String stage : requiredStageHeadings) {
            int stageIndex = anchor.indexOf(stage);
            assertTrue("Build-season anchor is missing stage: " + stage, stageIndex >= 0);
            assertTrue("Build-season stages are out of order: " + stage,
                    stageIndex > priorStage);
            priorStage = stageIndex;
        }

        String[] requiredEvidenceLabels = {
            "**Goal:**", "**Produce:**", "**Prove:**", "**Does not prove:**", "**Do next:**"
        };
        for (String label : requiredEvidenceLabels) {
            int occurrences = anchor.split(Pattern.quote(label), -1).length - 1;
            assertTrue("Every build-season stage must contain evidence label " + label
                            + "; found " + occurrences,
                    occurrences == requiredStageHeadings.length);
        }

        String[] requiredDestinations = {
            "learn-sushi/From Requirement to Robot.md",
            "../examples/Modern Starter Robot.md",
            "Test a Mechanism Without Hardware.md",
            "../testing-calibration/README.md",
            "../examples/Subsystem Experiments.md",
            "../design/Robot Capabilities & Mode Clients.md",
            "First Pedro Auto.md",
            "../troubleshooting/Common Problems.md"
        };
        for (String destination : requiredDestinations) {
            assertTrue("Build-season anchor is missing destination: " + destination,
                    anchor.contains(destination));
        }

        assertTrue("Build-season anchor must distinguish tester OpModes from production TeleOp",
                anchor.contains("dedicated tester or diagnostic OpModes")
                        && anchor.contains("production TeleOp")
                        && anchor.contains("raw device tester"));
        assertTrue("Build-season anchor must keep integration continuous",
                anchor.contains("Return to")
                        && anchor.contains("step 3 for the next subsystem slice"));
        assertTrue("Build-season anchor must include an accessible Mermaid title",
                anchor.contains("accTitle: Build-season robot workflow"));
        assertTrue("Build-season anchor must include an accessible Mermaid description",
                anchor.contains("accDescr:"));
        assertTrue("Build-season anchor must include a diagram text fallback",
                anchor.contains("**Text version:**"));
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
    public void firstContactLearningPagesStayWithinProgressiveDisclosureBudgets()
            throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        Path learningRoot = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started");
        Path overview = learningRoot.resolve("Framework Overview.md");
        Path firstRobotCode = learningRoot.resolve("First Sushi Robot Code.md");
        Path mechanismLesson = learningRoot.resolve("Test a Mechanism Without Hardware.md");
        Path hub = learningRoot.resolve("Beginner's Guide.md");
        Path topics = learningRoot.resolve("learn-sushi");

        assertTrue("First-contact overview exceeds 900 prose words",
                proseWordCount(overview) <= 900);
        assertTrue("First-contact overview exceeds three Java excerpts",
                javaFenceCount(overview) <= 3);
        assertTrue("First-contact overview exceeds 30 displayed Java lines",
                displayedJavaLineCount(overview) <= 30);
        assertTrue("First robot-code lesson is missing", Files.isRegularFile(firstRobotCode));
        assertTrue("First robot-code lesson exceeds 1,200 prose words",
                proseWordCount(firstRobotCode) <= 1200);
        assertTrue("First robot-code lesson exceeds five Java excerpts",
                javaFenceCount(firstRobotCode) <= 5);
        assertTrue("First robot-code lesson exceeds 35 displayed Java lines",
                displayedJavaLineCount(firstRobotCode) <= 35);
        String firstRobotCodeText = readUtf8(firstRobotCode);
        assertTrue("First robot-code lesson must link its compiled control check on GitHub",
                firstRobotCodeText.contains(
                        "github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/"
                                + "edu/ftcsushi/robots/examples/starter/robot/"
                                + "StarterFirstLessonTest.java"));
        assertTrue("First robot-code lesson must link the canonical Starter source",
                firstRobotCodeText.contains("robots/examples/starter"));
        assertTrue("Hardware-free mechanism lesson is missing",
                Files.isRegularFile(mechanismLesson));
        assertTrue("Hardware-free mechanism lesson exceeds 1,000 prose words",
                proseWordCount(mechanismLesson) <= 1000);
        assertTrue("Hardware-free mechanism lesson exceeds three Java excerpts",
                javaFenceCount(mechanismLesson) <= 3);
        assertTrue("Hardware-free mechanism lesson exceeds 35 displayed Java lines",
                displayedJavaLineCount(mechanismLesson) <= 35);
        String mechanismLessonText = readUtf8(mechanismLesson);
        assertTrue("Hardware-free mechanism lesson must link its compiled scenario on GitHub",
                mechanismLessonText.contains(
                        "github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/"
                                + "edu/ftcsushi/robots/examples/starter/robot/"
                                + "StarterMechanismLessonTest.java"));
        assertEvidenceBoundary("Hardware-free mechanism lesson", mechanismLessonText);

        Path referenceScenarios = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/examples/"
                        + "Hardware-free Reference Scenarios.md");
        String referenceScenarioText = readUtf8(referenceScenarios);
        assertTrue("Reference scenario guide must link the compiled lift scenario on GitHub",
                referenceScenarioText.contains(
                        "github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/"
                                + "edu/ftcsushi/robots/examples/reference/capability/lift/"
                                + "ReferenceLiftSoftwareScenarioTest.java"));
        assertTrue("Reference scenario guide must link the compiled launcher scenario on GitHub",
                referenceScenarioText.contains(
                        "github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/"
                                + "edu/ftcsushi/robots/examples/reference/capability/launcher/"
                                + "ReferenceLauncherSoftwareScenarioTest.java"));
        assertEvidenceBoundary("Hardware-free Reference scenarios", referenceScenarioText);
        assertTrue("Topic router exceeds 450 prose words", proseWordCount(hub) <= 450);

        int topicWords = 0;
        for (String topic : Arrays.asList(
                "Robot Roles.md",
                "Controls and Intent.md",
                "Plants and Hardware.md",
                "Tasks and Autonomous.md",
                "Evidence and Experiments.md",
                "From Requirement to Robot.md")) {
            topicWords += proseWordCount(topics.resolve(topic));
        }
        assertTrue("Six Sushi topic pages exceed 4,200 prose words: " + topicWords,
                topicWords <= 4200);
        assertTrue("Role Paths exceeds 500 prose words",
                proseWordCount(topics.resolve("Role Paths.md")) <= 500);
    }

    @Test
    public void acceptsLiteralAndEncodedSpacesAndDuplicateHeadingAnchors() throws IOException {
        Path root = temporaryFolder.getRoot().toPath();
        write(root, "Docs/Guide File.md",
                "# Hello, World!\n\n"
                        + "# Repeat\n\n# Repeat\n\n# Repeat-1\n\n"
                        + "Setext heading\n--------------\n");
        write(root, "README.md",
                "[literal](<Docs/Guide File.md#hello-world>)\n"
                        + "[encoded](Docs/Guide%20File.md#repeat-1)\n"
                        + "[collision](<Docs/Guide File.md#repeat-1-1>)\n"
                        + "[setext](<Docs/Guide File.md#setext-heading>)\n");

        assertNoFailures(MarkdownIntegrity.validateRepository(root));
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

    private static void write(Path root, String relativePath, String contents) throws IOException {
        Path file = root.resolve(relativePath);
        Files.createDirectories(file.getParent());
        Files.write(file, contents.getBytes(StandardCharsets.UTF_8));
    }

    private static void assertEvidenceBoundary(String pageName, String contents) {
        assertTrue(pageName + " must state what its evidence proves",
                contents.contains("### Proves"));
        assertTrue(pageName + " must state what its evidence does not prove",
                contents.contains("### Does not prove"));
        assertTrue(pageName + " must state the next evidence gate",
                contents.contains("### Next gate"));
    }

    private static void assertCompatibilityPage(Path repositoryRoot,
                                                String relativePath,
                                                String conceptTarget,
                                                String hardwareTarget) throws IOException {
        String contents = readUtf8(repositoryRoot.resolve(relativePath));
        assertTrue(relativePath + " must link to " + conceptTarget,
                contents.contains(conceptTarget));
        assertTrue(relativePath + " must link to " + hardwareTarget,
                contents.contains(hardwareTarget));
    }

    private static void assertNavigationEntries(String groupName,
                                                String groupContents,
                                                List<String> expectedLabels,
                                                List<String> expectedTargets) {
        Matcher entries = Pattern.compile(
                "\\{\\s*\"([^\"]+)\"\\s*=\\s*\"([^\"]+\\.md)\"\\s*}")
                .matcher(groupContents);
        List<String> actualLabels = new ArrayList<String>();
        List<String> actualTargets = new ArrayList<String>();
        while (entries.find()) {
            actualLabels.add(entries.group(1));
            actualTargets.add(entries.group(2));
        }
        assertTrue("Unexpected " + groupName + " navigation labels: " + actualLabels,
                expectedLabels.equals(actualLabels));
        assertTrue("Unexpected " + groupName + " navigation order: " + actualTargets,
                expectedTargets.equals(actualTargets));
    }

    private static String readUtf8(Path path) throws IOException {
        return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
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
        boolean insideFence = false;
        StringBuilder prose = new StringBuilder();
        for (String line : Files.readAllLines(path, StandardCharsets.UTF_8)) {
            String trimmed = line.trim();
            if (trimmed.startsWith("```") || trimmed.startsWith("~~~")) {
                insideFence = !insideFence;
            } else if (!insideFence) {
                prose.append(' ').append(trimmed);
            }
        }
        String text = prose.toString().trim();
        return text.isEmpty() ? 0 : text.split("\\s+").length;
    }

    private static int javaFenceCount(Path path) throws IOException {
        int count = 0;
        for (String line : Files.readAllLines(path, StandardCharsets.UTF_8)) {
            if (line.trim().equals("```java")) {
                count++;
            }
        }
        return count;
    }

    private static int displayedJavaLineCount(Path path) throws IOException {
        boolean insideJava = false;
        int count = 0;
        for (String line : Files.readAllLines(path, StandardCharsets.UTF_8)) {
            String trimmed = line.trim();
            if (!insideJava && trimmed.equals("```java")) {
                insideJava = true;
            } else if (insideJava && trimmed.equals("```")) {
                insideJava = false;
            } else if (insideJava && !trimmed.isEmpty()) {
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
        fail("Expected a failure containing '" + expectedText + "' but found:\n"
                + joinLines(failures));
    }

    private static void assertFailureStartsWith(List<String> failures, String expectedText) {
        for (String failure : failures) {
            if (failure.startsWith(expectedText)) {
                return;
            }
        }
        fail("Expected a failure starting with '" + expectedText + "' but found:\n"
                + joinLines(failures));
    }

    private static String joinLines(List<String> lines) {
        StringBuilder result = new StringBuilder();
        for (String line : lines) {
            if (result.length() > 0) {
                result.append('\n');
            }
            result.append(line);
        }
        return result.toString();
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
                "build",
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
                        if (!directory.equals(root) && isIgnoredDirectory(directory)) {
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

            private static boolean isIgnoredDirectory(Path directory) {
                Path name = directory.getFileName();
                return name != null && IGNORED_DIRECTORY_NAMES.contains(name.toString());
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
                        addUniqueAnchor(anchors, githubHeadingSlug(heading));
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
