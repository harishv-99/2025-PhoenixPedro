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
    public void studentLearningJavaExcerptsDeclareAndMatchTheirSourceTruth()
            throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        Path docsRoot = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs");
        List<Path> learningPages = new ArrayList<Path>();
        collectMarkdownFiles(docsRoot.resolve("getting-started"), learningPages);
        collectMarkdownFiles(docsRoot.resolve("examples"), learningPages);
        collectMarkdownFiles(docsRoot.resolve("testing-calibration"), learningPages);

        Pattern javaFence = Pattern.compile("(?m)^```java\\s*$");
        Pattern markedFence = Pattern.compile(
                "(?m)^<!-- (?:(source-excerpt|annotated-source-excerpt|source-file|"
                        + "annotated-source-file): "
                        + "([^>]+)|(teaching-shape)) -->\\r?\\n"
                        + "```java\\r?\\n([\\s\\S]*?)\\r?\\n```");
        List<String> failures = new ArrayList<String>();
        for (Path page : learningPages) {
            String markdown = readUtf8(page);
            int fenceCount = matcherCount(javaFence.matcher(markdown));
            Matcher marked = markedFence.matcher(markdown);
            int markedCount = 0;
            while (marked.find()) {
                markedCount++;
                String marker = marked.group(1);
                String sourcePath = marked.group(2);
                String snippet = marked.group(4);
                if (marked.group(3) != null) {
                    int contextStart = Math.max(0, marked.start() - 240);
                    String context = markdown.substring(contextStart, marked.start());
                    if (!context.contains("Abbreviated shape (omissions shown):")) {
                        failures.add(repositoryRelativePath(repositoryRoot, page)
                                + ": teaching shape lacks visible abbreviated-shape label");
                    }
                    if (!snippet.contains("// ...")) {
                        failures.add(repositoryRelativePath(repositoryRoot, page)
                                + ": teaching shape lacks // ... omission marker");
                    }
                    continue;
                }

                Path source = repositoryRoot.resolve(sourcePath.trim())
                        .toAbsolutePath().normalize();
                if (!source.startsWith(repositoryRoot.toAbsolutePath().normalize())
                        || !Files.isRegularFile(source)
                        || !source.getFileName().toString().endsWith(".java")) {
                    failures.add(repositoryRelativePath(repositoryRoot, page)
                            + ": invalid source excerpt path " + sourcePath.trim());
                    continue;
                }
                if (marker.startsWith("annotated-")) {
                    snippet = stripDocumentationAnnotations(
                            repositoryRoot, page, snippet, failures);
                } else if (snippet.contains("// docs:")) {
                    failures.add(repositoryRelativePath(repositoryRoot, page)
                            + ": // docs: comments require annotated-source-excerpt");
                }
                String normalizedSnippet = normalizeExcerpt(snippet);
                String sourceText = readUtf8(source);
                String normalizedSource = normalizeExcerpt(sourceText);
                boolean wholeFile = marker.endsWith("source-file");
                boolean matches = wholeFile
                        ? normalizedSource.equals(normalizedSnippet)
                        : containsDedentedBlock(sourceText, normalizedSnippet);
                if (normalizedSnippet.isEmpty() || !matches) {
                    failures.add(repositoryRelativePath(repositoryRoot, page)
                            + (wholeFile
                            ? ": complete file does not match "
                            : ": excerpt does not occur contiguously in ")
                            + sourcePath.trim());
                }
            }
            if (fenceCount != markedCount) {
                failures.add(repositoryRelativePath(repositoryRoot, page)
                        + ": found " + fenceCount + " Java fences but " + markedCount
                        + " provenance markers");
            }
        }
        assertTrue("Student learning excerpt provenance failures: " + failures,
                failures.isEmpty());
    }

    @Test
    public void substantiveStudentLearningPagesKeepInlineLearningBlocks() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        String docs = "TeamCode/src/main/java/edu/ftcsushi/fw/docs/";
        List<String> pages = Arrays.asList(
                "examples/Field-relative Drive.md",
                "examples/Hardware-free Reference Scenarios.md",
                "examples/Modern Starter Robot.md",
                "examples/Subsystem Experiments.md",
                "examples/Timestamped Adaptive Collection.md",
                "getting-started/Build a Robot Step by Step.md",
                "getting-started/Build and Run.md",
                "getting-started/First Pedro Auto.md",
                "getting-started/First Sushi Robot Code.md",
                "getting-started/Framework Overview.md",
                "getting-started/Test a Mechanism Without Hardware.md",
                "getting-started/learn-sushi/Controls and Intent.md",
                "getting-started/learn-sushi/Evidence and Experiments.md",
                "getting-started/learn-sushi/From Requirement to Robot.md",
                "getting-started/learn-sushi/Plants and Hardware.md",
                "getting-started/learn-sushi/Robot Roles.md",
                "getting-started/learn-sushi/Tasks and Autonomous.md",
                "testing-calibration/Actuator Bring-up.md",
                "testing-calibration/Control Tuning Workflow.md",
                "testing-calibration/Guided Calibration Walkthroughs.md",
                "testing-calibration/Robot Calibration Tutorials.md");

        List<String> failures = new ArrayList<String>();
        for (String page : pages) {
            String markdown = readUtf8(repositoryRoot.resolve(docs + page));
            if (!markdown.contains("### Critical code")) {
                failures.add(page + ": missing Critical code");
            }
            if (!markdown.contains("**What to notice**")) {
                failures.add(page + ": missing What to notice");
            }
            if (!markdown.contains("**Key APIs")) {
                failures.add(page + ": missing Key APIs");
            }
        }
        assertTrue("Student learning block failures: " + failures, failures.isEmpty());
    }

    @Test
    public void studentLearningPagesDeclareAndHonorTheirLearningMode() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        Path docsRoot = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs");
        List<Path> pages = new ArrayList<Path>();
        collectMarkdownFiles(docsRoot.resolve("getting-started"), pages);
        collectMarkdownFiles(docsRoot.resolve("examples"), pages);
        collectMarkdownFiles(docsRoot.resolve("testing-calibration"), pages);
        Pattern modePattern = Pattern.compile(
                "(?m)^\\*\\*Learning mode:\\*\\* (Buildable implementation|"
                        + "Guided course|Operational runbook|Architecture reference|Router)\\s*$");
        List<String> failures = new ArrayList<String>();
        for (Path page : pages) {
            String markdown = readUtf8(page);
            Matcher modes = modePattern.matcher(markdown);
            if (!modes.find()) {
                failures.add(repositoryRelativePath(repositoryRoot, page)
                        + ": missing supported Learning mode");
                continue;
            }
            String mode = modes.group(1);
            if (modes.find()) {
                failures.add(repositoryRelativePath(repositoryRoot, page)
                        + ": declares more than one Learning mode");
            }
            if (!"Buildable implementation".equals(mode)) {
                continue;
            }
            boolean visibleFirstRun = page.getFileName().toString()
                    .equals("First Sushi Robot Code.md");
            requireText(repositoryRoot, page, markdown,
                    "### Critical code", "Critical code", failures);
            requireText(repositoryRoot, page, markdown,
                    "**What to notice**", "What to notice", failures);
            requireText(repositoryRoot, page, markdown,
                    "**Key APIs", "Key APIs", failures);
            if (visibleFirstRun) {
                requireText(repositoryRoot, page, markdown,
                        "## 1. Copy the complete program", "one-file copy step", failures);
                requireText(repositoryRoot, page, markdown,
                        "## 3. Compile while motion remains disabled",
                        "disabled compile gate", failures);
                if (markdown.contains("<details>")) {
                    failures.add(repositoryRelativePath(repositoryRoot, page)
                            + ": first-run complete file must stay visible");
                }
            } else {
                requireText(repositoryRoot, page, markdown,
                        "## Files you will create", "Files you will create", failures);
                requireText(repositoryRoot, page, markdown,
                        "## Complete working slice", "Complete working slice", failures);
                requireText(repositoryRoot, page, markdown,
                        "## Verify the slice", "Verify the slice", failures);
                requireText(repositoryRoot, page, markdown,
                        "<details>", "collapsed complete files", failures);
            }
            Matcher manifest = Pattern.compile("<!-- buildable-files: ([^>]+) -->")
                    .matcher(markdown);
            if (!manifest.find()) {
                failures.add(repositoryRelativePath(repositoryRoot, page)
                        + ": buildable page lacks a buildable-files manifest");
            } else {
                Set<String> declaredFiles = new LinkedHashSet<String>(
                        Arrays.asList(manifest.group(1).trim().split("\\s*\\|\\s*")));
                Set<String> completeFiles = new LinkedHashSet<String>();
                Matcher complete = Pattern.compile(
                        "<!-- (?:annotated-)?source-file: ([^>]+) -->")
                        .matcher(markdown);
                while (complete.find()) {
                    completeFiles.add(complete.group(1).trim());
                    int openDetails = markdown.lastIndexOf("<details>", complete.start());
                    int closeDetails = markdown.lastIndexOf("</details>", complete.start());
                    if (!visibleFirstRun && (openDetails < 0 || closeDetails > openDetails)) {
                        failures.add(repositoryRelativePath(repositoryRoot, page)
                                + ": complete source file is not inside collapsed details: "
                                + complete.group(1).trim());
                    }
                }
                if (!declaredFiles.equals(completeFiles)) {
                    failures.add(repositoryRelativePath(repositoryRoot, page)
                            + ": buildable-files manifest " + declaredFiles
                            + " does not equal complete source files " + completeFiles);
                }
            }
            if (markdown.contains("<!-- teaching-shape -->")) {
                failures.add(repositoryRelativePath(repositoryRoot, page)
                        + ": buildable page contains teaching-shape pseudocode");
            }
            if (!markdown.contains("gradlew.bat")) {
                failures.add(repositoryRelativePath(repositoryRoot, page)
                        + ": buildable page lacks an exact Gradle verification command");
            }
        }
        assertTrue("Student learning-mode failures: " + failures, failures.isEmpty());
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
    public void learningNavigationKeepsOneCurrentCourseWithoutCompatibilityStubs()
            throws IOException {
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
                        "Set up and verify the project",
                        "Get your first robot driving",
                        "Build a robot step by step",
                        "Test a mechanism without hardware",
                        "Sushi in one picture",
                        "Choose a Sushi topic"),
                Arrays.asList(
                        "docs/getting-started/Build and Run.md",
                        "docs/getting-started/First Sushi Robot Code.md",
                        "docs/getting-started/Build a Robot Step by Step.md",
                        "docs/getting-started/Test a Mechanism Without Hardware.md",
                        "docs/getting-started/Framework Overview.md",
                        "docs/getting-started/Beginner's Guide.md"));

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
                        "From requirement to robot"),
                Arrays.asList(
                        "docs/getting-started/learn-sushi/Robot Roles.md",
                        "docs/getting-started/learn-sushi/Controls and Intent.md",
                        "docs/getting-started/learn-sushi/Plants and Hardware.md",
                        "docs/getting-started/learn-sushi/Tasks and Autonomous.md",
                        "docs/getting-started/learn-sushi/Evidence and Experiments.md",
                        "docs/getting-started/learn-sushi/From Requirement to Robot.md"));

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
                        "Field-relative drive",
                        "Your first Pedro Auto",
                        "Subsystem experiments"),
                Arrays.asList(
                        "docs/examples/README.md",
                        "docs/examples/Modern Starter Robot.md",
                        "docs/examples/Hardware-free Reference Scenarios.md",
                        "docs/examples/Field-relative Drive.md",
                        "docs/getting-started/First Pedro Auto.md",
                        "docs/examples/Subsystem Experiments.md"));

        String docs = "TeamCode/src/main/java/edu/ftcsushi/fw/docs/";
        for (String removed : Arrays.asList(
                "getting-started/First Mechanism.md",
                "getting-started/First TeleOp.md",
                "getting-started/First Task and Auto.md",
                "getting-started/README.md",
                "getting-started/learn-sushi/Role Paths.md",
                "examples/Framework Components Through Examples.md",
                "examples/BIOBUZZ Capability Map.md",
                "examples/Pedro Autonomous Reference.md")) {
            assertTrue("Redundant documentation page still exists: " + removed,
                    !Files.exists(repositoryRoot.resolve(docs + removed)));
        }
    }

    @Test
    public void studentCourseKeepsSevenRunnableRobotCheckpoints() throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        Path anchorPath = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/"
                        + "Build a Robot Step by Step.md");
        assertTrue("Build-season anchor is missing", Files.isRegularFile(anchorPath));

        String anchor = readUtf8(anchorPath);
        assertTrue("Build-season course exceeds 1,800 prose words",
                proseWordCount(anchorPath) <= 1800);
        assertTrue("Build-season course exceeds eight Java excerpts",
                javaFenceCount(anchorPath) <= 8);
        assertTrue("Build-season course exceeds 410 source lines",
                Files.readAllLines(anchorPath, StandardCharsets.UTF_8).size() <= 410);
        String[] requiredStageHeadings = {
            "## 1. Name the robot's subsystems and interfaces",
            "## 2. Test the interface without hardware",
            "## 3. Connect hardware and write TeleOp",
            "## 4. Run a bounded subsystem experiment",
            "## 5. Integrate the whole robot",
            "## 6. Test individual Auto paths",
            "## 7. Test end-to-end Auto"
        };
        String[] requiredEvidenceLabels = {
            "**Outcome:**", "**Change these files:**", "### Critical code", "**Run this:**",
            "**Expect this:**", "**What to notice**", "**Key APIs**", "**If it fails:**",
            "**Advance when:**"
        };
        String[][] requiredStageDestinations = {
            {"learn-sushi/From Requirement to Robot.md"},
            {"../examples/Modern Starter Robot.md",
                    "robots/examples/starter/robot/StarterTeleOpControls.java"},
            {"Test a Mechanism Without Hardware.md",
                    "../testing-calibration/Actuator Bring-up.md",
                    "../examples/Modern Starter Robot.md",
                    "robots/examples/starter/opmode/StarterTeleOp.java"},
            {"../examples/Subsystem Experiments.md"},
            {"../examples/Modern Starter Robot.md"},
            {"First Pedro Auto.md", "../../integrations/pedro/README.md"},
            {}
        };
        String[] requiredStageSourceMarkers = {
            "<!-- teaching-shape -->",
            "<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/"
                    + "starter/robot/StarterFirstLessonTest.java -->",
            "<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/"
                    + "starter/robot/StarterMechanismLessonTest.java -->",
            "<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/"
                    + "reference/tester/ReferenceFlywheelSpinUpExperiment.java -->",
            "<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/"
                    + "reference/robot/ReferenceRobot.java -->",
            "<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/"
                    + "pedro/autonomous/BasicPedroAutoRoutine.java -->",
            "<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/"
                    + "pedro/robot/BasicPedroAutoRobot.java -->"
        };
        int priorStage = -1;
        for (int stageIndex = 0; stageIndex < requiredStageHeadings.length; stageIndex++) {
            String stageHeading = requiredStageHeadings[stageIndex];
            int sectionStart = anchor.indexOf(stageHeading);
            assertTrue("Build-season anchor is missing stage: " + stageHeading,
                    sectionStart >= 0);
            assertTrue("Build-season stages are out of order: " + stageHeading,
                    sectionStart > priorStage);
            priorStage = sectionStart;
            int sectionEnd = stageIndex + 1 < requiredStageHeadings.length
                    ? anchor.indexOf(requiredStageHeadings[stageIndex + 1], sectionStart)
                    : anchor.indexOf("## Progress checklist", sectionStart);
            assertTrue("Build-season stage has no closing boundary: " + stageHeading,
                    sectionEnd > sectionStart);
            String section = anchor.substring(sectionStart, sectionEnd);

            for (String label : requiredEvidenceLabels) {
                int occurrences = matcherCount(Pattern.compile(Pattern.quote(label))
                        .matcher(section));
                assertTrue(stageHeading + " must contain exactly one " + label
                                + "; found " + occurrences,
                        occurrences == 1);
            }
            assertTrue(stageHeading + " is missing its maintained source marker",
                    section.contains(requiredStageSourceMarkers[stageIndex]));
            for (String destination : requiredStageDestinations[stageIndex]) {
                assertTrue(stageHeading + " is missing destination: " + destination,
                        section.contains(destination));
            }

            int apiStart = section.indexOf("**Key APIs**");
            int apiEnd = section.indexOf("**If it fails:**", apiStart);
            assertTrue(stageHeading + " has an invalid Key APIs boundary",
                    apiStart >= 0 && apiEnd > apiStart);
            String apiBlock = section.substring(apiStart, apiEnd);
            Matcher apiBullets = Pattern.compile("(?m)^- (.+)$").matcher(apiBlock);
            int apiCount = 0;
            while (apiBullets.find()) {
                apiCount++;
                assertTrue(stageHeading + " has a Key APIs bullet without a concrete API: "
                                + apiBullets.group(),
                        apiBullets.group(1).startsWith("`"));
            }
            assertTrue(stageHeading + " must name one to five key APIs; found " + apiCount,
                    apiCount >= 1 && apiCount <= 5);
        }

        int interfaceStageStart = anchor.indexOf(requiredStageHeadings[0]);
        int interfaceStageEnd = anchor.indexOf(requiredStageHeadings[1], interfaceStageStart);
        String interfaceStage = anchor.substring(interfaceStageStart, interfaceStageEnd);
        assertTrue("Stage 1's displayed teaching shape must use the three real capability methods",
                interfaceStage.contains("void setMode(Mode mode);")
                        && interfaceStage.contains(
                                "Task collectForSeconds(double durationSec);")
                        && interfaceStage.contains("Status status();"));
        String starterInterface = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/"
                        + "intake/StarterIntake.java"));
        assertTrue("The course's only teaching shape must use real Starter capability APIs",
                starterInterface.contains("void setMode(Mode mode);")
                        && starterInterface.contains(
                                "Task collectForSeconds(double durationSec);")
                        && starterInterface.contains("Status status();"));

        assertTrue("Course must start with the one-file physical milestone",
                anchor.contains("Get your first robot driving"));
        assertTrue("Course must distinguish software evidence from physical evidence",
                anchor.contains("Software proves the mapping, not the future mechanism or "
                                + "physical behavior")
                        && anchor.contains("Compilation does not prove direction, braking, "
                                + "clearance, or response"));
        assertTrue("Course must not authorize physical Pedro motion without route-time control",
                anchor.contains("ordinary Sushi route callers cannot currently set that")
                        && Pattern.compile("does not\\s+authorize a physical path test")
                                .matcher(anchor).find()
                        && anchor.contains("Do not enable either the generic host or an adapted host")
                        && anchor.contains("physical end-to-end rehearsal waits"));
        assertTrue("Starter TeleOp exposure must remain an explicit final physical gate",
                anchor.contains("remove `@Disabled`")
                        && anchor.contains("INIT with controls neutral"));
        assertTrue("Build-season anchor must include an accessible Mermaid title",
                anchor.contains("accTitle: Seven robot-building checkpoints"));
        assertTrue("Build-season anchor must include an accessible Mermaid description",
                anchor.contains("accDescr:"));
        assertTrue("Build-season anchor must include a diagram text fallback",
                anchor.contains("**Text version:**"));
    }

    @Test
    public void survivingPedroLessonKeepsTheCompleteTruthfulTeachingSurface()
            throws IOException {
        Path repositoryRoot = MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));
        Path lessonPath = repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/fw/docs/getting-started/"
                        + "First Pedro Auto.md");
        assertTrue("Surviving Pedro lesson is missing", Files.isRegularFile(lessonPath));
        String lesson = readUtf8(lessonPath);

        String sourcePrefix = "<!-- source-excerpt: TeamCode/src/main/java/"
                + "edu/ftcsushi/robots/examples/pedro/";
        assertTrue("Pedro lesson must retain source-backed fixed path geometry",
                lesson.contains(sourcePrefix
                        + "autonomous/BasicPedroAutoPaths.java -->"));
        assertTrue("Pedro lesson must retain source-backed route policy",
                lesson.contains(sourcePrefix
                        + "autonomous/BasicPedroAutoRoutine.java -->")
                        && lesson.contains("RouteTasks.follow(")
                        && lesson.contains("Tasks.branchOnOutcome("));
        assertTrue("Pedro lesson must retain its source-backed disabled host",
                lesson.contains(sourcePrefix + "opmode/BasicPedroAutoExample.java -->")
                        && lesson.contains("@Disabled")
                        && lesson.contains("allowRobotMotion"));
        assertTrue("Pedro lesson must distinguish exact route status from Task outcome",
                lesson.contains("RouteStatus")
                        && lesson.contains("TaskOutcome")
                        && lesson.contains("FOLLOWER_TIMEOUT_OR_STALL")
                        && lesson.contains("UNKNOWN_TERMINAL"));
        assertTrue("Pedro lesson must retain the exact integration contract link",
                lesson.contains("../../integrations/pedro/README.md"));
        assertTrue("Pedro lesson must keep physical motion blocked without route-time control",
                lesson.contains("Ordinary Sushi route callers cannot currently set")
                        && lesson.contains("does not authorize a later physical test")
                        && lesson.contains("Do not set it true for physical motion"));

        String profile = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/robot/"
                        + "BasicPedroProfile.java"));
        String robot = readUtf8(repositoryRoot.resolve(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/pedro/robot/"
                        + "BasicPedroAutoRobot.java"));
        assertTrue("Pedro example source must not present review alone as motion permission",
                profile.contains("Keep {@link #allowRobotMotion} false for physical use")
                        && profile.contains("physical motion remains blocked")
                        && robot.contains("Keep it false for physical")
                        && robot.contains("persistent Follower power limit"));
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
        assertTrue("First robot-code lesson exceeds 1,000 prose words",
                proseWordCount(firstRobotCode) <= 1000);
        assertTrue("First-drive lesson must show its complete program exactly once",
                javaFenceCount(firstRobotCode) == 1);
        assertTrue("First-drive lesson exceeds 55 displayed Java lines",
                displayedJavaLineCount(firstRobotCode) <= 55);
        String firstRobotCodeText = readUtf8(firstRobotCode);
        String firstDrivePath = "TeamCode/src/main/java/edu/ftcsushi/robots/examples/firstdrive/"
                + "FirstDriveTeleOp.java";
        Path firstDrive = repositoryRoot.resolve(firstDrivePath);
        assertTrue("Maintained first-drive example is missing", Files.isRegularFile(firstDrive));
        assertTrue("First-drive lesson must reproduce the maintained example",
                firstRobotCodeText.contains("<!-- source-file: " + firstDrivePath + " -->"));
        String firstDriveSource = readUtf8(firstDrive);
        assertTrue("First-drive example must stay disabled by default",
                Pattern.compile("(?m)^@Disabled[ \\t]*$")
                        .matcher(firstDriveSource).find());
        assertTrue("First-drive example must declare a real TeleOp",
                Pattern.compile("(?m)^@TeleOp\\([^\\r\\n]+\\)[ \\t]*$")
                        .matcher(firstDriveSource).find());
        assertTrue("First-drive example must remain one final managed OpMode",
                Pattern.compile("public\\s+final\\s+class\\s+FirstDriveTeleOp\\s+"
                                + "extends\\s+FtcRobotOpMode")
                        .matcher(firstDriveSource).find()
                        && !Pattern.compile("\\bvoid\\s+loop\\s*\\(")
                        .matcher(firstDriveSource).find());
        assertTrue("First-drive controls must own the three selected gamepad meanings",
                firstDriveSource.contains(
                        "new FirstDriveControls(new GamepadDevice(gamepad1))")
                        && firstDriveSource.contains("new GamepadDriveSource(")
                        && firstDriveSource.contains("driver.leftX()")
                        && firstDriveSource.contains("driver.leftY()")
                        && firstDriveSource.contains("driver.rightX()")
                        && firstDriveSource.contains("GamepadDriveSource.Config.defaults()")
                        && firstDriveSource.contains(".scaled(FIRST_RUN_TRANSLATION_SCALE, "
                                + "FIRST_RUN_TURN_SCALE)"));
        assertTrue("First-drive composition must expose reviewed wiring and one configured sink",
                firstDriveSource.contains("FtcDrives.MecanumConfig.defaults()")
                        && firstDriveSource.contains("drive.wiring.frontLeftName")
                        && firstDriveSource.contains("drive.wiring.frontRightName")
                        && firstDriveSource.contains("drive.wiring.backLeftName")
                        && firstDriveSource.contains("drive.wiring.backRightName")
                        && firstDriveSource.contains("drive.wiring.frontLeftDirection")
                        && firstDriveSource.contains("drive.wiring.frontRightDirection")
                        && firstDriveSource.contains("drive.wiring.backLeftDirection")
                        && firstDriveSource.contains("drive.wiring.backRightDirection")
                        && firstDriveSource.contains("drive.enableZeroPowerBrake = true")
                        && firstDriveSource.contains("program.drive(controls.driveSource(), "
                                + "FtcDrives.mecanum(hardwareMap, drive))")
                        && matcherCount(Pattern.compile("program\\.drive\\s*\\(")
                                .matcher(firstDriveSource)) == 1);
        assertTrue("First-drive example must not introduce another managed role or raw lookup",
                !firstDriveSource.contains("program.output(")
                        && !firstDriveSource.contains("program.service(")
                        && !firstDriveSource.contains("program.rootTask(")
                        && !firstDriveSource.contains("hardwareMap.get("));
        int firstDriveJavaFiles = 0;
        try (DirectoryStream<Path> children = Files.newDirectoryStream(firstDrive.getParent())) {
            for (Path child : children) {
                if (Files.isRegularFile(child)
                        && child.getFileName().toString().endsWith(".java")) {
                    firstDriveJavaFiles++;
                }
            }
        }
        assertTrue("First-drive milestone must remain exactly one Java file; found "
                        + firstDriveJavaFiles,
                firstDriveJavaFiles == 1);
        long nonblankFirstDriveLines = Files.readAllLines(firstDrive, StandardCharsets.UTF_8)
                .stream().filter(line -> !line.trim().isEmpty()).count();
        assertTrue("First-drive example exceeds 45 nonblank Java lines: "
                        + nonblankFirstDriveLines,
                nonblankFirstDriveLines <= 45);
        assertTrue("First-drive lesson must keep the physical evidence boundary visible",
                firstRobotCodeText.contains("compilation cannot prove wiring")
                        && firstRobotCodeText.contains("HW: Actuator Bring-up")
                        && firstRobotCodeText.contains("Verify wheel patterns on the stand")
                        && firstRobotCodeText.contains("Make the first controlled floor drive")
                        && firstRobotCodeText.contains("STOP"));
        int copyStep = firstRobotCodeText.indexOf("## 1. Copy the complete program");
        int wiringStep = firstRobotCodeText.indexOf(
                "## 2. Replace assumptions with your wiring facts");
        int disabledCompileStep = firstRobotCodeText.indexOf(
                "## 3. Compile while motion remains disabled");
        int standStep = firstRobotCodeText.indexOf("## 4. Verify wheel patterns on the stand");
        int floorStep = firstRobotCodeText.indexOf(
                "## 5. Make the first controlled floor drive");
        int growStep = firstRobotCodeText.indexOf("## 6. Grow beyond one file");
        assertTrue("First-drive physical gates are missing or out of order",
                copyStep >= 0
                        && copyStep < wiringStep
                        && wiringStep < disabledCompileStep
                        && disabledCompileStep < standStep
                        && standStep < floorStep
                        && floorStep < growStep);
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
        // Learning-mode orientation makes each topic's intended use explicit without adding code.
        assertTrue("Six Sushi topic pages exceed 4,800 prose words: " + topicWords,
                topicWords <= 4800);
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

    private static void collectMarkdownFiles(Path root, final List<Path> files)
            throws IOException {
        Files.walkFileTree(root, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attributes) {
                if (file.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".md")) {
                    files.add(file);
                }
                return FileVisitResult.CONTINUE;
            }
        });
    }

    private static int matcherCount(Matcher matcher) {
        int count = 0;
        while (matcher.find()) {
            count++;
        }
        return count;
    }

    private static void requireText(Path repositoryRoot,
                                    Path page,
                                    String markdown,
                                    String required,
                                    String description,
                                    List<String> failures) {
        if (!markdown.contains(required)) {
            failures.add(repositoryRelativePath(repositoryRoot, page)
                    + ": missing " + description);
        }
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

    private static String stripDocumentationAnnotations(Path repositoryRoot,
                                                         Path page,
                                                         String snippet,
                                                         List<String> failures) {
        StringBuilder stripped = new StringBuilder();
        String[] lines = snippet.replace("\r\n", "\n").replace('\r', '\n')
                .split("\n", -1);
        for (String line : lines) {
            if (line.trim().startsWith("// docs:")) {
                continue;
            }
            if (line.contains("// docs:")) {
                failures.add(repositoryRelativePath(repositoryRoot, page)
                        + ": // docs: must occupy a standalone comment line");
            }
            if (stripped.length() > 0) {
                stripped.append('\n');
            }
            stripped.append(line);
        }
        return stripped.toString();
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
            if (!insideJava && trimmed.equals("```java")) {
                insideJava = true;
            } else if (insideJava && trimmed.equals("```")) {
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
