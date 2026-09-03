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

    private static final List<String> GUIDE_AREAS = Arrays.asList(
            "Get Started",
            "Learn",
            "Build",
            "Test & Tune",
            "Advanced",
            "Reference");

    private static final List<String> BUILD_MARKDOWN_FILES = Arrays.asList(
            "Continuous Intake.md",
            "First Autonomous.md",
            "First Drive.md",
            "First Pedro Auto.md",
            "Named Claw.md",
            "README.md",
            "Referenced Lift.md");

    private static final List<String> BUILD_RECIPE_FILES = Arrays.asList(
            "First Drive.md",
            "Continuous Intake.md",
            "Named Claw.md",
            "Referenced Lift.md",
            "First Autonomous.md",
            "First Pedro Auto.md");

    private static final List<String> BUILD_NAV_TARGETS = Arrays.asList(
            "docs/build/README.md",
            "docs/build/First Drive.md",
            "docs/build/Continuous Intake.md",
            "docs/build/Named Claw.md",
            "docs/build/Referenced Lift.md",
            "docs/build/First Autonomous.md",
            "docs/build/First Pedro Auto.md");

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
            "(?m)^\\x60\\x60\\x60java[ \\t]*$");
    private static final Pattern SOURCE_EXCERPT = Pattern.compile(
            "(?m)^<!-- source-excerpt: ([^>]+) -->\\r?\\n"
                    + "\\x60\\x60\\x60java[ \\t]*\\r?\\n"
                    + "([\\s\\S]*?)\\r?\\n\\x60\\x60\\x60[ \\t]*$");
    private static final Pattern COMPLETE_SOURCE = Pattern.compile(
            "\\[Complete source:[^]]+]\\(<"
                    + Pattern.quote(MAINTAINED_REPOSITORY_ROOT)
                    + "(?:blob|tree)/master/([^>]+)>\\)");

    @Rule
    public final TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void maintainedRepositoryMarkdownHasValidLocalLinksAnchorsAndFences()
            throws IOException {
        Path repositoryRoot = repositoryRoot();

        assertNoFailures(MarkdownIntegrity.validateRepository(repositoryRoot));
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
    public void buildAreaHasExactlySixIndependentRecipes() throws IOException {
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
        assertTrue("Build index must keep mechanisms goal-selective",
                index.contains("independent checkpoint")
                        && index.contains("You do not need a lift to learn a claw"));
    }

    @Test
    public void everyBuildRecipeUsesTheSourceBackedEvidenceAnatomy() throws IOException {
        Path repositoryRoot = repositoryRoot();
        Path buildRoot = repositoryRoot.resolve(FRAMEWORK_DOCS_PATH).resolve("docs/build");
        List<String> failures = new ArrayList<String>();

        for (String fileName : BUILD_RECIPE_FILES) {
            Path page = buildRoot.resolve(fileName);
            String markdown = readUtf8(page);

            requireExactlyOnce(markdown, "**Outcome:**", fileName, failures);
            requireExactlyOnce(markdown, "**Prerequisites:**", fileName, failures);
            requireExactlyOnce(markdown, "## Critical production idea", fileName, failures);
            requireExactlyOnce(markdown, "## Files in this checkpoint", fileName, failures);
            requireExactlyOnce(markdown, "## Software checkpoint:", fileName, failures);
            requireExactlyOnce(markdown, "## Isolated hardware gate", fileName, failures);
            requireExactlyOnce(markdown, "**Next gate:**", fileName, failures);
            requireOrdered(markdown, fileName, failures,
                    "**Outcome:**",
                    "**Prerequisites:**",
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
            requireOrdered(markdown, fileName, failures,
                    "**Read the causal chain:**",
                    "**Proves:**",
                    "**Does not prove:**",
                    "**Next gate:**");

            boolean hasMainManifest = Pattern.compile(
                    "(?m)^\\*\\*Main(?: added here)?:\\*\\*$")
                    .matcher(markdown).find();
            boolean hasTestManifest = Pattern.compile("(?m)^\\*\\*Test:\\*\\*$")
                    .matcher(markdown).find();
            if (!hasMainManifest || !hasTestManifest) {
                failures.add(fileName + ": missing exact Main/Test checkpoint manifest");
            }
            if (!markdown.contains(".\\gradlew.bat --console=plain "
                    + ":TeamCode:testDebugUnitTest --tests")) {
                failures.add(fileName + ": missing focused Gradle scenario command");
            }
            if (!markdown.contains(PUBLISHED_API_ROOT)) {
                failures.add(fileName + ": missing generated API link");
            }
            if (markdown.contains("source-file:")
                    || markdown.contains("annotated-source")
                    || markdown.contains("teaching-shape")) {
                failures.add(fileName + ": contains a full, annotated, or invented Java source");
            }

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

            validateBuildSources(repositoryRoot, fileName, markdown, failures);
        }

        assertTrue("Build recipe contract failures: " + failures, failures.isEmpty());
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
        assertTrue("Test & Tune home must route to the testing philosophy first",
                testingHome.contains(
                        "[How to test a Sushi component](<How to test a Sushi component.md>)"));
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

        assertTrue("Root README must identify Sushi as the framework",
                rootReadme.startsWith("# Sushi framework"));
        assertTrue("Namespace README must identify Sushi as the framework",
                namespaceReadme.startsWith("# Sushi framework"));
        assertTrue("Framework doorway must identify Sushi",
                frameworkHome.contains("# Build a robot with Sushi"));
        assertTrue("Documentation hub must identify Sushi",
                docsHome.contains("# Sushi documentation"));
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
                beginner.contains("`Tasks.sequence(...)` starts its next child only after exact")
                        && beginner.contains("`Tasks.sequenceOnCompletion(...)`"));
        assertTrue("Adaptive park takeover must use explicit completion continuation",
                adaptive.contains(
                        "program.rootTask(Tasks.sequenceOnCompletion(boundedPrePark, park));")
                        && adaptive.contains("This continuation is not Java"));
        assertTrue("Cheat sheet must keep both sequence choices visible",
                cheatSheet.contains("`Tasks.sequence(...)` is the ordinary prerequisite chain")
                        && cheatSheet.contains("`Tasks.sequenceOnCompletion(...)` is only for"));
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
            topicWords += proseWordCount(topics.resolve(topic));
        }
        assertTrue("Six Learn pages exceed 5,400 prose words: " + topicWords,
                topicWords <= 5400);
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
        return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
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
                return;
            }
            if (found <= previous) {
                failures.add(page + ": teaching elements are out of order at " + token);
                return;
            }
            previous = found;
        }
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
            if (lines < 3 || lines > 12) {
                failures.add(pageName + ": source excerpt must contain 3–12 lines, found "
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
        if (excerptCount != 2 || javaFences != excerptCount || !excerptsMain || !excerptsTest) {
            failures.add(pageName + ": expected exactly one main and one test source excerpt; "
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
        boolean insideFence = false;
        StringBuilder prose = new StringBuilder();
        for (String line : Files.readAllLines(path, StandardCharsets.UTF_8)) {
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
        int count = 0;
        for (String line : Files.readAllLines(path, StandardCharsets.UTF_8)) {
            if (line.trim().equals(FENCE + "java")) {
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
            if (!insideJava && trimmed.equals(FENCE + "java")) {
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
