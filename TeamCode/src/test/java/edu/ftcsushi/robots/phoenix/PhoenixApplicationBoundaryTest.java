package edu.ftcsushi.robots.phoenix;

import org.junit.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Enforces the one-way boundary around the Phoenix production application. */
public final class PhoenixApplicationBoundaryTest {

    @Test
    public void maintainedJavaKeepsPhoenixInsideItsMainAndTestBubble() throws IOException {
        Path repositoryRoot = ApplicationBoundary.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));

        assertNoViolations(ApplicationBoundary.validateRepository(repositoryRoot));
    }

    @Test
    public void genericExamplesImportOnlyOtherGenericExamplesUnderTheRobotNamespace()
            throws IOException {
        Path repositoryRoot = ApplicationBoundary.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));

        assertNoViolations(ApplicationBoundary.validateExampleImports(repositoryRoot));
    }

    @Test
    public void rejectsApplicationNamesOutsideTheBubbleButAllowsExactInertEvidence() {
        String outsidePath =
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/Fixture.java";
        String source = lines(
                "package edu.ftcsushi.robots.examples;",
                "import edu.ftcsushi.robots.phoenix.PhoenixRobot;",
                "/** Phoenix application policy must not become teaching prose. */",
                "final class Fixture { PhoenixProfile profile; }"
        );

        List<String> violations = ApplicationBoundary.validateOutsideBubbleSource(
                outsidePath,
                source);

        assertEquals(3, violations.size());
        assertTrue(violations.get(0), violations.get(0).startsWith(outsidePath + ":2:"));
        assertTrue(violations.get(1), violations.get(1).startsWith(outsidePath + ":3:"));
        assertTrue(violations.get(2), violations.get(2).startsWith(outsidePath + ":4:"));

        assertNoViolations(ApplicationBoundary.validateOutsideBubbleSource(
                outsidePath,
                "final class Fixture { String repository = \"2025-PhoenixPedro\"; }\n"));
        assertEquals(1, ApplicationBoundary.validateOutsideBubbleSource(
                "TeamCode/src/main/java/edu/ftcsushi/fw/PhoenixPolicy.java",
                "package edu.ftcsushi.fw; final class Policy {}\n").size());
        assertNoViolations(ApplicationBoundary.validateOutsideBubbleSource(
                "TeamCode/src/main/java/edu/ftcsushi/robots/phoenix/Fixture.java",
                source));
        assertNoViolations(ApplicationBoundary.validateApplicationPathPackage(
                "TeamCode/src/main/java/edu/ftcsushi/robots/phoenix/Fixture.java",
                "package edu.ftcsushi.robots.phoenix;\n"));
        assertEquals(1, ApplicationBoundary.validateApplicationPathPackage(
                "TeamCode/src/main/java/edu/ftcsushi/robots/phoenix/Leaked.java",
                "package edu.ftcsushi.fw.foo;\n").size());
        assertNoViolations(ApplicationBoundary.validateOutsideBubbleSource(
                "TeamCode/src/test/java/edu/ftcsushi/fw/docs/DocumentationLinksTest.java",
                "String oldPage = \"First Phoenix Robot Code.md\";\n"));

        List<String> exampleImportViolations = ApplicationBoundary.validateExampleSource(
                outsidePath,
                lines(
                        "import edu.ftcsushi.robots.competition.*;",
                        "import static edu.ftcsushi.robots.competition.Constants.VALUE;",
                        "final class Fixture { "
                                + "edu.ftcsushi.robots.competition.CompetitionRobot robot; }",
                        "import edu /* split */ . ftcsushi . robots . competition . Service;",
                        "import edu.ftcsushi.robots.examples.reference.*;"
                ));
        assertEquals(4, exampleImportViolations.size());
    }

    private static String lines(String... lines) {
        StringBuilder source = new StringBuilder();
        for (String line : lines) {
            source.append(line).append('\n');
        }
        return source.toString();
    }

    private static void assertNoViolations(List<String> violations) {
        assertTrue("Application boundary violations:\n" + joinLines(violations),
                violations.isEmpty());
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

    static final class ApplicationBoundary {

        private static final String MAIN_SOURCE_ROOT = "TeamCode/src/main/java/";
        private static final String TEST_SOURCE_ROOT = "TeamCode/src/test/java/";
        private static final String MAIN_APPLICATION_ROOT =
                MAIN_SOURCE_ROOT + "edu/ftcsushi/robots/phoenix/";
        private static final String TEST_APPLICATION_ROOT =
                TEST_SOURCE_ROOT + "edu/ftcsushi/robots/phoenix/";
        private static final String MAIN_EXAMPLE_ROOT =
                MAIN_SOURCE_ROOT + "edu/ftcsushi/robots/examples/";
        private static final String TEST_EXAMPLE_ROOT =
                TEST_SOURCE_ROOT + "edu/ftcsushi/robots/examples/";
        private static final Pattern APPLICATION_REFERENCE = Pattern.compile("(?i)phoenix");
        private static final Pattern PACKAGE_DECLARATION = Pattern.compile(
                "(?m)^[ \\t]*package[ \\t]+"
                        + "([\\p{javaJavaIdentifierPart}.]+)[ \\t]*;");
        private static final String JAVA_IDENTIFIER =
                "\\p{javaJavaIdentifierStart}\\p{javaJavaIdentifierPart}*";
        private static final Pattern ROBOT_REFERENCE = Pattern.compile(
                "\\b(edu\\s*\\.\\s*ftcsushi\\s*\\.\\s*robots\\s*\\.\\s*"
                        + "(?:" + JAVA_IDENTIFIER + "\\s*\\.\\s*)*"
                        + "(?:" + JAVA_IDENTIFIER + "|\\*))");

        private ApplicationBoundary() {
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
            final Path root = repositoryRoot.toAbsolutePath().normalize();
            final List<String> violations = new ArrayList<String>();
            for (String sourceRoot : new String[]{MAIN_SOURCE_ROOT, TEST_SOURCE_ROOT}) {
                Path sources = root.resolve(sourceRoot);
                Files.walkFileTree(sources, new SimpleFileVisitor<Path>() {
                    @Override
                    public FileVisitResult visitFile(Path file, BasicFileAttributes attributes)
                            throws IOException {
                        if (attributes.isRegularFile()
                                && file.getFileName().toString().endsWith(".java")) {
                            String relativePath = relativePath(root, file);
                            String source = new String(
                                    Files.readAllBytes(file),
                                    StandardCharsets.UTF_8);
                            violations.addAll(validateApplicationPathPackage(
                                    relativePath,
                                    source));
                            violations.addAll(validateOutsideBubbleSource(relativePath, source));
                        }
                        return FileVisitResult.CONTINUE;
                    }
                });
            }
            Collections.sort(violations);
            return Collections.unmodifiableList(violations);
        }

        static List<String> validateExampleImports(Path repositoryRoot) throws IOException {
            final Path root = repositoryRoot.toAbsolutePath().normalize();
            final List<String> violations = new ArrayList<String>();
            for (String exampleRoot : new String[]{MAIN_EXAMPLE_ROOT, TEST_EXAMPLE_ROOT}) {
                Path examples = root.resolve(exampleRoot);
                if (!Files.isDirectory(examples)) {
                    continue;
                }
                Files.walkFileTree(examples, new SimpleFileVisitor<Path>() {
                    @Override
                    public FileVisitResult visitFile(Path file, BasicFileAttributes attributes)
                            throws IOException {
                        if (attributes.isRegularFile()
                                && file.getFileName().toString().endsWith(".java")) {
                            String source = new String(
                                    Files.readAllBytes(file),
                                    StandardCharsets.UTF_8);
                            violations.addAll(validateExampleSource(
                                    relativePath(root, file),
                                    source));
                        }
                        return FileVisitResult.CONTINUE;
                    }
                });
            }
            Collections.sort(violations);
            return Collections.unmodifiableList(violations);
        }

        static List<String> validateExampleSource(String relativePath, String source) {
            List<String> violations = new ArrayList<String>();
            Set<Integer> reportedOffsets = new LinkedHashSet<Integer>();
            collectExampleReferences(relativePath, source, source, reportedOffsets, violations);
            collectExampleReferences(
                    relativePath,
                    source,
                    maskComments(source),
                    reportedOffsets,
                    violations);
            return Collections.unmodifiableList(violations);
        }

        private static void collectExampleReferences(
                String relativePath,
                String originalSource,
                String searchableSource,
                Set<Integer> reportedOffsets,
                List<String> violations) {
            Matcher references = ROBOT_REFERENCE.matcher(searchableSource);
            while (references.find()) {
                String referencedName = removeWhitespace(references.group(1));
                if (!referencedName.equals("edu.ftcsushi.robots.examples")
                        && !referencedName.startsWith("edu.ftcsushi.robots.examples.")
                        && reportedOffsets.add(references.start(1))) {
                    violations.add(relativePath + ":"
                            + lineNumber(originalSource, references.start(1))
                            + ": generic example references production application '"
                            + referencedName + "'");
                }
            }
        }

        static List<String> validateApplicationPathPackage(String relativePath, String source) {
            String normalizedPath = relativePath.replace('\\', '/');
            if (!normalizedPath.startsWith(MAIN_APPLICATION_ROOT)
                    && !normalizedPath.startsWith(TEST_APPLICATION_ROOT)) {
                return Collections.emptyList();
            }

            String sourceRoot = normalizedPath.startsWith(MAIN_APPLICATION_ROOT)
                    ? MAIN_SOURCE_ROOT
                    : TEST_SOURCE_ROOT;
            int finalSlash = normalizedPath.lastIndexOf('/');
            String expectedPackage = normalizedPath.substring(
                    sourceRoot.length(),
                    finalSlash).replace('/', '.');
            Matcher declaration = PACKAGE_DECLARATION.matcher(source);
            boolean hasPackage = declaration.find();
            if (hasPackage && expectedPackage.equals(declaration.group(1))) {
                return Collections.emptyList();
            }

            String actualPackage = hasPackage ? declaration.group(1) : "<missing>";
            return Collections.singletonList(normalizedPath
                    + ": application source path requires package '" + expectedPackage
                    + "' but declares '" + actualPackage + "'");
        }

        static List<String> validateOutsideBubbleSource(String relativePath, String source) {
            String normalizedPath = relativePath.replace('\\', '/');
            if (normalizedPath.startsWith(MAIN_APPLICATION_ROOT)
                    || normalizedPath.startsWith(TEST_APPLICATION_ROOT)) {
                return Collections.emptyList();
            }

            List<String> violations = new ArrayList<String>();
            String semanticPath = normalizedPath.replace("2025-PhoenixPedro", "");
            Matcher pathReference = APPLICATION_REFERENCE.matcher(semanticPath);
            if (pathReference.find()) {
                violations.add(normalizedPath
                        + ": production application reference '" + pathReference.group()
                        + "' is present in a path outside " + MAIN_APPLICATION_ROOT + " or "
                        + TEST_APPLICATION_ROOT);
            }
            String[] lines = source.split("\\r?\\n", -1);
            for (int index = 0; index < lines.length; index++) {
                String semanticText = maskExplicitExceptions(normalizedPath, lines[index]);
                Matcher reference = APPLICATION_REFERENCE.matcher(semanticText);
                if (reference.find()) {
                    violations.add(normalizedPath + ":" + (index + 1)
                            + ": production application reference '" + reference.group()
                            + "' is outside " + MAIN_APPLICATION_ROOT + " or "
                            + TEST_APPLICATION_ROOT);
                }
            }
            return Collections.unmodifiableList(violations);
        }

        private static String maskExplicitExceptions(String relativePath, String line) {
            String masked = line.replace("2025-PhoenixPedro", "");
            if (relativePath.equals(
                    "TeamCode/src/test/java/edu/ftcsushi/fw/docs/DocumentationLinksTest.java")) {
                masked = masked
                        .replace("First Phoenix Robot Code.md", "")
                        .replace("learn-phoenix", "")
                        .replace("Phoenix Cheat Sheet.md", "")
                        .replace("ftcphoenix", "")
                        .replace("Phoenix framework", "")
                        .replace("phoenixJavadocs", "")
                        .replace("Pattern.compile(\"(?i)phoenix\")", "");
            } else if (relativePath.equals(
                    "TeamCode/src/test/java/edu/ftcsushi/fw/ftc/FtcFramesTest.java")) {
                masked = masked
                        .replace("Former Phoenix frame alias remains public", "")
                        .replace("\"phoenix\"", "");
            } else if (relativePath.equals(
                    "TeamCode/src/test/java/edu/ftcsushi/fw/integrations/pedro/"
                            + "PedroFieldTransformTest.java")) {
                masked = masked
                        .replace("Former Phoenix field alias remains public", "")
                        .replace("\"phoenix\"", "");
            }
            return masked;
        }

        private static String maskComments(String source) {
            StringBuilder masked = new StringBuilder(source.length());
            int state = 0;
            for (int index = 0; index < source.length(); index++) {
                char character = source.charAt(index);
                char next = index + 1 < source.length() ? source.charAt(index + 1) : '\0';

                if (state == 1) {
                    if (character == '\r' || character == '\n') {
                        masked.append(character);
                        state = 0;
                    } else {
                        masked.append(' ');
                    }
                    continue;
                }
                if (state == 2) {
                    if (character == '*' && next == '/') {
                        masked.append("  ");
                        index++;
                        state = 0;
                    } else {
                        masked.append(character == '\r' || character == '\n' ? character : ' ');
                    }
                    continue;
                }
                if (state == 3 || state == 4) {
                    masked.append(character);
                    if (character == '\\' && index + 1 < source.length()) {
                        masked.append(source.charAt(++index));
                    } else if ((state == 3 && character == '"')
                            || (state == 4 && character == '\'')) {
                        state = 0;
                    }
                    continue;
                }

                if (character == '/' && next == '/') {
                    masked.append("  ");
                    index++;
                    state = 1;
                } else if (character == '/' && next == '*') {
                    masked.append("  ");
                    index++;
                    state = 2;
                } else {
                    masked.append(character);
                    if (character == '"') {
                        state = 3;
                    } else if (character == '\'') {
                        state = 4;
                    }
                }
            }
            return masked.toString();
        }

        private static String removeWhitespace(String value) {
            StringBuilder compact = new StringBuilder(value.length());
            for (int index = 0; index < value.length(); index++) {
                char character = value.charAt(index);
                if (!Character.isWhitespace(character)) {
                    compact.append(character);
                }
            }
            return compact.toString();
        }

        private static int lineNumber(String source, int offset) {
            int line = 1;
            for (int index = 0; index < offset; index++) {
                if (source.charAt(index) == '\n') {
                    line++;
                }
            }
            return line;
        }

        private static String relativePath(Path root, Path path) {
            return root.relativize(path.toAbsolutePath().normalize())
                    .toString()
                    .replace('\\', '/');
        }
    }
}
