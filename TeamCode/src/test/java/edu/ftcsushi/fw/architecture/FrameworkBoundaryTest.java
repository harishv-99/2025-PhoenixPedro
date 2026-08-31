package edu.ftcsushi.fw.architecture;

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
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Prevents production dependencies from protected framework core to explicit edges. */
public final class FrameworkBoundaryTest {

    private static final String FIXTURE_PATH =
            "TeamCode/src/main/java/edu/ftcsushi/fw/future/Fixture.java";

    @Test
    public void maintainedProductionFrameworkHasNoProtectedCoreDependencyLeaks()
            throws IOException {
        Path repositoryRoot = FrameworkBoundary.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir")));

        assertNoViolations(FrameworkBoundary.validateProductionFramework(repositoryRoot));
    }

    @Test
    public void allowsJdkAndProtectedCoreImportsInAnUnknownFutureCorePackage() {
        String source = lines(
                "package edu.ftcsushi.fw.future.control;",
                "",
                "import java.util.List;",
                "import javax.crypto.Cipher;",
                "import edu.ftcsushi.fw.core.source.ScalarSource;",
                "import static java.lang.Math.max;",
                "import static edu.ftcsushi.fw.core.math.MathUtil.clamp;",
                "",
                "final class Fixture {}"
        );

        assertNoViolations(FrameworkBoundary.validateSource(FIXTURE_PATH, source));
    }

    @Test
    public void rejectsNormalAndStaticImportsOfBoundaryDependencies() {
        String source = lines(
                "package edu.ftcsushi.fw.future;",
                "",
                "import com.qualcomm.robotcore.hardware.Gamepad;",
                "import static com.pedropathing.pathgen.MathFunctions.normalizeAngle;",
                "",
                "final class Fixture {}"
        );

        List<String> violations = FrameworkBoundary.validateSource(FIXTURE_PATH, source);

        assertEquals(2, violations.size());
        assertViolation(violations, FIXTURE_PATH, 3, "com.qualcomm.");
        assertViolation(violations, FIXTURE_PATH, 4, "com.pedropathing.");
    }

    @Test
    public void rejectsKnownImportsWithWhitespaceAndCommentsAroundDots() {
        String source = lines(
                "package edu.ftcsushi.fw.future;",
                "",
                "import com /* FTC SDK */ . qualcomm . robotcore.hardware.Gamepad;",
                "",
                "final class Fixture {}"
        );

        List<String> violations = FrameworkBoundary.validateSource(FIXTURE_PATH, source);

        assertEquals(1, violations.size());
        assertViolation(violations, FIXTURE_PATH, 3, "com.qualcomm.");
    }

    @Test
    public void rejectsUnknownVendorImportsWithUnicodeJavaIdentifiers() {
        String source = lines(
                "package edu.ftcsushi.fw.future;",
                "",
                "import org.ex\u00e4mple.vendor.Device;",
                "",
                "final class Fixture {}"
        );

        List<String> violations = FrameworkBoundary.validateSource(FIXTURE_PATH, source);

        assertEquals(1, violations.size());
        assertViolation(violations, FIXTURE_PATH, 3, "org.ex\u00e4mple.vendor.Device");
    }

    @Test
    public void rejectsKnownFullyQualifiedNamesInCode() {
        String source = lines(
                "package edu.ftcsushi.fw.future;",
                "",
                "final class Fixture {",
                "    Class<?> type = android.util.Size.class;",
                "}"
        );

        List<String> violations = FrameworkBoundary.validateSource(FIXTURE_PATH, source);

        assertEquals(1, violations.size());
        assertViolation(violations, FIXTURE_PATH, 4, "android.");
    }

    @Test
    public void rejectsKnownCodeNamesWithWhitespaceAndCommentsAroundDots() {
        String source = lines(
                "package edu.ftcsushi.fw.future;",
                "",
                "final class Fixture {",
                "    Class<?> type = android /* platform */ . util . Size.class;",
                "}"
        );

        List<String> violations = FrameworkBoundary.validateSource(FIXTURE_PATH, source);

        assertEquals(1, violations.size());
        assertViolation(violations, FIXTURE_PATH, 4, "android.");
    }

    @Test
    public void rejectsKnownReflectionAndClassNameStrings() {
        String source = lines(
                "package edu.ftcsushi.fw.future;",
                "",
                "final class Fixture {",
                "    Class<?> type = Class.forName(\"org.opencv.core.Mat\");",
                "    String adapter = \"edu.ftcsushi.fw.ftc.FtcHardware\";",
                "}"
        );

        List<String> violations = FrameworkBoundary.validateSource(FIXTURE_PATH, source);

        assertEquals(2, violations.size());
        assertViolation(violations, FIXTURE_PATH, 4, "org.opencv.");
        assertViolation(violations, FIXTURE_PATH, 5, "edu.ftcsushi.fw.ftc.");
    }

    @Test
    public void masksLineBlockAndJavadocCommentsBeforeMatching() {
        String source = lines(
                "/** com.qualcomm.robotcore.hardware.Gamepad is mentioned only "
                        + "in documentation. */",
                "package edu.ftcsushi.fw.future;",
                "",
                "// import android.util.Size;",
                "/*",
                " * Class.forName(\"com.pedropathing.follower.Follower\");",
                " * edu.ftcsushi.robots.phoenix.PhoenixRobot",
                " */",
                "final class Fixture {}"
        );

        assertNoViolations(FrameworkBoundary.validateSource(FIXTURE_PATH, source));
    }

    @Test
    public void exemptsEachExplicitEdgeRootBasedOnItsDeclaredPackage() {
        List<String> edgePackages = Arrays.asList(
                "edu.ftcsushi.fw.ftc",
                "edu.ftcsushi.fw.integrations",
                "edu.ftcsushi.fw.tools"
        );

        for (String edgePackage : edgePackages) {
            String source = lines(
                    "package " + edgePackage + ";",
                    "import com.qualcomm.robotcore.hardware.Gamepad;",
                    "final class Fixture {",
                    "    String vendor = \"com.bylazar.configurables.PanelsConfigurables\";",
                    "}"
            );
            assertNoViolations(FrameworkBoundary.validateSource(
                    "TeamCode/src/main/java/edu/ftcsushi/fw/core/MisleadingPath.java",
                    source));
        }
    }

    @Test
    public void treatsEveryUnknownFrameworkPackageAsProtectedBasedOnDeclaration() {
        String source = lines(
                "package edu.ftcsushi.fw.brandnew;",
                "import org.example.vendor.Device;",
                "final class Fixture {}"
        );
        String misleadingEdgePath =
                "TeamCode/src/main/java/edu/ftcsushi/fw/ftc/MisleadingPath.java";

        List<String> violations = FrameworkBoundary.validateSource(misleadingEdgePath, source);

        assertEquals(1, violations.size());
        assertViolation(violations, misleadingEdgePath, 2, "org.example.vendor.Device");
    }

    @Test
    public void selectsOffPathSourcesThatDeclareProtectedFrameworkPackages() {
        String path = "TeamCode/src/main/java/misplaced/Fixture.java";
        String source = lines(
                "package edu.ftcsushi.fw.future;",
                "import org.example.vendor.Device;",
                "final class Fixture {}"
        );

        List<String> violations = FrameworkBoundary.validateProductionSource(path, source);

        assertEquals(1, violations.size());
        assertViolation(violations, path, 2, "org.example.vendor.Device");
    }

    @Test
    public void rejectsOutsideDeclarationsAtPhysicalFrameworkPaths() {
        String path = "TeamCode/src/main/java/edu/ftcsushi/fw/core/Misplaced.java";
        String source = lines(
                "package edu.ftcsushi.robots.misplaced;",
                "final class Misplaced {}"
        );

        List<String> violations = FrameworkBoundary.validateProductionSource(path, source);

        assertEquals(1, violations.size());
        assertTrue(violations.get(0), violations.get(0).startsWith(path + ":1:")
                && violations.get(0).contains("declared package "
                + "'edu.ftcsushi.robots.misplaced' is outside production "
                + "edu.ftcsushi.fw"));
    }

    @Test
    public void skipsUnrelatedProductionPackagesOutsideTheFrameworkPath() {
        String path = "TeamCode/src/main/java/org/example/application/App.java";
        String source = lines(
                "package org.example.application;",
                "import com.qualcomm.robotcore.hardware.Gamepad;",
                "final class App {}"
        );

        assertNoViolations(FrameworkBoundary.validateProductionSource(path, source));
    }

    private static String lines(String... lines) {
        StringBuilder source = new StringBuilder();
        for (String line : lines) {
            source.append(line).append('\n');
        }
        return source.toString();
    }

    private static void assertNoViolations(List<String> violations) {
        assertTrue("Framework boundary violations:\n" + joinLines(violations),
                violations.isEmpty());
    }

    private static void assertViolation(
            List<String> violations,
            String path,
            int line,
            String forbiddenPrefix
    ) {
        String location = path + ":" + line + ":";
        for (String violation : violations) {
            if (violation.startsWith(location)
                    && violation.contains("forbidden prefix '" + forbiddenPrefix + "'")) {
                assertTrue("Violation must name all explicit edge roots: " + violation,
                        violation.contains("edu.ftcsushi.fw.ftc")
                                && violation.contains("edu.ftcsushi.fw.integrations")
                                && violation.contains("edu.ftcsushi.fw.tools"));
                return;
            }
        }
        fail("Expected " + location + " to report forbidden prefix '" + forbiddenPrefix
                + "' but found:\n" + joinLines(violations));
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

    static final class FrameworkBoundary {

        private static final String FRAMEWORK_ROOT = "edu.ftcsushi.fw";
        private static final String PRODUCTION_SOURCE_ROOT = "TeamCode/src/main/java/";
        private static final String FRAMEWORK_SOURCE_ROOT =
                PRODUCTION_SOURCE_ROOT + "edu/ftcsushi/fw/";
        private static final List<String> EDGE_ROOTS = Arrays.asList(
                "edu.ftcsushi.fw.ftc",
                "edu.ftcsushi.fw.integrations",
                "edu.ftcsushi.fw.tools"
        );
        private static final List<String> KNOWN_FORBIDDEN_PREFIXES = Arrays.asList(
                "com.qualcomm.",
                "org.firstinspires.ftc.",
                "android.",
                "androidx.",
                "com.pedropathing.",
                "com.bylazar.",
                "org.opencv.",
                "edu.ftcsushi.fw.ftc.",
                "edu.ftcsushi.fw.integrations.",
                "edu.ftcsushi.fw.tools.",
                "edu.ftcsushi.robots."
        );
        private static final List<Pattern> KNOWN_FORBIDDEN_CODE_PATTERNS =
                knownForbiddenCodePatterns();
        private static final String JAVA_IDENTIFIER =
                "\\p{javaJavaIdentifierStart}\\p{javaJavaIdentifierPart}*";
        private static final Pattern PACKAGE_DECLARATION = Pattern.compile(
                "(?m)^[ \\t]*package\\s+(" + JAVA_IDENTIFIER + "(?:\\s*\\.\\s*"
                        + JAVA_IDENTIFIER + ")*)\\s*;");
        private static final Pattern IMPORT_DECLARATION = Pattern.compile(
                "(?m)^[ \\t]*import\\s+(?:static\\s+)?"
                        + "(" + JAVA_IDENTIFIER + "(?:\\s*\\.\\s*(?:"
                        + JAVA_IDENTIFIER + "|\\*))+)\\s*;");
        private static final String PERMITTED_BOUNDARIES =
                "protected core may import only java.*, javax.*, and other protected "
                        + "edu.ftcsushi.fw packages; edge code belongs under "
                        + "edu.ftcsushi.fw.ftc, edu.ftcsushi.fw.integrations, or "
                        + "edu.ftcsushi.fw.tools";

        private FrameworkBoundary() {
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

        static List<String> validateProductionFramework(final Path repositoryRoot)
                throws IOException {
            final Path root = repositoryRoot.toAbsolutePath().normalize();
            final Path productionSources = root.resolve("TeamCode/src/main/java");
            final Path frameworkSources = root.resolve(FRAMEWORK_SOURCE_ROOT);
            if (!Files.isDirectory(frameworkSources)) {
                return Collections.singletonList(
                        "TeamCode/src/main/java/edu/ftcsushi/fw:1: production framework source "
                                + "root is missing");
            }

            final List<Path> sources = new ArrayList<>();
            Files.walkFileTree(productionSources, new SimpleFileVisitor<Path>() {
                @Override
                public FileVisitResult visitFile(Path file, BasicFileAttributes attributes) {
                    if (attributes.isRegularFile()
                            && file.getFileName().toString().endsWith(".java")) {
                        sources.add(file);
                    }
                    return FileVisitResult.CONTINUE;
                }
            });
            Collections.sort(sources);

            List<String> violations = new ArrayList<>();
            for (Path source : sources) {
                String relativePath = relativePath(root, source);
                String contents = new String(Files.readAllBytes(source), StandardCharsets.UTF_8);
                violations.addAll(validateProductionSource(relativePath, contents));
            }
            return Collections.unmodifiableList(violations);
        }

        static List<String> validateProductionSource(String relativePath, String source) {
            String normalizedPath = relativePath.replace('\\', '/');
            String packageName = declaredPackage(source);
            if (!normalizedPath.startsWith(FRAMEWORK_SOURCE_ROOT)
                    && (packageName == null || !isWithin(packageName, FRAMEWORK_ROOT))) {
                return Collections.emptyList();
            }
            return validateSource(normalizedPath, source);
        }

        static List<String> validateSource(String relativePath, String source) {
            String normalizedPath = relativePath.replace('\\', '/');
            String withoutComments = maskComments(source);
            String syntaxOnly = maskStringAndCharacterLiterals(withoutComments);
            Matcher packageMatcher = PACKAGE_DECLARATION.matcher(syntaxOnly);
            if (!packageMatcher.find()) {
                return Collections.singletonList(normalizedPath
                        + ":1: production framework source has no declared package");
            }

            String packageName = removeWhitespace(packageMatcher.group(1));
            if (!isWithin(packageName, FRAMEWORK_ROOT)) {
                return Collections.singletonList(normalizedPath + ":"
                        + lineNumber(source, packageMatcher.start(1))
                        + ": declared package '" + packageName
                        + "' is outside production " + FRAMEWORK_ROOT);
            }
            if (isExplicitEdge(packageName)) {
                return Collections.emptyList();
            }

            List<String> violations = new ArrayList<>();
            Set<String> reported = new LinkedHashSet<>();
            Matcher importMatcher = IMPORT_DECLARATION.matcher(syntaxOnly);
            while (importMatcher.find()) {
                String importedName = removeWhitespace(importMatcher.group(1));
                if (!isAllowedProtectedImport(importedName)) {
                    String knownPrefix = knownForbiddenPrefix(importedName);
                    addViolation(
                            violations,
                            reported,
                            normalizedPath,
                            lineNumber(source, importMatcher.start(1)),
                            knownPrefix == null ? importedName : knownPrefix);
                }
            }

            for (int index = 0; index < KNOWN_FORBIDDEN_PREFIXES.size(); index++) {
                String forbiddenPrefix = KNOWN_FORBIDDEN_PREFIXES.get(index);
                Matcher codeMatcher = KNOWN_FORBIDDEN_CODE_PATTERNS.get(index).matcher(syntaxOnly);
                while (codeMatcher.find()) {
                    if (isQualifiedNameStart(syntaxOnly, codeMatcher.start())) {
                        addViolation(
                                violations,
                                reported,
                                normalizedPath,
                                lineNumber(source, codeMatcher.start()),
                                forbiddenPrefix);
                    }
                }

                int from = 0;
                while (from < withoutComments.length()) {
                    int match = withoutComments.indexOf(forbiddenPrefix, from);
                    if (match < 0) {
                        break;
                    }
                    if (isQualifiedNameStart(withoutComments, match)) {
                        addViolation(
                                violations,
                                reported,
                                normalizedPath,
                                lineNumber(source, match),
                                forbiddenPrefix);
                    }
                    from = match + forbiddenPrefix.length();
                }
            }

            return Collections.unmodifiableList(violations);
        }

        private static String declaredPackage(String source) {
            String withoutComments = maskComments(source);
            String syntaxOnly = maskStringAndCharacterLiterals(withoutComments);
            Matcher packageMatcher = PACKAGE_DECLARATION.matcher(syntaxOnly);
            return packageMatcher.find()
                    ? removeWhitespace(packageMatcher.group(1))
                    : null;
        }

        private static List<Pattern> knownForbiddenCodePatterns() {
            List<Pattern> patterns = new ArrayList<>();
            for (String forbiddenPrefix : KNOWN_FORBIDDEN_PREFIXES) {
                String qualifiedName = forbiddenPrefix.substring(0, forbiddenPrefix.length() - 1);
                String[] parts = qualifiedName.split("\\.");
                StringBuilder expression = new StringBuilder();
                for (int index = 0; index < parts.length; index++) {
                    if (index > 0) {
                        expression.append("\\s*\\.\\s*");
                    }
                    expression.append(Pattern.quote(parts[index]));
                }
                expression.append("\\s*\\.");
                patterns.add(Pattern.compile(expression.toString()));
            }
            return Collections.unmodifiableList(patterns);
        }

        private static boolean isQualifiedNameStart(String source, int offset) {
            if (offset == 0) {
                return true;
            }
            char preceding = source.charAt(offset - 1);
            if (Character.isJavaIdentifierPart(preceding) || preceding == '.') {
                return false;
            }
            if (!Character.isWhitespace(preceding)) {
                return true;
            }

            int index = offset - 1;
            while (index >= 0 && Character.isWhitespace(source.charAt(index))) {
                index--;
            }
            return index < 0 || source.charAt(index) != '.';
        }

        private static boolean isAllowedProtectedImport(String importedName) {
            return importedName.startsWith("java.")
                    || importedName.startsWith("javax.")
                    || (isWithin(importedName, FRAMEWORK_ROOT)
                    && !isExplicitEdge(importedName));
        }

        private static boolean isExplicitEdge(String qualifiedName) {
            for (String edgeRoot : EDGE_ROOTS) {
                if (isWithin(qualifiedName, edgeRoot)) {
                    return true;
                }
            }
            return false;
        }

        private static boolean isWithin(String qualifiedName, String root) {
            return qualifiedName.equals(root) || qualifiedName.startsWith(root + ".");
        }

        private static String knownForbiddenPrefix(String value) {
            for (String forbiddenPrefix : KNOWN_FORBIDDEN_PREFIXES) {
                if (value.startsWith(forbiddenPrefix)) {
                    return forbiddenPrefix;
                }
            }
            return null;
        }

        private static void addViolation(
                List<String> violations,
                Set<String> reported,
                String path,
                int line,
                String forbiddenPrefix
        ) {
            String key = line + "\u0000" + forbiddenPrefix;
            if (reported.add(key)) {
                violations.add(path + ":" + line + ": forbidden prefix '" + forbiddenPrefix
                        + "'; " + PERMITTED_BOUNDARIES);
            }
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

        private static String maskStringAndCharacterLiterals(String sourceWithoutComments) {
            StringBuilder masked = new StringBuilder(sourceWithoutComments.length());
            char delimiter = '\0';
            for (int index = 0; index < sourceWithoutComments.length(); index++) {
                char character = sourceWithoutComments.charAt(index);
                if (delimiter == '\0') {
                    if (character == '"' || character == '\'') {
                        delimiter = character;
                        masked.append(' ');
                    } else {
                        masked.append(character);
                    }
                } else if (character == '\r' || character == '\n') {
                    masked.append(character);
                } else if (character == '\\' && index + 1 < sourceWithoutComments.length()) {
                    masked.append(' ');
                    char escaped = sourceWithoutComments.charAt(++index);
                    masked.append(escaped == '\r' || escaped == '\n' ? escaped : ' ');
                } else {
                    masked.append(' ');
                    if (character == delimiter) {
                        delimiter = '\0';
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
