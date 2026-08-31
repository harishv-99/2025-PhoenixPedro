package edu.ftcsushi.fw.architecture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.GenericArrayType;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.lang.reflect.TypeVariable;
import java.lang.reflect.WildcardType;
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
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.FtcTeleOpTesterOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.testfixtures.host.CrossScopeRawMemberOwner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Enforces the one ordinary FTC host and explicit custom-host boundary. */
public final class ModernFtcHostBoundaryTest {

    private static final String PRODUCTION_JAVA_ROOT = "TeamCode/src/main/java";
    private static final Set<String> SHARED_HOST_PACKAGE_ROOTS =
            Collections.unmodifiableSet(new LinkedHashSet<String>(Arrays.asList(
                    "edu.ftcsushi.fw",
                    "edu.ftcsushi.robots.examples"
            )));
    private static final Set<String> FIXTURE_PACKAGE_ROOTS =
            Collections.singleton("edu.ftcsushi.fw.architecture");
    private static final List<String> FTC_CALLBACKS = Arrays.asList(
            "init",
            "init_loop",
            "start",
            "loop",
            "stop"
    );
    private static final Set<String> APPROVED_RAW_HOST_ROOTS =
            Collections.unmodifiableSet(new LinkedHashSet<String>(Arrays.asList(
                    FtcRobotOpMode.class.getName(),
                    FtcTeleOpTesterOpMode.class.getName()
            )));

    private static boolean initializationProbeRan;

    @Test
    public void productionOpModesUseOnlyApprovedOrReasonedHostFamilies() throws IOException {
        List<String> violations = new ArrayList<String>(validateScopedProductionHosts(
                Paths.get(System.getProperty("user.dir")),
                SHARED_HOST_PACKAGE_ROOTS,
                Collections.<String, String>emptyMap(),
                true
        ));
        validateRobotProgramBoundary(violations);

        assertNoViolations(violations);
    }

    /**
     * Test-only verifier for one framework, example, or application-owned package scope.
     *
     * <p>The caller owns every exact custom-host name and rationale. Production sources are still
     * discovered from the complete main tree so an off-path declaration cannot evade its selected
     * scope.</p>
     */
    public static List<String> validateScopedProductionHosts(
            Path startingPath,
            Set<String> packageRoots,
            Map<String, String> customHostExemptions,
            boolean requireApprovedRawRoots) throws IOException {
        Set<String> requiredPackageRoots = Collections.unmodifiableSet(
                new LinkedHashSet<String>(packageRoots));
        Map<String, String> requiredExemptions = Collections.unmodifiableMap(
                new LinkedHashMap<String, String>(customHostExemptions));
        List<String> violations = new ArrayList<String>();
        if (requiredPackageRoots.isEmpty()) {
            violations.add("host verification requires at least one package root");
            return Collections.unmodifiableList(violations);
        }
        for (String packageRoot : requiredPackageRoots) {
            if (packageRoot == null || packageRoot.trim().isEmpty()
                    || !packageRoot.equals(packageRoot.trim())) {
                violations.add("host verification requires nonblank exact package roots");
            }
        }
        if (!violations.isEmpty()) {
            return Collections.unmodifiableList(violations);
        }

        Path repositoryRoot = findRepositoryRoot(startingPath);
        List<TypeDeclaration> declarations = discoverProductionTypes(
                repositoryRoot,
                requiredPackageRoots);
        Map<String, TypeDeclaration> topLevelDeclarations = declarationsByName(declarations);
        Map<String, Class<?>> topLevelTypes = loadProductionTypes(declarations);
        Map<String, TypeDeclaration> declarationsByName =
                new LinkedHashMap<String, TypeDeclaration>(topLevelDeclarations);
        Map<String, Class<?>> typesByName = collectInspectableTypes(
                topLevelTypes,
                topLevelDeclarations,
                declarationsByName,
                requiredPackageRoots,
                violations
        );

        validateExemptionScope(requiredExemptions, requiredPackageRoots, violations);
        validateCustomHostExemptions(requiredExemptions, typesByName, violations);
        if (requireApprovedRawRoots) {
            validateApprovedRawRoots(declarationsByName, typesByName, violations);
        }
        int opModeCount = validateOpModeTypes(
                typesByName,
                declarationsByName,
                requiredExemptions,
                violations
        );

        if (opModeCount == 0) {
            violations.add(PRODUCTION_JAVA_ROOT
                    + ": discovery found no FTC OpMode subtype; the production scan or classpath "
                    + "is not exercising the intended namespace");
        }
        validateNoTypedRobotProgramExposure(
                typesByName,
                declarationsByName,
                violations
        );
        return Collections.unmodifiableList(violations);
    }

    @Test
    public void discoversMultiplePackagePrivateTopLevelTypesButNotLookalikes() {
        String source = lines(
                "package edu.ftcsushi.fixture;",
                "",
                "// class LineCommentLookalike {}",
                "/* interface BlockCommentLookalike {} */",
                "@SuppressWarnings({\"enum StringLookalike {}\", \"class AlsoAString {}\"})",
                "public final class PublicType {",
                "    char brace = '}';",
                "    final class NestedType {}",
                "    interface NestedInterface {}",
                "}",
                "final class PackagePrivateType {}",
                "interface PackagePrivateInterface {}",
                "enum PackagePrivateEnum { VALUE }",
                "@interface PackagePrivateAnnotation {}"
        );

        ParsedSource parsed = parseSource("fixtures/SeveralTypes.java", source);

        assertEquals("edu.ftcsushi.fixture", parsed.packageName);
        assertEquals(
                new LinkedHashSet<String>(Arrays.asList(
                        "edu.ftcsushi.fixture.PublicType",
                        "edu.ftcsushi.fixture.PackagePrivateType",
                        "edu.ftcsushi.fixture.PackagePrivateInterface",
                        "edu.ftcsushi.fixture.PackagePrivateEnum",
                        "edu.ftcsushi.fixture.PackagePrivateAnnotation"
                )),
                declarationNames(parsed.declarations)
        );

        try {
            parseSource(
                    "fixtures/Unterminated.java",
                    lines("package edu.ftcsushi.fixture;", "/* never closed")
            );
            fail("Expected an actionable lexical discovery failure");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "fixtures/Unterminated.java:2"));
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "unterminated block comment"));
        }
    }

    @Test
    public void loadsWithoutInitializationAndReportsMissingOrBrokenTypes() {
        initializationProbeRan = false;
        TypeDeclaration probe = new TypeDeclaration(
                InitializationProbe.class.getName(),
                "fixtures/InitializationProbe.java",
                7
        );

        Class<?> loaded = loadWithoutInitialization(
                probe,
                ModernFtcHostBoundaryTest.class.getClassLoader()
        );

        assertSame(InitializationProbe.class, loaded);
        assertFalse("Class.forName must not run production static initialization",
                initializationProbeRan);

        assertLoadFailureContains(
                new TypeDeclaration(
                        "edu.ftcsushi.fixture.DoesNotExist",
                        "fixtures/Missing.java",
                        11
                ),
                ModernFtcHostBoundaryTest.class.getClassLoader(),
                "ClassNotFoundException"
        );

        final String brokenName = "edu.ftcsushi.fixture.BrokenLink";
        ClassLoader brokenLoader = new ClassLoader(
                ModernFtcHostBoundaryTest.class.getClassLoader()) {
            @Override
            protected Class<?> loadClass(String name, boolean resolve)
                    throws ClassNotFoundException {
                if (brokenName.equals(name)) {
                    throw new NoClassDefFoundError("fixture dependency is unavailable");
                }
                return super.loadClass(name, resolve);
            }
        };
        assertLoadFailureContains(
                new TypeDeclaration(brokenName, "fixtures/BrokenLink.java", 19),
                brokenLoader,
                "NoClassDefFoundError"
        );
    }

    @Test
    public void customHostExemptionsRejectBlankStaleAndAlreadyApprovedEntries() {
        Map<String, Class<?>> discovered = new LinkedHashMap<String, Class<?>>();
        discovered.put(FtcRobotOpMode.class.getName(), FtcRobotOpMode.class);
        discovered.put(UnapprovedHostFixture.class.getName(), UnapprovedHostFixture.class);

        Map<String, String> invalid = new LinkedHashMap<String, String>();
        invalid.put("   ", "a name is still required");
        invalid.put("edu.ftcsushi.fixture.StaleHost", "no discovered type has this name");
        invalid.put(FtcRobotOpMode.class.getName(), "the managed family already covers this");
        invalid.put(UnapprovedHostFixture.class.getName(), "   ");

        List<String> violations = new ArrayList<String>();
        validateCustomHostExemptions(invalid, discovered, violations);

        assertEquals(4, violations.size());
        assertContains(violations, "nonblank exact type name");
        assertContains(violations, "stale");
        assertContains(violations, "already belongs to an approved host family");
        assertContains(violations, "nonblank rationale");

        Map<String, String> outOfScope = Collections.singletonMap(
                "edu.ftcsushi.robots.application.RawHost",
                "fixture rationale");
        List<String> scopeViolations = new ArrayList<String>();
        validateExemptionScope(
                outOfScope,
                Collections.singleton("edu.ftcsushi.fw"),
                scopeViolations);
        assertEquals(1, scopeViolations.size());
        assertContains(scopeViolations, "outside the verified package scope");
    }

    @Test
    public void offPathSushiDeclarationCannotBypassNamespaceDiscovery() {
        Map<String, TypeDeclaration> declarations =
                new LinkedHashMap<String, TypeDeclaration>();
        List<String> failures = new ArrayList<String>();

        collectProductionSource(
                "misplaced/HiddenHost.java",
                lines(
                        "package edu.ftcsushi.robots.hidden;",
                        "public abstract class HiddenHost extends OpMode {}"
                ),
                Collections.singleton("edu.ftcsushi.robots"),
                declarations,
                failures
        );

        assertTrue(declarations.isEmpty());
        assertContains(failures, "misplaced/HiddenHost.java");
        assertContains(failures, "does not match production path package misplaced");
    }

    @Test
    public void reachableNestedRawOpModeIsInspectedAndExactNameCanBeReviewed() {
        Map<String, Class<?>> topLevelTypes = new LinkedHashMap<String, Class<?>>();
        topLevelTypes.put(
                ModernFtcHostBoundaryTest.class.getName(),
                ModernFtcHostBoundaryTest.class
        );
        Map<String, TypeDeclaration> declarations =
                new LinkedHashMap<String, TypeDeclaration>();
        declarations.put(
                ModernFtcHostBoundaryTest.class.getName(),
                new TypeDeclaration(
                        ModernFtcHostBoundaryTest.class.getName(),
                        "fixtures/NestedFactory.java",
                        3
                )
        );
        List<String> violations = new ArrayList<String>();
        Map<String, TypeDeclaration> inspectableDeclarations =
                new LinkedHashMap<String, TypeDeclaration>(declarations);
        Map<String, Class<?>> inspectableTypes = collectInspectableTypes(
                topLevelTypes,
                declarations,
                inspectableDeclarations,
                FIXTURE_PACKAGE_ROOTS,
                violations
        );

        validateOpModeTypes(
                inspectableTypes,
                inspectableDeclarations,
                Collections.<String, String>emptyMap(),
                violations
        );

        assertTrue(inspectableTypes.containsKey(PublicNestedRawOpModeFixture.class.getName()));
        assertContains(violations, PublicNestedRawOpModeFixture.class.getName());
        assertContains(violations, "directly extends raw FTC OpMode");
        String inheritedRawName =
                SkippedPackagePrivateBaseFixture.InheritedPublicRawOpModeFixture.class.getName();
        assertTrue(inspectableTypes.containsKey(inheritedRawName));
        assertContains(violations, inheritedRawName);
        assertTrue(inheritedRawName, inheritedRawName.contains(
                "$SkippedPackagePrivateBaseFixture$"));

        Map<String, String> exactNestedExemption = new LinkedHashMap<String, String>();
        exactNestedExemption.put(
                PublicNestedRawOpModeFixture.class.getName(),
                "fixture proves exact binary-name exemption lookup"
        );
        List<String> exemptionViolations = new ArrayList<String>();
        validateCustomHostExemptions(
                exactNestedExemption,
                inspectableTypes,
                exemptionViolations
        );
        assertTrue(exemptionViolations.toString(), exemptionViolations.isEmpty());
        List<String> reviewedViolations = new ArrayList<String>();
        validateOpModeTypes(
                inspectableTypes,
                inspectableDeclarations,
                exactNestedExemption,
                reviewedViolations);
        for (String violation : reviewedViolations) {
            assertFalse(violation,
                    violation.contains(PublicNestedRawOpModeFixture.class.getName()));
        }
        assertTrue(PublicNestedRawOpModeFixture.class.getName().contains("$"));
    }

    @Test
    public void inheritedMemberDiscoveryDoesNotCrossCallerPackageScope() {
        Map<String, Class<?>> topLevelTypes = new LinkedHashMap<String, Class<?>>();
        topLevelTypes.put(
                CrossScopeInheritanceEntryFixture.class.getName(),
                CrossScopeInheritanceEntryFixture.class
        );
        TypeDeclaration declaration = new TypeDeclaration(
                CrossScopeInheritanceEntryFixture.class.getName(),
                "fixtures/CrossScopeInheritanceEntry.java",
                3
        );
        Map<String, TypeDeclaration> declarations =
                new LinkedHashMap<String, TypeDeclaration>();
        declarations.put(CrossScopeInheritanceEntryFixture.class.getName(), declaration);
        Map<String, TypeDeclaration> inspectableDeclarations =
                new LinkedHashMap<String, TypeDeclaration>(declarations);
        List<String> violations = new ArrayList<String>();

        Map<String, Class<?>> inspectableTypes = collectInspectableTypes(
                topLevelTypes,
                declarations,
                inspectableDeclarations,
                FIXTURE_PACKAGE_ROOTS,
                violations
        );

        assertTrue(violations.toString(), violations.isEmpty());
        assertTrue(inspectableTypes.containsKey(CrossScopeInheritanceEntryFixture.class.getName()));
        assertFalse(inspectableTypes.containsKey(
                CrossScopeRawMemberOwner.InheritedPublicRawOpModeFixture.class.getName()));
        assertFalse(inspectableDeclarations.containsKey(
                CrossScopeRawMemberOwner.InheritedPublicRawOpModeFixture.class.getName()));
    }

    @Test
    public void exposedNestedRobotProgramTypesAreRejected() {
        Map<String, Class<?>> topLevelTypes = new LinkedHashMap<String, Class<?>>();
        topLevelTypes.put(
                ModernFtcHostBoundaryTest.class.getName(),
                ModernFtcHostBoundaryTest.class
        );
        Map<String, TypeDeclaration> declarations =
                new LinkedHashMap<String, TypeDeclaration>();
        declarations.put(
                ModernFtcHostBoundaryTest.class.getName(),
                new TypeDeclaration(
                        ModernFtcHostBoundaryTest.class.getName(),
                        "fixtures/NestedExposure.java",
                        3
                )
        );
        List<String> violations = new ArrayList<String>();
        Map<String, TypeDeclaration> inspectableDeclarations =
                new LinkedHashMap<String, TypeDeclaration>(declarations);
        Map<String, Class<?>> inspectableTypes = collectInspectableTypes(
                topLevelTypes,
                declarations,
                inspectableDeclarations,
                FIXTURE_PACKAGE_ROOTS,
                violations
        );

        validateNoTypedRobotProgramExposure(
                inspectableTypes,
                inspectableDeclarations,
                violations
        );

        assertContains(violations, PublicNestedFactoryFixture.class.getName());
        assertContains(violations, "createProgramSupplier");
        assertContains(violations, "exposedPrograms");
        assertContains(violations, PublicNestedProgramCarrierFixture.class.getName());
        assertContains(violations, "generic interface");
        assertContains(violations, PublicNestedOwnerExposureFixture.class.getName());
        assertContains(violations, "createOwnedProgramView");
        assertContains(violations, "ownedProgramView");
    }

    private static Map<String, Class<?>> collectInspectableTypes(
            Map<String, Class<?>> topLevelTypes,
            Map<String, TypeDeclaration> topLevelDeclarations,
            Map<String, TypeDeclaration> declarationsByName,
            Set<String> packageRoots,
            List<String> violations) {
        Map<String, Class<?>> inspectable =
                new LinkedHashMap<String, Class<?>>(topLevelTypes);
        Set<String> inspectedTypeNames = new LinkedHashSet<String>();
        for (Class<?> topLevelType : topLevelTypes.values()) {
            if (!Modifier.isPublic(topLevelType.getModifiers())) {
                continue;
            }
            collectReachableMemberTypes(
                    topLevelType,
                    topLevelDeclarations.get(topLevelType.getName()),
                    inspectable,
                    declarationsByName,
                    inspectedTypeNames,
                    packageRoots,
                    violations
            );
        }
        return inspectable;
    }

    private static void collectReachableMemberTypes(
            Class<?> owner,
            TypeDeclaration topLevelDeclaration,
            Map<String, Class<?>> inspectable,
            Map<String, TypeDeclaration> declarationsByName,
            Set<String> inspectedTypeNames,
            Set<String> packageRoots,
            List<String> violations) {
        if (!inspectedTypeNames.add(owner.getName())) {
            return;
        }

        Map<String, Class<?>> memberTypes = new LinkedHashMap<String, Class<?>>();
        try {
            addVisibleScopedMemberTypes(memberTypes, owner.getDeclaredClasses(), packageRoots);
        } catch (LinkageError error) {
            violations.add(locationOf(topLevelDeclaration)
                    + ": could not inspect declared member types of " + owner.getName() + ": "
                    + describeFailure(error));
        } catch (SecurityException error) {
            violations.add(locationOf(topLevelDeclaration)
                    + ": could not inspect declared member types of " + owner.getName() + ": "
                    + describeFailure(error));
        }

        try {
            addVisibleScopedMemberTypes(memberTypes, owner.getClasses(), packageRoots);
        } catch (LinkageError error) {
            violations.add(locationOf(topLevelDeclaration)
                    + ": could not inspect inherited public member types of " + owner.getName()
                    + ": " + describeFailure(error));
        } catch (SecurityException error) {
            violations.add(locationOf(topLevelDeclaration)
                    + ": could not inspect inherited public member types of " + owner.getName()
                    + ": " + describeFailure(error));
        }

        Class<?> superclass = owner.getSuperclass();
        while (superclass != null) {
            try {
                addVisibleScopedMemberTypes(
                        memberTypes,
                        superclass.getDeclaredClasses(),
                        packageRoots);
            } catch (LinkageError error) {
                violations.add(locationOf(topLevelDeclaration)
                        + ": could not inspect inherited protected member types declared by "
                        + superclass.getName() + ": " + describeFailure(error));
            } catch (SecurityException error) {
                violations.add(locationOf(topLevelDeclaration)
                        + ": could not inspect inherited protected member types declared by "
                        + superclass.getName() + ": " + describeFailure(error));
            }
            superclass = superclass.getSuperclass();
        }

        for (Class<?> memberType : memberTypes.values()) {
            TypeDeclaration memberDeclaration = new TypeDeclaration(
                    memberType.getName(),
                    topLevelDeclaration.sourcePath,
                    topLevelDeclaration.line
            );
            Class<?> loaded;
            try {
                loaded = loadWithoutInitialization(
                        memberDeclaration,
                        ModernFtcHostBoundaryTest.class.getClassLoader()
                );
            } catch (AssertionError loadFailure) {
                violations.add(loadFailure.getMessage());
                continue;
            }
            Class<?> previous = inspectable.put(memberType.getName(), loaded);
            if (previous != null && previous != loaded) {
                violations.add(locationOf(topLevelDeclaration)
                        + ": externally reachable member type " + memberType.getName()
                        + " resolved to two different runtime classes");
                continue;
            }
            declarationsByName.put(memberType.getName(), memberDeclaration);
            collectReachableMemberTypes(
                    loaded,
                    topLevelDeclaration,
                    inspectable,
                    declarationsByName,
                    inspectedTypeNames,
                    packageRoots,
                    violations
            );
        }
    }

    private static void addVisibleScopedMemberTypes(
            Map<String, Class<?>> destination,
            Class<?>[] candidates,
            Set<String> packageRoots) {
        for (Class<?> candidate : candidates) {
            int modifiers = candidate.getModifiers();
            if ((Modifier.isPublic(modifiers) || Modifier.isProtected(modifiers))
                    && isWithinAny(candidate.getName(), packageRoots)) {
                destination.put(candidate.getName(), candidate);
            }
        }
    }

    private static int validateOpModeTypes(
            Map<String, Class<?>> typesByName,
            Map<String, TypeDeclaration> declarationsByName,
            Map<String, String> exemptions,
            List<String> violations) {
        int opModeCount = 0;
        for (Class<?> type : typesByName.values()) {
            if (!OpMode.class.isAssignableFrom(type)) {
                continue;
            }
            opModeCount++;
            TypeDeclaration declaration = declarationsByName.get(type.getName());
            boolean approvedFamily = isApprovedHostFamily(type);
            boolean exactExemption = exemptions.containsKey(type.getName());
            if (!approvedFamily && !exactExemption) {
                violations.add(locationOf(declaration)
                        + ": FTC OpMode " + type.getName()
                        + " is outside the managed FtcRobotOpMode and tester families; add an "
                        + "exact custom-host exemption with a nonblank reviewed rationale only "
                        + "when the managed hosts cannot own its lifecycle");
            }
            if (type.getSuperclass() == OpMode.class
                    && !APPROVED_RAW_HOST_ROOTS.contains(type.getName())
                    && !exactExemption) {
                violations.add(locationOf(declaration)
                        + ": " + type.getName() + " directly extends raw FTC OpMode; only "
                        + APPROVED_RAW_HOST_ROOTS + " or an exact reasoned exemption may do so");
            }
            if (!exactExemption) {
                validateFinalCallbacks(type, declaration, violations);
            }
        }
        return opModeCount;
    }

    private static void validateApprovedRawRoots(
            Map<String, TypeDeclaration> declarationsByName,
            Map<String, Class<?>> typesByName,
            List<String> violations) {
        for (String approvedRoot : APPROVED_RAW_HOST_ROOTS) {
            TypeDeclaration declaration = declarationsByName.get(approvedRoot);
            Class<?> type = typesByName.get(approvedRoot);
            if (declaration == null || type == null) {
                violations.add(PRODUCTION_JAVA_ROOT + ": approved raw host root " + approvedRoot
                        + " was not discovered and loaded");
            } else if (type.getSuperclass() != OpMode.class) {
                violations.add(locationOf(declaration) + ": approved raw host root " + approvedRoot
                        + " must directly extend " + OpMode.class.getName());
            }
        }
    }

    private static void validateCustomHostExemptions(
            Map<String, String> exemptions,
            Map<String, Class<?>> typesByName,
            List<String> violations) {
        for (Map.Entry<String, String> exemption : exemptions.entrySet()) {
            String typeName = exemption.getKey();
            String rationale = exemption.getValue();
            if (typeName == null || typeName.trim().isEmpty()
                    || !typeName.equals(typeName.trim())) {
                violations.add("custom-host exemption requires a nonblank exact type name");
                continue;
            }
            if (rationale == null || rationale.trim().isEmpty()) {
                violations.add("custom-host exemption for " + typeName
                        + " requires a nonblank rationale");
            }

            Class<?> type = typesByName.get(typeName);
            if (type == null) {
                violations.add("custom-host exemption for " + typeName
                        + " is stale: no inspectable production type with that exact name was "
                        + "discovered");
                continue;
            }
            if (!OpMode.class.isAssignableFrom(type)) {
                violations.add("custom-host exemption for " + typeName
                        + " is stale: the discovered type is not an FTC OpMode");
            } else if (isApprovedHostFamily(type)) {
                violations.add("custom-host exemption for " + typeName
                        + " is redundant: it already belongs to an approved host family");
            }
        }
    }

    private static void validateExemptionScope(
            Map<String, String> exemptions,
            Set<String> packageRoots,
            List<String> violations) {
        for (String typeName : exemptions.keySet()) {
            if (typeName != null && !typeName.trim().isEmpty()
                    && !isWithinAny(typeName, packageRoots)) {
                violations.add("custom-host exemption for " + typeName
                        + " is outside the verified package scope " + packageRoots);
            }
        }
    }

    private static boolean isApprovedHostFamily(Class<?> type) {
        return FtcRobotOpMode.class.isAssignableFrom(type)
                || FtcTeleOpTesterOpMode.class.isAssignableFrom(type);
    }

    private static void validateFinalCallbacks(
            Class<?> type,
            TypeDeclaration declaration,
            List<String> violations) {
        for (String callbackName : FTC_CALLBACKS) {
            try {
                Method callback = type.getMethod(callbackName);
                if (!Modifier.isFinal(callback.getModifiers())) {
                    violations.add(locationOf(declaration) + ": " + type.getName() + " inherits "
                            + "or declares non-final FTC callback " + callbackName + "()");
                }
                if (callback.getReturnType() != void.class) {
                    violations.add(locationOf(declaration) + ": " + type.getName() + " callback "
                            + callbackName + "() must return void");
                }
            } catch (NoSuchMethodException missingCallback) {
                violations.add(locationOf(declaration) + ": " + type.getName()
                        + " has no public no-argument FTC callback " + callbackName + "()");
            } catch (LinkageError error) {
                violations.add(locationOf(declaration) + ": could not inspect callback "
                        + type.getName() + "#" + callbackName + "(): "
                        + describeFailure(error));
            } catch (SecurityException error) {
                violations.add(locationOf(declaration) + ": could not inspect callback "
                        + type.getName() + "#" + callbackName + "(): "
                        + describeFailure(error));
            }
        }
    }

    private static void validateRobotProgramBoundary(List<String> violations) {
        Constructor<?>[] constructors = RobotProgram.class.getDeclaredConstructors();
        if (constructors.length != 1) {
            violations.add(RobotProgram.class.getName() + " must retain exactly one framework-only "
                    + "constructor, but declares " + constructors.length);
        }
        try {
            Constructor<RobotProgram> constructor =
                    RobotProgram.class.getDeclaredConstructor(Telemetry.class);
            if (!isPackagePrivate(constructor.getModifiers())) {
                violations.add(RobotProgram.class.getName()
                        + "(Telemetry) must remain package-private");
            }
        } catch (NoSuchMethodException missingConstructor) {
            violations.add(RobotProgram.class.getName()
                    + " must retain its one package-private Telemetry constructor");
        }

        Map<String, MethodExpectation> declarations = declarationMethods();
        int publicMethodCount = 0;
        Set<String> publicMethodNames = new LinkedHashSet<String>();
        for (Method method : RobotProgram.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethodCount++;
                publicMethodNames.add(method.getName());
            }
        }
        if (publicMethodCount != declarations.size()
                || !publicMethodNames.equals(declarations.keySet())) {
            violations.add(RobotProgram.class.getName() + " must expose exactly the nine "
                    + "declaration methods " + declarations.keySet() + ", but found "
                    + publicMethodCount + " public declarations named " + publicMethodNames);
        }
        for (Map.Entry<String, MethodExpectation> expected : declarations.entrySet()) {
            validateMethodShape(
                    RobotProgram.class,
                    expected.getKey(),
                    expected.getValue(),
                    true,
                    violations
            );
        }

        Map<String, MethodExpectation> lifecycle = lifecycleMethods();
        for (Map.Entry<String, MethodExpectation> expected : lifecycle.entrySet()) {
            validateMethodShape(
                    RobotProgram.class,
                    expected.getKey(),
                    expected.getValue(),
                    false,
                    violations
            );
        }

        for (Method method : RobotProgram.class.getDeclaredMethods()) {
            if (lifecycle.containsKey(method.getName())
                    && (Modifier.isPublic(method.getModifiers())
                    || Modifier.isProtected(method.getModifiers()))) {
                violations.add(RobotProgram.class.getName() + " lifecycle method "
                        + method.getName() + " must not be public or protected");
            }
        }

        try {
            Method configure = FtcRobotOpMode.class.getDeclaredMethod(
                    "configure",
                    RobotProgram.class
            );
            if (!Modifier.isProtected(configure.getModifiers())
                    || !Modifier.isAbstract(configure.getModifiers())) {
                violations.add(FtcRobotOpMode.class.getName()
                        + "#configure(RobotProgram) must remain protected and abstract");
            }
        } catch (NoSuchMethodException missingConfigure) {
            violations.add(FtcRobotOpMode.class.getName()
                    + " must retain protected abstract configure(RobotProgram)");
        }
    }

    private static void validateMethodShape(
            Class<?> owner,
            String methodName,
            MethodExpectation expected,
            boolean mustBePublic,
            List<String> violations) {
        try {
            Method method = owner.getDeclaredMethod(methodName, expected.parameterTypes);
            if (method.getReturnType() != expected.returnType) {
                violations.add(owner.getName() + "#" + methodName + " has return type "
                        + method.getReturnType().getName() + "; expected "
                        + expected.returnType.getName());
            }
            if (mustBePublic) {
                if (!Modifier.isPublic(method.getModifiers())
                        || Modifier.isStatic(method.getModifiers())) {
                    violations.add(owner.getName() + "#" + methodName
                            + " must remain a public instance declaration method");
                }
            } else if (!isPackagePrivate(method.getModifiers())) {
                violations.add(owner.getName() + "#" + methodName
                        + " must remain package-private lifecycle machinery");
            }
        } catch (NoSuchMethodException missingMethod) {
            violations.add(owner.getName() + " must retain " + expected.returnType.getName() + " "
                    + methodName + parameterList(expected.parameterTypes));
        } catch (LinkageError error) {
            violations.add("Could not inspect " + owner.getName() + "#" + methodName + ": "
                    + describeFailure(error));
        }
    }

    private static Map<String, MethodExpectation> declarationMethods() {
        Map<String, MethodExpectation> methods =
                new LinkedHashMap<String, MethodExpectation>();
        methods.put("callbackBindings", methodReturning(CallbackBindings.class));
        methods.put("taskBindings", methodReturning(TaskBindings.class));
        methods.put("prestart", methodReturning(
                RobotProgram.Prestart.class,
                RobotProgram.Prestart.class
        ));
        methods.put("service", methodReturning(
                RobotProgram.Service.class,
                RobotProgram.Service.class
        ));
        methods.put("output", methodReturning(
                RobotProgram.Output.class,
                RobotProgram.Output.class
        ));
        methods.put("drive", methodReturning(
                DriveCommandSink.class,
                DriveSource.class,
                DriveCommandSink.class
        ));
        methods.put("rootTask", methodReturning(void.class, Task.class));
        methods.put("presenter", methodReturning(
                void.class,
                RobotProgram.Presenter.class
        ));
        methods.put("stopHandoff", methodReturning(
                void.class,
                Supplier.class,
                Consumer.class,
                Runnable.class
        ));
        return methods;
    }

    private static Map<String, MethodExpectation> lifecycleMethods() {
        Map<String, MethodExpectation> methods =
                new LinkedHashMap<String, MethodExpectation>();
        methods.put("beginInit", methodReturning(void.class, double.class));
        methods.put("finishConfiguration", methodReturning(void.class));
        methods.put("presentConfiguredInit", methodReturning(void.class));
        methods.put("initLoop", methodReturning(void.class, double.class));
        methods.put("start", methodReturning(void.class, double.class));
        methods.put("loop", methodReturning(void.class, double.class));
        methods.put("stop", methodReturning(void.class));
        methods.put("stopAfterFailure", methodReturning(
                RuntimeException.class,
                RuntimeException.class
        ));
        methods.put("isTerminal", methodReturning(boolean.class));
        return methods;
    }

    private static MethodExpectation methodReturning(
            Class<?> returnType,
            Class<?>... parameterTypes) {
        return new MethodExpectation(returnType, parameterTypes);
    }

    private static boolean isPackagePrivate(int modifiers) {
        return !Modifier.isPublic(modifiers)
                && !Modifier.isProtected(modifiers)
                && !Modifier.isPrivate(modifiers);
    }

    /**
     * Enforces explicit reflection-visible RobotProgram types. A semantically equivalent facade
     * with no typed RobotProgram exposure remains a design-review and documentation concern.
     */
    private static void validateNoTypedRobotProgramExposure(
            Map<String, Class<?>> typesByName,
            Map<String, TypeDeclaration> declarationsByName,
            List<String> violations) {
        for (Class<?> type : typesByName.values()) {
            TypeDeclaration declaration = declarationsByName.get(type.getName());
            validateRobotProgramExposureOn(type, declaration, violations);
        }
    }

    private static void validateRobotProgramExposureOn(
            Class<?> type,
            TypeDeclaration declaration,
            List<String> violations) {
        try {
            for (Method method : type.getDeclaredMethods()) {
                int modifiers = method.getModifiers();
                if (isPublicOrProtected(modifiers)
                        && containsRobotProgram(method.getGenericReturnType())) {
                    violations.add(locationOf(declaration) + ": "
                            + type.getName() + "#" + method.getName()
                            + " exposes RobotProgram through public/protected return type "
                            + method.getGenericReturnType().getTypeName()
                            + "; FtcRobotOpMode must remain the sole runtime owner");
                }
            }
            for (Field field : type.getDeclaredFields()) {
                int modifiers = field.getModifiers();
                if (isPublicOrProtected(modifiers)
                        && containsRobotProgram(field.getGenericType())) {
                    violations.add(locationOf(declaration) + ": "
                            + type.getName() + "#" + field.getName()
                            + " exposes RobotProgram through public/protected field type "
                            + field.getGenericType().getTypeName()
                            + "; FtcRobotOpMode must remain the sole runtime owner");
                }
            }

            Type genericSuperclass = type.getGenericSuperclass();
            if (genericSuperclass != null && containsRobotProgram(genericSuperclass)) {
                violations.add(locationOf(declaration) + ": " + type.getName()
                        + " exposes RobotProgram through reachable generic superclass "
                        + genericSuperclass.getTypeName());
            }
            for (Type genericInterface : type.getGenericInterfaces()) {
                if (containsRobotProgram(genericInterface)) {
                    violations.add(locationOf(declaration) + ": " + type.getName()
                            + " exposes RobotProgram through reachable generic interface "
                            + genericInterface.getTypeName());
                }
            }
            for (TypeVariable<?> typeParameter : type.getTypeParameters()) {
                if (containsRobotProgram(typeParameter)) {
                    violations.add(locationOf(declaration) + ": " + type.getName()
                            + " exposes RobotProgram through public type parameter "
                            + typeParameter.getTypeName());
                }
            }
        } catch (LinkageError error) {
            violations.add(locationOf(declaration)
                    + ": could not inspect RobotProgram exposure on " + type.getName() + ": "
                    + describeFailure(error));
        } catch (RuntimeException error) {
            violations.add(locationOf(declaration)
                    + ": could not inspect RobotProgram exposure on " + type.getName() + ": "
                    + describeFailure(error));
        }
    }

    private static boolean containsRobotProgram(Type type) {
        return containsRobotProgram(type, new LinkedHashSet<Type>());
    }

    private static boolean containsRobotProgram(Type type, Set<Type> visited) {
        if (type == RobotProgram.class) {
            return true;
        }
        if (type == null || !visited.add(type)) {
            return false;
        }
        if (type instanceof Class<?>) {
            Class<?> rawClass = (Class<?>) type;
            return rawClass.isArray()
                    && containsRobotProgram(rawClass.getComponentType(), visited);
        }
        if (type instanceof ParameterizedType) {
            ParameterizedType parameterized = (ParameterizedType) type;
            if (containsRobotProgram(parameterized.getOwnerType(), visited)
                    || containsRobotProgram(parameterized.getRawType(), visited)) {
                return true;
            }
            for (Type argument : parameterized.getActualTypeArguments()) {
                if (containsRobotProgram(argument, visited)) {
                    return true;
                }
            }
            return false;
        }
        if (type instanceof GenericArrayType) {
            return containsRobotProgram(
                    ((GenericArrayType) type).getGenericComponentType(),
                    visited
            );
        }
        if (type instanceof WildcardType) {
            WildcardType wildcard = (WildcardType) type;
            for (Type upperBound : wildcard.getUpperBounds()) {
                if (containsRobotProgram(upperBound, visited)) {
                    return true;
                }
            }
            for (Type lowerBound : wildcard.getLowerBounds()) {
                if (containsRobotProgram(lowerBound, visited)) {
                    return true;
                }
            }
            return false;
        }
        if (type instanceof TypeVariable<?>) {
            for (Type bound : ((TypeVariable<?>) type).getBounds()) {
                if (containsRobotProgram(bound, visited)) {
                    return true;
                }
            }
        }
        return false;
    }

    private static boolean isPublicOrProtected(int modifiers) {
        return Modifier.isPublic(modifiers) || Modifier.isProtected(modifiers);
    }

    private static Path findRepositoryRoot(Path startingPath) {
        Path candidate = startingPath.toAbsolutePath().normalize();
        while (candidate != null) {
            if (Files.isDirectory(candidate.resolve(PRODUCTION_JAVA_ROOT))) {
                return candidate;
            }
            candidate = candidate.getParent();
        }
        throw new AssertionError("Could not locate " + PRODUCTION_JAVA_ROOT
                + " from test working directory " + startingPath.toAbsolutePath().normalize());
    }

    private static List<TypeDeclaration> discoverProductionTypes(
            Path repositoryRoot,
            Set<String> packageRoots)
            throws IOException {
        final Path sourceRoot = repositoryRoot.resolve(PRODUCTION_JAVA_ROOT).normalize();
        final List<Path> javaSources = new ArrayList<Path>();
        Files.walkFileTree(sourceRoot, new SimpleFileVisitor<Path>() {
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attributes) {
                if (attributes.isRegularFile() && file.getFileName().toString().endsWith(".java")) {
                    javaSources.add(file);
                }
                return FileVisitResult.CONTINUE;
            }
        });
        Collections.sort(javaSources);

        List<String> failures = new ArrayList<String>();
        Map<String, TypeDeclaration> declarations =
                new LinkedHashMap<String, TypeDeclaration>();
        for (Path sourceFile : javaSources) {
            String relativePath = normalizedRelativePath(sourceRoot.relativize(sourceFile));
            String source = new String(
                    Files.readAllBytes(sourceFile),
                    StandardCharsets.UTF_8
            );
            collectProductionSource(
                    relativePath,
                    source,
                    packageRoots,
                    declarations,
                    failures);
        }

        if (javaSources.isEmpty()) {
            failures.add(sourceRoot + ": no production Java sources were found");
        }
        if (declarations.isEmpty()) {
            failures.add(sourceRoot + ": no top-level production types were discovered");
        }
        if (!failures.isEmpty()) {
            throw new AssertionError(formatFailures("Production type discovery failed", failures));
        }
        return new ArrayList<TypeDeclaration>(declarations.values());
    }

    private static void collectProductionSource(
            String mainSourceRelativePath,
            String source,
            Set<String> packageRoots,
            Map<String, TypeDeclaration> declarations,
            List<String> failures) {
        String normalizedPath = mainSourceRelativePath.replace('\\', '/');
        String displayPath = PRODUCTION_JAVA_ROOT + "/" + normalizedPath;
        boolean physicallyUnderScope = isUnderPackagePath(normalizedPath, packageRoots);
        ParsedSource parsed;
        try {
            parsed = parseSource(displayPath, source);
        } catch (IllegalArgumentException discoveryFailure) {
            if (physicallyUnderScope) {
                failures.add(discoveryFailure.getMessage());
            }
            return;
        }

        boolean declaresScope = isWithinAny(parsed.packageName, packageRoots);
        if (!physicallyUnderScope && !declaresScope) {
            return;
        }

        String expectedPackage = expectedPackage(Paths.get(normalizedPath));
        if (!expectedPackage.equals(parsed.packageName)) {
            failures.add(displayPath + ": declared package " + parsed.packageName
                    + " does not match production path package " + expectedPackage);
            return;
        }
        for (TypeDeclaration declaration : parsed.declarations) {
            TypeDeclaration previous = declarations.put(declaration.className, declaration);
            if (previous != null) {
                failures.add(locationOf(declaration) + ": duplicate top-level production type "
                        + declaration.className + "; first discovered at "
                        + locationOf(previous));
            }
        }
    }

    private static boolean isUnderPackagePath(String relativePath, Set<String> packageRoots) {
        for (String packageRoot : packageRoots) {
            String pathRoot = packageRoot.replace('.', '/') + "/";
            if (relativePath.startsWith(pathRoot)) {
                return true;
            }
        }
        return false;
    }

    private static boolean isWithinAny(String classOrPackageName, Set<String> packageRoots) {
        for (String packageRoot : packageRoots) {
            if (classOrPackageName.equals(packageRoot)
                    || classOrPackageName.startsWith(packageRoot + ".")) {
                return true;
            }
        }
        return false;
    }

    private static ParsedSource parseSource(String sourcePath, String source) {
        List<Token> tokens;
        try {
            tokens = tokenize(source);
        } catch (IllegalArgumentException lexicalFailure) {
            throw new IllegalArgumentException(
                    sourcePath + ":" + lexicalFailure.getMessage(),
                    lexicalFailure
            );
        }
        String packageName = findPackageName(sourcePath, tokens);
        List<TypeDeclaration> declarations = new ArrayList<TypeDeclaration>();
        int braceDepth = 0;
        for (int index = 0; index < tokens.size(); index++) {
            Token token = tokens.get(index);
            if ("{".equals(token.text)) {
                braceDepth++;
                continue;
            }
            if ("}".equals(token.text)) {
                braceDepth--;
                if (braceDepth < 0) {
                    throw new IllegalArgumentException(sourcePath + ":" + token.line
                            + ": unmatched closing brace while discovering top-level types");
                }
                continue;
            }
            if (braceDepth != 0 || !isTypeKeyword(token.text)) {
                continue;
            }
            if (index > 0 && ".".equals(tokens.get(index - 1).text)) {
                continue;
            }
            if (index + 1 >= tokens.size() || !tokens.get(index + 1).identifier) {
                continue;
            }

            Token name = tokens.get(index + 1);
            declarations.add(new TypeDeclaration(
                    packageName + "." + name.text,
                    sourcePath,
                    name.line
            ));
        }
        if (braceDepth != 0) {
            throw new IllegalArgumentException(sourcePath
                    + ": unmatched opening brace while discovering top-level types");
        }
        return new ParsedSource(packageName, declarations);
    }

    private static String findPackageName(String sourcePath, List<Token> tokens) {
        int braceDepth = 0;
        for (int index = 0; index < tokens.size(); index++) {
            Token token = tokens.get(index);
            if ("{".equals(token.text)) {
                braceDepth++;
            } else if ("}".equals(token.text)) {
                braceDepth--;
            } else if (braceDepth == 0 && "package".equals(token.text)) {
                StringBuilder packageName = new StringBuilder();
                boolean expectIdentifier = true;
                for (int part = index + 1; part < tokens.size(); part++) {
                    Token packageToken = tokens.get(part);
                    if (";".equals(packageToken.text)) {
                        if (packageName.length() == 0 || expectIdentifier) {
                            break;
                        }
                        return packageName.toString();
                    }
                    if (expectIdentifier && packageToken.identifier) {
                        packageName.append(packageToken.text);
                        expectIdentifier = false;
                    } else if (!expectIdentifier && ".".equals(packageToken.text)) {
                        packageName.append('.');
                        expectIdentifier = true;
                    } else {
                        break;
                    }
                }
                throw new IllegalArgumentException(sourcePath + ":" + token.line
                        + ": malformed package declaration");
            }
        }
        throw new IllegalArgumentException(sourcePath
                + ": no declared package was found while discovering production types");
    }

    private static List<Token> tokenize(String source) {
        List<Token> tokens = new ArrayList<Token>();
        int line = 1;
        int index = 0;
        while (index < source.length()) {
            char current = source.charAt(index);
            if (current == '\n') {
                line++;
                index++;
                continue;
            }
            if (Character.isWhitespace(current)) {
                index++;
                continue;
            }
            if (current == '/' && index + 1 < source.length()
                    && source.charAt(index + 1) == '/') {
                index += 2;
                while (index < source.length() && source.charAt(index) != '\n') {
                    index++;
                }
                continue;
            }
            if (current == '/' && index + 1 < source.length()
                    && source.charAt(index + 1) == '*') {
                int commentStartLine = line;
                index += 2;
                boolean closed = false;
                while (index < source.length()) {
                    if (source.charAt(index) == '\n') {
                        line++;
                    }
                    if (source.charAt(index) == '*' && index + 1 < source.length()
                            && source.charAt(index + 1) == '/') {
                        index += 2;
                        closed = true;
                        break;
                    }
                    index++;
                }
                if (!closed) {
                    throw new IllegalArgumentException(
                            commentStartLine + ": unterminated block comment while discovering "
                                    + "production types");
                }
                continue;
            }
            if (current == '"' || current == '\'') {
                int literalStartLine = line;
                char delimiter = current;
                boolean textBlock = delimiter == '"'
                        && index + 2 < source.length()
                        && source.charAt(index + 1) == '"'
                        && source.charAt(index + 2) == '"';
                index += textBlock ? 3 : 1;
                boolean closed = false;
                while (index < source.length()) {
                    if (source.charAt(index) == '\n') {
                        line++;
                    }
                    if (textBlock && index + 2 < source.length()
                            && source.charAt(index) == '"'
                            && source.charAt(index + 1) == '"'
                            && source.charAt(index + 2) == '"') {
                        index += 3;
                        closed = true;
                        break;
                    }
                    if (!textBlock && source.charAt(index) == delimiter) {
                        index++;
                        closed = true;
                        break;
                    }
                    if (source.charAt(index) == '\\' && index + 1 < source.length()) {
                        if (source.charAt(index + 1) == '\n') {
                            line++;
                        }
                        index += 2;
                    } else {
                        index++;
                    }
                }
                if (!closed) {
                    throw new IllegalArgumentException(
                            literalStartLine + ": unterminated literal while discovering "
                                    + "production types");
                }
                continue;
            }
            if (Character.isJavaIdentifierStart(current)) {
                int start = index;
                int tokenLine = line;
                index++;
                while (index < source.length()
                        && Character.isJavaIdentifierPart(source.charAt(index))) {
                    index++;
                }
                tokens.add(new Token(source.substring(start, index), tokenLine, true));
                continue;
            }

            tokens.add(new Token(String.valueOf(current), line, false));
            index++;
        }
        return tokens;
    }

    private static boolean isTypeKeyword(String token) {
        return "class".equals(token)
                || "interface".equals(token)
                || "enum".equals(token)
                || "record".equals(token);
    }

    private static String expectedPackage(Path relativeSourcePath) {
        Path parent = relativeSourcePath.getParent();
        if (parent == null) {
            return "";
        }
        return parent.toString().replace(File.separatorChar, '.');
    }

    private static String normalizedRelativePath(Path path) {
        return path.toString().replace(File.separatorChar, '/');
    }

    private static Map<String, TypeDeclaration> declarationsByName(
            List<TypeDeclaration> declarations) {
        Map<String, TypeDeclaration> byName = new LinkedHashMap<String, TypeDeclaration>();
        for (TypeDeclaration declaration : declarations) {
            byName.put(declaration.className, declaration);
        }
        return byName;
    }

    private static Set<String> declarationNames(List<TypeDeclaration> declarations) {
        Set<String> names = new LinkedHashSet<String>();
        for (TypeDeclaration declaration : declarations) {
            names.add(declaration.className);
        }
        return names;
    }

    private static Map<String, Class<?>> loadProductionTypes(
            List<TypeDeclaration> declarations) {
        Map<String, Class<?>> loaded = new LinkedHashMap<String, Class<?>>();
        List<String> failures = new ArrayList<String>();
        ClassLoader loader = ModernFtcHostBoundaryTest.class.getClassLoader();
        for (TypeDeclaration declaration : declarations) {
            try {
                loaded.put(
                        declaration.className,
                        loadWithoutInitialization(declaration, loader)
                );
            } catch (AssertionError loadFailure) {
                failures.add(loadFailure.getMessage());
            }
        }
        if (!failures.isEmpty()) {
            throw new AssertionError(formatFailures(
                    "Discovered production types could not be loaded",
                    failures
            ));
        }
        return loaded;
    }

    private static Class<?> loadWithoutInitialization(
            TypeDeclaration declaration,
            ClassLoader loader) {
        try {
            return Class.forName(declaration.className, false, loader);
        } catch (ClassNotFoundException error) {
            throw loadFailure(declaration, error);
        } catch (LinkageError error) {
            throw loadFailure(declaration, error);
        } catch (SecurityException error) {
            throw loadFailure(declaration, error);
        }
    }

    private static AssertionError loadFailure(
            TypeDeclaration declaration,
            Throwable cause) {
        AssertionError failure = new AssertionError(locationOf(declaration)
                + ": discovered top-level production type " + declaration.className
                + " could not be loaded without initialization ("
                + describeFailure(cause)
                + "); ensure the declaration, compiled class, and test runtime dependencies agree");
        failure.initCause(cause);
        return failure;
    }

    private static void assertLoadFailureContains(
            TypeDeclaration declaration,
            ClassLoader loader,
            String expectedFailureType) {
        try {
            loadWithoutInitialization(declaration, loader);
            fail("Expected an actionable load failure for " + declaration.className);
        } catch (AssertionError expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(declaration.className));
            assertTrue(expected.getMessage(), expected.getMessage().contains(declaration.sourcePath));
            assertTrue(expected.getMessage(), expected.getMessage().contains(expectedFailureType));
        }
    }

    private static String describeFailure(Throwable failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + (message == null || message.trim().isEmpty() ? "" : ": " + message);
    }

    private static String locationOf(TypeDeclaration declaration) {
        if (declaration == null) {
            return PRODUCTION_JAVA_ROOT;
        }
        return declaration.sourcePath + ":" + declaration.line;
    }

    private static String parameterList(Class<?>[] parameterTypes) {
        StringBuilder result = new StringBuilder("(");
        for (int index = 0; index < parameterTypes.length; index++) {
            if (index > 0) {
                result.append(", ");
            }
            result.append(parameterTypes[index].getName());
        }
        return result.append(')').toString();
    }

    private static void assertNoViolations(List<String> violations) {
        if (!violations.isEmpty()) {
            fail(formatFailures("Modern FTC host boundary violations", violations));
        }
    }

    private static String formatFailures(String heading, List<String> failures) {
        StringBuilder message = new StringBuilder(heading);
        for (String failure : failures) {
            message.append(System.lineSeparator()).append(" - ").append(failure);
        }
        return message.toString();
    }

    private static void assertContains(List<String> values, String expectedPart) {
        for (String value : values) {
            if (value.contains(expectedPart)) {
                return;
            }
        }
        fail("Expected one of " + values + " to contain " + expectedPart);
    }

    private static String lines(String... lines) {
        StringBuilder source = new StringBuilder();
        for (String line : lines) {
            source.append(line).append('\n');
        }
        return source.toString();
    }

    private static final class MethodExpectation {
        private final Class<?> returnType;
        private final Class<?>[] parameterTypes;

        private MethodExpectation(Class<?> returnType, Class<?>[] parameterTypes) {
            this.returnType = returnType;
            this.parameterTypes = parameterTypes.clone();
        }
    }

    private static final class ParsedSource {
        private final String packageName;
        private final List<TypeDeclaration> declarations;

        private ParsedSource(String packageName, List<TypeDeclaration> declarations) {
            this.packageName = packageName;
            this.declarations = declarations;
        }
    }

    private static final class TypeDeclaration {
        private final String className;
        private final String sourcePath;
        private final int line;

        private TypeDeclaration(String className, String sourcePath, int line) {
            this.className = className;
            this.sourcePath = sourcePath;
            this.line = line;
        }
    }

    private static final class Token {
        private final String text;
        private final int line;
        private final boolean identifier;

        private Token(String text, int line, boolean identifier) {
            this.text = text;
            this.line = line;
            this.identifier = identifier;
        }
    }

    private static final class InitializationProbe {
        static {
            initializationProbeRan = true;
        }
    }

    public static final class PublicNestedFactoryFixture {
        public RobotProgram[] exposedPrograms;

        public static RobotProgram createProgram() {
            return null;
        }

        public Supplier<RobotProgram> createProgramSupplier() {
            return null;
        }
    }

    public interface PublicNestedProgramCarrierFixture extends Supplier<RobotProgram> {
    }

    public static final class GenericOuterFixture<T> {
        public final class Inner<U> {
        }
    }

    public static final class PublicNestedOwnerExposureFixture {
        public GenericOuterFixture<RobotProgram>.Inner<String> ownedProgramView;

        public GenericOuterFixture<RobotProgram>.Inner<String> createOwnedProgramView() {
            return null;
        }
    }

    public abstract static class PublicNestedRawOpModeFixture extends OpMode {
    }

    static class SkippedPackagePrivateBaseFixture {
        public abstract static class InheritedPublicRawOpModeFixture extends OpMode {
        }
    }

    public static final class PublicInheritedMemberEntryFixture
            extends SkippedPackagePrivateBaseFixture {
    }

    public static final class CrossScopeInheritanceEntryFixture
            extends CrossScopeRawMemberOwner {
    }

    private abstract static class UnapprovedHostFixture extends OpMode {
    }
}
