package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.AprilTagPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.localization.AprilTagLocalizationTester;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Locks CONFIG-06's Camera and April-only tester configuration ownership boundaries. */
public final class CameraAprilTesterConfigurationTest {

    @Test
    public void cameraOwnerHasOneConstructorAndDefaultsOnlyFiveFieldConfig() throws Exception {
        assertOwnerConstructor(CameraMountCalibrator.class, CameraMountCalibrator.Config.class);
        assertConfigShape(
                CameraMountCalibrator.Config.class,
                Arrays.asList(
                        "preferredVisionDeviceName",
                        "visionDeviceType",
                        "visionPickerTitle",
                        "fixedTagLayout",
                        "maxDetectionAgeSec"
                ),
                Arrays.<Class<?>>asList(
                        String.class,
                        Class.class,
                        String.class,
                        TagLayout.class,
                        double.class
                )
        );
    }

    @Test
    public void aprilOwnerHasOneConstructorAndDefaultsOnlyFiveFieldConfig() throws Exception {
        assertOwnerConstructor(
                AprilTagLocalizationTester.class,
                AprilTagLocalizationTester.Config.class
        );
        assertConfigShape(
                AprilTagLocalizationTester.Config.class,
                Arrays.asList(
                        "preferredVisionDeviceName",
                        "visionDeviceType",
                        "visionPickerTitle",
                        "fixedTagLayout",
                        "aprilTags"
                ),
                Arrays.<Class<?>>asList(
                        String.class,
                        Class.class,
                        String.class,
                        TagLayout.class,
                        AprilTagLocalizationConfig.class
                )
        );
    }

    @Test
    public void defaultsAreFreshSoftwareBaselines() {
        CameraMountCalibrator.Config firstCamera = CameraMountCalibrator.Config.defaults();
        CameraMountCalibrator.Config secondCamera = CameraMountCalibrator.Config.defaults();
        assertNotSame(firstCamera, secondCamera);
        assertSame(WebcamName.class, firstCamera.visionDeviceType);
        assertEquals("Select Camera", firstCamera.visionPickerTitle);
        assertSame(null, firstCamera.preferredVisionDeviceName);
        assertNotNull(firstCamera.fixedTagLayout);
        assertEquals(0.35, firstCamera.maxDetectionAgeSec, 0.0);

        AprilTagLocalizationTester.Config firstApril =
                AprilTagLocalizationTester.Config.defaults();
        AprilTagLocalizationTester.Config secondApril =
                AprilTagLocalizationTester.Config.defaults();
        assertNotSame(firstApril, secondApril);
        assertNotSame(firstApril.aprilTags, secondApril.aprilTags);
        assertNotSame(firstApril.aprilTags.fieldPoseSolver, secondApril.aprilTags.fieldPoseSolver);
        assertSame(WebcamName.class, firstApril.visionDeviceType);
        assertEquals("Select Camera", firstApril.visionPickerTitle);
        assertSame(null, firstApril.preferredVisionDeviceName);
        assertNotNull(firstApril.fixedTagLayout);
        assertEquals(0.35, firstApril.aprilTags.maxDetectionAgeSec, 0.0);
    }

    @Test
    public void preferredNameIsNormalizedAndBuilderIsCapturedExactlyOnceWithoutOpening() {
        final int[] cameraApplies = {0};
        final int[] cameraOpens = {0};
        final String[] cameraName = {null};
        CameraMountCalibrator.Config camera = CameraMountCalibrator.Config.defaults();
        camera.preferredVisionDeviceName = "  frontCam  ";
        camera.fixedTagLayout = new SimpleTagLayout();
        new CameraMountCalibrator(camera, name -> {
            cameraApplies[0]++;
            cameraName[0] = name;
            return hardwareMap -> {
                cameraOpens[0]++;
                return null;
            };
        });
        assertEquals(1, cameraApplies[0]);
        assertEquals("frontCam", cameraName[0]);
        assertEquals(0, cameraOpens[0]);

        final int[] aprilApplies = {0};
        final int[] aprilOpens = {0};
        final String[] aprilName = {null};
        AprilTagLocalizationTester.Config april = AprilTagLocalizationTester.Config.defaults();
        april.preferredVisionDeviceName = "  limelight  ";
        april.fixedTagLayout = new SimpleTagLayout();
        new AprilTagLocalizationTester(april, name -> {
            aprilApplies[0]++;
            aprilName[0] = name;
            return hardwareMap -> {
                aprilOpens[0]++;
                return null;
            };
        });
        assertEquals(1, aprilApplies[0]);
        assertEquals("limelight", aprilName[0]);
        assertEquals(0, aprilOpens[0]);
    }

    @Test
    public void pickerPathDoesNotApplyBuilderUntilADeviceIsChosen() {
        final int[] applies = {0};
        Function<String, AprilTagVisionLaneFactory> builder = name -> {
            applies[0]++;
            return hardwareMap -> null;
        };

        CameraMountCalibrator.Config camera = CameraMountCalibrator.Config.defaults();
        camera.fixedTagLayout = new SimpleTagLayout();
        new CameraMountCalibrator(camera, builder);

        AprilTagLocalizationTester.Config april = AprilTagLocalizationTester.Config.defaults();
        april.fixedTagLayout = new SimpleTagLayout();
        new AprilTagLocalizationTester(april, builder);

        assertEquals(0, applies[0]);
    }

    @Test
    public void preferredBuilderMustReturnAFactory() {
        CameraMountCalibrator.Config camera = CameraMountCalibrator.Config.defaults();
        camera.preferredVisionDeviceName = "vision";
        assertMessageContains(
                () -> new CameraMountCalibrator(camera, name -> null),
                "visionLaneFactoryBuilder returned null for vision"
        );

        AprilTagLocalizationTester.Config april = AprilTagLocalizationTester.Config.defaults();
        april.preferredVisionDeviceName = "vision";
        assertMessageContains(
                () -> new AprilTagLocalizationTester(april, name -> null),
                "visionLaneFactoryBuilder returned null for vision"
        );
    }

    @Test
    public void currentGamePolicySummaryIsRetainedBesideTheImmutableSnapshot() throws Exception {
        CameraMountCalibrator.Config camera = CameraMountCalibrator.Config.defaults();
        String cameraSummary = ((FtcGameTagLayout) camera.fixedTagLayout).policySummaryLine();
        CameraMountCalibrator cameraOwner = new CameraMountCalibrator(
                camera,
                name -> hardwareMap -> null
        );
        assertEquals(cameraSummary, field(cameraOwner, "layoutPolicySummary"));
        assertFalse(field(cameraOwner, "layout") instanceof FtcGameTagLayout);

        AprilTagLocalizationTester.Config april = AprilTagLocalizationTester.Config.defaults();
        String aprilSummary = ((FtcGameTagLayout) april.fixedTagLayout).policySummaryLine();
        AprilTagLocalizationTester aprilOwner = new AprilTagLocalizationTester(
                april,
                name -> hardwareMap -> null
        );
        assertEquals(aprilSummary, field(aprilOwner, "layoutPolicySummary"));
        assertFalse(field(aprilOwner, "layout") instanceof FtcGameTagLayout);
    }

    @Test
    public void ownersSnapshotLayoutAndAprilPolicyBeforeCallerMutation() throws Exception {
        Pose3d firstPose = new Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        SimpleTagLayout cameraSource = new SimpleTagLayout().addPose(1, firstPose);
        CameraMountCalibrator.Config camera = CameraMountCalibrator.Config.defaults();
        camera.preferredVisionDeviceName = null;
        camera.visionDeviceType = HardwareDevice.class;
        camera.visionPickerTitle = "  Pick Vision  ";
        camera.fixedTagLayout = cameraSource;
        camera.maxDetectionAgeSec = 0.0;
        CameraMountCalibrator cameraOwner = new CameraMountCalibrator(
                camera,
                name -> hardwareMap -> null
        );

        cameraSource.clear();
        cameraSource.addPose(2, Pose3d.zero());
        camera.visionDeviceType = WebcamName.class;
        camera.visionPickerTitle = "changed";
        camera.maxDetectionAgeSec = 9.0;

        TagLayout cameraSnapshot = (TagLayout) field(cameraOwner, "layout");
        assertTrue(cameraSnapshot.has(1));
        assertFalse(cameraSnapshot.has(2));
        assertSame(firstPose, cameraSnapshot.requireFieldToTagPose(1));
        assertSame(HardwareDevice.class, field(cameraOwner, "visionDeviceType"));
        assertEquals("Pick Vision", field(cameraOwner, "visionPickerTitle"));
        assertEquals(0.0, (Double) field(cameraOwner, "maxDetectionAgeSec"), 0.0);

        SimpleTagLayout aprilSource = new SimpleTagLayout().addPose(7, firstPose);
        AprilTagLocalizationTester.Config april = AprilTagLocalizationTester.Config.defaults();
        april.visionDeviceType = HardwareDevice.class;
        april.visionPickerTitle = "  April Vision  ";
        april.fixedTagLayout = aprilSource;
        april.aprilTags.maxDetectionAgeSec = 0.0;
        april.aprilTags.fieldPoseSolver.maxAbsBearingRad = 0.75;
        april.aprilTags.fieldPoseSolver.preferObservationFieldPose = false;
        AprilTagLocalizationTester aprilOwner = new AprilTagLocalizationTester(
                april,
                name -> hardwareMap -> null
        );

        aprilSource.clear();
        aprilSource.addPose(8, Pose3d.zero());
        april.visionDeviceType = WebcamName.class;
        april.visionPickerTitle = "changed";
        april.aprilTags.maxDetectionAgeSec = 5.0;
        april.aprilTags.fieldPoseSolver.maxAbsBearingRad = 0.0;
        april.aprilTags.fieldPoseSolver.preferObservationFieldPose = true;

        TagLayout aprilSnapshot = (TagLayout) field(aprilOwner, "layout");
        assertTrue(aprilSnapshot.has(7));
        assertFalse(aprilSnapshot.has(8));
        assertSame(HardwareDevice.class, field(aprilOwner, "visionDeviceType"));
        assertEquals("April Vision", field(aprilOwner, "visionPickerTitle"));
        AprilTagLocalizationConfig capturedPolicy =
                (AprilTagLocalizationConfig) field(aprilOwner, "aprilTags");
        assertNotSame(april.aprilTags, capturedPolicy);
        assertNotSame(april.aprilTags.fieldPoseSolver, capturedPolicy.fieldPoseSolver);
        assertEquals(0.0, capturedPolicy.maxDetectionAgeSec, 0.0);
        assertEquals(0.75, capturedPolicy.fieldPoseSolver.maxAbsBearingRad, 0.0);
        assertFalse(capturedPolicy.fieldPoseSolver.preferObservationFieldPose);
    }

    @Test
    public void eachOwnerReadsTheAuthoredLayoutOnce() {
        CountingLayout cameraLayout = new CountingLayout();
        CameraMountCalibrator.Config camera = CameraMountCalibrator.Config.defaults();
        camera.fixedTagLayout = cameraLayout;
        new CameraMountCalibrator(camera, name -> hardwareMap -> null);
        assertEquals(1, cameraLayout.idsReads);
        assertEquals(1, cameraLayout.poseReads);

        CountingLayout aprilLayout = new CountingLayout();
        AprilTagLocalizationTester.Config april = AprilTagLocalizationTester.Config.defaults();
        april.fixedTagLayout = aprilLayout;
        new AprilTagLocalizationTester(april, name -> hardwareMap -> null);
        assertEquals(1, aprilLayout.idsReads);
        assertEquals(1, aprilLayout.poseReads);
    }

    @Test
    public void emptyLayoutsKeepRawDetectionsWithoutInventingFieldTruth() throws Exception {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        Pose3d cameraToTag = new Pose3d(24.0, 1.0, 2.0, 0.1, 0.0, 0.0);
        AprilTagDetections frame = AprilTagDetections.fromFrame(
                clock.nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(1, cameraToTag))
        );
        AprilTagSensor sensor = ignoredClock -> frame;

        CameraMountCalibrator.Config camera = CameraMountCalibrator.Config.defaults();
        camera.preferredVisionDeviceName = "vision";
        camera.fixedTagLayout = new SimpleTagLayout();
        camera.maxDetectionAgeSec = 0.0;
        CameraMountCalibrator cameraOwner = new CameraMountCalibrator(
                camera,
                name -> hardwareMap -> new ReadyLane(sensor)
        );
        cameraOwner.init(context(clock));
        cameraOwner.initLoop(0.0);
        assertSame(cameraToTag, field(cameraOwner, "lastObservedCameraToTag"));
        assertSame(null, field(cameraOwner, "lastRobotToCameraSample"));
        cameraOwner.stop();

        AprilTagLocalizationTester.Config april = AprilTagLocalizationTester.Config.defaults();
        april.preferredVisionDeviceName = "vision";
        april.fixedTagLayout = new SimpleTagLayout();
        april.aprilTags.maxDetectionAgeSec = 0.0;
        AprilTagLocalizationTester aprilOwner = new AprilTagLocalizationTester(
                april,
                name -> hardwareMap -> new ReadyLane(sensor)
        );
        aprilOwner.init(context(clock));
        aprilOwner.initLoop(0.0);

        TagSelectionSource selection = (TagSelectionSource) field(aprilOwner, "selection");
        TagSelectionResult selected = selection.get(clock);
        assertTrue(selected.hasFreshSelectedObservation);
        assertEquals(1, selected.selectedObservation.id);
        AprilTagPoseEstimator estimator =
                (AprilTagPoseEstimator) field(aprilOwner, "poseEstimator");
        PoseEstimate estimate = estimator.getEstimate();
        assertFalse(estimate.hasPose);
        aprilOwner.stop();
    }

    @Test
    public void invalidCameraDataFailsBeforePreferredBuilderApplication() {
        double[] invalidAges = {-1.0, Double.NaN, Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY};
        for (double age : invalidAges) {
            final int[] applies = {0};
            CameraMountCalibrator.Config config = CameraMountCalibrator.Config.defaults();
            config.preferredVisionDeviceName = "vision";
            config.maxDetectionAgeSec = age;
            RuntimeException failure = captureFailure(() -> new CameraMountCalibrator(
                    config,
                    name -> {
                        applies[0]++;
                        return hardwareMap -> null;
                    }
            ));
            assertTrue(failure.getMessage(), failure.getMessage().contains(
                    "CameraMountCalibrator.Config.maxDetectionAgeSec"
            ));
            assertEquals(0, applies[0]);
        }
    }

    @Test
    public void invalidAprilPolicyFailsBeforePreferredBuilderApplication() {
        final int[] applies = {0};
        AprilTagLocalizationTester.Config config = AprilTagLocalizationTester.Config.defaults();
        config.preferredVisionDeviceName = "vision";
        config.aprilTags.fieldPoseSolver.maxAbsBearingRad = Math.PI + 0.01;
        RuntimeException failure = captureFailure(() -> new AprilTagLocalizationTester(
                config,
                name -> {
                    applies[0]++;
                    return hardwareMap -> null;
                }
        ));
        assertTrue(failure.getMessage(), failure.getMessage().contains(
                "AprilTagLocalizationTester.Config.aprilTags.fieldPoseSolver.maxAbsBearingRad"
        ));
        assertEquals(0, applies[0]);
    }

    @Test
    public void malformedLayoutsAreRejectedWithTheOwningFieldContext() {
        TagLayout malformed = new TagLayout() {
            @Override
            public Pose3d getFieldToTagPose(int id) {
                return Pose3d.zero();
            }

            @Override
            public Set<Integer> ids() {
                return null;
            }
        };

        CameraMountCalibrator.Config camera = CameraMountCalibrator.Config.defaults();
        camera.fixedTagLayout = malformed;
        assertMessageContains(
                () -> new CameraMountCalibrator(camera, name -> hardwareMap -> null),
                "CameraMountCalibrator.Config.fixedTagLayout"
        );

        AprilTagLocalizationTester.Config april = AprilTagLocalizationTester.Config.defaults();
        april.fixedTagLayout = malformed;
        assertMessageContains(
                () -> new AprilTagLocalizationTester(april, name -> hardwareMap -> null),
                "AprilTagLocalizationTester.Config.fixedTagLayout"
        );
    }

    @Test
    public void nullAndBlankActiveAnswersAreRejectedWithOwnerContext() {
        assertMessageContains(
                () -> new CameraMountCalibrator(null, name -> hardwareMap -> null),
                "CameraMountCalibrator.Config"
        );

        CameraMountCalibrator.Config blankCamera = CameraMountCalibrator.Config.defaults();
        blankCamera.preferredVisionDeviceName = "   ";
        assertMessageContains(
                () -> new CameraMountCalibrator(blankCamera, name -> hardwareMap -> null),
                "CameraMountCalibrator.Config.preferredVisionDeviceName"
        );

        CameraMountCalibrator.Config nullCameraType = CameraMountCalibrator.Config.defaults();
        nullCameraType.visionDeviceType = null;
        assertMessageContains(
                () -> new CameraMountCalibrator(nullCameraType, name -> hardwareMap -> null),
                "CameraMountCalibrator.Config.visionDeviceType"
        );

        CameraMountCalibrator.Config blankCameraTitle = CameraMountCalibrator.Config.defaults();
        blankCameraTitle.visionPickerTitle = "\t";
        assertMessageContains(
                () -> new CameraMountCalibrator(blankCameraTitle, name -> hardwareMap -> null),
                "CameraMountCalibrator.Config.visionPickerTitle"
        );

        CameraMountCalibrator.Config nullCameraLayout = CameraMountCalibrator.Config.defaults();
        nullCameraLayout.fixedTagLayout = null;
        assertMessageContains(
                () -> new CameraMountCalibrator(nullCameraLayout, name -> hardwareMap -> null),
                "CameraMountCalibrator.Config.fixedTagLayout"
        );

        assertMessageContains(
                () -> new CameraMountCalibrator(
                        CameraMountCalibrator.Config.defaults(),
                        null
                ),
                "visionLaneFactoryBuilder"
        );

        AprilTagLocalizationTester.Config blankApril =
                AprilTagLocalizationTester.Config.defaults();
        blankApril.preferredVisionDeviceName = "   ";
        assertMessageContains(
                () -> new AprilTagLocalizationTester(blankApril, name -> hardwareMap -> null),
                "AprilTagLocalizationTester.Config.preferredVisionDeviceName"
        );

        AprilTagLocalizationTester.Config nullAprilPolicy =
                AprilTagLocalizationTester.Config.defaults();
        nullAprilPolicy.aprilTags = null;
        assertMessageContains(
                () -> new AprilTagLocalizationTester(
                        nullAprilPolicy,
                        name -> hardwareMap -> null
                ),
                "AprilTagLocalizationTester.Config.aprilTags"
        );

        AprilTagLocalizationTester.Config nullSolver =
                AprilTagLocalizationTester.Config.defaults();
        nullSolver.aprilTags.fieldPoseSolver = null;
        assertMessageContains(
                () -> new AprilTagLocalizationTester(nullSolver, name -> hardwareMap -> null),
                "AprilTagLocalizationTester.Config.aprilTags.fieldPoseSolver"
        );

        assertMessageContains(
                () -> new AprilTagLocalizationTester(
                        AprilTagLocalizationTester.Config.defaults(),
                        null
                ),
                "visionLaneFactoryBuilder"
        );

        assertMessageContains(
                () -> new AprilTagLocalizationTester(null, name -> hardwareMap -> null),
                "AprilTagLocalizationTester.Config"
        );
    }

    private static void assertOwnerConstructor(Class<?> owner, Class<?> configType) throws Exception {
        List<Constructor<?>> publicConstructors = Arrays.stream(owner.getDeclaredConstructors())
                .filter(constructor -> Modifier.isPublic(constructor.getModifiers()))
                .collect(Collectors.toList());
        assertEquals(1, publicConstructors.size());
        Constructor<?> selected = owner.getDeclaredConstructor(configType, Function.class);
        assertTrue(Modifier.isPublic(selected.getModifiers()));
    }

    private static void assertConfigShape(
            Class<?> configType,
            List<String> expectedNames,
            List<Class<?>> expectedTypes
    ) {
        int modifiers = configType.getModifiers();
        assertTrue(Modifier.isPublic(modifiers));
        assertTrue(Modifier.isStatic(modifiers));
        assertTrue(Modifier.isFinal(modifiers));

        Constructor<?>[] constructors = configType.getDeclaredConstructors();
        assertEquals(1, constructors.length);
        assertTrue(Modifier.isPrivate(constructors[0].getModifiers()));

        List<Field> publicFields = Arrays.stream(configType.getDeclaredFields())
                .filter(field -> Modifier.isPublic(field.getModifiers()))
                .collect(Collectors.toList());
        assertEquals(expectedNames, publicFields.stream()
                .map(Field::getName)
                .collect(Collectors.toList()));
        assertEquals(expectedTypes, publicFields.stream()
                .map(Field::getType)
                .collect(Collectors.toList()));

        List<Method> publicMethods = Arrays.stream(configType.getDeclaredMethods())
                .filter(method -> Modifier.isPublic(method.getModifiers()))
                .filter(method -> !method.isSynthetic())
                .collect(Collectors.toList());
        assertEquals(1, publicMethods.size());
        assertEquals("defaults", publicMethods.get(0).getName());
        assertTrue(Modifier.isStatic(publicMethods.get(0).getModifiers()));
        assertEquals(configType, publicMethods.get(0).getReturnType());
    }

    private static Object field(Object owner, String name) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }

    private static TesterContext context(LoopClock clock) {
        return new TesterContext(
                new HardwareMap(null, null),
                noOpTelemetry(),
                new Gamepad(),
                new Gamepad(),
                clock
        );
    }

    private static Telemetry noOpTelemetry() {
        InvocationHandler handler = (proxy, method, args) -> defaultValue(method.getReturnType());
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                handler
        );
    }

    private static Object defaultValue(Class<?> type) {
        if (type == boolean.class) return true;
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0.0f;
        if (type == double.class) return 0.0;
        if (type == char.class) return '\0';
        return null;
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            throw new AssertionError("Expected configuration failure");
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static void assertMessageContains(Runnable action, String expectedText) {
        RuntimeException failure = captureFailure(action);
        assertTrue(failure.getMessage(), failure.getMessage().contains(expectedText));
    }

    private static final class CountingLayout implements TagLayout {
        private final Pose3d pose = new Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        private int idsReads;
        private int poseReads;

        @Override
        public Pose3d getFieldToTagPose(int id) {
            poseReads++;
            return id == 4 ? pose : null;
        }

        @Override
        public Set<Integer> ids() {
            idsReads++;
            return new LinkedHashSet<Integer>(Arrays.asList(4));
        }
    }

    private static final class ReadyLane implements AprilTagVisionLane {
        private final AprilTagSensor sensor;

        ReadyLane(AprilTagSensor sensor) {
            this.sensor = sensor;
        }

        @Override
        public AprilTagSensor tagSensor() {
            return sensor;
        }

        @Override
        public CameraMountConfig cameraMountConfig() {
            return CameraMountConfig.identity();
        }

        @Override
        public VisionReadiness readiness(LoopClock clock) {
            return VisionReadiness.ready();
        }

        @Override
        public void close() {
            // No resources in this fake.
        }
    }
}
