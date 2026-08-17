package edu.ftcphoenix.fw.tools.tester.localization;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** API and effect-free capture locks for the corrected-localization tester. */
public final class PinpointAprilTagCorrectedLocalizationTesterConfigTest {

    @Test
    public void publicSurfaceIsOneDefaultsOnlyConfigAndOneBehaviorConstructor()
            throws Exception {
        Class<PinpointAprilTagCorrectedLocalizationTester.Config> configType =
                PinpointAprilTagCorrectedLocalizationTester.Config.class;
        assertTrue(Modifier.isPublic(configType.getModifiers()));
        assertTrue(Modifier.isStatic(configType.getModifiers()));
        assertTrue(Modifier.isFinal(configType.getModifiers()));

        Constructor<?>[] configConstructors = configType.getDeclaredConstructors();
        assertEquals(1, configConstructors.length);
        assertTrue(Modifier.isPrivate(configConstructors[0].getModifiers()));
        List<Field> publicFields = publicFields(configType);
        assertEquals(
                Arrays.asList(
                        "preferredVisionDeviceName",
                        "visionDeviceType",
                        "visionPickerTitle",
                        "fixedTagLayout",
                        "localization"
                ),
                fieldNames(publicFields)
        );
        assertEquals(
                Arrays.asList(
                        String.class,
                        Class.class,
                        String.class,
                        TagLayout.class,
                        FtcOdometryAprilTagLocalizationLane.Config.class
                ),
                fieldTypes(publicFields)
        );
        assertEquals(Collections.singletonList("defaults"), publicMethodNames(configType));
        Method defaults = configType.getMethod("defaults");
        assertTrue(Modifier.isStatic(defaults.getModifiers()));
        assertSame(configType, defaults.getReturnType());

        Constructor<?>[] ownerConstructors =
                PinpointAprilTagCorrectedLocalizationTester.class.getConstructors();
        assertEquals(1, ownerConstructors.length);
        assertEquals(
                Arrays.asList(ConfigClass(), Function.class),
                Arrays.asList(ownerConstructors[0].getParameterTypes())
        );
        assertEquals(
                Arrays.asList("name", "onBackPressed"),
                publicMethodNames(PinpointAprilTagCorrectedLocalizationTester.class)
        );
        for (Class<?> nested : PinpointAprilTagCorrectedLocalizationTester.class.getDeclaredClasses()) {
            assertFalse("tester-local estimator enum must be removed", nested.isEnum());
        }
    }

    @Test
    public void defaultsAreFreshAndRetainTheApprovedFiveAnswers() {
        PinpointAprilTagCorrectedLocalizationTester.Config first =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        PinpointAprilTagCorrectedLocalizationTester.Config second =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();

        assertNotSame(first, second);
        assertNull(first.preferredVisionDeviceName);
        assertSame(WebcamName.class, first.visionDeviceType);
        assertEquals("Select Camera", first.visionPickerTitle);
        assertNotNull(first.fixedTagLayout);
        assertNotNull(first.localization);
        assertNotSame(first.localization, second.localization);
        assertNotSame(first.localization.predictor, second.localization.predictor);
        assertNotSame(first.localization.estimation, second.localization.estimation);
        assertEquals(
                0.50,
                first.localization.estimation.aprilTags.maxDetectionAgeSec,
                0.0
        );
        assertSame(
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION,
                first.localization.estimation.correctedEstimatorMode
        );
    }

    @Test
    public void constructorCapturesAllConfigBeforeApplyingPreferredBuilder() throws Exception {
        PinpointAprilTagCorrectedLocalizationTester.Config authored =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        CountingLayout layout = new CountingLayout();
        authored.preferredVisionDeviceName = "  frontVision  ";
        authored.visionDeviceType = HardwareDevice.class;
        authored.visionPickerTitle = "  Choose Vision  ";
        authored.fixedTagLayout = layout;
        authored.localization.estimation.aprilTags.maxDetectionAgeSec = 0.0;
        authored.localization.estimation.correctionFusion.correctionPositionGain = 0.27;
        authored.localization.estimation.correctionEkf = null;
        authored.localization.estimation.correctionSource.limelightFieldPose.maxResultAgeSec =
                Double.NaN;

        AtomicInteger builderCalls = new AtomicInteger();
        AtomicReference<String> selectedName = new AtomicReference<String>();
        Function<String, AprilTagVisionLaneFactory> builder = name -> {
            assertEquals("layout snapshot must precede preferred factory capture", 1,
                    layout.idsReads);
            assertEquals("layout snapshot must precede preferred factory capture", 1,
                    layout.poseReads);
            builderCalls.incrementAndGet();
            selectedName.set(name);
            authored.localization.estimation.correctionFusion.correctionPositionGain = 0.91;
            return hardwareMap -> null;
        };

        PinpointAprilTagCorrectedLocalizationTester tester =
                new PinpointAprilTagCorrectedLocalizationTester(authored, builder);

        assertEquals(1, builderCalls.get());
        assertEquals("frontVision", selectedName.get());
        assertEquals(1, layout.idsReads);
        assertEquals(1, layout.poseReads);
        assertEquals("frontVision", field(tester, "preferredVisionDeviceName"));
        assertEquals("Choose Vision", field(tester, "visionPickerTitle"));
        assertSame(HardwareDevice.class, field(tester, "visionDeviceType"));

        TagLayout capturedLayout = (TagLayout) field(tester, "fixedTagLayout");
        FtcOdometryAprilTagLocalizationLane.Config capturedLocalization =
                (FtcOdometryAprilTagLocalizationLane.Config) field(tester, "localizationConfig");
        assertNotSame(layout, capturedLayout);
        assertNotSame(authored.localization, capturedLocalization);
        assertNotSame(authored.localization.predictor, capturedLocalization.predictor);
        assertNotSame(authored.localization.estimation, capturedLocalization.estimation);
        assertEquals(0.0, capturedLocalization.estimation.aprilTags.maxDetectionAgeSec, 0.0);
        assertEquals(
                0.27,
                capturedLocalization.estimation.correctionFusion.correctionPositionGain,
                0.0
        );
        assertNull(capturedLocalization.estimation.correctionEkf);
        assertTrue(Double.isNaN(
                capturedLocalization.estimation.correctionSource
                        .limelightFieldPose.maxResultAgeSec
        ));

        authored.preferredVisionDeviceName = "other";
        authored.visionPickerTitle = "Other";
        authored.localization.estimation.aprilTags.maxDetectionAgeSec = 0.4;
        authored.localization.estimation.correctionFusion.correctionPositionGain = 0.8;
        layout.pose = new Pose3d(-12.0, 3.0, 9.0, 0.2, 0.0, 0.0);

        assertEquals("frontVision", field(tester, "preferredVisionDeviceName"));
        assertEquals("Choose Vision", field(tester, "visionPickerTitle"));
        assertEquals(0.0, capturedLocalization.estimation.aprilTags.maxDetectionAgeSec, 0.0);
        assertEquals(
                0.27,
                capturedLocalization.estimation.correctionFusion.correctionPositionGain,
                0.0
        );
        assertEquals(new Pose3d(8.0, 4.0, 2.0, 0.1, 0.0, 0.0),
                capturedLayout.getFieldToTagPose(9));
        assertEquals(1, layout.idsReads);
        assertEquals(1, layout.poseReads);
    }

    @Test
    public void pickerPathDefersBuilderAndPreservesCurrentGamePolicySummary() throws Exception {
        PinpointAprilTagCorrectedLocalizationTester.Config authored =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        FtcGameTagLayout gameLayout = (FtcGameTagLayout) authored.fixedTagLayout;
        String expectedPolicy = gameLayout.policySummaryLine();
        AtomicInteger builderCalls = new AtomicInteger();

        PinpointAprilTagCorrectedLocalizationTester tester =
                new PinpointAprilTagCorrectedLocalizationTester(
                        authored,
                        ignored -> {
                            builderCalls.incrementAndGet();
                            return hardwareMap -> null;
                        }
                );

        assertEquals(0, builderCalls.get());
        assertEquals(expectedPolicy, field(tester, "fixedTagLayoutPolicySummary"));
        assertNotSame(gameLayout, field(tester, "fixedTagLayout"));
    }

    @Test
    public void emptyLayoutAndDormantEstimatorDraftsAreCapturedWithoutInventedDefaults()
            throws Exception {
        PinpointAprilTagCorrectedLocalizationTester.Config authored =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        authored.fixedTagLayout = new SimpleEmptyLayout();
        authored.localization.estimation.correctedEstimatorMode =
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
        authored.localization.estimation.correctionEkf = null;
        authored.localization.estimation.correctionSource.mode =
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;
        authored.localization.estimation.correctionSource.limelightFieldPose.maxResultAgeSec =
                Double.NaN;
        AtomicInteger builderCalls = new AtomicInteger();

        PinpointAprilTagCorrectedLocalizationTester tester =
                new PinpointAprilTagCorrectedLocalizationTester(
                        authored,
                        ignored -> {
                            builderCalls.incrementAndGet();
                            return hardwareMap -> null;
                        }
                );

        assertEquals(0, builderCalls.get());
        assertTrue(((TagLayout) field(tester, "fixedTagLayout")).ids().isEmpty());
        FtcOdometryAprilTagLocalizationLane.Config captured =
                (FtcOdometryAprilTagLocalizationLane.Config) field(
                        tester,
                        "localizationConfig"
                );
        assertNull(captured.estimation.correctionEkf);
        assertTrue(Double.isNaN(
                captured.estimation.correctionSource.limelightFieldPose.maxResultAgeSec
        ));
    }

    @Test
    public void everyIntrinsicFailurePrecedesPreferredBuilderApplication() {
        assertInvalidBeforeBuilder(config -> config.preferredVisionDeviceName = "  ",
                "preferredVisionDeviceName");
        assertInvalidBeforeBuilder(config -> config.visionDeviceType = null,
                "visionDeviceType");
        assertInvalidBeforeBuilder(config -> config.visionPickerTitle = "  ",
                "visionPickerTitle");
        assertInvalidBeforeBuilder(config -> config.fixedTagLayout = null,
                "fixedTagLayout");
        assertInvalidBeforeBuilder(config -> config.localization = null,
                "localization");
        assertInvalidBeforeBuilder(config ->
                        config.localization.estimation.aprilTags.maxDetectionAgeSec = Double.NaN,
                "localization.estimation.aprilTags.maxDetectionAgeSec");
        assertInvalidBeforeBuilder(config ->
                        config.localization.estimation.correctionFusion = null,
                "localization.estimation.correctionFusion");
    }

    @Test
    public void invalidLayoutIsWrappedWithTheToolFieldContextBeforeBuilder() {
        PinpointAprilTagCorrectedLocalizationTester.Config authored =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        authored.preferredVisionDeviceName = "vision";
        authored.fixedTagLayout = new TagLayout() {
            @Override
            public Pose3d getFieldToTagPose(int id) {
                return null;
            }

            @Override
            public Set<Integer> ids() {
                return Collections.singleton(3);
            }
        };
        AtomicInteger builderCalls = new AtomicInteger();

        RuntimeException failure = captureFailure(() ->
                new PinpointAprilTagCorrectedLocalizationTester(
                        authored,
                        ignored -> {
                            builderCalls.incrementAndGet();
                            return hardwareMap -> null;
                        }
                ));

        assertEquals(0, builderCalls.get());
        assertTrue(failure.getMessage().contains(
                "PinpointAprilTagCorrectedLocalizationTester.Config.fixedTagLayout"
        ));
        assertTrue(failure.getMessage().contains("id=3"));
    }

    @Test
    public void behaviorPeerIsRequiredWithoutInventingAWebcamFallback() {
        PinpointAprilTagCorrectedLocalizationTester.Config authored =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        authored.fixedTagLayout = new SimpleEmptyLayout();

        RuntimeException failure = captureFailure(() ->
                new PinpointAprilTagCorrectedLocalizationTester(authored, null));

        assertTrue(failure.getMessage().contains(
                "PinpointAprilTagCorrectedLocalizationTester.visionLaneFactoryBuilder"
        ));
    }

    private interface ConfigMutation {
        void apply(PinpointAprilTagCorrectedLocalizationTester.Config config);
    }

    private static void assertInvalidBeforeBuilder(ConfigMutation mutation,
                                                   String expectedMessage) {
        PinpointAprilTagCorrectedLocalizationTester.Config authored =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        authored.preferredVisionDeviceName = "vision";
        mutation.apply(authored);
        AtomicInteger builderCalls = new AtomicInteger();

        RuntimeException failure = captureFailure(() ->
                new PinpointAprilTagCorrectedLocalizationTester(
                        authored,
                        ignored -> {
                            builderCalls.incrementAndGet();
                            return hardwareMap -> null;
                        }
                ));

        assertEquals(expectedMessage, 0, builderCalls.get());
        assertTrue(
                String.valueOf(failure.getMessage()),
                String.valueOf(failure.getMessage()).contains(expectedMessage)
        );
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected construction failure");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static Object field(Object owner, String fieldName) throws Exception {
        Field field = owner.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        return field.get(owner);
    }

    private static List<Field> publicFields(Class<?> type) {
        ArrayList<Field> fields = new ArrayList<Field>();
        for (Field field : type.getDeclaredFields()) {
            if (Modifier.isPublic(field.getModifiers()) && !field.isSynthetic()) {
                fields.add(field);
            }
        }
        return fields;
    }

    private static List<String> fieldNames(List<Field> fields) {
        ArrayList<String> names = new ArrayList<String>();
        for (Field field : fields) {
            names.add(field.getName());
        }
        return names;
    }

    private static List<Class<?>> fieldTypes(List<Field> fields) {
        ArrayList<Class<?>> types = new ArrayList<Class<?>>();
        for (Field field : fields) {
            types.add(field.getType());
        }
        return types;
    }

    private static List<String> publicMethodNames(Class<?> type) {
        ArrayList<String> names = new ArrayList<String>();
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && !method.isSynthetic()) {
                names.add(method.getName());
            }
        }
        Collections.sort(names);
        return names;
    }

    private static Class<?> ConfigClass() {
        return PinpointAprilTagCorrectedLocalizationTester.Config.class;
    }

    private static final class CountingLayout implements TagLayout {
        int idsReads;
        int poseReads;
        Pose3d pose = new Pose3d(8.0, 4.0, 2.0, 0.1, 0.0, 0.0);

        @Override
        public Pose3d getFieldToTagPose(int id) {
            poseReads++;
            return pose;
        }

        @Override
        public Set<Integer> ids() {
            idsReads++;
            return Collections.singleton(9);
        }
    }

    private static final class SimpleEmptyLayout implements TagLayout {
        @Override
        public Pose3d getFieldToTagPose(int id) {
            return null;
        }

        @Override
        public Set<Integer> ids() {
            return Collections.emptySet();
        }
    }
}
