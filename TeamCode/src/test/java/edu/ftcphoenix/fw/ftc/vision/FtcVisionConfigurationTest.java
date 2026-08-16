package edu.ftcphoenix.fw.ftc.vision;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.junit.Test;
import org.opencv.core.Mat;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Configuration-boundary and API-shape coverage for the FTC vision owners. */
public final class FtcVisionConfigurationTest {

    private static final FtcWebcamVisionPortalLane.ResolutionReader VALID_RESOLUTION_READER =
            new FtcWebcamVisionPortalLane.ResolutionReader() {
                @Override
                public int width(Size size) {
                    return 640;
                }

                @Override
                public int height(Size size) {
                    return 480;
                }
            };

    @Test
    public void rawWebcamConfigCopyRetainsBorrowedLibraryWithoutValidation() {
        AprilTagLibrary malformed = new AprilTagLibrary.Builder().build();
        FtcWebcamAprilTagVisionLane.Config authored =
                FtcWebcamAprilTagVisionLane.Config.defaults();
        authored.webcamName = "  Webcam Left  ";
        authored.tagLibrary = malformed;

        FtcWebcamAprilTagVisionLane.Config copy = authored.copy();

        assertNotSame(authored, copy);
        assertSame(malformed, copy.tagLibrary);
        assertEquals("  Webcam Left  ", copy.webcamName);
    }

    @Test
    public void activeWebcamCaptureCanonicalizesUnitsAndOwnsEveryMutableSdkValue() {
        VectorF authoredPosition = new VectorF(25.4f, -50.8f, 76.2f);
        Quaternion authoredOrientation = new Quaternion(0.5f, 0.5f, 0.5f, 0.5f, 123L);
        AprilTagMetadata authoredMetadata = new AprilTagMetadata(
                7,
                null,
                50.8,
                authoredPosition,
                DistanceUnit.MM,
                authoredOrientation
        );
        AprilTagLibrary source = library(
                authoredMetadata,
                metadata(2, DistanceUnit.INCH, 2.0,
                        new VectorF(4.0f, 5.0f, 6.0f), Quaternion.identityQuaternion()),
                metadata(4, DistanceUnit.INCH, 3.0,
                        new VectorF(7.0f, 8.0f, 9.0f), Quaternion.identityQuaternion())
        );
        FtcWebcamAprilTagVisionLane.Config cfg =
                FtcWebcamAprilTagVisionLane.Config.defaults();
        cfg.webcamName = "  Webcam Left  ";
        cfg.tagLibrary = source;

        FtcWebcamAprilTagVisionLane.ActiveConfig captured =
                FtcWebcamAprilTagVisionLane.captureActiveConfig(
                        cfg,
                        VALID_RESOLUTION_READER
                );

        assertEquals("Webcam Left", captured.webcamName());
        assertEquals("custom", captured.tagLibraryProvenance());

        // Mutate every SDK alias held by the source after the active boundary captured it.
        authoredPosition.put(0, 999.0f);
        authoredOrientation.w = 0.0f;
        source.getAllTags()[0] = metadata(99, DistanceUnit.INCH, 1.0,
                new VectorF(9.0f, 9.0f, 9.0f), Quaternion.identityQuaternion());

        AprilTagLibrary firstLibrary = captured.freshTagLibrary();
        AprilTagMetadata first = firstLibrary.getAllTags()[0];
        assertEquals(3, firstLibrary.getAllTags().length);
        assertEquals(7, first.id);
        assertEquals(2, firstLibrary.getAllTags()[1].id);
        assertEquals(4, firstLibrary.getAllTags()[2].id);
        assertEquals(null, first.name);
        assertEquals(DistanceUnit.INCH, first.distanceUnit);
        assertEquals(2.0, first.tagsize, 1e-9);
        assertEquals(1.0, first.fieldPosition.get(0), 1e-5);
        assertEquals(-2.0, first.fieldPosition.get(1), 1e-5);
        assertEquals(3.0, first.fieldPosition.get(2), 1e-5);
        assertEquals(0.5, first.fieldOrientation.w, 0.0);
        assertEquals(0.5, first.fieldOrientation.x, 0.0);
        assertEquals(0.5, first.fieldOrientation.y, 0.0);
        assertEquals(0.5, first.fieldOrientation.z, 0.0);
        assertEquals(123L, first.fieldOrientation.acquisitionTime);

        // A processor/open receives another independent library, not the retained snapshot.
        first.fieldPosition.put(0, -88.0f);
        first.fieldOrientation.w = 0.0f;
        firstLibrary.getAllTags()[0] = metadata(88, DistanceUnit.INCH, 1.0,
                new VectorF(8.0f, 8.0f, 8.0f), Quaternion.identityQuaternion());
        AprilTagLibrary secondLibrary = captured.freshTagLibrary();
        assertNotSame(firstLibrary, secondLibrary);
        assertNotSame(firstLibrary.getAllTags(), secondLibrary.getAllTags());
        assertEquals(7, secondLibrary.getAllTags()[0].id);
        assertEquals(1.0, secondLibrary.getAllTags()[0].fieldPosition.get(0), 1e-5);
        assertEquals(0.5, secondLibrary.getAllTags()[0].fieldOrientation.w, 0.0);
        assertEquals(0.5, secondLibrary.getAllTags()[0].fieldOrientation.x, 0.0);
        assertEquals(0.5, secondLibrary.getAllTags()[0].fieldOrientation.y, 0.0);
        assertEquals(0.5, secondLibrary.getAllTags()[0].fieldOrientation.z, 0.0);
    }

    @Test
    public void currentGameMeaningIsResolvedAtCaptureAndFreshLibrariesAreIndependent() {
        FtcWebcamAprilTagVisionLane.Config cfg =
                FtcWebcamAprilTagVisionLane.Config.defaults();

        FtcWebcamAprilTagVisionLane.ActiveConfig captured =
                FtcWebcamAprilTagVisionLane.captureActiveConfig(
                        cfg,
                        VALID_RESOLUTION_READER
                );
        AprilTagLibrary first = captured.freshTagLibrary();
        AprilTagLibrary second = captured.freshTagLibrary();

        assertEquals("currentGame", captured.tagLibraryProvenance());
        assertNotSame(first, second);
        assertNotSame(first.getAllTags(), second.getAllTags());
        assertTrue(first.getAllTags().length > 0);
        assertEquals(first.getAllTags().length, second.getAllTags().length);
        for (int i = 0; i < first.getAllTags().length; i++) {
            assertEquals(first.getAllTags()[i].id, second.getAllTags()[i].id);
        }
    }

    @Test
    public void activeLibraryCaptureRejectsMalformedMetadataWithExactLocation() {
        assertLibraryRejected(
                rawLibrary((AprilTagMetadata) null),
                "Config.tagLibrary.metadata[0] must not be null"
        );
        assertLibraryRejected(
                rawLibrary(metadata(-1, DistanceUnit.INCH, 1.0,
                        new VectorF(0.0f, 0.0f, 0.0f), Quaternion.identityQuaternion())),
                "metadata[0].id must be non-negative, got -1"
        );
        assertLibraryRejected(
                rawLibrary(new AprilTagMetadata(
                        1, "bad unit", 1.0, new VectorF(0.0f, 0.0f, 0.0f), null,
                        Quaternion.identityQuaternion())),
                "metadata[0].distanceUnit must not be null"
        );
        assertLibraryRejected(
                rawLibrary(new AprilTagMetadata(
                        1, "bad position", 1.0, null, DistanceUnit.INCH,
                        Quaternion.identityQuaternion())),
                "metadata[0].fieldPosition must not be null"
        );
        assertLibraryRejected(
                rawLibrary(metadata(1, DistanceUnit.INCH, 1.0,
                        new VectorF(1.0f, 2.0f), Quaternion.identityQuaternion())),
                "fieldPosition must have exactly three components, got 2"
        );
        assertLibraryRejected(
                rawLibrary(metadata(1, DistanceUnit.INCH, 1.0,
                        new VectorF(1.0f, 2.0f, 3.0f, 4.0f),
                        Quaternion.identityQuaternion())),
                "fieldPosition must have exactly three components, got 4"
        );
        assertLibraryRejected(
                rawLibrary(new AprilTagMetadata(
                        1, "bad orientation", 1.0,
                        new VectorF(0.0f, 0.0f, 0.0f), DistanceUnit.INCH, null)),
                "metadata[0].fieldOrientation must not be null"
        );
        assertLibraryRejected(
                new AprilTagLibrary.Builder().build(),
                "Config.tagLibrary must contain at least one tag"
        );

        AprilTagLibrary duplicate = library(
                metadata(3, DistanceUnit.INCH, 1.0,
                        new VectorF(0.0f, 0.0f, 0.0f), Quaternion.identityQuaternion()),
                metadata(4, DistanceUnit.INCH, 1.0,
                        new VectorF(1.0f, 0.0f, 0.0f), Quaternion.identityQuaternion())
        );
        duplicate.getAllTags()[1] = metadata(3, DistanceUnit.INCH, 1.0,
                new VectorF(2.0f, 0.0f, 0.0f), Quaternion.identityQuaternion());
        assertLibraryRejected(
                duplicate,
                "duplicate id 3 at metadata[0] and metadata[1]"
        );
    }

    @Test
    public void activeLibraryCaptureRejectsEveryNonFiniteFieldAndFloatConversionOverflow() {
        double[] invalidSizes = {
                0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
        };
        for (double value : invalidSizes) {
            assertLibraryRejected(
                    rawLibrary(metadata(1, DistanceUnit.INCH, value,
                            new VectorF(0.0f, 0.0f, 0.0f),
                            Quaternion.identityQuaternion())),
                    ".tagsize"
            );
        }
        assertLibraryRejected(
                rawLibrary(metadata(1, DistanceUnit.METER, Double.MAX_VALUE,
                        new VectorF(0.0f, 0.0f, 0.0f),
                        Quaternion.identityQuaternion())),
                ".tagsize"
        );
        assertLibraryRejected(
                rawLibrary(metadata(1, DistanceUnit.METER, Double.MAX_VALUE / 2.0,
                        new VectorF(0.0f, 0.0f, 0.0f),
                        Quaternion.identityQuaternion())),
                ".tagsize"
        );
        assertLibraryRejected(
                rawLibrary(metadata(1, DistanceUnit.MM, Double.MIN_VALUE,
                        new VectorF(0.0f, 0.0f, 0.0f),
                        Quaternion.identityQuaternion())),
                ".tagsize"
        );
        float[] invalidFloats = {
                Float.NaN, Float.POSITIVE_INFINITY, Float.NEGATIVE_INFINITY
        };
        for (int slot = 0; slot < 3; slot++) {
            for (float value : invalidFloats) {
                float[] position = {1.0f, 2.0f, 3.0f};
                position[slot] = value;
                assertLibraryRejected(
                        rawLibrary(metadata(1, DistanceUnit.INCH, 1.0,
                                new VectorF(position), Quaternion.identityQuaternion())),
                        ".fieldPosition[" + slot + "]"
                );
            }
        }
        assertLibraryRejected(
                rawLibrary(metadata(1, DistanceUnit.METER, 1.0,
                        new VectorF(Float.MAX_VALUE, 0.0f, 0.0f),
                        Quaternion.identityQuaternion())),
                ".fieldPosition[0]"
        );

        for (int slot = 0; slot < 4; slot++) {
            for (float value : invalidFloats) {
                float[] quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
                quaternion[slot] = value;
                assertLibraryRejected(
                        rawLibrary(metadata(1, DistanceUnit.INCH, 1.0,
                                new VectorF(0.0f, 0.0f, 0.0f),
                                new Quaternion(
                                        quaternion[0], quaternion[1], quaternion[2], quaternion[3],
                                        0L))),
                        ".fieldOrientation."
                );
            }
        }
    }

    @Test
    public void quaternionToleranceAcceptsNearUnitWithoutNormalizingAndRejectsOutsideIt() {
        Quaternion accepted = new Quaternion(1.000009f, 0.0f, 0.0f, 0.0f, 456L);
        FtcWebcamAprilTagVisionLane.ActiveConfig captured = capture(
                library(metadata(1, DistanceUnit.INCH, 1.0,
                        new VectorF(0.0f, 0.0f, 0.0f), accepted))
        );

        Quaternion retained = captured.freshTagLibrary().getAllTags()[0].fieldOrientation;
        assertEquals(accepted.w, retained.w, 0.0);
        assertEquals(456L, retained.acquisitionTime);

        assertLibraryRejected(
                rawLibrary(metadata(1, DistanceUnit.INCH, 1.0,
                        new VectorF(0.0f, 0.0f, 0.0f),
                        new Quaternion(1.000011f, 0.0f, 0.0f, 0.0f, 0L))),
                "normalize the authored quaternion"
        );
        assertLibraryRejected(
                rawLibrary(metadata(1, DistanceUnit.INCH, 1.0,
                        new VectorF(0.0f, 0.0f, 0.0f),
                        new Quaternion(0.0f, 0.0f, 0.0f, 0.0f, 0L))),
                "magnitude=0.0"
        );
    }

    @Test
    public void deferredFactoriesValidateAndCaptureBeforeHardwareOpen() {
        FtcLimelightAprilTagVisionLane.Config limelight =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        limelight.hardwareName = "  limelight-main  ";
        AprilTagVisionLaneFactory limelightFactory =
                AprilTagVisionLaneFactories.limelight(limelight);
        limelight.hardwareName = "changed";
        limelight.pipelineIndex = 9;
        assertEquals("limelight: limelight-main", limelightFactory.description());

        assertLimelightFactoryRejected("pipelineIndex", "-1", config -> config.pipelineIndex = -1);
        assertLimelightFactoryRejected("pipelineIndex", "10", config -> config.pipelineIndex = 10);
        assertLimelightFactoryRejected("pollRateHz", "0", config -> config.pollRateHz = 0);
        assertLimelightFactoryRejected("pollRateHz", "251", config -> config.pollRateHz = 251);
        assertLimelightFactoryRejected(
                "maxResultAgeSec", "NaN", config -> config.maxResultAgeSec = Double.NaN);
        assertLimelightFactoryRejected(
                "cameraMount", "null", config -> config.cameraMount = null);

        FtcWebcamAprilTagVisionLane.Config webcam =
                FtcWebcamAprilTagVisionLane.Config.defaults();
        webcam.webcamName = "  Webcam Right  ";
        webcam.tagLibrary = library(metadata(1, DistanceUnit.INCH, 1.0,
                new VectorF(0.0f, 0.0f, 0.0f), Quaternion.identityQuaternion()));
        AprilTagVisionLaneFactory webcamFactory =
                AprilTagVisionLaneFactories.webcam(webcam, VALID_RESOLUTION_READER);
        webcam.webcamName = "changed";
        webcam.tagLibrary.getAllTags()[0] = metadata(99, DistanceUnit.INCH, 1.0,
                new VectorF(0.0f, 0.0f, 0.0f), Quaternion.identityQuaternion());
        assertEquals("webcam: Webcam Right", webcamFactory.description());

        webcam = FtcWebcamAprilTagVisionLane.Config.defaults();
        webcam.webcamName = "  ";
        FtcWebcamAprilTagVisionLane.Config invalidWebcam = webcam;
        expectFailureContaining(
                IllegalArgumentException.class,
                "FtcWebcamAprilTagVisionLane.Config.webcamName",
                () -> AprilTagVisionLaneFactories.webcam(
                        invalidWebcam,
                        VALID_RESOLUTION_READER
                )
        );
    }

    @Test
    public void deferredFactoriesOwnAllFieldsAndProduceIndependentRepeatedOpenSnapshots() {
        FtcWebcamAprilTagVisionLane.Config webcam =
                FtcWebcamAprilTagVisionLane.Config.defaults();
        Size authoredResolution = webcam.cameraResolution;
        CameraMountConfig authoredMount = CameraMountConfig.of(
                1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        webcam.webcamName = "  Webcam Factory  ";
        webcam.cameraMount = authoredMount;
        webcam.tagLibrary = library(metadata(7, DistanceUnit.INCH, 2.0,
                new VectorF(1.0f, 2.0f, 3.0f), Quaternion.identityQuaternion()));

        List<FtcWebcamAprilTagVisionLane.ActiveConfig> webcamOwners =
                new ArrayList<FtcWebcamAprilTagVisionLane.ActiveConfig>();
        List<AprilTagLibrary> openedLibraries = new ArrayList<AprilTagLibrary>();
        AprilTagVisionLaneFactory webcamFactory = AprilTagVisionLaneFactories.webcam(
                webcam,
                VALID_RESOLUTION_READER,
                (hardwareMap, captured) -> {
                    FtcWebcamAprilTagVisionLane.ActiveConfig owner =
                            captured.recaptured(VALID_RESOLUTION_READER);
                    webcamOwners.add(owner);
                    openedLibraries.add(owner.freshTagLibrary());
                    return fakeLane(owner.cameraMount());
                }
        );

        webcam.webcamName = "mutated";
        webcam.cameraResolution = null;
        webcam.cameraMount = CameraMountConfig.identity();
        webcam.tagLibrary.getAllTags()[0] = metadata(99, DistanceUnit.INCH, 9.0,
                new VectorF(9.0f, 9.0f, 9.0f), Quaternion.identityQuaternion());

        webcamFactory.open(null);
        webcamFactory.open(null);
        assertEquals(2, webcamOwners.size());
        assertNotSame(webcamOwners.get(0), webcamOwners.get(1));
        for (FtcWebcamAprilTagVisionLane.ActiveConfig owner : webcamOwners) {
            assertEquals("Webcam Factory", owner.webcamName());
            assertSame(authoredResolution, owner.cameraResolution());
            assertSame(authoredMount, owner.cameraMount());
        }
        assertNotSame(openedLibraries.get(0), openedLibraries.get(1));
        assertNotSame(openedLibraries.get(0).getAllTags(), openedLibraries.get(1).getAllTags());
        assertEquals(7, openedLibraries.get(0).getAllTags()[0].id);
        assertEquals(7, openedLibraries.get(1).getAllTags()[0].id);

        FtcLimelightAprilTagVisionLane.Config limelight =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        CameraMountConfig limelightMount = CameraMountConfig.of(
                -1.0, -2.0, 4.0, -0.1, 0.0, 0.2);
        limelight.hardwareName = "  limelight-factory  ";
        limelight.pipelineIndex = 9;
        limelight.pollRateHz = 250;
        limelight.maxResultAgeSec = Double.MIN_VALUE;
        limelight.cameraMount = limelightMount;
        List<FtcLimelightAprilTagVisionLane.Config> limelightOwners =
                new ArrayList<FtcLimelightAprilTagVisionLane.Config>();
        AprilTagVisionLaneFactory limelightFactory = AprilTagVisionLaneFactories.limelight(
                limelight,
                (hardwareMap, captured) -> {
                    FtcLimelightAprilTagVisionLane.Config owner =
                            FtcLimelightAprilTagVisionLane.validatedCopy(captured);
                    limelightOwners.add(owner);
                    return fakeLane(owner.cameraMount);
                }
        );

        limelight.hardwareName = "mutated";
        limelight.pipelineIndex = 0;
        limelight.pollRateHz = 1;
        limelight.maxResultAgeSec = 10.0;
        limelight.cameraMount = CameraMountConfig.identity();
        limelightFactory.open(null);
        limelightFactory.open(null);

        assertEquals(2, limelightOwners.size());
        assertNotSame(limelightOwners.get(0), limelightOwners.get(1));
        for (FtcLimelightAprilTagVisionLane.Config owner : limelightOwners) {
            assertEquals("limelight-factory", owner.hardwareName);
            assertEquals(9, owner.pipelineIndex);
            assertEquals(250, owner.pollRateHz);
            assertEquals(Double.MIN_VALUE, owner.maxResultAgeSec, 0.0);
            assertSame(limelightMount, owner.cameraMount);
        }
    }

    @Test
    public void specializedWebcamPreflightRejectsBeforeProcessorOrPortalEffects() {
        int[] processorCreates = {0};
        int[] portalOpens = {0};
        FtcWebcamAprilTagVisionLane.AprilTagProcessorFactory processorFactory =
                (mount, tagLibrary) -> {
                    processorCreates[0]++;
                    return new org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder()
                            .setTagLibrary(tagLibrary)
                            .build();
                };
        FtcWebcamVisionPortalLane.PortalFactory portalFactory = (config, processors) -> {
            portalOpens[0]++;
            return null;
        };

        expectFailureMessage(
                NullPointerException.class,
                "FtcWebcamAprilTagVisionLane.Config",
                () -> specializedOwner(null, processorFactory, portalFactory,
                        VALID_RESOLUTION_READER)
        );

        FtcWebcamAprilTagVisionLane.Config blank = validWebcamConfig();
        blank.webcamName = " ";
        expectFailureMessage(
                IllegalArgumentException.class,
                "FtcWebcamAprilTagVisionLane.Config.webcamName must not be blank",
                () -> specializedOwner(blank, processorFactory, portalFactory,
                        VALID_RESOLUTION_READER)
        );

        FtcWebcamAprilTagVisionLane.Config nullName = validWebcamConfig();
        nullName.webcamName = null;
        expectFailureMessage(
                NullPointerException.class,
                "FtcWebcamAprilTagVisionLane.Config.webcamName",
                () -> specializedOwner(nullName, processorFactory, portalFactory,
                        VALID_RESOLUTION_READER)
        );

        FtcWebcamAprilTagVisionLane.Config nullResolution = validWebcamConfig();
        nullResolution.cameraResolution = null;
        expectFailureContaining(NullPointerException.class, ".cameraResolution",
                () -> specializedOwner(nullResolution, processorFactory, portalFactory,
                        VALID_RESOLUTION_READER));

        expectFailureContaining(IllegalArgumentException.class, ".cameraResolution",
                () -> specializedOwner(validWebcamConfig(), processorFactory, portalFactory,
                        new FtcWebcamVisionPortalLane.ResolutionReader() {
                            @Override
                            public int width(Size size) {
                                return 0;
                            }

                            @Override
                            public int height(Size size) {
                                return 480;
                            }
                        }));

        FtcWebcamAprilTagVisionLane.Config nullMount = validWebcamConfig();
        nullMount.cameraMount = null;
        expectFailureContaining(NullPointerException.class, ".cameraMount",
                () -> specializedOwner(nullMount, processorFactory, portalFactory,
                        VALID_RESOLUTION_READER));

        FtcWebcamAprilTagVisionLane.Config emptyLibrary = validWebcamConfig();
        emptyLibrary.tagLibrary = new AprilTagLibrary.Builder().build();
        expectFailureContaining(IllegalArgumentException.class, ".tagLibrary",
                () -> specializedOwner(emptyLibrary, processorFactory, portalFactory,
                        VALID_RESOLUTION_READER));

        VisionProcessor duplicate = new NoopProcessor();
        expectFailureContaining(NullPointerException.class, "additionalProcessors",
                () -> specializedOwner(validWebcamConfig(), processorFactory, portalFactory,
                        VALID_RESOLUTION_READER, (VisionProcessor[]) null));
        expectFailureContaining(NullPointerException.class, "additionalProcessors[0]",
                () -> specializedOwner(validWebcamConfig(), processorFactory, portalFactory,
                        VALID_RESOLUTION_READER, (VisionProcessor) null));
        expectFailureContaining(IllegalArgumentException.class, "duplicate",
                () -> specializedOwner(validWebcamConfig(), processorFactory, portalFactory,
                        VALID_RESOLUTION_READER, duplicate, duplicate));

        assertEquals(0, processorCreates[0]);
        assertEquals(0, portalOpens[0]);

        expectFailureMessage(
                NullPointerException.class,
                "hardwareMap",
                () -> new FtcWebcamAprilTagVisionLane(
                        null,
                        FtcWebcamAprilTagVisionLane.Config.defaults()
                )
        );
        assertEquals(0, processorCreates[0]);
        assertEquals(0, portalOpens[0]);
    }

    @Test
    public void ownerSpecificNullConfigAndLimelightBoundaryDiagnosticsAreExact() {
        expectFailureMessage(
                NullPointerException.class,
                "FtcWebcamVisionPortalLane.Config",
                () -> new FtcWebcamVisionPortalLane(
                        null,
                        (config, processors) -> null,
                        () -> 0L,
                        VALID_RESOLUTION_READER
                )
        );
        expectFailureMessage(
                NullPointerException.class,
                "FtcLimelightVisionLane.Config",
                () -> new FtcLimelightVisionLane(null, hardwareName -> null)
        );
        expectFailureMessage(
                NullPointerException.class,
                "FtcLimelightAprilTagVisionLane.Config",
                () -> new FtcLimelightAprilTagVisionLane(null, hardwareName -> null)
        );
        expectFailureMessage(
                NullPointerException.class,
                "FtcWebcamAprilTagVisionLane.Config",
                () -> AprilTagVisionLaneFactories.webcam(null, VALID_RESOLUTION_READER)
        );
        expectFailureMessage(
                NullPointerException.class,
                "FtcLimelightAprilTagVisionLane.Config",
                () -> AprilTagVisionLaneFactories.limelight(null)
        );

        FtcLimelightAprilTagVisionLane.Config lower =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        lower.pipelineIndex = 0;
        lower.pollRateHz = 1;
        lower.maxResultAgeSec = Double.MIN_VALUE;
        FtcLimelightAprilTagVisionLane.Config lowerSnapshot =
                FtcLimelightAprilTagVisionLane.validatedCopy(lower);
        assertEquals(0, lowerSnapshot.pipelineIndex);
        assertEquals(1, lowerSnapshot.pollRateHz);
        assertEquals(Double.MIN_VALUE, lowerSnapshot.maxResultAgeSec, 0.0);

        FtcLimelightAprilTagVisionLane.Config upper =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        upper.pipelineIndex = 9;
        upper.pollRateHz = 250;
        FtcLimelightAprilTagVisionLane.Config upperSnapshot =
                FtcLimelightAprilTagVisionLane.validatedCopy(upper);
        assertEquals(9, upperSnapshot.pipelineIndex);
        assertEquals(250, upperSnapshot.pollRateHz);

        assertLimelightFailureMessage(
                config -> config.pipelineIndex = -1,
                "FtcLimelightAprilTagVisionLane.Config.pipelineIndex must be within [0, 9], got -1"
        );
        assertLimelightFailureMessage(
                config -> config.pollRateHz = 0,
                "FtcLimelightAprilTagVisionLane.Config.pollRateHz must be within [1, 250], got 0"
        );
        assertLimelightFailureMessage(
                config -> config.maxResultAgeSec = Double.POSITIVE_INFINITY,
                "FtcLimelightAprilTagVisionLane.Config.maxResultAgeSec must be finite and > 0, "
                        + "got Infinity"
        );
    }

    @Test
    public void retainedVisionApiHasOneConstructorPerOwnerAndOnlyConfigFactories() {
        assertPublicConstructors(
                FtcWebcamVisionPortalLane.class,
                signature(HardwareMap.class, FtcWebcamVisionPortalLane.Config.class,
                        VisionProcessor[].class)
        );
        assertPublicConstructors(
                FtcWebcamAprilTagVisionLane.class,
                signature(HardwareMap.class, FtcWebcamAprilTagVisionLane.Config.class,
                        VisionProcessor[].class)
        );
        assertPublicConstructors(
                FtcLimelightVisionLane.class,
                signature(HardwareMap.class, FtcLimelightVisionLane.Config.class)
        );
        assertPublicConstructors(
                FtcLimelightAprilTagVisionLane.class,
                signature(HardwareMap.class, FtcLimelightAprilTagVisionLane.Config.class)
        );

        assertNoDeclaredMethod(FtcWebcamVisionPortalLane.class, "portalConfig");
        assertNoDeclaredMethod(FtcWebcamAprilTagVisionLane.class, "config");
        assertNoDeclaredMethod(FtcLimelightVisionLane.class, "visionConfig");
        assertNoDeclaredMethod(FtcLimelightAprilTagVisionLane.class, "config");

        List<String> publicFactories = new ArrayList<String>();
        for (Method method : AprilTagVisionLaneFactories.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicFactories.add(method.getName() + Arrays.toString(method.getParameterTypes()));
            }
        }
        assertEquals(Arrays.asList(
                "limelight[class edu.ftcphoenix.fw.ftc.vision.FtcLimelightAprilTagVisionLane$Config]",
                "webcam[class edu.ftcphoenix.fw.ftc.vision.FtcWebcamAprilTagVisionLane$Config]"
        ), sorted(publicFactories));

        assertConfigSurface(FtcWebcamVisionPortalLane.Config.class);
        assertConfigSurface(FtcWebcamAprilTagVisionLane.Config.class);
        assertConfigSurface(FtcLimelightVisionLane.Config.class);
        assertConfigSurface(FtcLimelightAprilTagVisionLane.Config.class);
    }

    @Test
    public void everyRetainedVisionConfigHasCompactFieldBearingToString() {
        FtcWebcamVisionPortalLane.Config genericWebcamConfig =
                FtcWebcamVisionPortalLane.Config.defaults();
        // Android's pure-JVM stub does not implement Size.toString(); a raw null draft still
        // exercises the config formatter and is intentionally not validated by toString().
        genericWebcamConfig.cameraResolution = null;
        String genericWebcam = genericWebcamConfig.toString();
        assertTrue(genericWebcam, genericWebcam.contains("webcamName"));
        assertTrue(genericWebcam, genericWebcam.contains("cameraResolution"));

        FtcWebcamAprilTagVisionLane.Config tagWebcamConfig =
                FtcWebcamAprilTagVisionLane.Config.defaults();
        tagWebcamConfig.cameraResolution = null;
        String tagWebcam = tagWebcamConfig.toString();
        assertTrue(tagWebcam, tagWebcam.contains("cameraMount"));
        assertTrue(tagWebcam, tagWebcam.contains("tagLibrary=currentGame"));

        String genericLimelight = FtcLimelightVisionLane.Config.defaults().toString();
        assertTrue(genericLimelight, genericLimelight.contains("pipelineIndex=0"));
        assertTrue(genericLimelight, genericLimelight.contains("pollRateHz=100"));
        assertTrue(genericLimelight, genericLimelight.contains("maxResultAgeSec=0.25"));

        String tagLimelight = FtcLimelightAprilTagVisionLane.Config.defaults().toString();
        assertTrue(tagLimelight, tagLimelight.contains("cameraMount"));
    }

    private static FtcWebcamAprilTagVisionLane.Config validWebcamConfig() {
        FtcWebcamAprilTagVisionLane.Config config =
                FtcWebcamAprilTagVisionLane.Config.defaults();
        config.tagLibrary = library(metadata(
                1,
                DistanceUnit.INCH,
                1.0,
                new VectorF(0.0f, 0.0f, 0.0f),
                Quaternion.identityQuaternion()
        ));
        return config;
    }

    private static void specializedOwner(
            FtcWebcamAprilTagVisionLane.Config config,
            FtcWebcamAprilTagVisionLane.AprilTagProcessorFactory processorFactory,
            FtcWebcamVisionPortalLane.PortalFactory portalFactory,
            FtcWebcamVisionPortalLane.ResolutionReader resolutionReader,
            VisionProcessor... additionalProcessors
    ) {
        new FtcWebcamAprilTagVisionLane(
                config,
                processorFactory,
                portalFactory,
                () -> 0L,
                resolutionReader,
                additionalProcessors
        );
    }

    private static AprilTagVisionLane fakeLane(CameraMountConfig mount) {
        return new AprilTagVisionLane() {
            @Override
            public edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor tagSensor() {
                return clock -> edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections.none();
            }

            @Override
            public CameraMountConfig cameraMountConfig() {
                return mount;
            }

            @Override
            public VisionReadiness readiness(edu.ftcphoenix.fw.core.time.LoopClock clock) {
                return VisionReadiness.ready();
            }

            @Override
            public void close() {
                // No resource in this factory-capture test value.
            }
        };
    }

    private static void assertLimelightFailureMessage(
            ConfigMutation mutation,
            String expectedMessage
    ) {
        FtcLimelightAprilTagVisionLane.Config config =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        mutation.apply(config);
        expectFailureMessage(
                IllegalArgumentException.class,
                expectedMessage,
                () -> FtcLimelightAprilTagVisionLane.validatedCopy(config)
        );
    }

    private static FtcWebcamAprilTagVisionLane.ActiveConfig capture(AprilTagLibrary library) {
        FtcWebcamAprilTagVisionLane.Config cfg =
                FtcWebcamAprilTagVisionLane.Config.defaults();
        cfg.tagLibrary = library;
        return FtcWebcamAprilTagVisionLane.captureActiveConfig(cfg, VALID_RESOLUTION_READER);
    }

    private static AprilTagMetadata metadata(
            int id,
            DistanceUnit unit,
            double size,
            VectorF position,
            Quaternion orientation
    ) {
        return new AprilTagMetadata(id, "tag-" + id, size, position, unit, orientation);
    }

    private static AprilTagLibrary library(AprilTagMetadata... metadata) {
        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
        for (AprilTagMetadata entry : metadata) {
            builder.addTag(entry);
        }
        return builder.build();
    }

    private static AprilTagLibrary rawLibrary(AprilTagMetadata metadata) {
        AprilTagLibrary library = library(FtcVisionConfigurationTest.metadata(
                100,
                DistanceUnit.INCH,
                1.0,
                new VectorF(0.0f, 0.0f, 0.0f),
                Quaternion.identityQuaternion()
        ));
        library.getAllTags()[0] = metadata;
        return library;
    }

    private static void assertLibraryRejected(AprilTagLibrary library, String messageFragment) {
        expectFailureContaining(
                IllegalArgumentException.class,
                messageFragment,
                () -> capture(library)
        );
    }

    private static void assertLimelightFactoryRejected(
            String field,
            String value,
            ConfigMutation mutation
    ) {
        FtcLimelightAprilTagVisionLane.Config cfg =
                FtcLimelightAprilTagVisionLane.Config.defaults();
        mutation.apply(cfg);
        try {
            AprilTagVisionLaneFactories.limelight(cfg);
            fail("Expected invalid Limelight config");
        } catch (RuntimeException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "FtcLimelightAprilTagVisionLane.Config." + field));
            if (!"null".equals(value)) {
                assertTrue(expected.getMessage(), expected.getMessage().contains(value));
            }
        }
    }

    private interface ConfigMutation {
        void apply(FtcLimelightAprilTagVisionLane.Config config);
    }

    private static final class NoopProcessor implements VisionProcessor {
        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            return null;
        }

        @Override
        public void onDrawFrame(
                Canvas canvas,
                int onscreenWidth,
                int onscreenHeight,
                float scaleBmpPxToCanvasPx,
                float scaleCanvasDensity,
                Object userContext
        ) {
        }
    }

    private static Class<?>[] signature(Class<?>... parameterTypes) {
        return parameterTypes;
    }

    private static void assertPublicConstructors(Class<?> type, Class<?>[] expectedSignature) {
        Constructor<?>[] constructors = type.getConstructors();
        assertEquals(type.getName(), 1, constructors.length);
        assertTrue(
                type.getName() + " constructor parameters",
                Arrays.equals(expectedSignature, constructors[0].getParameterTypes())
        );
    }

    private static void assertNoDeclaredMethod(Class<?> type, String name) {
        for (Method method : type.getDeclaredMethods()) {
            assertFalse(type.getName() + " still declares " + name, method.getName().equals(name));
        }
    }

    private static void assertConfigSurface(Class<?> configType) {
        assertTrue(configType.getName(), Modifier.isFinal(configType.getModifiers()));
        List<String> publicDeclaredMethods = new ArrayList<String>();
        for (Method method : configType.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicDeclaredMethods.add(method.getName());
            }
        }
        assertEquals(
                configType.getName(),
                Arrays.asList("copy", "defaults", "toString"),
                sorted(publicDeclaredMethods)
        );
    }

    private static List<String> sorted(List<String> values) {
        java.util.Collections.sort(values);
        return values;
    }

    private static void expectFailureContaining(
            Class<? extends RuntimeException> expectedType,
            String messageFragment,
            Runnable action
    ) {
        try {
            action.run();
            fail("Expected " + expectedType.getSimpleName());
        } catch (RuntimeException actual) {
            assertTrue("Unexpected failure: " + actual, expectedType.isInstance(actual));
            assertTrue("Unexpected message: " + actual.getMessage(),
                    actual.getMessage() != null
                            && actual.getMessage().contains(messageFragment));
        }
    }

    private static void expectFailureMessage(
            Class<? extends RuntimeException> expectedType,
            String expectedMessage,
            Runnable action
    ) {
        try {
            action.run();
            fail("Expected " + expectedType.getSimpleName());
        } catch (RuntimeException actual) {
            assertTrue("Unexpected failure: " + actual, expectedType.isInstance(actual));
            assertEquals(expectedMessage, actual.getMessage());
        }
    }
}
