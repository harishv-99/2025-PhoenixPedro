# AprilTag practice setups (outside a full game field)

Sushi supports AprilTag-based localization in two ways:

- **Current official game policy** via `FtcGameTagLayout.currentGameFieldFixed()`
- **Custom practice layout** (e.g. one printed tag taped to a wall)

Important distinction:

- the FTC `AprilTagLibrary` is detection metadata (IDs, tag size, FTC-known metadata)
- the Sushi `TagLayout` is the set of tags your solver is allowed to treat as fixed field landmarks

This note shows how to configure the framework for the custom case. For the full framework policy, see [`AprilTag Localization & Fixed Layouts`](<AprilTag Localization & Fixed Layouts.md>).

## 1) Print a tag and know its size

The most important detail for good pose accuracy is the **physical size** of the printed tag (edge length). The AprilTag processor uses this size to estimate distance.

A handy generator is Limelight's tool:

```text
https://tools.limelightvision.io/apriltag-generator
```

Make sure you measure or record the printed size (in inches or mm) and use that same size in your code.

## 2) Create a tag library (size + IDs)

Sushi's webcam AprilTag lane supports overriding the FTC `AprilTagLibrary`.

For a **single printed tag**, build a library like:

```java
import edu.ftcsushi.fw.ftc.FtcAprilTags;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

AprilTagLibrary lib = FtcAprilTags.singleTagLibrary(
        1,              // your printed tag ID
        "PracticeTag",  // optional name
        2.0             // tag size in inches (edge length)
);
```

Then pass it into the explicit webcam AprilTag owner config:

```java
FtcWebcamAprilTagVisionLane.Config visionCfg =
        FtcWebcamAprilTagVisionLane.Config.defaults();
visionCfg.webcamName = "Webcam 1";
visionCfg.tagLibrary = lib;

FtcWebcamAprilTagVisionLane vision =
        new FtcWebcamAprilTagVisionLane(hardwareMap, visionCfg);
```

Keep `vision` as the owner and call `vision.close()` during shutdown. Consumers borrow
`vision.tagSensor()`; they do not own the webcam lifecycle.

The config is an authoring draft. Its raw `copy()` isolates the config container, but deliberately
keeps a supplied SDK `AprilTagLibrary` as a borrowed reference. The active webcam owner validates
and deep-snapshots that library before it acquires hardware, converts tag size and field positions
to inches, and gives the processor its own private metadata. Mutating `lib`, its metadata array, a
position vector, or an orientation after owner construction therefore does not tune the running
camera. Construct a new owner to adopt a changed library. A selectable tester's
`AprilTagVisionLaneFactories.webcam(visionCfg)` performs the same validation and capture when the
factory is created, then gives every open a fresh private library snapshot.

A custom library must be nonempty. Tag IDs must be unique and non-negative; sizes must be finite
and positive; field positions must contain exactly three finite coordinates; and orientations
must be finite quaternions whose magnitude is within `1e-5` of `1.0`. These are software
metadata checks, not proof that the printed size or physical placement is correct.

## 3) Create a simple tag layout (field pose)

To get a full field-centric robot pose from a tag, the system needs to know where the tag is in your chosen **field frame**.

For a practice setup, you can choose a convenient frame yourself (for example: origin at the tag, or origin at the center of your practice area).

Example: a single tag at the origin, facing +X:

```java
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.field.SimpleTagLayout;

SimpleTagLayout layout = new SimpleTagLayout()
        .addPose(
                1,
                new Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        );
```

Tag IDs must be non-negative, and every authored position and orientation component must be finite.
`SimpleTagLayout` is a convenient mutable authoring surface: finish adding or replacing poses before
constructing the query or estimator that will use it. Retaining protected-core owners take an
immutable semantic snapshot; editing `layout` afterward is not live field-layout tuning. Construct
a new owner when the practice layout changes.

## 4) Use the layout/library with Sushi testers

Keep the two facts with their actual owners: put the trusted fixed-tag `layout` and localization
policy in the tester Config, while the webcam backend Config owns `lib` and the camera mount. Pass
the backend recipe separately as behavior:

```java
FtcWebcamAprilTagVisionLane.Config webcamTemplate =
        FtcWebcamAprilTagVisionLane.Config.defaults();
webcamTemplate.tagLibrary = lib;
webcamTemplate.cameraMount = solvedCameraMount;

Function<String, AprilTagVisionLaneFactory> webcamBuilder = selectedName -> {
    FtcWebcamAprilTagVisionLane.Config backend = webcamTemplate.copy();
    backend.webcamName = selectedName;
    return AprilTagVisionLaneFactories.webcam(backend);
};

AprilTagLocalizationTester.Config april =
        AprilTagLocalizationTester.Config.defaults();
april.fixedTagLayout = layout;
april.aprilTags = FtcOdometryAprilTagLocalizationLane
        .AprilTagLocalizationConfig.defaults();
new AprilTagLocalizationTester(april, webcamBuilder);
```

The same one-Config-plus-builder shape applies to all four AprilTag tools:

- `CameraMountCalibrator.Config.fixedTagLayout` supplies the calibration landmarks.
- `AprilTagLocalizationTester.Config` supplies the layout and mount-free age/solver policy.
- `PinpointAprilTagCorrectedLocalizationTester.Config` supplies the layout and complete corrected-
  localization policy.
- `PinpointPodOffsetCalibrator.Config` supplies the layout and the same mount-free AprilTag policy;
  passing a non-null builder enables its optional assist path.

None of those tool Configs has a second camera-mount or tag-library answer. A tester snapshots its
active Config and layout. The deferred builder may be applied again after a clean picker retry, so
keep `webcamTemplate` and the borrowed SDK `lib` stable for the tester's full lifetime. Each factory
`open(...)` still returns a fresh lane owner. An empty layout remains an honest empty snapshot: raw
detections may still be visible, but no fixed-layout calibration sample or AprilTag field correction
can be produced.


## 5) Related calibration docs

If you are using this practice setup as part of a full robot bring-up, also read:

- [`Robot Calibration Tutorials`](<../testing-calibration/Robot Calibration Tutorials.md>)
- [`Robot Calibration Tutorials → Camera mount`](<../testing-calibration/Robot Calibration Tutorials.md#camera-mount>)
- [`Robot Calibration Tutorials → AprilTag-only localization check`](<../testing-calibration/Robot Calibration Tutorials.md#apriltag-only-localization-check>)
