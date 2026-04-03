# AprilTag practice setups (outside a full game field)

Phoenix supports AprilTag-based localization in two ways:

- **Current game layout** (FTC-provided tag IDs, sizes, and field poses)
- **Custom practice layout** (e.g. one printed tag taped to a wall)

This note shows how to configure the framework for the custom case.

## 1) Print a tag and know its size

The most important detail for good pose accuracy is the **physical size** of the printed tag (edge length). The AprilTag processor uses this size to estimate distance.

A handy generator is Limelight's tool:

```text
https://tools.limelightvision.io/apriltag-generator
```

Make sure you measure or record the printed size (in inches or mm) and use that same size in your code.

## 2) Create a tag library (size + IDs)

Phoenix's FTC vision wrapper supports overriding the FTC `AprilTagLibrary`.

For a **single printed tag**, build a library like:

```java
import edu.ftcphoenix.fw.ftc.FtcAprilTags;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

AprilTagLibrary lib = FtcAprilTags.singleTagLibrary(
        1,              // your printed tag ID
        "PracticeTag",  // optional name
        2.0             // tag size in inches (edge length)
);
```

Then pass it into `FtcVision.Config`:

```java
FtcVision.Config visionCfg = FtcVision.Config.defaults()
        .withTagLibrary(lib);
```

## 3) Create a simple tag layout (field pose)

To get a full field-centric robot pose from a tag, the system needs to know where the tag is in your chosen **field frame**.

For a practice setup, you can choose a convenient frame yourself (for example: origin at the tag, or origin at the center of your practice area).

Example: a single tag at the origin, facing +X:

```java
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.field.SimpleTagLayout;

SimpleTagLayout layout = new SimpleTagLayout()
        .addPose(
                1,
                new Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        );
```

## 4) Use the layout/library with Phoenix testers

The testers that use AprilTags now support:

- A **tag layout override** (e.g. `SimpleTagLayout`)
- A **tag library override** (e.g. `FtcAprilTags.singleTagLibrary(...)`)

That includes:

- `AprilTagLocalizationTester`
- `PinpointAprilTagFusionLocalizationTester`
- `CameraMountCalibrator`

If you want to run outside a full game field, pass your custom `layout` and `lib` into the tester's constructor/config.
