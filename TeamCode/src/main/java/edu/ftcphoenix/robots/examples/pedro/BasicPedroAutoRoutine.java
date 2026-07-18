package edu.ftcphoenix.robots.examples.pedro;

import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;

/** Composes the basic route, success action, and timeout fallback with ordinary Task factories. */
public final class BasicPedroAutoRoutine {

    private static final double ROUTE_TIMEOUT_SEC = 4.0;
    private static final double COLLECT_DURATION_SEC = 0.50;

    private BasicPedroAutoRoutine() {
        // Robot-owned Task factory only.
    }

    /**
     * Builds one fresh routine.
     *
     * <p>Confirmed route completion starts collection. A follower or Task timeout restores the
     * mechanism's idle request. Cancellation-like route endings abort without either branch and
     * deliberately leave any unrelated prior request unchanged. The basic host enters at idle;
     * another caller must establish the entry state its policy requires.</p>
     */
    public static Task build(RouteFollower<PathChain> routes,
                             PathChain practiceRoute,
                             BasicPedroAutoMechanism mechanism) {
        Task followPracticeRoute = RouteTasks.follow(
                "BasicPracticeRoute",
                Objects.requireNonNull(routes, "routes"),
                Objects.requireNonNull(practiceRoute, "practiceRoute"),
                ROUTE_TIMEOUT_SEC
        );

        BasicPedroAutoMechanism requiredMechanism = Objects.requireNonNull(
                mechanism,
                "mechanism"
        );
        return Tasks.branchOnOutcome(
                followPracticeRoute,
                requiredMechanism.collectTask(COLLECT_DURATION_SEC),
                requiredMechanism.idleTask()
        );
    }
}
