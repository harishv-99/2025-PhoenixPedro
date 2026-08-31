package edu.ftcsushi.testfixtures.host;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/** Test fixture whose inherited member must not leak into another verifier package scope. */
public class CrossScopeRawMemberOwner {

    public abstract static class InheritedPublicRawOpModeFixture extends OpMode {
    }
}
