package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class GenericOpMode extends LinearOpMode {
    public static final String GROUP_SKYSTONE = "SkyStone";
    public static final String GROUP_TESTINNG = "Testing";

    //For use in other classes during autonomous
    public void addTelemetry(Object msg) {
        telemetry.addData("Autonomous", msg);
    }

    public void addTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    public void addTelemetry(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
    }

    //For use in the Robot class during autonomous
    public void updateTelemetry() {
        telemetry.update();
    }

    public void waitForStart() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
    }

    public abstract void runOpMode() throws InterruptedException;
}
