package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class GenericOpMode extends LinearOpMode {
    //For use in other classes during autonomous
    void addTelemetry(String msg) {
        telemetry.addData("Autonomous", msg);
    }

    void addTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    void addTelemetry(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
    }

    //For use in the Hardware class during autonomous
    void updateTelemetry() {
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