package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Android Studio", group = "Android Studio")
public class MecanumAndroidStudio extends GenericOpMode {
    private Hardware<MecanumAndroidStudio> robot = new Hardware<>(this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        double x, y, r;

        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            robot.setMecanumMotorPowers(x, y, r);

            telemetry.addData("x", "%.2f", x);
            telemetry.addData("y", "%.2f", y);
            telemetry.addData("r", "%.2f", r);
            telemetry.update();
        }
    }
}
