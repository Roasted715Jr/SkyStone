package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Android Studio", group = "Android Studio")
public class MecanumAndroidStudio extends GenericOpMode {
    private Hardware<MecanumAndroidStudio> robot = new Hardware<>(this);

    private final double MOTOR_MULT = 1;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double x, y, r;

        waitForStart();

        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            robot.setMecanumMotorPowers(x * MOTOR_MULT, y * MOTOR_MULT, r * MOTOR_MULT);

            telemetry.addData("x", "%.5f", x);
            telemetry.addData("y", "%.5f", y);
            telemetry.addData("r", "%.5f", r);
            telemetry.update();
        }
    }
}
