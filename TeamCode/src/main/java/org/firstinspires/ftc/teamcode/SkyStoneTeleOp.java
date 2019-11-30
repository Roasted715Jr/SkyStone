package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SkyStone TeleOp", group = "SkyStone")
public class SkyStoneTeleOp extends GenericOpMode {
    private Hardware<SkyStoneTeleOp> robot = new Hardware<>(this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double x, y, r;

        waitForStart();

        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            robot.setMecanumMotorPowers(x, y, r);

            telemetry.addData("x", "%.5f", x);
            telemetry.addData("y", "%.5f", y);
            telemetry.addData("r", "%.5f", r);
            telemetry.update();

            if (gamepad2.right_trigger > 0.5)
                robot.clawServo.setPosition(0.95);
            else if (gamepad2.right_bumper)
                robot.clawServo.setPosition(0.25);

            if (gamepad2.left_trigger > 0.5)
                robot.armServo.setPosition(robot.armServo.getPosition() + 0.003);
            else if (gamepad2.left_bumper)
                robot.armServo.setPosition(robot.armServo.getPosition() - 0.003);

            if (gamepad2.a) {
                robot.rFoundationServo.setPosition(1);
                robot.lFoundationServo.setPosition(0);
            } else if (gamepad2.b) {
                robot.rFoundationServo.setPosition(0);
                robot.lFoundationServo.setPosition(1);
            }

        }
    }
}
