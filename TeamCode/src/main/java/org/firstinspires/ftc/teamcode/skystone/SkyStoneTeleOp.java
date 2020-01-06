package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "SkyStone TeleOp", group = GenericOpMode.GROUP_SKYSTONE)
public class SkyStoneTeleOp extends GenericOpMode {
    private Robot<SkyStoneTeleOp> robot = new Robot<>(this);

    public void runOpMode() throws InterruptedException {
        boolean aPressed = false, multiplierToggle = false;
//        double speedMultiplier = 1, currentPos, targetPos = 0;
        double speedMultiplier, currentPos, targetPos = 0;

        robot.init(hardwareMap);

        double x, y, r;

        waitForStart();

        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            if (gamepad1.a) {
                if (!aPressed)
                    multiplierToggle = !multiplierToggle;
                aPressed = true;
            } else
                aPressed = false;

//            speedMultiplier = multiplierToggle ? 0.25 : 1;
            speedMultiplier = multiplierToggle ? 0.25 : 0.75;

            robot.setMecanumMotorPowers(x * speedMultiplier, y * speedMultiplier, r * speedMultiplier);

            if (gamepad2.right_trigger > 0.5)
                robot.clawServo.setPosition(0.95);
            else if (gamepad2.right_bumper)
                robot.clawServo.setPosition(0.25);

            currentPos = robot.armServo.getPosition();
//            if (gamepad2.left_trigger > 0.25)
//                robot.armServo.setPosition(currentPos + 0.003);
//            else if (gamepad2.left_bumper)
//                robot.armServo.setPosition(currentPos - 0.003);
            if (gamepad2.dpad_up)
                robot.armServo.setPosition(currentPos + 0.003);
            else if (gamepad2.dpad_down)
                robot.armServo.setPosition(currentPos - 0.003);

            if (gamepad2.a)
                robot.moveHooks(true);
            else if (gamepad2.b)
                robot.moveHooks(false);

            telemetry.addData("speedMultiplier", speedMultiplier);
            telemetry.addData("x", "%.5f", x);
            telemetry.addData("y", "%.5f", y);
            telemetry.addData("r", "%.5f", r);
            telemetry.update();
        }
    }
}
