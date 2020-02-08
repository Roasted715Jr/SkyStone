package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "SkyStone TeleOp", group = GenericOpMode.GROUP_SKYSTONE)
public class SkyStoneTeleOp extends GenericOpMode {
    private static final double SPEED_MAX = 0.75;
    private static final double SPEED_MIN = 0.25;
    private static final double ARM_SPEED = 0.5;

    private Robot robot = new Robot(this);

    public void runOpMode() throws InterruptedException {
        boolean aPressed = false, multiplierToggle = false;
//        double speedMultiplier = 1, currentPos, targetPos = 0;
        double speedMultiplier;
        int currentPos, previousPos;
//        int targetPos;
//        boolean isMoving = false;

        robot.init(hardwareMap);

        double x, y, r;

        waitForStart();

        robot.retractOdometers();

//        targetPos = robot.armMotor.getCurrentPosition();
        previousPos = robot.armMotor.getCurrentPosition();

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

            speedMultiplier = multiplierToggle ? SPEED_MIN : SPEED_MAX;

            robot.setMecanumMotorPowers(x * speedMultiplier, y * speedMultiplier, r * speedMultiplier);

            if (gamepad2.right_trigger > 0.5)
                robot.clawServo.setPosition(0.95);
            else if (gamepad2.right_bumper)
                robot.clawServo.setPosition(0.25);


//            targetPos = robot.armMotor.getCurrentPosition();
            double y2 = -gamepad2.left_stick_y;
//            if (y2 > 0.1 && robot.armMotor.getCurrentPosition() <= ARM_MAX_POS)
//                robot.armMotor.setPower(y2 * ARM_SPEED);
//            else if (y2 < -0.1 && robot.armMotor.getCurrentPosition() >= 0)
//                robot.armMotor.setPower(y2 * ARM_SPEED);
//            else
//                robot.armMotor.setPower(0);

//            robot.armMotor.setTargetPosition(100);


            //Working increment code
//            if (y2 > 0.1 && targetPos <= ARM_MAX_POS - ARM_INCREMENT)
//                targetPos += ARM_INCREMENT * y2;
//            else if (y2 < -0.1 && targetPos >= ARM_INCREMENT)
//                targetPos += ARM_INCREMENT * y2; //y2 is negative, so we add instead of subtract
//
//            robot.armMotor.setTargetPosition(targetPos);
//            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.armMotor.setPower(ARM_SPEED);

            currentPos = robot.armMotor.getCurrentPosition(); //Since we reverse the motor, the encoder will be negative as well
            if (Math.abs(y2) > 0.1) {
                robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.armMotor.setPower(-y2 * (y2 > 0 ? 0.5 : 1) * ARM_SPEED);
                previousPos = currentPos;
            } else {
//                robot.armMotor.setTargetPosition(previousPos);
//                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.armMotor.setPower(1);

                robot.moveArmMotor(previousPos);
            }

            if (gamepad2.a)
                robot.moveHooks(true);
            else if (gamepad2.b)
                robot.moveHooks(false);

            telemetry.addData("speedMultiplier", speedMultiplier);
            telemetry.addData("x", "%.5f", x);
            telemetry.addData("y", "%.5f", y);
            telemetry.addData("r", "%.5f", r);
            telemetry.addData("y2", "%.5f", y2);
            telemetry.addData("armMotor Position", robot.armMotor.getCurrentPosition());
            telemetry.addData("armMotor Previous Position", previousPos);
//            telemetry.addData("armMotor Target Position", robot.armMotor.getTargetPosition());
            telemetry.update();
        }
    }
}
