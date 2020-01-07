package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Odometers Test", group = GenericOpMode.GROUP_TESTINNG)
@Disabled
public class OdometerTest extends GenericOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.blMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.lOdometer.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            robot.blMotor.setPower(1);
            telemetry.addData("lOdometer", robot.lOdometer.getCurrentPosition());
            telemetry.update();
        }

        robot.blMotor.setPower(0);
    }
}
