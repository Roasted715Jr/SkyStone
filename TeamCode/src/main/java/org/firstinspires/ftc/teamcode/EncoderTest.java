package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Encoder Test", group = GenericOpMode.GROUP_TESTINNG)
public class EncoderTest extends GenericOpMode {
    Robot<EncoderTest> robot = new Robot<>(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("fl", robot.flMotor.getCurrentPosition());
            telemetry.addData("bl", robot.blMotor.getCurrentPosition());
            telemetry.addData("fr", robot.frMotor.getCurrentPosition());
            telemetry.addData("br", robot.brMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
