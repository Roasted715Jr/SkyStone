package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.GenericOpMode;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous(name = "Autonomous", group = "SkyStone")
@Disabled
public class Auton extends GenericOpMode {
    private Robot robot = new Robot(this);
    private AutonProcedures autonProcedures = new AutonProcedures();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this, true); //This calls init in the Robot class

//        Thread thread = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (!interrupted())
//                    autonProcedures.start();
//            }
//        });

        waitForStart();

//        thread.start();

//        autonProcedures.startWithStartSpot(2);
        autonProcedures.start();

        while (opModeIsActive()) {
//            if (isStopRequested()) {
////                autonProcedures.running = false; //For testing purposes
//            }

//            telemetry.update();
        }

//        thread.interrupt();
//        thread.stop();
        robot.stop();
    }
}
