package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static java.lang.Thread.interrupted;

@Autonomous(name = "Autonomous", group = "SkyStone")
@Disabled
public class Auton extends GenericOpMode {
    Hardware<Auton> robot = new Hardware<>(this);
    AutonProcedures<Auton> autonProcedures = new AutonProcedures<>();

    @Override
    public void runOpMode() throws InterruptedException {
        autonProcedures.init(robot, hardwareMap, this); //This calls init in the Hardware class

//        Thread thread = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (!interrupted())
//                    autonProcedures.start();
//            }
//        });

        waitForStart();

//        thread.start();

        autonProcedures.start();

        while (opModeIsActive()) {
//            if (isStopRequested()) {
////                autonProcedures.running = false; //For testing purposes
//            }

//            telemetry.update();
        }

//        thread.interrupt();
//        thread.stop();
    }
}
