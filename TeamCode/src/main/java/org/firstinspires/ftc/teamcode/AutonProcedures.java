package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class AutonProcedures<T extends GenericOpMode> {
    private Hardware robot;
    private HardwareMap hardwareMap;
    private T runningOpMode;
    private VuforiaLocalizer vuforia;

    void init(Hardware robot, HardwareMap hardwareMap, T runningOpMode) {
        this.robot = robot;
        this.hardwareMap = hardwareMap;
        this.runningOpMode = runningOpMode;

        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = Hardware.VUFORIA_LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();
    }

    void start() {
        runningOpMode.addTelemetry("This is the first step");
        runningOpMode.updateTelemetry();
        //func();
        runningOpMode.addTelemetry("Done");
        runningOpMode.updateTelemetry();
    }


}
