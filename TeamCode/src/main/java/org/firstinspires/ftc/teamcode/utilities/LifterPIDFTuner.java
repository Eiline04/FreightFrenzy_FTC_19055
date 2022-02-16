package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.Lifter;

@TeleOp
public class LifterPIDFTuner extends LinearOpMode {

    Lifter lifter;
    ControllerInput controller1;

    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new ControllerInput(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lifter = new Lifter(hardwareMap, telemetry);


        telemetry.addLine();
        telemetry.addLine("Press Play when ready");
        telemetry.update();

        waitForStart();

        telemetryThread threadObj = new telemetryThread();
        Thread thread1 = new Thread(threadObj);
        thread1.start();

        while (opModeIsActive()) {
            controller1.update();

            if (controller1.AOnce()) {
                //go up
                lifter.goToPosition(0, 35000);
            }

            if (controller1.BOnce()) {
                //go down
                lifter.goToPosition(0, 0);
            }
        }
    }

    class telemetryThread implements Runnable {
        @Override
        public void run() {
            telemetry.log().clear();
            while (opModeIsActive()) {
                lifter.update();
                telemetry.addData("position", lifter.getLifterPosition());
                telemetry.update();
            }
        }
    }
}
