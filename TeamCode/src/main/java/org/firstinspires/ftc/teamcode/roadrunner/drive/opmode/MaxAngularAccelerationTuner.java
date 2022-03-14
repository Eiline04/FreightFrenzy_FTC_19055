package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDriveImpl;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Autonomous(group = "drive")
public class MaxAngularAccelerationTuner extends LinearOpMode {
    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngAcc = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveImpl drive = new MecanumDriveImpl(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        double currentAngularVelocity = 0.0;
        double previousAngularVelocity = 0.0;

        ElapsedTime deltaTimer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            currentAngularVelocity = poseVelo.getHeading();
            double acc = (currentAngularVelocity - previousAngularVelocity) / deltaTimer.seconds();
            telemetry.addData("Acc", acc);
            telemetry.update();

            maxAngAcc = Math.max(acc, maxAngAcc);
            deltaTimer.reset();
            previousAngularVelocity = currentAngularVelocity;
        }

        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Angular Acceleration (rad)", maxAngAcc);
        telemetry.addData("Max Angular Acceleration (deg)", Math.toDegrees(maxAngAcc));
        telemetry.update();

        while (!isStopRequested()) idle();
    }
}
