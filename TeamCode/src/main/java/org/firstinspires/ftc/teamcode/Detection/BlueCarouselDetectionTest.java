package org.firstinspires.ftc.teamcode.Detection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Lifter;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp(name ="Blue -Carousel- Detection", group = "Detection")
public class BlueCarouselDetectionTest extends LinearOpMode {

    OpenCvCamera webcam;
    BlueCarouselCameraThread cameraThread;
    Lifter.LEVEL result;

    @Override
    public void runOpMode() throws InterruptedException {
        initWebcam();
        sleep(1000);
        cameraThread = new BlueCarouselCameraThread(webcam);
        Thread cameraRunner = new Thread(cameraThread);
        cameraRunner.start();

        cameraThread.setBlueCarouselState(BlueCarouselCameraThread.CAMERA_STATE.INIT);
        sleep(1000);
        cameraThread.setBlueCarouselState(BlueCarouselCameraThread.CAMERA_STATE.STREAM);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Lifter.LEVEL result = BlueCarouselCameraThread.getBlueCarouselResult();
            telemetry.addData("Height", result);

            Rect detection = BlueCarouselCameraThread.detectionRect;
            telemetry.addData("Area", detection.area());
            telemetry.addData("Rectangle", detection.x + " " + detection.y);
            telemetry.update();
        }
    }

    public void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }
}
