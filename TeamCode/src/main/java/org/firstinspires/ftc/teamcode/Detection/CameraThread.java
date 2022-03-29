package org.firstinspires.ftc.teamcode.Detection;

//import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.wrappers.Lifter;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Thread that handles the webcam init and streaming for RingPipeline.
 */

public class CameraThread implements Runnable {

    public static int THRESHOLD = 109;
    public static int BLUR_KERNEL_SIZE = 9;
    //LEFT 20 - 95
    //MIDDLE 150 - 260
    //RIGHT - otherwise
    //x axis

    public enum CAMERA_STATE {
        NULL,
        INIT,
        STREAM,
        DETECT,
        KILL
    }

    private final OpenCvCamera camera;
    private volatile CAMERA_STATE state;
    private boolean active;
    private static volatile boolean kill = false;

    public static volatile Rect detectionRect;

    public CameraThread(OpenCvCamera camera) {
        this.camera = camera;
        kill = false;
        state = CAMERA_STATE.NULL;
    }

    @Override
    public void run() {
        //Must add isInterrupted() since it crashes when OpMode is force stopped.
        while (!kill && !Thread.currentThread().isInterrupted()) {
            if (active) {
                if (state == CAMERA_STATE.INIT) {
                    try {
                        camera.openCameraDevice();
                    } catch (OpenCvCameraException e) {
                        e.printStackTrace();
                    }
                    camera.setPipeline(new TSEPipeline());
                }

                if (state == CAMERA_STATE.STREAM) {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                if (state == CAMERA_STATE.KILL) {
                    camera.closeCameraDevice();
                    killThread();
                }
                active = false;
            }
        }
    }

    private static void killThread() {
        kill = true;
    }

    public void setState(CAMERA_STATE state) {
        this.state = state;
        this.active = true;
    }

    public static Lifter.LEVEL getRedResult() {
        Rect resultRect = CameraThread.detectionRect;
        if(resultRect.area() < 1000) return Lifter.LEVEL.THIRD;
        double x = resultRect.x;
        if(x >= 15 && x <= 110) return Lifter.LEVEL.FIRST;
        else if(x >= 150 && x <= 250) return Lifter.LEVEL.SECOND;
        else return Lifter.LEVEL.THIRD;
    }

    public static Lifter.LEVEL getBlueResult() {
        Rect resultRect = CameraThread.detectionRect;
        if(resultRect.area() < 1000) return Lifter.LEVEL.FIRST;
        double x = resultRect.x;
        if(x >= 0 && x <= 80) return Lifter.LEVEL.SECOND;
        else if(x >= 80 && x <= 240) return Lifter.LEVEL.THIRD;
        else return Lifter.LEVEL.FIRST;
    }

    public boolean isKilled() {
        return kill;
    }

    /**
     * Advanced Stage-Switching OpenCV pipeline that detects clusters of blue pixels
     * and calculates their height. It detects anywhere in the frame.
     */
    public static class TSEPipeline extends OpenCvPipeline {

        Mat YCbCr = new Mat();
        Mat medianBlur = new Mat();
        Mat Cr = new Mat();
        Mat CrInv = new Mat();
        Mat thresholdMat = new Mat();
        Mat newmat = new Mat();
        MatOfPoint m = new MatOfPoint();
        Rect TSE = new Rect();
        private final List<MatOfPoint> contoursList = new ArrayList<>();
        private final List<Mat> splitMat = new ArrayList<>();
        Scalar scalar = new Scalar(0, 255, 0);

        double maxArea = 0, currentArea;
        int i;

//        Point p1 = new Point(5, 120);
//        Point p2 = new Point(319, 120);
//        Point p3 = new Point(5, 239);
//        Point p4 = new Point(319, 239);
//        Rect rectCrop = new Rect((int) p1.x, (int) p1.y, (int) (p4.x - p1.x + 1), (int) (p4.y - p1.y + 1));

        @Override
        public Mat processFrame(Mat input) {

            //Crop the image to fit
            //input = input.submat(rectCrop);

            //Convert to YCbCr
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //Apply the median blur effect
            Imgproc.medianBlur(YCbCr, medianBlur, BLUR_KERNEL_SIZE);

            //extract Cr channel and invert it
            Core.split(medianBlur, splitMat);
            Cr = splitMat.get(2);
            Core.bitwise_not(Cr, CrInv);

            //Threshold the image to isolate blue
            Imgproc.threshold(CrInv, thresholdMat, THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

            //Find the biggest white contour and place a rectangle on it
            Imgproc.findContours(thresholdMat, contoursList, newmat, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            //find the biggest blue spot
            TSE.set(null);
            if (contoursList.size() > 0) {
                maxArea = 0;
                for (i = 0; i < contoursList.size(); i++) {
                    m = contoursList.get(i);
                    currentArea = Imgproc.contourArea(m);
                    if (currentArea > maxArea) {
                        maxArea = currentArea;
                        TSE = Imgproc.boundingRect(m);
                    }
                }
            }
            detectionRect = TSE;

            Imgproc.rectangle(input, TSE, scalar, 5);

            for (MatOfPoint m : contoursList) {
                m.release();
            }
            contoursList.clear();

            //Clearing the vector should release the mats. Causes a memory leak otherwise.
            for (Mat m : splitMat) {
                m.release();
            }
            splitMat.clear();

            return input;
        }
    }
}