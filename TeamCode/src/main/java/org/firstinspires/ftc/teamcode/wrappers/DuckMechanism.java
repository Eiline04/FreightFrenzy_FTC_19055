package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckMechanism {
    private final DcMotorEx duckMotor;
    public volatile boolean running = false;

    public DuckMechanism(HardwareMap hardwareMap) {
        duckMotor = hardwareMap.get(DcMotorEx.class, "duck");
        duckMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        duckMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        duckMotor.setPower(0.0);
    }

    public void reverseMotor() {
        duckMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void startSpin() {
        duckMotor.setPower(0.7);
        running = true;
    }

    public void stopSpin() {
        duckMotor.setPower(0.0);
        running = false;
    }

    public void spinForMs(long ms) {
        if(ms <= 0) return;
        new Thread(() -> {
            startSpin();
            if(Thread.currentThread().isInterrupted()) return;
            try {
                Thread.sleep(ms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopSpin();
        }).start();
    }
}