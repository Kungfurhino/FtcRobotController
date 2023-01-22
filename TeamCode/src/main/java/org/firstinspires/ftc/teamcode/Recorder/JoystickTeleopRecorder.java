package org.firstinspires.ftc.teamcode.Recorder;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

import androidx.annotation.RequiresApi;

@TeleOp
public class JoystickTeleopRecorder extends LinearOpMode {
    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        JoystickRecorder recorder = new JoystickRecorder("SlowBack" + ".txt", telemetry);

        waitForStart();
        recorder.start();
        while (!isStopRequested()) {
            double throttle = -gamepad1.right_stick_x;
            double direction = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;

            double FR = throttle + direction + strafe;
            double FL = -throttle + direction - strafe;
            double BR = throttle + direction - strafe;
            double BL = -throttle + direction + strafe;

            FR = Range.clip(FR, -0.7, 0.7);
            FL = Range.clip(FL, -0.7, 0.7);
            BR = Range.clip(BR, -0.7, 0.7);
            BL = Range.clip(BL, -0.7, 0.7);

            drive.setMotorPowers(FL, BL, BR, FR);
            recorder.collectSample(Arrays.asList(direction, strafe, throttle));

            if(gamepad1.a){
                recorder.stop();
                break;
            }
        }
    }
}