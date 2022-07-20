package org.firstinspires.ftc.teamcode.Recorder;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import androidx.annotation.RequiresApi;

//Simple op mode to test your recorded movements
@TeleOp
public class Playback extends LinearOpMode {

    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Make sure to change to file name to the path you want to test
        JoystickRecorder recorder = new JoystickRecorder("SlowBack" + ".txt", telemetry);

        waitForStart();

        recorder.loadPlayback();
        recorder.playSamples(drive);
    }
}