package org.firstinspires.ftc.teamcode.recorder;

import android.os.Build;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import ii.SampleMecanumDrive;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import androidx.annotation.RequiresApi;

public class JoystickRecorder {

    private String filename;
    private Telemetry telemetry;

    private List<List<Double>> samples = new ArrayList<>();

    private int collectIntervalMillis = 5;

    public JoystickRecorder(String filename, Telemetry telemetry) {
        this.filename = filename;
        this.telemetry = telemetry;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void writeToFile(String text) {
        File file = new File(AppUtil.FIRST_FOLDER, filename);
        try (FileOutputStream outputStream = new FileOutputStream(file)) {
            outputStream.write(text.getBytes());
        } catch (IOException e) {
            throw new RuntimeException("Failed to write to file:" + filename, e);
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public String readFromFile() {
        StringBuilder content = new StringBuilder();
        File file = new File(AppUtil.FIRST_FOLDER, filename);
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String sCurrentLine;
            while ((sCurrentLine = br.readLine()) != null) {
                content.append(sCurrentLine).append(System.lineSeparator());
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to read from file:" + filename, e);
        }
        return content.toString();
    }

    public void start() {
        AppUtil.getInstance().showToast(UILocation.BOTH, "Start Recording to " + AppUtil.FIRST_FOLDER + filename);
        telemetry.addData("recording to : ", new File(AppUtil.FIRST_FOLDER, filename));
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void stop() {
        telemetry.addData("Writing file", "now");
        telemetry.update();
        StringBuilder content = new StringBuilder();
        for (int i = 0; i < samples.size(); i++) {
            content.append(i).append(System.lineSeparator());
            content.append(samples.get(i).get(0)).append(",").append(samples.get(i).get(1)).append(",").append(samples.get(i).get(2)).append(System.lineSeparator());
        }
        telemetry.addData("sample size: ", samples.size());
        telemetry.addData("content: ", content.toString());
        telemetry.update();
        writeToFile(content.toString());
    }

    public void collectSample(List<Double> sampleList) throws InterruptedException {
        samples.add(sampleList);
        Thread.sleep(collectIntervalMillis);
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void loadPlayback() {
        String content = readFromFile();
        telemetry.addData("content", content);
        telemetry.update();
        samples.clear();

        for (String line : content.split(System.lineSeparator())) {
            if (line.contains(",")) {
                // sample line
                String[] sampleText = line.split(",");
                List<Double> sampleList = new ArrayList<>();
                sampleList.add(Double.parseDouble(sampleText[0]));
                sampleList.add(Double.parseDouble(sampleText[1]));
                sampleList.add(Double.parseDouble(sampleText[2]));
                samples.add(sampleList);
            }
        }
        telemetry.addData("Playing back", "samples now " + samples.size());
        telemetry.update();
    }

    public void playSamples(SampleMecanumDrive drive) throws InterruptedException {
        for (int i = 0; i < samples.size(); i++) {
            double throttle = samples.get(i).get(2);
            double direction = samples.get(i).get(0);
            double strafe = samples.get(i).get(1);

            double FR = (throttle + direction + strafe);
            double FL = (-throttle + direction - strafe);
            double BR = (throttle + direction - strafe);
            double BL = (-throttle + direction + strafe);

            FR = Range.clip(FR, -1, 1);
            FL = Range.clip(FL, -1, 1);
            BR = Range.clip(BR, -1, 1);
            BL = Range.clip(BL, -1, 1);

            drive.setMotorPowers(FL, BL, BR, FR);
            Thread.sleep(collectIntervalMillis);
        }
    }

    public void playSamplesWithTicks(SampleMecanumDrive drive, int ticks) throws InterruptedException {
        if (ticks < drive.rightRear.getCurrentPosition()) {
            for (int i = 0; i < samples.size(); i++) {
                double throttle = samples.get(i).get(2);
                double direction = samples.get(i).get(0);
                double strafe = samples.get(i).get(1);

                double FR = (throttle + direction + strafe);
                double FL = (-throttle + direction - strafe);
                double BR = (throttle + direction - strafe);
                double BL = (-throttle + direction + strafe);

                FR = Range.clip(FR, -1, 1);
                FL = Range.clip(FL, -1, 1);
                BR = Range.clip(BR, -1, 1);
                BL = Range.clip(BL, -1, 1);

                drive.setMotorPowers(FL, BL, BR, FR);
                Thread.sleep(collectIntervalMillis);

                if (drive.rightRear.getCurrentPosition() < ticks) {
                    telemetry.addData("Current Position: ", drive.rightRear.getCurrentPosition());
                    drive.setMotorPowers(0, 0, 0, 0);
                    return;
                }
            }
        } else {
            for (int i = 0; i < samples.size(); i++) {
                double throttle = samples.get(i).get(2);
                double direction = samples.get(i).get(0);
                double strafe = samples.get(i).get(1);

                double FR = (throttle + direction + strafe);
                double FL = (-throttle + direction - strafe);
                double BR = (throttle + direction - strafe);
                double BL = (-throttle + direction + strafe);

                FR = Range.clip(FR, -1, 1);
                FL = Range.clip(FL, -1, 1);
                BR = Range.clip(BR, -1, 1);
                BL = Range.clip(BL, -1, 1);

                drive.setMotorPowers(FL, BL, BR, FR);
                Thread.sleep(collectIntervalMillis);

                if (drive.rightRear.getCurrentPosition() > ticks) {
                    telemetry.addData("Current Position: ", drive.rightRear.getCurrentPosition());
                    drive.setMotorPowers(0, 0, 0, 0);
                    return;
                }
            }
        }
    }
}