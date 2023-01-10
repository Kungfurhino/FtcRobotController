package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous
public class ThreadTesting extends LinearOpMode {

    SampleMecanumDrive robot;
    private ExecutorService service;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.rightVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.rightVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //service = Executors.newFixedThreadPool(4);
        robot.config.rightPivot.setPosition(0.22);
        robot.config.leftPivot.setPosition(0.78);
        waitForStart();

        whileLoopScore(0, robot);
        whileLoopScore(1, robot);
        whileLoopScore(2, robot);
        whileLoopScore(3, robot);
        whileLoopScore(4, robot);
    }

    public void whileLoopScore(int iter, SampleMecanumDrive drive) { //extend slides to get cone
        double angle;
        boolean horizontal = false;
        boolean horizontalFinal = false;
        boolean vertical = false;
        boolean height = true;

        int ticks = 2300;
        if (iter == 0) {
            angle = .37;
            ticks = 2250;
        } else if (iter == 1) {
            angle = .35;
            ticks = 2200;
        } else if (iter == 2) {
            angle = .33;
            ticks = 2150;
        } else if (iter == 3) {
            angle = .28;
            ticks = 2100;
        } else {
            angle = .25;
        }
        //pivot the claw

        drive.config.rightPivot.setPosition(1 - angle); //set pivots on forward positions
        drive.config.leftPivot.setPosition(angle);
        sleep(700);
        drive.config.claw.setPosition(0.4); //open claw



        //horizontal
        drive.config.intakeDrawerSlideLeft.setPower(1); //extend drawer slides out
        drive.config.intakeDrawerSlideRight.setPower(1);

        //vert
        drive.config.leftVerticalSlide.setPower(1);
        drive.config.rightVerticalSlide.setPower(1);
        while (true) {
            if ((horizontal && vertical)) {
                drive.config.leftVerticalSlide.setPower(0);
                drive.config.rightVerticalSlide.setPower(0);
                drive.config.intakeDrawerSlideLeft.setPower(0); //extend drawer slides out
                drive.config.intakeDrawerSlideRight.setPower(0);
                break;
            }
            if(gamepad2.left_bumper){
                break;
            }
            if (drive.config.intakeDrawerSlideLeft.getCurrentPosition() >= ticks || drive.config.intakeDrawerSlideRight.getCurrentPosition() >= ticks) {
                horizontalFinal = true;
                drive.config.intakeDrawerSlideLeft.setPower(0); //stop extending
                drive.config.intakeDrawerSlideRight.setPower(0);
                drive.config.claw.setPosition(0.85); //close the claw to grab cone
                sleep(500);
                drive.config.rightPivot.setPosition(0.4);
                drive.config.leftPivot.setPosition(0.6);
                sleep(500);
                drive.config.intakeDrawerSlideLeft.setPower(-1); //start retracting
                drive.config.intakeDrawerSlideRight.setPower(-1);
            }
            if (horizontalFinal && drive.config.intakeDrawerSlideLeft.getCurrentPosition() <= 0){
                robot.config.rightPivot.setPosition(0.24);
                robot.config.leftPivot.setPosition(0.76);
                sleep(400);
                robot.config.claw.setPosition(0.7);
                sleep(500);
                robot.config.rightPivot.setPosition(0.35);
                robot.config.leftPivot.setPosition(0.65);
                sleep(400);
                horizontal = true;
            }
            if (drive.config.leftVerticalSlide.getCurrentPosition() >= 2300 && height) {
                height = false;
                drive.config.leftVerticalSlide.setPower(-1);
                drive.config.rightVerticalSlide.setPower(-1);
            }
            if(!height && drive.config.leftVerticalSlide.getCurrentPosition() <= 0)
            {
                drive.config.leftVerticalSlide.setPower(0); //pull in
                drive.config.rightVerticalSlide.setPower(0);
                vertical = true;
            }
            telemetry.addData("boolean vars", vertical + ", " + horizontal + ", " + height);
            telemetry.addData("horizontal slides", "left slide" + drive.config.intakeDrawerSlideLeft.getCurrentPosition() + ", Right slide" + drive.config.intakeDrawerSlideRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
