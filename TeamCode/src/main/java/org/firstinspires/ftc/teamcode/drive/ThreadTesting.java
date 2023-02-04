package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        waitForStart();

        while(true){
            if(gamepad1.a){
                sensorGrabCone(robot);
            }
            if(gamepad1.b){
                break;
            }
        }
    }

    public void sensorGrabCone(SampleMecanumDrive drive){
        drive.config.claw.setPosition(0);
        while(drive.config.clawSensor.red() <= 60){
            drive.setMotorPowers(0.19, 0.19, -0.19, -0.19);
        }
        drive.closeClaw();
        drive.stop();
    }

    public void whileLoopScore(int iter, SampleMecanumDrive drive) { //extend slides to get cone
        double angleLeft = 0;
        double angleRight = 0;
        boolean horizontal = false;
        boolean horizontalFinal = false;
        boolean vertical = false;
        boolean height = true;

        int ticks = 2300;
        if (iter == 0) {
            angleLeft = 0.35;
            angleRight = 0.65;
            ticks = 2057;
        } else if (iter == 1) {
            angleLeft = 0.32;
            angleRight = 0.680;
            ticks = 1940;
        } else if (iter == 2) {
            angleLeft = 0.3;
            angleRight = 0.7;
            ticks = 1900;
        } else if (iter == 3) {
            angleLeft = .247;
            angleRight = 0.6988;
            ticks = 1830;
        } else {
            angleLeft = .238;
            angleRight = 0.709;
            ticks = 1800;
        }
        //pivot the claw

        drive.config.rightPivot.setPosition(angleRight); //set pivots on forward positions
        drive.config.leftPivot.setPosition(angleLeft);
        sleep(700);
        drive.config.claw.setPosition(0.6); //open claw



        //horizontal
        drive.config.intakeDrawerSlideLeft.setPower(0.4); //extend drawer slides out
        drive.config.intakeDrawerSlideRight.setPower(0.4);

        //vert
        drive.config.leftVerticalSlide.setPower(0.7);
        drive.config.rightVerticalSlide.setPower(0.7);
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
                drive.config.claw.setPosition(0.89); //close the claw to grab cone
                sleep(700);
                drive.config.rightPivot.setPosition(0.6);
                drive.config.leftPivot.setPosition(0.4);
                sleep(300);
                drive.config.intakeDrawerSlideLeft.setPower(-0.4); //start retracting
                drive.config.intakeDrawerSlideRight.setPower(-0.4);
            }
            if (horizontalFinal && drive.config.intakeDrawerSlideLeft.getCurrentPosition() <= 0){
                drive.config.rightPivot.setPosition(0.276);
                drive.config.leftPivot.setPosition(0.666);
                sleep(400);
                drive.config.claw.setPosition(0.75);
                sleep(500);
                drive.config.rightPivot.setPosition(0.5);
                drive.config.leftPivot.setPosition(0.5);
                sleep(400);
                horizontal = true;
            }
            if (drive.config.leftVerticalSlide.getCurrentPosition() >= 2600 && height) {
                height = false;
                drive.config.leftVerticalSlide.setPower(-0.7);
                drive.config.rightVerticalSlide.setPower(-0.7);
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
