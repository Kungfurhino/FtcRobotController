package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Teleop extends LinearOpMode {

    private static SampleMecanumDrive robot;
    private double positionLeft = 0.73;
    private double positionRight = 0.27;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        while(opModeIsActive()){
            robot.config.rightPivot.setPosition(positionRight);
            robot.config.leftPivot.setPosition(positionLeft);
            if(gamepad2.a){
                positionLeft -= 0.01;
                positionRight += 0.01;
            }else if(gamepad2.y){
                positionLeft += 0.01;
                positionRight -= 0.01;
            }
            sleep(100);
            telemetry.addData("Left position", positionLeft);
            telemetry.addData("Right position", positionRight);
            telemetry.update();
        }


    }



}
