package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Teleop extends LinearOpMode {

    enum State{
        PULL_IN,
        PIVOT1,
        FLIP_SERVO,
        PIVOT2,
        IDLE,

        ROLLER_ON,
        LIFT_UP,
        ROLLER_OFF,
        LIFT_DOWN,
    }
    private double pivotConstant = 0.0;
    private int pivotTicks;
    private long time;
    private boolean liftDefault;
    private static SampleMecanumDrive robot;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);
        robot.config.clawLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.clawLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        if(isStopRequested()){
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {
                switch(currentState){
                    case PIVOT1:
                        robot.config.clawLift.setPower(0.55);
                        if(pivotTicks - robot.config.clawLift.getCurrentPosition() < 150){
                            currentState = State.FLIP_SERVO;
                            robot.config.clawLift.setPower(0.1);
                        }
                        break;
                    case FLIP_SERVO:
                        robot.config.clawPivot.setPosition(1);
                        currentState = State.PIVOT2;
                        robot.config.clawLift.setPower(0.45);
                        break;
                    case PIVOT2:
                        if(pivotTicks - robot.config.clawLift.getCurrentPosition() < 0){
                            currentState = State.IDLE;
                            robot.config.clawLift.setPower(0);
                        }
                        break;
                    case IDLE:
                        //finished with all
                        time = System.currentTimeMillis();
                        pivotTicks = robot.config.clawLift.getCurrentPosition();
                        break;
                }

            //Do while the automated program runs
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepad1.left_bumper){
                currentState = State.PIVOT1;
            }else if(gamepad1.left_trigger != 0){
                currentState = State.IDLE;
            }

            if(gamepad1.dpad_down){
                robot.config.intakeDrawerSlide.setPower(-1);
            } else if(gamepad1.dpad_up){
                robot.config.intakeDrawerSlide.setPower(1);
            } else if(!gamepad1.dpad_down && !gamepad1.dpad_up){
                robot.config.intakeDrawerSlide.setPower(0);
            }

            if(gamepad1.dpad_left){
                robot.config.clawLift.setPower(-0.5);
            } else if(gamepad1.dpad_right){
                robot.config.clawLift.setPower(0.5);
            } else if(!gamepad1.dpad_left && !gamepad1.dpad_right && currentState == State.IDLE){
                robot.config.clawLift.setPower(0);
            }

            if(gamepad1.a){
                robot.config.clawPivot.setPosition(1);
            }else if(gamepad1.y){
                robot.config.clawPivot.setPosition(0.4);
            }else if(gamepad1.x){
                robot.config.rightClaw.setPosition(0.9);
            }else if(gamepad1.b){
                robot.config.rightClaw.setPosition(0.35);
            }else if(gamepad1.right_bumper){
                robot.config.rightClaw.setPosition(0.8);
            }

            if (-gamepad2.left_stick_y != 0) {
                robot.config.rightVerticalSlide.setPower(-gamepad2.left_stick_y);
                robot.config.leftVerticalSlide.setPower(gamepad2.left_stick_y);
            }else if(liftDefault){
                robot.config.rightVerticalSlide.setPower(0.05);
                robot.config.leftVerticalSlide.setPower(-0.05);
            }else {
                robot.config.rightVerticalSlide.setPower(0);
                robot.config.leftVerticalSlide.setPower(0);
            }

            if(gamepad2.a){
                robot.config.horizontalSlide.setPower(-1);
            }else if(gamepad2.y){
                robot.config.horizontalSlide.setPower(1);
            }else if(!gamepad2.a && !gamepad2.y){
                robot.config.horizontalSlide.setPower(0);
            }

            if(gamepad2.left_bumper){
                robot.config.rightRoller.setPower(-1);
                robot.config.leftRoller.setPower(1);
            }else if(gamepad2.right_bumper){
                robot.config.rightRoller.setPower(1);
                robot.config.leftRoller.setPower(-1);
            }else if(!gamepad2.left_bumper && !gamepad2.right_bumper){
                robot.config.rightRoller.setPower(0);
                robot.config.leftRoller.setPower(0);
            }

            if(gamepad2.b){
                liftDefault = true;
            }else if(gamepad2.x){
                liftDefault = false;
            }



            telemetry.addData("State", currentState);
            telemetry.addData("Pivot ticks", robot.config.clawLift.getCurrentPosition());
            telemetry.addData("pivot difference", pivotTicks - robot.config.clawLift.getCurrentPosition());
            telemetry.addData("Left Front", robot.leftFront.getCurrentPosition());
            telemetry.addData("right Rear", robot.rightRear.getCurrentPosition());
            telemetry.update();
        }

    }

}
