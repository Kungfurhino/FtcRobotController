package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EndcoderTesting extends LinearOpMode {

    //2600 ticks for high goal
    enum State{
        IDLE,
        SLOW_SERVO_DOWN,
        SlOW_SERVO_UP,
        PULL_IN_SLIDES,
        RAISE_VERT,
        PULL_IN_VERT,
        DROP_CONE
    }

    private int verticalLiftTicks;
    private int drawerSlidesTicks;
    private int initialDrawerTicks = 0;
    private int initialVerticalTicks = 0;

    State currentState = State.IDLE;

    private long currentTime;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case SlOW_SERVO_UP:
                    double progress = System.currentTimeMillis() - currentTime;
                    double angleRight = map(progress, 0, 7000, 0.9, 0.2);
                    double angleLeft = map(progress, 0, 7000, 0.1, 0.8);
                    robot.config.rightPivot.setPosition(angleRight);
                    robot.config.leftPivot.setPosition(angleLeft);
                    if (progress >= 7000) {
                        currentState = State.IDLE;
                    }
                    break;
                case SLOW_SERVO_DOWN:
                    double timeProgress = System.currentTimeMillis() - currentTime;
                    double right = map(timeProgress, 0, 7000, 0.25, 0.9);
                    double left = map(timeProgress, 0, 7000, 0.75, 0.1);
                    robot.config.rightPivot.setPosition(right);
                    robot.config.leftPivot.setPosition(left);
                    if (timeProgress >= 7000) {
                        currentState = State.IDLE;
                    }
                    break;
                case PULL_IN_SLIDES:
                    robot.config.rightPivot.setPosition(0.5);
                    robot.config.leftPivot.setPosition(0.5);
                    robot.config.intakeDrawerSlideRight.setPower(-1);
                    robot.config.intakeDrawerSlideLeft.setPower(-1);
                    if(robot.config.intakeDrawerSlideLeft.getCurrentPosition() <= initialDrawerTicks){
                        initialDrawerTicks = robot.config.intakeDrawerSlideLeft.getCurrentPosition();
                        robot.config.rightPivot.setPosition(0.22);
                        robot.config.leftPivot.setPosition(0.78);
                        currentTime = System.currentTimeMillis();
                        currentState = State.DROP_CONE;
                    }
                    break;
                case DROP_CONE:
                    if(System.currentTimeMillis() - currentTime >= 1200){
                        robot.config.claw.setPosition(0.6);
                        if(System.currentTimeMillis() - currentTime >= 1500 ){
                            robot.config.rightPivot.setPosition(0.8);
                            robot.config.leftPivot.setPosition(0.2);
                            currentState = State.IDLE;
                        }
                    }
                    break;
                case RAISE_VERT:
                    robot.config.leftVerticalSlide.setPower(1);
                    robot.config.rightVerticalSlide.setPower(1);
                    if(robot.config.leftVerticalSlide.getCurrentPosition() >= 2350){//-2350
                        robot.config.leftVerticalSlide.setPower(-1);
                        robot.config.rightVerticalSlide.setPower(-1);
                        currentState = State.PULL_IN_VERT;
                    }
                    break;
                case PULL_IN_VERT:
                    if(robot.config.leftVerticalSlide.getCurrentPosition() <= 0){
                        initialVerticalTicks = robot.config.leftVerticalSlide.getCurrentPosition();
                        robot.config.leftVerticalSlide.setPower(0);
                        robot.config.rightVerticalSlide.setPower(0);
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    currentTime = System.currentTimeMillis();
                    drawerSlidesTicks = robot.config.intakeDrawerSlideLeft.getCurrentPosition();
                    verticalLiftTicks = robot.config.leftVerticalSlide.getCurrentPosition();
                    break;
            }
            if (-gamepad2.left_stick_y != 0) {
                robot.config.intakeDrawerSlideLeft.setPower(-gamepad2.left_stick_y);
                robot.config.intakeDrawerSlideRight.setPower(-gamepad2.left_stick_y);
            }else if(currentState != State.PULL_IN_SLIDES){
                robot.config.intakeDrawerSlideLeft.setPower(0);
                robot.config.intakeDrawerSlideRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                robot.config.leftVerticalSlide.setPower(0.7);
                robot.config.rightVerticalSlide.setPower(0.7);
            }else if (gamepad1.right_bumper){
                robot.config.leftVerticalSlide.setPower(-0.7);
                robot.config.rightVerticalSlide.setPower(-0.7);
            }else if(currentState != State.RAISE_VERT && currentState != State.PULL_IN_VERT){
                robot.config.leftVerticalSlide.setPower(0);
                robot.config.rightVerticalSlide.setPower(0);
            }
            if (gamepad2.dpad_left) { //up
                robot.config.rightPivot.setPosition(0.245);
                robot.config.leftPivot.setPosition(0.755);
            }
            if (gamepad2.dpad_right) { // down
                robot.config.rightPivot.setPosition(0.75);//8
                robot.config.leftPivot.setPosition(0.25);//2
            }
            if (gamepad2.b) {
                robot.config.claw.setPosition(0.6);
            }
            if (gamepad2.x) {
                robot.config.claw.setPosition(0.85);
            }
            if (gamepad2.right_bumper) {
                robot.config.claw.setPosition(0.7);
            }
            if (gamepad2.dpad_up) {
                currentState = State.PULL_IN_SLIDES;
            }
            //if(gamepad2.dpad_down){
            //    currentState = State.DROP_CONE;
            //}
            if(gamepad1.left_trigger != 0){
                currentState = State.RAISE_VERT;
            }
            if(gamepad2.left_bumper){
                currentState = State.IDLE;
            }
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            telemetry.addData("rightRear", robot.rightRear.getCurrentPosition());
            telemetry.addData("leftFront", robot.leftFront.getCurrentPosition());
            telemetry.addData("leftRear", robot.leftRear.getCurrentPosition());
            telemetry.addData("Right Vertical Slide", robot.config.rightVerticalSlide.getCurrentPosition());
            telemetry.addData("Left Vertical Slide", robot.config.leftVerticalSlide.getCurrentPosition());
            telemetry.addData("Intake Drawer slides", robot.config.intakeDrawerSlideRight.getCurrentPosition() + ", " + robot.config.intakeDrawerSlideLeft.getCurrentPosition());
            telemetry.addData("Initial Ticks", initialDrawerTicks);
            telemetry.update();
            robot.update();
        }
    }

    public double map(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
