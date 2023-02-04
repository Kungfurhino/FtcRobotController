package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class EndcoderTesting extends LinearOpMode {

    //2600 ticks for high goal
    enum State{
        IDLE,
        SLOW_SERVO_DOWN,
        SlOW_SERVO_UP,
        PULL_IN_SLIDES,
        MEDIUM_CYCLE,
        MEDIUM_CYCLE_DOWN,
        GRAB_CONE,
        PULL_IN_SLOW,
        RAISE_VERT,
        PULL_IN_VERT,
        DROP_CONE,
        ARM_ONLY,
        PULL_DOWN_SLOW
    }

    private int verticalLiftTicks;
    private int drawerSlidesTicks;
    private int initialDrawerTicks = 0;
    private int alignNum = 0;
    private int initialVerticalTicks = 0;
    private double leftServo;
    private double rightServo;
    private boolean auto = false;
    private boolean slowDrive = false;

    State currentState = State.IDLE;

    private long currentTime;
   
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.intakeDrawerSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.config.leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case SlOW_SERVO_UP://use motor ticks for progress and go from 1-0.3
                    double progress = robot.config.leftVerticalSlide.getCurrentPosition();
                    double powerRight = map(progress, 0, 2300, 1.0, 0.2);
                    double powerLeft = map(progress, 0, 2300, 1.0, 0.2);
                    robot.config.rightPivot.setPosition(powerRight);
                    robot.config.leftPivot.setPosition(powerLeft);
                    if (progress >= 2300) {
                        robot.config.leftVerticalSlide.setPower(0);
                        robot.config.rightVerticalSlide.setPower(0);
                        currentState = State.SLOW_SERVO_DOWN;
                    }
                    break;
                case SLOW_SERVO_DOWN://use motor ticks for progress and go from 1-0.3
                    double timeProgress = robot.config.leftVerticalSlide.getCurrentPosition();
                    double right = map(timeProgress, 2300, 0, -1, 0.2);
                    double left = map(timeProgress, 2300, 0, -1, 0.2);
                    robot.config.rightPivot.setPosition(right);
                    robot.config.leftPivot.setPosition(left);
                    if (timeProgress >= 7000) {
                        currentState = State.IDLE;
                    }
                    break;
                case GRAB_CONE:
                    robot.config.intakeDrawerSlideRight.setPower(0.4);
                    robot.config.intakeDrawerSlideLeft.setPower(0.4);
                    if(robot.config.clawSensor.blue() >= 110 || robot.config.clawSensor.red() >= 110){
                        robot.closeClaw();
                        sleep(200);
                        robot.config.intakeDrawerSlideRight.setPower(0);
                        robot.config.intakeDrawerSlideLeft.setPower(0);
                        currentState = State.PULL_IN_SLIDES;
                    }
                    break;
                case PULL_IN_SLIDES:
                    if(robot.config.leftVerticalSlide.getCurrentPosition() > 60){
                        currentState = State.IDLE;
                    }else{
                        robot.config.rightPivot.setPosition(0.5);
                        robot.config.leftPivot.setPosition(0.5);
                        robot.config.intakeDrawerSlideRight.setPower(-0.7);
                        robot.config.intakeDrawerSlideLeft.setPower(-0.7);
                        if(robot.config.intakeDrawerSlideLeft.getCurrentPosition() <= 0){
                            robot.config.intakeDrawerSlideLeft.setPower(0);
                            robot.config.intakeDrawerSlideRight.setPower(0);
                            robot.armUp();
                            currentTime = System.currentTimeMillis();
                            currentState = State.DROP_CONE;
                        }
                    }
                    break;
                case PULL_IN_SLOW:
                    if(robot.config.intakeDrawerSlideLeft.getCurrentPosition() <= 0){
                        //initialDrawerTicks = robot.config.intakeDrawerSlideLeft.getCurrentPosition();
                        robot.armUp();
                        currentTime = System.currentTimeMillis();
                        currentState = State.DROP_CONE;
                    }
                    break;
                case ARM_ONLY:
                    robot.armUp();
                    currentTime = System.currentTimeMillis();
                    currentState = State.DROP_CONE;
                    break;
                case DROP_CONE:
                    if(System.currentTimeMillis() - currentTime >= 500){
                        robot.slightOpen();
                        if(auto){
                            if(System.currentTimeMillis() - currentTime >= 800){
                                //robot.config.rightPivot.setPosition(0.58);
                                //robot.config.leftPivot.setPosition(0.42);
                                auto = false;
                                currentState = State.IDLE;
                            }
                        }else if(System.currentTimeMillis() - currentTime >= 800 ){
                            robot.armDown();
                            if(System.currentTimeMillis() - currentTime >= 1050){
                                robot.openClaw();
                                currentState = State.IDLE;
                            }
                        }
                    }
                    break;
                case RAISE_VERT:
                    if(robot.config.magnet.isPressed()){
                        robot.config.leftVerticalSlide.setPower(0);
                        robot.config.rightVerticalSlide.setPower(0);
                        currentState = State.IDLE;
                    }else{
                        robot.config.leftVerticalSlide.setPower(1);
                        robot.config.rightVerticalSlide.setPower(1);
                    }
                    if(robot.config.leftVerticalSlide.getCurrentPosition() >= 2275){//-2350
                        currentTime = System.currentTimeMillis();
                        currentState = State.PULL_DOWN_SLOW;
                    }
                    break;
                case MEDIUM_CYCLE:
                    if(robot.config.magnet.isPressed()){
                        robot.config.leftVerticalSlide.setPower(0);
                        robot.config.rightVerticalSlide.setPower(0);
                        currentState = State.IDLE;
                    }else{
                        robot.config.leftVerticalSlide.setPower(1);
                        robot.config.rightVerticalSlide.setPower(1);
                    }
                    if(robot.config.leftVerticalSlide.getCurrentPosition() >= 1704){
                        robot.config.leftVerticalSlide.setPower(-1);
                        robot.config.rightVerticalSlide.setPower(-1);
                        currentState = State.MEDIUM_CYCLE_DOWN;
                    }
                    break;
                case MEDIUM_CYCLE_DOWN:
                    robot.config.leftVerticalSlide.setPower(-1);
                    robot.config.rightVerticalSlide.setPower(-1);
                    if(robot.config.leftVerticalSlide.getCurrentPosition() <= 0){
                        robot.config.leftVerticalSlide.setPower(0);
                        robot.config.rightVerticalSlide.setPower(0);
                        currentState = State.IDLE;
                    }
                    break;
                case PULL_DOWN_SLOW:
                    if(System.currentTimeMillis() - currentTime > 250){
                        if(robot.config.leftVerticalSlide.getCurrentPosition() >= 2200){
                            robot.config.leftVerticalSlide.setPower(-1);
                            robot.config.rightVerticalSlide.setPower(-1);
                            currentState = State.PULL_IN_VERT;
                        }
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
                    //drawerSlidesTicks = robot.config.intakeDrawerSlideLeft.getCurrentPosition();
                    verticalLiftTicks = robot.config.leftVerticalSlide.getCurrentPosition();
                    rightServo = robot.config.rightPivot.getPosition();
                    leftServo = robot.config.leftPivot.getPosition();
                    if(robot.config.leftVerticalSlide.getCurrentPosition() > 60 && robot.config.rightPivot.getPosition() > 0.6){
                        robot.ninetyDegreeArm();
                    }
                    if(robot.config.magnet.isPressed()){
                        robot.config.leftVerticalSlide.setPower(0);
                        robot.config.rightVerticalSlide.setPower(0);
                    }
                    break;
            }
            if (-gamepad2.left_stick_y != 0) {
                robot.config.intakeDrawerSlideLeft.setPower(-gamepad2.left_stick_y);
                robot.config.intakeDrawerSlideRight.setPower(-gamepad2.left_stick_y);
            }else if(currentState != State.PULL_IN_SLIDES && currentState != State.GRAB_CONE){
                robot.config.intakeDrawerSlideLeft.setPower(0);
                robot.config.intakeDrawerSlideRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                robot.config.leftVerticalSlide.setPower(0.7);
                robot.config.rightVerticalSlide.setPower(0.7);
            }else if (gamepad1.right_bumper){
                robot.config.leftVerticalSlide.setPower(-0.7);
                robot.config.rightVerticalSlide.setPower(-0.7);
            }else if(currentState != State.RAISE_VERT && currentState != State.PULL_IN_VERT && currentState != State.MEDIUM_CYCLE && currentState != State.MEDIUM_CYCLE_DOWN){
                robot.config.leftVerticalSlide.setPower(0);
                robot.config.rightVerticalSlide.setPower(0);
            }
            if (gamepad2.dpad_left) { //up
                robot.armUp();
            }
            if (gamepad2.dpad_right) { // down
                robot.armDown();
            }
            if (gamepad2.b) {
                robot.openClaw();
            }
            if (gamepad2.x) {
                robot.closeClaw();
            }
            if (gamepad2.right_bumper) {
                robot.slightOpen();
            }
            if (gamepad2.dpad_up) {
                currentState = State.PULL_IN_SLIDES;
            }
            if(gamepad2.a){
                auto = true;
            }
            if(gamepad2.y){
                robot.config.rightPivot.setPosition(0.58);
                robot.config.leftPivot.setPosition(0.42);
            }
            if(gamepad1.x){
                robot.ninetyDegreeArm();
            }
            if(gamepad2.left_trigger != 0){
                rightServo += 0.01;
                leftServo -= 0.01;
                robot.config.rightPivot.setPosition(rightServo); //set pivots on forward positions
                robot.config.leftPivot.setPosition(leftServo);
            }
            if(gamepad2.right_trigger != 0){
                rightServo -= 0.01;
                leftServo += 0.01;
                robot.config.rightPivot.setPosition(rightServo); //set pivots on forward positions
                robot.config.leftPivot.setPosition(leftServo);
            }
            if(gamepad1.left_trigger != 0){
                currentState = State.RAISE_VERT;
            }
            if(gamepad1.right_trigger != 0){
                currentState = State.MEDIUM_CYCLE;
            }
            if (gamepad2.dpad_down) {
                currentState = State.GRAB_CONE;
            }
            if(gamepad1.a){
                robot.config.alignmentTool.setPosition(0.5);
            }
            if(gamepad1.y){
                robot.config.alignmentTool.setPosition(0.85);
            }
            if(gamepad2.left_bumper){
                currentState = State.IDLE;
            }
            if(gamepad1.b){
                if(slowDrive){
                    slowDrive = false;
                }else{
                    slowDrive = true;
                }
            }
            if(slowDrive){
                mecanumDrive(robot, 0.3);
            }else{
                mecanumDrive(robot, 1);
            }
            telemetry.addData("front distance: ", robot.config.distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("back distance: ", robot.config.backSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("claw color: ", robot.config.clawSensor.red() + " blue: " + robot.config.clawSensor.blue() + " Distance: " + robot.config.clawSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("magnet: ", robot.config.magnet.isPressed());
            telemetry.addData("rightRear", robot.rightRear.getCurrentPosition());
            telemetry.addData("leftFront", robot.leftFront.getCurrentPosition());
            telemetry.addData("Rightfront", robot.rightFront.getCurrentPosition());
            telemetry.addData("Right Vertical Slide", robot.config.rightVerticalSlide.getCurrentPosition());
            telemetry.addData("Left Vertical Slide", robot.config.leftVerticalSlide.getCurrentPosition());
            telemetry.addData("Intake Drawer slides", robot.config.intakeDrawerSlideRight.getCurrentPosition() + ", " + robot.config.intakeDrawerSlideLeft.getCurrentPosition());
            telemetry.addData("Initial Ticks", initialDrawerTicks);
            telemetry.addData("right servo", rightServo + ", left servo: " + leftServo);
            telemetry.update();
            robot.update();
        }
    }

    public double map(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public void mecanumDrive(SampleMecanumDrive drive, double multiplier){
        double throttle = -gamepad1.right_stick_x;
        double direction = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;

        double FR = throttle + direction + strafe;
        double FL = -throttle + direction - strafe;
        double BR = throttle + direction - strafe;
        double BL = -throttle + direction + strafe;

        FR = Range.clip(FR, -1, 1);
        FL = Range.clip(FL, -1, 1);
        BR = Range.clip(BR, -1, 1);
        BL = Range.clip(BL, -1, 1);

        drive.setMotorPowers(FL * multiplier, BL * multiplier, BR * multiplier, FR * multiplier);
    }
}
