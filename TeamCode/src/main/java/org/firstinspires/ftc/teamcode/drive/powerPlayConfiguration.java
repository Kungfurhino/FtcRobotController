package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

public class powerPlayConfiguration {
        //Motor
        public static final String MOTOR_FR = "wheelFR";
        public static final String MOTOR_FL = "wheelFL";
        public static final String MOTOR_BR = "wheelBR";
        public static final String MOTOR_BL = "wheelBL";
        public DcMotorEx rightFront;
        public DcMotorEx leftFront;
        public DcMotorEx leftRear;
        public DcMotorEx rightRear;

        public static final String LEFT_VERTICAL_SLIDE = "leftVerticalSlide";
        public static final String RIGHT_VERTICAL_SLIDE = "rightVerticalSlide";
        public DcMotorEx leftVerticalSlide;
        public DcMotorEx rightVerticalSlide;

        public DcMotorEx intakeDrawerSlideRight;
        public DcMotorEx intakeDrawerSlideLeft;

        public Servo claw;
        public ServoImplEx leftPivot;
        public ServoImplEx rightPivot;
        public DistanceSensor distanceSensor;

        public Servo alignmentTool;

        HardwareMap hwMap           =  null;

        public BNO055IMU imu;

        public static powerPlayConfiguration newConfig(HardwareMap hardwareMap) {

            powerPlayConfiguration config = new powerPlayConfiguration();
            config.init(hardwareMap);
            return config;
        }

        protected void init(HardwareMap hardwareMap) {

            hwMap = hardwareMap;

            rightFront = hwMap.get(DcMotorEx.class, "rightFront");
            leftFront = hwMap.get(DcMotorEx.class, "leftFront");
            rightRear = hwMap.get(DcMotorEx.class, "rightRear");
            leftRear = hwMap.get(DcMotorEx.class, "leftRear");


            rightVerticalSlide = hwMap.get(DcMotorEx.class, "rightVerticalSlide");
            leftVerticalSlide = hwMap.get(DcMotorEx.class, "leftVerticalSlide");

            intakeDrawerSlideLeft = hwMap.get(DcMotorEx.class, "intakeDrawerSlideLeft");
            intakeDrawerSlideRight = hwMap.get(DcMotorEx.class, "intakeDrawerSlideRight");

            claw = hwMap.get(Servo.class, "claw");

            rightPivot = hwMap.get(ServoImplEx.class, "rightPivot");
            leftPivot = hwMap.get(ServoImplEx.class, "leftPivot");
            rightPivot.setPwmRange(new PwmControl.PwmRange(500, 2500));
            leftPivot.setPwmRange(new PwmControl.PwmRange(500, 2500));

            alignmentTool = hwMap.get(Servo.class, "alignmentPivot");

            distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

            imu = hardwareMap.get(BNO055IMU.class, "imu");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
            BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        }

        public void zeroMotors(){
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }

        public void zeroServos(){
        }
}
