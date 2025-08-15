package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DrivingCrossaints.ARM_EXTEND_HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.ARM_EXTEND_PICK_UP;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.ARM_SCORE_SAMPLE_IN_HIGH;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.ARM_SPEED_DOWN;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.ARM_SPEED_UP;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.EXTEND_SPEED;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.INTAKE_COLLECT;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.INTAKE_DEPOSIT;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.WRIST_FOLDED_IN;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.WRIST_FOLDED_OUT;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.ANGLE_GO_SLOW;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.ANGLE_MAX_ERROR;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.MINIMUM_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.RANGE_GO_SLOW;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.RANGE_MAX_ERROR;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.RobotAutoDriveToAprilTagOmni.TURN_GAIN;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name="Autonomous Specimen+Sample", group="Robot")
public class AutonomousSpecimenSample extends PlatinumBase {
    static public double STRAFE_TIME_FIRST = 800;
    static public double STRAFE_TIME_SECOND = 800;
    static public double STRAFE_POWER = 0.4;
    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotor.class, "arm"); // Update with actual name
        extend = hardwareMap.get(DcMotor.class, "extend"); // Update with actual name
        // Set zero power behavior for arm and extension motors
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */

        wrist.setPosition(WRIST_FOLDED_IN);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) extend).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        extend.setTargetPosition(0);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor directions (Reverse motors on one side)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        initAprilTag();
        setManualExposure(WEBCAM_EXPOSURE, WEBCAM_GAIN);  // Use low exposure time to reduce motion blur

        waitForStart();
        intake.setPower(INTAKE_COLLECT);
        //extend and lift the arm

        armMotor.setTargetPosition(ARM_SPECIMEN_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(ARM_SPEED_UP);
        waitForMotorsToFinish(FIRST_ARM_LIFT_WAIT_TIME);
        wrist.setPosition(WRIST_FOLDED_OUT);
        betterSleep(WRIST_FOLD_OUT_WAIT_TIME);
        extend.setTargetPosition(EXTEND_SPECIMEN_POSITION);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) extend).setVelocity(EXTEND_SPEED);
        drive(FORWARD_SPEED, FORWARD_TIME);
        extend.setTargetPosition(0);
        betterSleep(.300);
        armMotor.setTargetPosition(0);
        ((DcMotorEx) armMotor).setVelocity(ARM_SPEED_DOWN);
        betterSleep(.500);
        turnRightToAprilTag(0.5);
        driveToTag(DESIRED_DISTANCE1, 9.0);
        driveFromAprilTagToSamplePickupDepositAndGoBack(STRAFE_TIME_FIRST,STRAFE_POWER);
        driveToTag(DESIRED_DISTANCE3, 2.0);
        driveFromAprilTagToSamplePickupDepositAndGoBack(STRAFE_TIME_SECOND,STRAFE_POWER);
        intake.setPower(INTAKE_OFF);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }



}