package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DrivingCrossaints.ARM_EXTEND_HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.WRIST_FOLDED_IN;
import static org.firstinspires.ftc.teamcode.DrivingCrossaints.ARM_SCORE_SAMPLE_IN_HIGH;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
@Autonomous(name="Basic Autonomous", group="Robot")
public class AutonomousCrossaints extends LinearOpMode {
    // Declare motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private DcMotor extend = null;

    // Constants for arm positions
    public static double STRAFE_TIME = 2.5;
    public static final double ARM_COLLAPSED_INTO_ROBOT = 0;
    public static final double ARM_COLLECT = 0;
    public static final double ARM_SCORE_SAMPLE_IN_HIGH = 1800; // Example position
    public static final double ARM_EXTEND_PICK_UP = -200; // Example position
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    private ElapsedTime runtime = new ElapsedTime();
    public double wristPosition = WRIST_FOLDED_IN;
    // Constants for motor power and timing
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    double extendPosition = 0;
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

        waitForStart();

        // Step 1: Strafe right for STRAFE_TIME seconds
        runtime.reset();
        strafe(0.25, STRAFE_TIME);
        armMotor.setTargetPosition((int)ARM_SCORE_SAMPLE_IN_HIGH);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(3000);
        runtime.reset();
        extend.setTargetPosition((int)ARM_EXTEND_HIGH_BASKET);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) extend).setVelocity(3000);
        while (runtime.seconds() < 4.0) {
            telemetry.addData("arm pos", armMotor.getCurrentPosition());
            telemetry.update();
        }
        sleep(10000);
        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 1.0) {
//            telemetry.addData("Path", "Moving forward: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//            armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
//            extendPosition = ARM_EXTEND_HIGH_BASKET;
//            armMotor.setTargetPosition((int) (armPosition));
//            ((DcMotorEx) armMotor).setVelocity(2100);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            extend.setTargetPosition((int) (extendPosition));
//            ((DcMotorEx) extend).setVelocity(1000);
//            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
////        // Step 2: Turn right for 1.3 seconds
//        turnRight(TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 1.3) {
//            telemetry.addData("Path", "Turning Right: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3: Drive forward for another 2 seconds
//        allMotorsPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 2.0) {
//            telemetry.addData("Path", "Moving forward: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    // Helper methods for robot movement
    private void drive(double power, double time) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Moving forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        // Arm motor power is not included here to avoid unintended movement
    }

    private void strafe(double power, double time) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Moving forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        // Arm motor power is not included here to avoid unintended movement
    }

    private void turnRight(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
    }


}
