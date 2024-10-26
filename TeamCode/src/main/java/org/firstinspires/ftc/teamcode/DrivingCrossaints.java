package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@TeleOp
//@Disabled
public class DrivingCrossaints extends OpMode {

    /* Declare OpMode members. */
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo
    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)
    public DcMotor  extend    = null;

    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    public double ARM_COLLAPSED_INTO_ROBOT  = 0;
    public double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    public double ARM_CLEAR_BARRIER         = 220 * ARM_TICKS_PER_DEGREE;
    public double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    public double ARM_SCORE_SAMPLE_IN_HIGH  = 130 * ARM_TICKS_PER_DEGREE;
    public double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    public double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
    public double ARM_EXTEND_HIGH_BASKET    = -1931;
    public double ARM_EXTEND_PICK_UP    = -100;
    public double ARM_EXTEND_RESET = 0;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    public double INTAKE_COLLECT    = -1.0;
    public double INTAKE_OFF        =  0.0;
    public double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public double WRIST_FOLDED_IN   = 1;
    public double WRIST_FOLDED_OUT  = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    public double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    double extendPosition = 0;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */

        extend = hardwareMap.get(DcMotor.class, "extend"); //the arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm"); //the arm motor


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/



        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */

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

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

    }


    @Override
    public void loop() {

        if (gamepad1.a) {
            intake.setPower(INTAKE_COLLECT);
        }
        else if (gamepad1.x) {
            intake.setPower(INTAKE_OFF);
        }
        else if (gamepad1.b) {
            intake.setPower(INTAKE_DEPOSIT);
        }
        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

//      this on the gamepad 2(aka driver 2) makes it slower for more presion for the robot



        if(gamepad1.right_bumper) {
            armPosition = ARM_COLLECT;
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_COLLECT);
            extendPosition = ARM_EXTEND_PICK_UP;
        }

        else if (gamepad1.left_bumper){
            armPosition = ARM_CLEAR_BARRIER;
            extendPosition = ARM_EXTEND_RESET;
        }

        else if (gamepad1.y){
            armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
            extendPosition = ARM_EXTEND_HIGH_BASKET;
        }

        else if (gamepad1.dpad_left) { /* This turns off the intake, folds in the wrist, and moves the arm back to folded inside the robot. This is also the starting configuration */
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
            extendPosition = ARM_EXTEND_RESET;
        }

        else if (gamepad1.dpad_right){
            /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
            armPosition = ARM_SCORE_SPECIMEN;
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        else if (gamepad1.dpad_up){
            /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
            armPosition = ARM_ATTACH_HANGING_HOOK;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        else if (gamepad1.dpad_down){
            /* this moves the arm down to lift the robot up once it has been hooked */
            armPosition = ARM_WINCH_ROBOT;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
        armMotor.setTargetPosition((int) (armPosition  +armPositionFudgeFactor));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extend.setTargetPosition((int) (extendPosition));
        ((DcMotorEx) extend).setVelocity(1000);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */

        /* Check to see if our arm is over the current limit, and report via telemetry. */
        if (((DcMotorEx) armMotor).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }


        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());


        telemetry.addData("extendTarget: ", extend.getTargetPosition());
        telemetry.addData("extend Encoder: ", extend.getCurrentPosition());
        telemetry.update();

        double drive = -gamepad1.left_stick_y; // Forward/Backward
        double strafe = gamepad1.left_stick_x; // Left/Right
        double rotate = gamepad1.right_stick_x;




        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;
        if(gamepad2.b) {
            frontLeftPower = frontLeftPower*0.5;
            frontRightPower = frontRightPower*0.5;
            backLeftPower = frontRightPower*0.5;

        }
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}