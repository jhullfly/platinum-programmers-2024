package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
//@Disabled
public class SpinEachWheel extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double speed = 0.25;
        if(gamepad2.a){
            telemetry.addLine("this should be front left");
            frontLeft.setPower(speed);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        } else if (gamepad2.b){
            telemetry.addLine("this should be front right");
            frontLeft.setPower(0);
            frontRight.setPower(speed);
            backLeft.setPower(0);
            backRight.setPower(0);
        } else if (gamepad2.x){
            telemetry.addLine("this should be back left");
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(speed);
            backRight.setPower(0);
        } else if (gamepad2.y){
            telemetry.addLine("this should be back right");
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(speed);
        } else {
            telemetry.addLine("this should be OFF");
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        telemetry.update();
    }
}