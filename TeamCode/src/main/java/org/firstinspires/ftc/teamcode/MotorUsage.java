package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ServoMotor;

@TeleOp()
public class MotorUsage extends OpMode{
    ServoMotor board = new ServoMotor();
    @Override
    public void init(){
        board.init(hardwareMap);
    }   String clawPosition;

    @Override
    public void loop(){
        if(gamepad1.a){
            board.setServoPosition(1.0); // 1.0 means 100% open
            clawPosition = "closed";
        }
        else if (gamepad1.b) {
            board.setServoPosition(0.0);
            clawPosition = "open";

        }
        telemetry.addData("claw position", clawPosition);

        if (gamepad1.x){
            board.setMotorSpeed(1.0);//set's motor speed to 100%
        }
        else if (gamepad1.y) {
            board.setMotorSpeed(-1.0);//sets motor to move backwards
        }
        else {
            board.setMotorSpeed(0.0);
        }
    }
}
