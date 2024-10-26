package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
 public class Couch extends OpMode {
    @Override
    public void init() {
         String myName = "Ethan Jackson";

         telemetry.addData("Hello", myName);
         telemetry.addData("Right stick y", gamepad1.right_stick_y);

    }

    @Override
    public void loop() {
        double speedForward = gamepad1.left_stick_y;

        if (gamepad1.a) {
            speedForward = gamepad1.left_stick_y * 2.0;

        }
        telemetry.addData("left stick y", speedForward);
    }


}

