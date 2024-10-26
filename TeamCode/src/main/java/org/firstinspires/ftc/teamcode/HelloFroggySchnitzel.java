package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp()
public class HelloFroggySchnitzel extends OpMode{

    @Override
    public void init() {
        telemetry.addData("Hello", "FroggySchnitzel");
    } //output for package

    @Override
    public void loop() {

    }
}
