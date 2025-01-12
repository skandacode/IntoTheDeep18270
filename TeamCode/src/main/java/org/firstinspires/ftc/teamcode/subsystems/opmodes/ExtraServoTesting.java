package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ExtraServoTesting extends LinearOpMode {
    Servo extraServo;
    public static double servoPosition=0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        extraServo=hardwareMap.servo.get("extraServo");
        waitForStart();
        while (opModeIsActive()){
            extraServo.setPosition(servoPosition);
        }
    }
}
