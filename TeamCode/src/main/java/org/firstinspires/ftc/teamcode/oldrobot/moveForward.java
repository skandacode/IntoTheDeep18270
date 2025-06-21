package org.firstinspires.ftc.teamcode.oldrobot;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;

@Autonomous
@Config
public class moveForward extends LinearOpMode {
    //HardwareMap hwMap;
    MotorEx leftFront;
    MotorEx leftBack;
    MotorEx rightFront;
    MotorEx rightBack;

    @Override
    public void runOpMode() {
        leftFront=new MotorEx(hardwareMap, "frontleft");
        leftBack=new MotorEx(hardwareMap, "backleft");
        rightFront=new MotorEx(hardwareMap, "frontright");
        rightBack=new MotorEx(hardwareMap, "backright");


    waitForStart();
    while (opModeIsActive()) {
        leftFront.set(1);
        leftBack.set(-1);
        rightFront.set(1);
        rightBack.set(-1);

    }
}}