package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

@Config
@TeleOp
public class testing extends LinearOpMode {
    CachedMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public static double frontLeftPower = 0; //motors
    public static double backLeftPower = 0;
    public static double frontRightPower = 0;
    public static double backRightPower = 0;
    public static double claw_pos = 0.7;
    public static double turret_pos = 0.5;
    public static double arm_pos = 0.5;
    public static double wrist_pos = 0.5;
    public static double height = 17; //height of the limeLight
    public static double angle = 45; //angle of the limeLight
    public static double alpha = 0.003; //for low pass filter
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Servo claw = hardwareMap.servo.get("claw");
        Servo turret = hardwareMap.servo.get("turret");
        Servo arm = hardwareMap.servo.get("arm");
        Servo wrist = hardwareMap.servo.get("wrist");
        frontLeftMotor=new CachedMotorEx(hardwareMap, "frontleft");
        frontRightMotor=new CachedMotorEx(hardwareMap, "frontright");
        backLeftMotor=new CachedMotorEx(hardwareMap, "backleft");
        backRightMotor=new CachedMotorEx(hardwareMap, "backright");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); //frames per sec
        limelight.start();
        limelight.pipelineSwitch(0); //blue samp

        waitForStart();
        double filtered_x = 0;
        double filtered_y = 0;
        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);
            claw.setPosition(claw_pos);
            turret.setPosition(turret_pos); //to turn the claw
            arm.setPosition(arm_pos);
            wrist.setPosition(wrist_pos);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)
                double distance_from_Samp_y = height*Math.tan(Math.toRadians(angle+ty)); //trig
                double distance_from_Samp_x = distance_from_Samp_y*Math.tan(Math.toRadians(tx));
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.addData("Y Value", height);
                telemetry.addData("Distance to Sample Y", distance_from_Samp_y);
                telemetry.addData("Distance to Sample X", distance_from_Samp_x);
                filtered_x = alpha*distance_from_Samp_x+(1-alpha)*filtered_x; //low pass filter
                filtered_y = alpha*distance_from_Samp_y+(1-alpha)*filtered_y; //low pass filter
                telemetry.addData("filtered_x", filtered_x);
                telemetry.addData("filtered_y", filtered_y);
                packet.fieldOverlay()
                        .setFill("blue") //set color
                        .fillCircle(distance_from_Samp_x, distance_from_Samp_y, 2);
                packet.fieldOverlay()
                        .setFill("red") //set color
                        .fillCircle(filtered_x, filtered_y, 2);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
