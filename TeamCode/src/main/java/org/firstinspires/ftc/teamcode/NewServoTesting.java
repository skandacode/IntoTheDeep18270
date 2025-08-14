package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class NewServoTesting extends LinearOpMode {
    IntakeCopy intake;
    private NewServoMP profiledServo;
    public static double maxVel = 1.0;
    public static double maxAccel = 2.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intake = new IntakeCopy(hardwareMap, telemetry);
        Servo myServo = hardwareMap.get(Servo.class, "arm");
        TelemetryPacket packet = new TelemetryPacket();
        // Create motion profiled servo
        // Parameters: maxVelocity = 2.0 pos/sec, maxAcceleration = 4.0 pos/sec²

        profiledServo = new NewServoMP(myServo, maxVel, maxAccel);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                profiledServo.setTargetPosition(0.0);
            } else if (gamepad1.b) {
                profiledServo.setTargetPosition(1.0);
            } else if (gamepad1.x) {
                profiledServo.setTargetPosition(0.5);
            }


            // Update the motion profile (call every loop!)
            profiledServo.update();

            // Telemetry
            telemetry.addData("Current Position", "%.3f", profiledServo.getCurrentPosition());
            telemetry.addData("Target Position", "%.3f", profiledServo.getTargetPosition());
            telemetry.addData("Profile Active", profiledServo.isProfileActive());
            telemetry.addData("Error", "%.3f", profiledServo.getError());
            telemetry.update();
            packet.put("current position", profiledServo.getCurrentPosition());
            packet.put("target position", profiledServo.getTargetPosition());
            packet.put("current velocity", profiledServo.getCurrentVel());
            packet.put("error", profiledServo.getError());


            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
