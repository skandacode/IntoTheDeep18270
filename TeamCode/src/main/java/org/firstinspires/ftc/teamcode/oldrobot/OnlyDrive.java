package org.firstinspires.ftc.teamcode.oldrobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldrobot.subsystems.MecanumDrivetrain;

import java.util.List;

@TeleOp
public class OnlyDrive extends LinearOpMode {
    MecanumDrivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Enable bulk caching to improve performance
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        
        // Set up telemetry with Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize only the drive subsystem
        drive = new MecanumDrivetrain(hardwareMap, telemetry, FtcDashboard.getInstance());
        
        telemetry.addData("Status", "Initialized - Press Play to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Clear bulk cache for performance
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // Drive controls
            if (gamepad1.right_bumper) {
                // Normal speed driving
                drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                // Precision/slow driving when right bumper is held
                drive.setWeightedPowers(-gamepad1.left_stick_y / 5, -gamepad1.left_stick_x / 5, -gamepad1.right_stick_x / 8);
            }
            
            // Display drive information
            telemetry.addData("Drive Mode", gamepad1.right_bumper ? "Precision" : "Normal");
            telemetry.addData("Left Stick Y", -gamepad1.left_stick_y);
            telemetry.addData("Left Stick X", -gamepad1.left_stick_x);
            telemetry.addData("Right Stick X (Turn)", -gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
