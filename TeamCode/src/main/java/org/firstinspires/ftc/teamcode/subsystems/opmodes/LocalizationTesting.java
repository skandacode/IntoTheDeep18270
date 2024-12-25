package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

@Config
@TeleOp
public class LocalizationTesting extends LinearOpMode {
    MecanumDrivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry= new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive=new MecanumDrivetrain(hardwareMap, telemetry, dashboard);
        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, 2, -63, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));
        waitForStart();
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        drive.calibrateIMU();
        while (opModeIsActive()){
            if (!gamepad1.right_bumper) {
                drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                drive.setWeightedPowers(-gamepad1.left_stick_y / 5, -gamepad1.left_stick_x / 5, -gamepad1.right_stick_x / 8);
            }
            drive.update();
            telemetry.update();
        }
    }
}
