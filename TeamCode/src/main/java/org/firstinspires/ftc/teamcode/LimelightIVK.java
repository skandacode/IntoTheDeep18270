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
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.oldrobot.subsystems.Subsystem;

@Config
public class LimelightIVK {
    Limelight3A limelight;
    public static double height = 14; //height of the limeLight
    public static double angle = 45; //angle of the limeLight

    public LimelightIVK(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); //frames per sec
        limelight.start();
        limelight.pipelineSwitch(0); //yellow samp
    }
    public Position getPosition(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double distance_from_Samp_y = height * Math.tan(Math.toRadians(angle + ty)); //trig
            double distance_from_Samp_x = distance_from_Samp_y * Math.tan(Math.toRadians(tx));
            return new Position(distance_from_Samp_x, distance_from_Samp_y);
        }
        return null;
    }
}
