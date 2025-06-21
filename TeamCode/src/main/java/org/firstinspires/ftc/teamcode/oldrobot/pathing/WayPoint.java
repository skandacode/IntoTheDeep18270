package org.firstinspires.ftc.teamcode.oldrobot.pathing;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class WayPoint {
    Pose2D position;
    Pose2D tolerance;

    public WayPoint(Pose2D pos, Pose2D tolerance){
        this.position=pos;
        this.tolerance=tolerance;
    }
    public Pose2D getPosition(){
        return position;
    }

    public Pose2D getTolerance() {
        return tolerance;
    }
}