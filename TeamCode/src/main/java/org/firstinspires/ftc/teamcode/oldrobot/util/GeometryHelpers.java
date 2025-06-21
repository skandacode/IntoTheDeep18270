package org.firstinspires.ftc.teamcode.oldrobot.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GeometryHelpers {
    public static Pose2D rotateBy(Pose2D original, double amount, AngleUnit unit){
        double x = original.getX(DistanceUnit.INCH);
        double y = original.getY(DistanceUnit.INCH);
        double newX = x * Math.cos(unit.toRadians(amount)) + y * Math.sin(unit.toRadians(amount));
        double newY = y * Math.cos(unit.toRadians(amount)) - x * Math.sin(unit.toRadians(amount));
        return new Pose2D(DistanceUnit.INCH, newX, newY,
                unit, original.getHeading(unit)+amount);
    }
    public static Pose2D add(Pose2D originalPose, Pose2D transformBy){
        double originalPoseX = originalPose.getX(DistanceUnit.INCH);
        double originalPoseY = originalPose.getY(DistanceUnit.INCH);
        Pose2D globalTransform = rotateBy(transformBy, originalPose.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.INCH,
                originalPoseX+ globalTransform.getX(DistanceUnit.INCH),
                originalPoseY + globalTransform.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS, globalTransform.getHeading(AngleUnit.RADIANS));
    }
    public static Pose2D subtract(Pose2D from, Pose2D to){
        return add(from,
                new Pose2D(DistanceUnit.INCH, to.getX(DistanceUnit.INCH),
                        to.getY(DistanceUnit.INCH),
                        AngleUnit.RADIANS, to.getHeading(AngleUnit.RADIANS)));
    }
}
