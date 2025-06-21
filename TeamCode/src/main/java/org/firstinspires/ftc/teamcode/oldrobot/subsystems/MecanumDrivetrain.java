package org.firstinspires.ftc.teamcode.oldrobot.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.oldrobot.pathing.WayPoint;

@Config
public class MecanumDrivetrain implements Subsystem{
    CachedMotorEx leftFront;
    CachedMotorEx leftBack;
    CachedMotorEx rightFront;
    CachedMotorEx rightBack;
    SimpleMotorFeedforward forwardFeedforward=new SimpleMotorFeedforward(0.12, 1);
    SimpleMotorFeedforward strafeFeedforward=new SimpleMotorFeedforward(0.26, 1);
    SimpleMotorFeedforward headingFeedforward=new SimpleMotorFeedforward(0.135, 1);

    PIDFController translationalControllerY=new PIDFController(0.09, 0, 0.01, 0);
    PIDFController translationalControllerX=new PIDFController(
            translationalControllerY.getP(),
            translationalControllerY.getI(),
            translationalControllerY.getD(),
            translationalControllerY.getF());
    PIDFController headingController=new PIDFController(1, 0, 0, 0);

    Telemetry telemetry;
    FtcDashboard dashboard;

    public SparkFunOTOS odometry;

    public Pose2D position;

    public MecanumDrivetrain(HardwareMap hwMap, Telemetry telemetry, FtcDashboard dashboard){
        this.telemetry=telemetry;
        this.dashboard=dashboard;
        leftFront=new CachedMotorEx(hwMap, "frontleft");
        leftBack=new CachedMotorEx(hwMap, "backleft");
        rightFront=new CachedMotorEx(hwMap, "frontright");
        rightBack=new CachedMotorEx(hwMap, "backright");
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        odometry.calibrateImu();
        odometry.setLinearScalar(1.0726534030253347);
        odometry.setAngularScalar(0.9885441764832054);
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-5, 0, 90);
        odometry.setOffset(offset);
        odometry.resetTracking();

        odometry.setPosition(new SparkFunOTOS.Pose2D());
    }

    public void setRawPowers(double frontleft, double frontright, double backleft, double backright){
        double maximum=Math.max(frontleft, frontright);
        maximum=Math.max(maximum, backleft);
        maximum=Math.max(maximum, backright);
        if (maximum>1){
            frontleft=frontleft/maximum;
            frontright=frontright/maximum;
            backleft=backleft/maximum;
            backright=backright/maximum;

        }
        leftFront.set(-frontleft);
        leftBack.set(-backleft);
        rightFront.set(frontright);
        rightBack.set(backright);
    }
    public void setWeightedPowers(double front, double strafe, double heading){
        double weightedFront=forwardFeedforward.calculate(front);
        double weightedStrafe=strafeFeedforward.calculate(strafe);
        double weightedHeading=headingFeedforward.calculate(heading);

        setRawPowers(
                (weightedFront - weightedStrafe - weightedHeading),
                (weightedFront + weightedStrafe + weightedHeading),
                (weightedFront + weightedStrafe - weightedHeading),
                (weightedFront - weightedStrafe + weightedHeading)
        );
    }

    public void driveFieldCentric(double XPower, double YPower, double turnPower, double currHeading){
        double x = XPower * Math.cos(currHeading) + YPower * Math.sin(currHeading);
        double y = YPower * Math.cos(currHeading) - XPower * Math.sin(currHeading);
        setWeightedPowers(x, y, turnPower);
    }
    public void setTarget(WayPoint target){
        translationalControllerX.setSetPoint(target.getPosition().getX(DistanceUnit.INCH));
        translationalControllerY.setSetPoint(target.getPosition().getY(DistanceUnit.INCH));
        headingController.setSetPoint(target.getPosition().getHeading(AngleUnit.RADIANS));

        translationalControllerX.setTolerance(target.getTolerance().getX(DistanceUnit.INCH));
        translationalControllerY.setTolerance(target.getTolerance().getY(DistanceUnit.INCH));
        headingController.setTolerance(target.getTolerance().getHeading(AngleUnit.RADIANS));
    }
    public void update() {
        SparkFunOTOS.Pose2D rawposition= odometry.getPosition();
        position=new Pose2D(DistanceUnit.INCH, rawposition.x, rawposition.y, AngleUnit.DEGREES, rawposition.h);

        telemetry.addData("position", position.getX(DistanceUnit.INCH)+" "+position.getY(DistanceUnit.INCH)+" "+position.getHeading(AngleUnit.DEGREES));
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setFill("blue")
                .strokeCircle(position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH), 5)
                .strokeLine(position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH),
                        (Math.cos(position.getHeading(AngleUnit.RADIANS))*5)+ position.getX(DistanceUnit.INCH),
                        (Math.sin(position.getHeading(AngleUnit.RADIANS))*5)+ position.getY(DistanceUnit.INCH));

        dashboard.sendTelemetryPacket(packet);
    }
    public SparkFunOTOS.Pose2D getVelocity(){
        return odometry.getVelocity();
    }
    public void updatePIDS(){
        double heading=position.getHeading(AngleUnit.RADIANS);
        while (Math.abs(heading-headingController.getSetPoint())>Math.PI){
            if (heading<headingController.getSetPoint()){
                heading=heading+2*Math.PI;
            }else{
                heading=heading-2*Math.PI;
            }
        }
        double x_velo=translationalControllerX.calculate(position.getX(DistanceUnit.INCH));
        double y_velo=translationalControllerY.calculate(position.getY(DistanceUnit.INCH));
        double heading_velo=headingController.calculate(heading);
        telemetry.addData("velocity x", x_velo);
        telemetry.addData("velocity y", y_velo);
        telemetry.addData("velocity heading", heading_velo);

        if (translationalControllerY.atSetPoint()){
            y_velo=0;
        }
        if (translationalControllerX.atSetPoint()){
            x_velo=0;
        }
        if (headingController.atSetPoint()){
            heading_velo=0;
        }


        driveFieldCentric(x_velo, y_velo,heading_velo, heading);
    }
    public boolean atTarget(){
        return translationalControllerX.atSetPoint() && translationalControllerY.atSetPoint() && headingController.atSetPoint();
    }
    public void setPosition(Pose2D targetPosition){

        odometry.setPosition(new SparkFunOTOS.Pose2D(targetPosition.getX(DistanceUnit.INCH),
                targetPosition.getY(DistanceUnit.INCH), targetPosition.getHeading(AngleUnit.DEGREES)));
    }
    public void calibrateIMU(){
        odometry.calibrateImu();
    }
}