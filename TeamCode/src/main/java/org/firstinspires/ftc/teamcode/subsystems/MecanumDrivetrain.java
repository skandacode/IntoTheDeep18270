package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;

@Config
public class MecanumDrivetrain implements Subsystem{
    MotorEx leftFront;
    MotorEx leftBack;
    MotorEx rightFront;
    MotorEx rightBack;
    SimpleMotorFeedforward forwardFeedforward=new SimpleMotorFeedforward(0.085, 1);
    SimpleMotorFeedforward strafeFeedforward=new SimpleMotorFeedforward(0.22, 1);
    SimpleMotorFeedforward headingFeedforward=new SimpleMotorFeedforward(0.115, 1);

    PIDFController translationalControllerY=new PIDFController(0.1, 0, 0.008, 0);
    PIDFController translationalControllerX=new PIDFController(
            translationalControllerY.getP(),
            translationalControllerY.getI(),
            translationalControllerY.getD(),
            translationalControllerY.getF());
    PIDFController headingController=new PIDFController(1, 0, 0, 0);

    Telemetry telemetry;
    FtcDashboard dashboard;

    GoBildaPinpointDriver odometry;

    public MecanumDrivetrain(HardwareMap hwMap, Telemetry telemetry, FtcDashboard dashboard){
        this.telemetry=telemetry;
        this.dashboard=dashboard;
        leftFront=new MotorEx(hwMap, "frontleft");
        leftBack=new MotorEx(hwMap, "backleft");
        rightFront=new MotorEx(hwMap, "frontright");
        rightBack=new MotorEx(hwMap, "backright");

        odometry = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setOffsets(100,  100);
        odometry.setYawScalar(1);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
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
        odometry.update();
        Pose2D position= odometry.getPosition();
        telemetry.addLine(position.toString());
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();


        packet.fieldOverlay().setFill("blue")
                .strokeCircle(position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH), 5)
                .strokeLine(position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH),
                        (Math.cos(position.getHeading(AngleUnit.RADIANS)*5))+ position.getX(DistanceUnit.INCH),
                        (Math.sin(position.getHeading(AngleUnit.RADIANS))*5)+ position.getY(DistanceUnit.INCH));

        dashboard.sendTelemetryPacket(packet);
    }
    public Pose2D getVelocity(){
        return odometry.getVelocity();
    }
    public void updatePIDS(){
        double heading=odometry.getPosition().getHeading(AngleUnit.RADIANS);
        while (Math.abs(heading-headingController.getSetPoint())>Math.PI){
            if (heading<headingController.getSetPoint()){
                heading=heading+2*Math.PI;
            }else{
                heading=heading-2*Math.PI;
            }
        }
        double x_velo=translationalControllerX.calculate(odometry.getPosition().getX(DistanceUnit.INCH));
        double y_velo=translationalControllerY.calculate(odometry.getPosition().getY(DistanceUnit.INCH));
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
        odometry.setPosition(targetPosition);
    }
}