package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.oldrobot.subsystems.Subsystem;

public class IntakeCopy implements Subsystem {
    private final CachedMotorEx slides;
    private Servo turret, arm, wrist, claw;
    private int targetPos=0;
    private PIDFController controller;
    Telemetry telemetry;

    public IntakeCopy (HardwareMap hwMap, Telemetry telemetry){
        slides = new CachedMotorEx(hwMap, "slides");

        turret=hwMap.servo.get("turret");
        arm=hwMap.servo.get("arm");

        wrist=hwMap.servo.get("wrist");
        claw=hwMap.servo.get("claw");

        resetEncoder();

        controller = new PIDFController(0.01, 0, 0, 0);

        controller.setTolerance(30);

        this.telemetry=telemetry;
    }

    public void setPower(double power){
        slides.setPower(power);
    }

    public int getLiftPos(){
        return slides.getCurrentPosition();
    }

    public void openClaw(){
        claw.setPosition(0.4);
    }
    public void closeClaw(){
        claw.setPosition(0.7);
    }
    public void setWristPos(double pos){ // wrist
        wrist.setPosition(pos);
    }
    public void setTurretPos(double pos){
        turret.setPosition(pos);
    }
    public void setArmPos(double pos){
        arm.setPosition(pos);
    }
    public void resetEncoder(){
        slides.resetEncoder();
    }
    public void setTurretAngle(double angle){
        setTurretPos(-0.00611765*angle+0.52);
    }
    public void armGrab(){
        setArmPos(0);
    }
    public void armBack(){
        setArmPos(0.7);
    }

    @Override
    public void update() {
        double controller_output=controller.calculate(getLiftPos());
        telemetry.addData("Intake applied power", controller_output);
        setPower(controller_output);
    }
    public boolean atTarget(){
        return controller.atSetPoint();
    }

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
        controller.setSetPoint(targetPos);
    }

    public int getTargetPos() {
        return targetPos;
    }
    public double getCurrent(){
        return slides.getCurrent();
    }
}
