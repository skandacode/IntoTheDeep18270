package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.oldrobot.subsystems.Subsystem;

public class IntakeCopyCopy implements Subsystem {
    private final CachedMotorEx slides;
    private Servo turret, wrist, claw;
    private int targetPos=0;
    public NewServoMP arm;
    private PIDFController controller;
    public static double ticks_per_inch = 145.1*105/25.4;
    public static double arm_length = 7; // inch
    Telemetry telemetry;

    public IntakeCopyCopy (HardwareMap hwMap, Telemetry telemetry){
        slides = new CachedMotorEx(hwMap, "slides");

        turret=hwMap.servo.get("turret");
        Servo myServo = hwMap.get(Servo.class, "arm");
        arm = new NewServoMP(myServo, 1, 2);

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

    public int getIntakePos(){
        return slides.getCurrentPosition();
    }

    public void openClaw(){
        claw.setPosition(0.1);
    }
    public void closeClaw(){
        claw.setPosition(0.6);
    }
    public void depositPos(){
        arm.setTargetPosition(0.5);
        turret.setPosition(0);
        wrist.setPosition(0.5);
    }
    public void intakePos(){
        claw.setPosition(0.4);
        arm.setTargetPosition(0.1);
    }
    public void setWristPos(double pos){ // wrist
        wrist.setPosition(pos);
    }
    public void setTurretPos(double pos){
        turret.setPosition(pos);
    }
    public void setArmPos(double pos){
        arm.setTargetPosition(pos);
    }
    public void resetEncoder(){
        slides.resetEncoder();
    }
    public void setTurretAngle(double angle){
        setTurretPos(-0.00611765*angle+0.52);
    }
    public void setDistance (double distance){

        setTargetPos((int) (25.4/113*145.1*distance*20/21));
    }
    public void armGrab(){
        setArmPos(0);
    }
    public void armBack(){
        setArmPos(0.7);
    }

    @Override
    public void update() {
        double controller_output=controller.calculate(getIntakePos());
        arm.update();
        telemetry.addData("Intake applied power", controller_output);
        setPower(controller_output);
    }
    public boolean atTarget(){
        return controller.atSetPoint();
    }

    public void setTargetPos(int targetPos) {
        this.targetPos = -targetPos;
        controller.setSetPoint(-targetPos);
    }

    public int getTargetPos() {
        return targetPos;
    }
    public double getCurrent(){
        return slides.getCurrent();
    }
    public void goToPosition(Position pos){
        double angRad;
        if (Math.abs(pos.x)>arm_length){
            angRad=Math.signum(pos.x)*Math.PI/2;
        }else{
            angRad =Math.asin(pos.x/arm_length);
        }
        setTurretAngle(Math.toDegrees(angRad));
        setDistance(pos.y-arm_length*Math.cos(angRad));
        setWristPos(0.5);
    }
}