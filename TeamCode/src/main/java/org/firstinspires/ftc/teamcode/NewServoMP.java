package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class NewServoMP {
    private Servo servo;
    private double currentPosition;
    private double targetPosition;
    private double maxVelocity;
    private double maxAcceleration;
    private ElapsedTime timer;

    // Motion profile state variables
    private double profileStartTime;
    private double profileStartPosition;
    private double accelerationTime;
    private double cruiseTime;
    private double decelerationTime;
    private double totalTime;
    private boolean isProfileActive;

    /**
     * Constructor for MotionProfiledServo
     * @param servo The servo hardware object
     * @param maxVel Maximum velocity (positions/second)
     * @param maxAccel Maximum acceleration (positions/second²)
     */
    public NewServoMP(Servo servo, double maxVel, double maxAccel) {
        this.servo = servo;
        this.maxVelocity = maxVel;
        this.maxAcceleration = maxAccel;
        this.timer = new ElapsedTime();
        this.currentPosition = servo.getPosition();
        this.targetPosition = currentPosition;
        this.isProfileActive = true;
    }

    /**
     * Set a new target position with motion profiling
     * @param target Target position (0.0 to 1.0)
     */
    public void setTargetPosition(double target) {
        // Clamp target to valid servo range
        target = Math.max(0.0, Math.min(1.0, target));

        if (Math.abs(target - targetPosition) < 0.001) {
            return; // Target hasn't changed significantly
        }

        this.targetPosition = target;
        this.profileStartPosition = currentPosition;
        this.profileStartTime = timer.seconds();

        calculateMotionProfile();
        this.isProfileActive = true;
    }

    /**
     * Calculate the motion profile parameters
     */
    private void calculateMotionProfile() {
        double distance = Math.abs(targetPosition - profileStartPosition);

        // Time to reach max velocity
        double timeToMaxVel = maxVelocity / maxAcceleration;

        // Distance covered during acceleration to max velocity
        double accelDistance = 0.5 * maxAcceleration * timeToMaxVel * timeToMaxVel;

        if (2 * accelDistance >= distance) {
            // Triangular profile (never reach max velocity)
            accelerationTime = Math.sqrt(distance / maxAcceleration);
            cruiseTime = 0;
            decelerationTime = accelerationTime;
        } else {
            // Trapezoidal profile (reach max velocity)
            accelerationTime = timeToMaxVel;
            decelerationTime = timeToMaxVel;
            cruiseTime = (distance - 2 * accelDistance) / maxVelocity;
        }

        totalTime = accelerationTime + cruiseTime + decelerationTime;
    }

    /**
     * Update the servo position based on the motion profile
     * Call this method in your OpMode loop
     */

    public void update() {
        if (!isProfileActive) {
            return;
        }
        double elapsedTime = timer.seconds() - profileStartTime;

        if (elapsedTime >= totalTime) {
            // Profile complete
            currentPosition = targetPosition;
            isProfileActive = false;
        } else {
            // Calculate position based on current phase of profile
            currentPosition = calculateProfilePosition(elapsedTime);
        }

        servo.setPosition(currentPosition);
    }

    double previousPos = servo.getPosition();
    double previousTime = timer.seconds();

    public double CurrentVel() {
        double currentVel = (servo.getPosition() - previousPos)/(timer.seconds()-previousTime);
        double previousPos = servo.getPosition();
        double previousTime = timer.seconds();
        return currentVel;
    }
    /**
     * Calculate the position at a given time in the motion profile
     */
    private double calculateProfilePosition(double t) {
        double direction = targetPosition > profileStartPosition ? 1.0 : -1.0;
        double position = profileStartPosition;

        if (t <= accelerationTime) {
            // Acceleration phase
            position += direction * 0.5 * maxAcceleration * t * t;
        } else if (t <= accelerationTime + cruiseTime) {
            // Cruise phase
            double accelDistance = 0.5 * maxAcceleration * accelerationTime * accelerationTime;
            double cruiseDistance = maxVelocity * (t - accelerationTime);
            position += direction * (accelDistance + cruiseDistance);
        } else {
            // Deceleration phase
            double accelDistance = 0.5 * maxAcceleration * accelerationTime * accelerationTime;
            double cruiseDistance = maxVelocity * cruiseTime;
            double decelTime = t - accelerationTime - cruiseTime;
            double decelDistance = maxVelocity * decelTime - 0.5 * maxAcceleration * decelTime * decelTime;
            position += direction * (accelDistance + cruiseDistance + decelDistance);
        }

        return position;
    }

    // Getter methods
    public double getCurrentPosition() { return currentPosition; }
    public double getTargetPosition() { return targetPosition; }
    public boolean isProfileActive() { return isProfileActive; }
    public double getError() { return Math.abs(targetPosition - currentPosition); }
    public double getCurrentVel() { return CurrentVel(); }

    // Setter methods for tuning
    public void setMaxVelocity(double maxVel) { this.maxVelocity = maxVel; }
    public void setMaxAcceleration(double maxAccel) { this.maxAcceleration = maxAccel; }
}

// Example usage in an OpMode:
/*
public class ServoMotionProfileExample extends LinearOpMode {
    private MotionProfiledServo profiledServo;

    @Override
    public void runOpMode() {
        // Initialize hardware
        Servo myServo = hardwareMap.get(Servo.class, "servo_name");

        // Create motion profiled servo
        // Parameters: maxVelocity = 2.0 pos/sec, maxAcceleration = 4.0 pos/sec²
        profiledServo = new MotionProfiledServo(myServo, 2.0, 4.0);

        waitForStart();

        while (opModeIsActive()) {
            // Set target based on gamepad input
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
        }
    }
} */