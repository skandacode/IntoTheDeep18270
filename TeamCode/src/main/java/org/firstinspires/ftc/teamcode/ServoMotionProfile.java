package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Motion Profiling system for FTC Axon Max Servo
 * Provides smooth acceleration and deceleration for precise servo control
 */
public class ServoMotionProfile {

    private Servo servo;
    private ElapsedTime timer;

    // Motion profile parameters
    private double maxVelocity = 1.0;        // Maximum velocity (positions/second)
    private double acceleration = 2.0;       // Acceleration (positions/second²)
    private double deceleration = 2.0;       // Deceleration (positions/second²)

    // Current motion state
    private double currentPosition = 0.0;
    private double targetPosition = 0.0;
    private double currentVelocity = 0.0;
    private boolean isMoving = false;

    // Profile calculation variables
    private double accelTime;
    private double cruiseTime;
    private double decelTime;
    private double totalTime;
    private double accelDistance;
    private double cruiseDistance;
    private double decelDistance;
    private double startPosition;
    private double profileStartTime;

    /**
     * Constructor
     * @param servo The servo to control
     */
    public ServoMotionProfile(Servo servo) {
        this.servo = servo;
        this.timer = new ElapsedTime();
        this.currentPosition = servo.getPosition();
    }

    /**
     * Set motion profile parameters
     * @param maxVel Maximum velocity (positions/second)
     * @param accel Acceleration (positions/second²)
     * @param decel Deceleration (positions/second²)
     */
    public void setMotionParameters(double maxVel, double accel, double decel) {
        this.maxVelocity = Math.abs(maxVel);
        this.acceleration = Math.abs(accel);
        this.deceleration = Math.abs(decel);
    }

    /**
     * Move servo to target position using motion profiling
     * @param target Target position (0.0 to 1.0)
     */
    public void setTargetPosition(double target) {
        // Clamp target to valid servo range
        target = Math.max(0.0, Math.min(1.0, target));

        if (Math.abs(target - currentPosition) < 0.001) {
            return; // Already at target
        }

        this.targetPosition = target;
        this.startPosition = currentPosition;
        this.profileStartTime = timer.seconds();

        calculateMotionProfile();
        isMoving = true;
    }

    /**
     * Calculate the motion profile segments
     */
    private void calculateMotionProfile() {
        double distance = Math.abs(targetPosition - startPosition);
        double direction = Math.signum(targetPosition - startPosition);

        // Calculate time to reach max velocity
        double timeToMaxVel = maxVelocity / acceleration;
        double timeToStop = maxVelocity / deceleration;

        // Distance covered during acceleration and deceleration
        double accelDist = 0.5 * acceleration * timeToMaxVel * timeToMaxVel;
        double decelDist = 0.5 * deceleration * timeToStop * timeToStop;

        // Check if we can reach max velocity
        if (accelDist + decelDist > distance) {
            // Triangular profile - can't reach max velocity
            double switchTime = Math.sqrt(distance * acceleration * deceleration /
                    (acceleration + deceleration));

            accelTime = switchTime / acceleration;
            cruiseTime = 0.0;
            decelTime = switchTime / deceleration;

            accelDistance = 0.5 * acceleration * accelTime * accelTime;
            cruiseDistance = 0.0;
            decelDistance = distance - accelDistance;

            maxVelocity = acceleration * accelTime; // Actual max velocity reached
        } else {
            // Trapezoidal profile - reaches max velocity
            accelTime = timeToMaxVel;
            decelTime = timeToStop;
            cruiseTime = (distance - accelDist - decelDist) / maxVelocity;

            accelDistance = accelDist;
            cruiseDistance = maxVelocity * cruiseTime;
            decelDistance = decelDist;
        }

        totalTime = accelTime + cruiseTime + decelTime;

        // Apply direction
        accelDistance *= direction;
        cruiseDistance *= direction;
        decelDistance *= direction;
    }

    /**
     * Update the servo position based on the motion profile
     * Call this method in your OpMode loop
     */
    public void update() {
        if (!isMoving) {
            return;
        }

        double elapsed = timer.seconds() - profileStartTime;

        if (elapsed >= totalTime) {
            // Motion complete
            currentPosition = targetPosition;
            currentVelocity = 0.0;
            isMoving = false;
        } else {
            // Calculate current position based on profile phase
            if (elapsed <= accelTime) {
                // Acceleration phase
                double t = elapsed;
                currentVelocity = acceleration * t * Math.signum(accelDistance);
                currentPosition = startPosition + 0.5 * acceleration * t * t * Math.signum(accelDistance);
            } else if (elapsed <= accelTime + cruiseTime) {
                // Cruise phase
                double t = elapsed - accelTime;
                currentVelocity = maxVelocity * Math.signum(cruiseDistance);
                currentPosition = startPosition + accelDistance + maxVelocity * t * Math.signum(cruiseDistance);
            } else {
                // Deceleration phase
                double t = elapsed - accelTime - cruiseTime;
                double remainingTime = decelTime - t;
                currentVelocity = deceleration * remainingTime * Math.signum(decelDistance);
                currentPosition = targetPosition - 0.5 * deceleration * remainingTime * remainingTime * Math.signum(decelDistance);
            }
        }

        // Update servo position
        servo.setPosition(currentPosition);
    }

    /**
     * Check if the servo is currently moving
     * @return true if moving, false if at target
     */
    public boolean isMoving() {
        return isMoving;
    }

    /**
     * Get the current position
     * @return current position (0.0 to 1.0)
     */
    public double getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Get the target position
     * @return target position (0.0 to 1.0)
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Get the current velocity
     * @return current velocity (positions/second)
     */
    public double getCurrentVelocity() {
        return currentVelocity;
    }

    /**
     * Stop the current motion immediately
     */
    public void stop() {
        targetPosition = currentPosition;
        isMoving = false;
        currentVelocity = 0.0;
    }

    /**
     * Get the estimated time remaining for current motion
     * @return time remaining in seconds
     */
    public double getTimeRemaining() {
        if (!isMoving) {
            return 0.0;
        }

        double elapsed = timer.seconds() - profileStartTime;
        return Math.max(0.0, totalTime - elapsed);
    }
}

// Example OpMode showing how to use the motion profiling system
/*
@Autonomous(name = "Axon Servo Motion Profile Test", group = "Test")
public class AxonServoTest extends OpMode {

    private AxonServoMotionProfile servoProfile;
    private Servo axonServo;
    private ElapsedTime programTimer;

    @Override
    public void init() {
        axonServo = hardwareMap.get(Servo.class, "axon_servo");
        servoProfile = new AxonServoMotionProfile(axonServo);
        programTimer = new ElapsedTime();

        // Configure motion parameters
        servoProfile.setMotionParameters(
            0.8,    // Max velocity: 0.8 positions/second
            1.5,    // Acceleration: 1.5 positions/second²
            1.5     // Deceleration: 1.5 positions/second²
        );
    }

    @Override
    public void loop() {
        double time = programTimer.seconds();

        // Automated sequence - move between positions
        if (time < 2.0) {
            servoProfile.setTargetPosition(0.0);  // Move to position 0
        } else if (time < 4.0) {
            servoProfile.setTargetPosition(1.0);  // Move to position 1
        } else if (time < 6.0) {
            servoProfile.setTargetPosition(0.5);  // Move to middle
        } else {
            // Reset timer for continuous operation
            programTimer.reset();
        }

        // Update the motion profile
        servoProfile.update();

        // Telemetry
        telemetry.addData("Program Time", "%.2f s", time);
        telemetry.addData("Current Position", "%.3f", servoProfile.getCurrentPosition());
        telemetry.addData("Target Position", "%.3f", servoProfile.getTargetPosition());
        telemetry.addData("Current Velocity", "%.3f", servoProfile.getCurrentVelocity());
        telemetry.addData("Is Moving", servoProfile.isMoving());
        telemetry.addData("Time Remaining", "%.2f s", servoProfile.getTimeRemaining());
        telemetry.update();
    }
}
*/