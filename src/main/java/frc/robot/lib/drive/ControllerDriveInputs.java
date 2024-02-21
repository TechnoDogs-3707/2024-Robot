package frc.robot.lib.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.Utility;
import frc.robot.lib.util.Util;

/**
 * A class that processes drivetrain inputs from a joystick/controller.
 * I'm pretty sure most of this is from 3476.
 */
public class ControllerDriveInputs {

    private double x, y, rotation;

    /**
     * 
     * @param forwards This input should be positive to move the robot forwards.
     * @param left This input should be positive to move the robot left.
     * @param rotateLeft This input should be positive to rotate the robot left
     */
    public ControllerDriveInputs(double forwards, double left, double rotateLeft) {
        this.x = forwards;
        this.y = left;
        this.rotation = rotateLeft;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getRotation() {
        return rotation;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setRotation(double rotation) {
        this.rotation = rotation;
    }

    /**
     * Apply a circular deadzone on the controller and then scale the joystick values
     *
     * @param snappingDeadZoneX An X value smaller than this value will get set to 0 (This help you with driving straight forwards
     *                          and backwards) Larger x values will be scaled so that the edge of the deadzone will be 0
     * @param snappingDeadZoneY An Y value smaller than this value will get set to 0 (This help you with driving straight
     *                          sideways) Larger y values will be scaled so that the edge of the deadzone will be 0
     * @param rotationDeadZone  A rotation value smaller than this value will get set to 0. Larger rotation values will be scaled
     *                          so that the edge of the deadzone will be 0
     * @param circularDeadZone  If the controller input is inside the circle with a radius of this value we'll set the X and Y
     *                          values to 0. If its higher we'll scale the joystick values to make the edge of the deadzone have a
     *                          value of 0
     * @return {@link ControllerDriveInputs}
     */
    public ControllerDriveInputs applyDeadZone(double snappingDeadZoneX, double snappingDeadZoneY, double rotationDeadZone,
                                               double circularDeadZone) {
        double amplitudeSquared = x * x + y * y;
        if (amplitudeSquared < circularDeadZone * circularDeadZone) {
            x = 0;
            y = 0;
        } else {
            double angle = Math.atan2(x, y);
            double minx = Math.sin(angle) * circularDeadZone;
            double miny = Math.cos(angle) * circularDeadZone;

            //System.out.println("min controller: x: " + minx1 + " y: " +  minx2);

            x = Math.copySign(Utility.coercedNormalize(Math.abs(x), Math.abs(minx), 1, 0, 1), x);
            y = Math.copySign(Utility.coercedNormalize(Math.abs(y), Math.abs(miny), 1, 0, 1), y);
        }

        //coercedNormalize should do this for us
        // if(Math.abs(x)<snappingDeadZoneX) x = 0;
        // if(Math.abs(y)<snappingDeadZoneY) y = 0;
        // if(Math.abs(rotation)<rotationDeadZone) rotation = 0;

        rotation = Math.copySign(Utility.coercedNormalize(Math.abs(rotation), Math.abs(rotationDeadZone), 1, 0, 1),
                rotation);
        x = Math.copySign(Utility.coercedNormalize(Math.abs(x), Math.abs(snappingDeadZoneX), 1, 0, 1), x);
        y = Math.copySign(Utility.coercedNormalize(Math.abs(y), Math.abs(snappingDeadZoneY), 1, 0, 1), y);

        return this;
    }

    /**
     * Squares Inputs to allow for more precise control at low speeds.
     *
     * @return {@link ControllerDriveInputs}
     */
    public ControllerDriveInputs squareInputs() {
        x = Math.copySign(x * x, x);
        y = Math.copySign(y * y, y);
        rotation = Math.copySign(rotation * rotation, rotation);
        return this;
    }

    /**
     * Cubes Inputs to allow for more precise control at low speeds.
     *
     * @return {@link ControllerDriveInputs}
     */
    public ControllerDriveInputs cubeInputs() {
        x = x * x * x;
        y = y * y * y;
        rotation = rotation * rotation * rotation;
        return this;
    }

    public ChassisSpeeds getVelocity(double maxVelocity, double maxAngularVelocity) {
        return new ChassisSpeeds(
            x * maxVelocity,
            y * maxVelocity,
            rotation * maxAngularVelocity
        );
    }

    public ChassisSpeeds getVelocityFieldOriented(double maxVelocity, double maxAngularVelocity, Rotation2d angle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            x * maxVelocity,
            y * maxVelocity,
            rotation * maxAngularVelocity,
            angle
        );
    }

    public ControllerDriveInputs exponent(double power) {
        x = Math.pow(x, power);
        y = Math.pow(y, power);
        rotation = Math.pow(rotation, power);
        return this;
    }

    public ControllerDriveInputs times(double factor) {
        return new ControllerDriveInputs(x*factor, y*factor, rotation*factor);
    }

    public ControllerDriveInputs times(double linearFactor, double angularFactor) {
        return new ControllerDriveInputs(x*linearFactor, y*linearFactor, rotation*angularFactor);
    }

    public Twist2d getVelocityVector() {
        return new Twist2d(x, y, rotation);
    }

    /**
     * Applies a power function to the control inputs without limiting max values when the stick
     * is not aligned with a cardinal direction.
     * @return
     */
    public ControllerDriveInputs powerPolar(double power) {
        // note: we preserve the rotation value since it's not affected by this issue

        // convert x and y input to polar coordinates, theta and magnitude.
        var translation = new Translation2d(x, y);
        var theta = translation.getAngle();
        var magnitude = translation.getNorm();

        // limit magnitude to 0..1
        magnitude = Util.limit(magnitude, 1);
        magnitude = Math.pow(magnitude, power);

        // convert back to cartesian coords
        var normalizedTranslation = new Translation2d(magnitude, theta);

        return new ControllerDriveInputs(normalizedTranslation.getX(), normalizedTranslation.getY(), Math.copySign(Math.pow(rotation, power), rotation));
    }

}
