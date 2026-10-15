package org.openbot.app.robot.robot;

/**
 * Turns a heading error into left/right motor commands for a 2WD differential-drive robot (two
 * independently driven wheels, no steering axle).
 *
 * <ul>
 *   <li>Small error: drive forward and steer by making one wheel faster than the other.
 *   <li>Large error: stop moving forward and rotate in place (left and right wheels in opposite
 *       directions). The rotation continues until the error is small again, so it does not chatter
 *       around the switching point.
 * </ul>
 *
 * <p>The heading error is positive when the target is to the RIGHT (clockwise): the left wheel
 * must then run faster than the right wheel. Commands are in the range -1..1 and are meant for
 * Vehicle.setControl(left, right). All values are initial guesses to be tuned on the real robot.
 *
 * <p>Plain Java on purpose (no Android classes), so it can be tested on a desktop.
 */
public class DifferentialSteering {

    /** Above this error the robot rotates in place. */
    public static final double PIVOT_ENTER_DEG = 50.0;
    /** An in-place rotation continues until the error is below this. */
    public static final double PIVOT_EXIT_DEG = 25.0;
    /** Rotate in place at this wheel speed ... */
    public static final float PIVOT_SPEED = 0.25f;
    /** ... or at this slower speed once the remaining error is below PIVOT_FINE_BELOW_DEG. */
    public static final float PIVOT_FINE_SPEED = 0.18f;
    public static final double PIVOT_FINE_BELOW_DEG = 35.0;
    /** Forward speed on a straight path. */
    public static final float DRIVE_SPEED = 0.25f;
    /** At the pivot threshold the forward speed has dropped to this fraction. */
    public static final float MIN_SPEED_FACTOR = 0.35f;
    /** Steering trim per degree of error. */
    public static final float TRIM_PER_DEG = 0.006f;
    /** Largest steering trim. */
    public static final float MAX_TRIM = 0.12f;

    /** Left and right wheel commands. */
    public static final class Command {
        public final float left;
        public final float right;

        Command(float left, float right) {
            this.left = left;
            this.right = right;
        }
    }

    private boolean pivoting = false;

    public void reset() {
        pivoting = false;
    }

    public boolean isPivoting() {
        return pivoting;
    }

    /** @param headingErrorDeg positive = target to the right (clockwise); NaN = no command */
    public Command compute(double headingErrorDeg) {
        if (Double.isNaN(headingErrorDeg)) {
            return new Command(0f, 0f);
        }
        double abs = Math.abs(headingErrorDeg);

        if (pivoting) {
            if (abs < PIVOT_EXIT_DEG) {
                pivoting = false;
            }
        } else if (abs >= PIVOT_ENTER_DEG) {
            pivoting = true;
        }

        float sign = headingErrorDeg >= 0 ? 1f : -1f;

        if (pivoting) {
            float speed = abs < PIVOT_FINE_BELOW_DEG ? PIVOT_FINE_SPEED : PIVOT_SPEED;
            // Target to the right: left wheel forward, right wheel backward.
            return new Command(sign * speed, -sign * speed);
        }

        float factor = (float) (1.0 - abs / PIVOT_ENTER_DEG * (1.0 - MIN_SPEED_FACTOR));
        float speed = DRIVE_SPEED * Math.max(MIN_SPEED_FACTOR, factor);
        float trim = (float) Math.max(-MAX_TRIM, Math.min(MAX_TRIM, TRIM_PER_DEG * headingErrorDeg));
        // Target to the right (positive error): left wheel faster than right wheel.
        return new Command(clamp(speed + trim), clamp(speed - trim));
    }

    private static float clamp(float value) {
        return Math.max(-1f, Math.min(1f, value));
    }
}