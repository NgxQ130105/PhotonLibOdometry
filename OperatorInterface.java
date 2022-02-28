import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

public class OperatorInterface {
    private XboxController opCtrl = new XboxController(0);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
    private SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    public OperatorInterface() {}

    public double getFwdRevSpdCmd() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        return -speedLimiter.calculate(opCtrl.getLeftY()) * Constants.kMaxSpeed;
    }

    public double getRotateSpdCmd() {
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        return -rotLimiter.calculate(opCtrl.getRightX()) * Constants.kMaxAngularSpeed;
    }

    public boolean getSimKickCmd() {
        return opCtrl.getXButtonPressed();
    }
}