package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class ControlBoard {
    public static final XboxController driver = new XboxController(0);

    public static Supplier<Double> getLeftYC0 () {
        return () -> {
            return MathUtil.applyDeadband(driver.getLeftY(), 0.2);
        };
    }

    public static Supplier<Double> getLeftXC0 () {
        return () -> {
            return MathUtil.applyDeadband(driver.getLeftX(), 0.2);
        };
    }

    public static Supplier<Double> getRightXC0 () {
        return () -> {
            return MathUtil.applyDeadband(driver.getRightX(), 0.2);
        };
    }
}
