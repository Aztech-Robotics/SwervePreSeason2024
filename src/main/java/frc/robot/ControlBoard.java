package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class ControlBoard {
    public static final XboxController driver = new XboxController(0); 
    public static final XboxController operator = new XboxController(1); 

    public static DoubleSupplier getLeftYC0 () {
        return () -> MathUtil.applyDeadband(-driver.getLeftY(), 0.2);
    }

    public static DoubleSupplier getLeftXC0 () {
        return () -> MathUtil.applyDeadband(-driver.getLeftX(), 0.2);
    }

    public static DoubleSupplier getRightXC0 () {
        return () -> MathUtil.applyDeadband(-driver.getRightX(), 0.2);
    }

    public static DoubleSupplier getRightYC1 () {
        return () -> MathUtil.applyDeadband(-operator.getRightY(), 0.2);
    }
}
