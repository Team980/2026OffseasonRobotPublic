package frc.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import frc.robot.RobotContainer;

public class DriveInputSmoother {
    public static final double joystickDeadband = 0.2;
    public static double translationJoystickDeadband = joystickDeadband;
    public static double rotationJoystickDeadband = joystickDeadband;
    private static final double kTranslationCurveExponent = 1.5;
    private static final double kRotationCurveExponent = 1.5;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rotationInput;

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput, DoubleSupplier rotationInput) {
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.rotationInput = rotationInput;
    }

    public ManualDriveInput getSmoothedInput() { 
        final Vector<N2> rawTranslationInput = VecBuilder.fill(forwardInput.getAsDouble(), leftInput.getAsDouble());
        final Vector<N2> deadbandedTranslationInput = MathUtil.applyDeadband(rawTranslationInput, translationJoystickDeadband);
        final Vector<N2> curvedTranslationInput = MathUtil.copyDirectionPow(deadbandedTranslationInput, kTranslationCurveExponent);
        

        final double rawRotationInput = rotationInput.getAsDouble();
        final double deadbandedRotationInput = MathUtil.applyDeadband(rawRotationInput, rotationJoystickDeadband);
        final double curvedRotationInput = MathUtil.copyDirectionPow(deadbandedRotationInput, kRotationCurveExponent);
        return new ManualDriveInput(
            curvedTranslationInput.get(0), 
            curvedTranslationInput.get(1), 
            curvedRotationInput
        );
    }
}
