
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldTelemetry {
    private final Field2d m_field = new Field2d(); 
    private static FieldTelemetry instance;
    private FieldTelemetry() {
        SmartDashboard.putData("Field", m_field);
    }

    public static FieldTelemetry getInstance() {
        if (instance == null) {
            instance = new FieldTelemetry();
        }
        return instance;
    }

    public void updateRobotPose(Pose2d pose) {
        FieldObject2d targetPose = m_field.getRobotObject();
        targetPose.setPose(pose);
    }
    public void updateTargetPose(Pose2d pose) {
        FieldObject2d targetPose = m_field.getObject("Target");
        if (targetPose == null) {
            targetPose = m_field.getObject("Target");
        }
        targetPose.setPose(pose);
    }
}
