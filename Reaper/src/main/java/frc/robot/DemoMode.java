package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DemoMode extends SubsystemBase{

    private boolean m_enabled = false;
    private boolean m_kid = false;

    public DemoMode(){
        SmartDashboard.putBoolean("Demo Mode", m_enabled);
    }
    
    public boolean isEnabled(){
        return m_enabled;
    }


 @Override
 public void periodic() {
    
    m_enabled = SmartDashboard.getBoolean("Demo Mode", false);
    if (m_kid && m_enabled){
        DriveConstants.kMaxSpeedMetersPerSecond = 1.5;
    } else {
        DriveConstants.kMaxSpeedMetersPerSecond = 4.25;
    }
 }

}
