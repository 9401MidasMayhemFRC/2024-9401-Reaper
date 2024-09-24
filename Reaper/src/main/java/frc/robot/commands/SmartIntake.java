package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class SmartIntake extends Command{

    private LimelightHelpers m_limelight = new LimelightHelpers();
    private boolean m_finished;


    @Override
    public void initialize() {
        m_finished = false;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }


    
}
