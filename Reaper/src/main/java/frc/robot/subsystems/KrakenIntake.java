package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenIntake extends SubsystemBase {

    private TalonFX m_motor = new TalonFX(9,"rio");
    private TalonFXConfigurator m_leftTalonFXConfigurator = m_motor.getConfigurator();
    private MotorOutputConfigs m_leftMotorConfigs = new MotorOutputConfigs();
    private StatusSignal<Double> m_motorCurrentSignal = m_motor.getTorqueCurrent();
    private StatusSignal<Double> m_motorVeloSignal = m_motor.getVelocity();

    private VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private Slot0Configs m_PIDSlot0Configs = new Slot0Configs();
    private CurrentLimitsConfigs m_currentLimitsConfigs = new CurrentLimitsConfigs();
    
    private double m_shooterVelo = 0.0;
    private boolean m_shooterPIDEnabled = false;

    

    public KrakenIntake(){

        m_currentLimitsConfigs.StatorCurrentLimit = 100.0;
        m_currentLimitsConfigs.StatorCurrentLimitEnable = true;
        m_currentLimitsConfigs.SupplyCurrentLimit = 50.0;
        m_currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        m_PIDSlot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        m_PIDSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        m_PIDSlot0Configs.kA = 0.0;
        m_PIDSlot0Configs.kP = 0.01; // An error of 1 rps results in 0.01 V output
        m_PIDSlot0Configs.kI = 0; // no output for integrated error
        m_PIDSlot0Configs.kD = 0; // no output for error derivative

        m_leftTalonFXConfigurator.apply(new TalonFXConfiguration());
        //m_leftMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_leftMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        m_leftTalonFXConfigurator.apply(m_PIDSlot0Configs);
        m_leftTalonFXConfigurator.apply(m_currentLimitsConfigs);
        m_leftTalonFXConfigurator.apply(m_leftMotorConfigs);

    }

    /**
     * Sets the Velocity of the Shooter's Krakens taking in RPM and giving the Krakens RPS
     * as well as enables the PID
     * @param shooterVelo Velocity in RPM
     * 
     */
    public void setIntakeVelo(int RPS){
        m_shooterPIDEnabled = true;
        m_shooterVelo = RPS;
    }

    public void setIntakeVelo(double RPM){
        m_shooterPIDEnabled = true;
        m_shooterVelo = RPM / 60;
    }

    public void RunShooter(){
        m_shooterPIDEnabled = true;
    }

    public void setVelo(int RPS){
        m_shooterVelo = RPS;
    }

    public void setVelo(double RPM){
        m_shooterVelo = RPM / 60;
    }

    public double getMotorCurrent(){
        m_motorCurrentSignal.refresh();
        return m_motorCurrentSignal.getValueAsDouble();
    }

   

    public void stopShooter(){
        m_motor.stopMotor();
        m_shooterPIDEnabled = false;
    }

    public double getVelocity(){
        m_motorVeloSignal.refresh();
        return m_motorVeloSignal.getValueAsDouble();
    }
        

    @Override
    public void periodic() {
        
        if(m_shooterPIDEnabled){
            m_motor.setControl(m_request.withVelocity(m_shooterVelo));
        } else {
            m_motor.stopMotor();
        }

        SmartDashboard.putNumber("Target Shooter Velo.",m_shooterVelo);
        SmartDashboard.putNumber("Actual Motor Velo.",m_motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Motor Current",m_motor.getTorqueCurrent().getValueAsDouble());
        

        

    }

}
