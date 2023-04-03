package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Arm implements Sendable {
	// Initializing Variables
	private final CANSparkMax liftMotor;
	private final CANSparkMax extendMotor;
	//private final WPI_VictorSPX clawMotor;

	ShuffleboardTab m_armTab;

	public Arm(int liftCanId, int extendCanId, int intakeCanId, int maxLiftId, int minLiftId, int maxExtendId, int minExtendId) {
		liftMotor = new CANSparkMax(liftCanId, CANSparkMaxLowLevel.MotorType.kBrushless); // controlled by CAN SparkMax
		extendMotor = new CANSparkMax(extendCanId,CANSparkMaxLowLevel.MotorType.kBrushless);// controlled by CAN SparkMax
		//clawMotor = new WPI_VictorSPX(intakeCanId); // controlled by TalonSRX (for claw)
		m_armTab = Shuffleboard.getTab("Arm");
		m_armTab.add("Telemetry", this);

	}

	public double getExtensionSpeed(){
		return extendMotor.getEncoder().getVelocity();
	}	

	public double getLiftSpeed(){
		return liftMotor.getEncoder().getVelocity();
	}

	public double getExtensionLength(){
		return extendMotor.getEncoder().getPosition();
	}

	public double getLiftAngle(){
		return liftMotor.getEncoder().getPosition();
	}

	public void setExtensionSpeed(double speed){
		extendMotor.set(speed);
	}

	public void setLiftSpeed(double speed){
		liftMotor.set(speed);
	}

	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Swerve");
		builder.addDoubleProperty("lift-angle", this::getLiftAngle, null);
	}
}