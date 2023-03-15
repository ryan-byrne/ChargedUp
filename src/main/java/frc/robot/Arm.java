package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Arm {
	// Initializing Variables
	private final CANSparkMax liftMotor;
	private final CANSparkMax extendMotor;
	//private final WPI_VictorSPX clawMotor;

	public Arm(int liftCanId, int extendCanId, int intakeCanId, int maxLiftId, int minLiftId, int maxExtendId, int minExtendId) {
		liftMotor = new CANSparkMax(liftCanId, CANSparkMaxLowLevel.MotorType.kBrushless); // controlled by CAN SparkMax
		extendMotor = new CANSparkMax(extendCanId,CANSparkMaxLowLevel.MotorType.kBrushless);// controlled by CAN SparkMax
		//clawMotor = new WPI_VictorSPX(intakeCanId); // controlled by TalonSRX (for claw)
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
		//double length = getExtensionLength();
		//SmartDashboard.putNumber("Extension", length);
		extendMotor.set(speed);
	}

	public void setLiftSpeed(double speed){
		//double angle = getLiftAngle();
		//SmartDashboard.putNumber("Lift Angle", angle);
		liftMotor.set(speed);
	}

	// public void setPosition(){
	// 	liftMotor.getEncoder().setPosition(100);
	// }

}