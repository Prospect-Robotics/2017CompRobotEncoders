package org.usfirst.frc2813.Robot4.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
/**
 * 
 * PID controller, but the loop speed is NOT regulated; instead, constants are adjusted on the fly
 * to account for time delta since the last time through the loop.  Useful if the roboRIO is under
 * heavy load (i.e. trying to process vision without a GPU).
 * 
 * @author lisaw
 *
 */
public class RandomPIDController2 extends PIDController {
	
	private final Timer timer = new Timer();

	public RandomPIDController2(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output) {
		super(Kp, Ki, Kd, source, output);
	}

	public RandomPIDController2(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output, double period) {
		super(Kp, Ki, Kd, source, output, period);
	}

	public RandomPIDController2(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output) {
		super(Kp, Ki, Kd, Kf, source, output);
	}

	public RandomPIDController2(double Kp, double Ki, double Kd, double Kf, PIDSource source, PIDOutput output,
			double period) {
		super(Kp, Ki, Kd, Kf, source, output, period);
	}
	
	@Override
	 @SuppressWarnings("LocalVariableName")
	  protected void calculate() {
	    if (m_origSource == null || m_pidOutput == null) {
	      return;
	    }

	    boolean enabled;

	    m_thisMutex.lock();
	    try {
	      enabled = m_enabled;
	    } finally {
	      m_thisMutex.unlock();
	    }

	    if (enabled) {
	      double input;

	      // Storage for function inputs
	      PIDSourceType pidSourceType;
	      double P;
	      double I;
	      double D;
	      double feedForward = calculateFeedForward();
	      double minimumOutput;
	      double maximumOutput;

	      // Storage for function input-outputs
	      double prevError;
	      double error;
	      double totalError;

	      m_thisMutex.lock();
	      try {
	        input = m_pidInput.pidGet();

	        pidSourceType = m_pidInput.getPIDSourceType();
	        P = m_P;
	        I = m_I;
	        D = m_D;
	        minimumOutput = m_minimumOutput;
	        maximumOutput = m_maximumOutput;

	        prevError = m_prevError;
	        error = getContinuousError(m_setpoint - input);
	        totalError = m_totalError;
	      } finally {
	        m_thisMutex.unlock();
	      }

	      // Storage for function outputs
	      double result;

	      if (pidSourceType.equals(PIDSourceType.kRate)) {
	        if (P != 0) {
	          totalError = clamp(totalError + error, minimumOutput / P,
	              maximumOutput / P);
	        }

	        result = P * totalError + D * error + feedForward;
	      } else {
	        if (I != 0) {
	          totalError = clamp(totalError + error, minimumOutput / I,
	              maximumOutput / I);
	        }

	        result = P * error + I * totalError + D * (error - prevError)
	            + feedForward;
	      }

	      result = clamp(result, minimumOutput, maximumOutput);

	      // Ensures m_enabled check and pidWrite() call occur atomically
	      m_pidWriteMutex.lock();
	      try {
	        m_thisMutex.lock();
	        try {
	          if (m_enabled) {
	            // Don't block other PIDController operations on pidWrite()
	            m_thisMutex.unlock();

	            m_pidOutput.pidWrite(result);
	          }
	        } finally {
	          if (m_thisMutex.isHeldByCurrentThread()) {
	            m_thisMutex.unlock();
	          }
	        }
	      } finally {
	        m_pidWriteMutex.unlock();
	      }

	      m_thisMutex.lock();
	      try {
	        m_prevError = error;
	        m_error = error;
	        m_totalError = totalError;
	        m_result = result;
	      } finally {
	        m_thisMutex.unlock();
	      }
	    }
	  }

}
