package actr.tasks.driving;

import java.awt.BorderLayout;
// import java.io.BufferedWriter;
// import java.io.FileWriter;
// import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Vector;

import javax.swing.JLabel;

import actr.model.Model;
import actr.task.Result;
import actr.task.Task;

/**
 * The main Driving task class that sets up the simulation and starts periodic updates.
 *  
 * @author Dario Salvucci
 */
public class Driving extends actr.task.Task
{
	static Simulator simulator = null;

	Simulation simulation;
	JLabel nearLabel, carLabel, speedsign, instructions, warning, speedometer;

	final double scale = .6; // .85
	final double steerFactor_dfa = (16 * scale);
	final double steerFactor_dna = (4 * scale);
	final double steerFactor_na  = (3 * scale);
	final double steerFactor_fa  = (0 * scale);
	final double accelFactor_thw  = (1 * .40);
	final double accelFactor_dthw = (3 * .40);
	final double steerNaMax = .07;
	final double thwFollow = 1.0;
	final double thwMax = 4.0;

	double startTime=0, endTime=180;
	double accelBrake=0, speed=0;

	static int minX=174, maxX=(238+24), minY=94, maxY=(262+32);
	static int centerX=(minX+maxX)/2, centerY=(minY+maxY)/2; 

	//make it bigger if the experiment is longer than 60s or signs appear more often
	static String currentLimit = "60"; //starting speed
	String previousLimit = "0";
	int nback_count = 0;
	double signOnset = 0; //a local variable would be better but this shows how long the sign has been visible.
	double instructionsOnset = 0;
	double warningOnset = 0;
	boolean signSeen = false; //same
	static boolean instructionsSeen = false;
	static boolean warningSeen = false;
	static int speedI = 0;
	static Coordinate signPos;
	static String currentNBack = "";
	String[] nBack_list = {"2back", "3back", "0back", "1back", "4back", "0back", "3back", "4back", "1back", "2back"};
	double sign_count = 0;
	int rehearsal_count = 0;
	static String imaginedSpeedlimit = "";
	List<String> output = new ArrayList<String>();	

	public Driving ()
	{
		super ();
		nearLabel = new JLabel (".");
		carLabel = new JLabel ("X");
		speedsign = new JLabel ("*");
		instructions = new JLabel (",");
		warning = new JLabel (";");
		speedometer = new JLabel("~");
	}

	public void start ()
	{
		simulation = new Simulation (getModel());

		if (getModel().getRealTime())
		{
			setLayout (new BorderLayout());
			if (simulator == null) simulator = new Simulator ();
			add (simulator, BorderLayout.CENTER);
			simulator.useSimulation (simulation);
		}
		else
		{
			add (nearLabel);
			nearLabel.setSize (20, 20);
			nearLabel.setLocation (250, 250);
			add (carLabel);
			carLabel.setSize (20, 20);
			carLabel.setLocation (250, 250);
			add (speedsign);
			speedsign.setSize (20, 20);
			speedsign.setLocation (250, 250);
			add (instructions);
			instructions.setSize(20,20);
			instructions.setLocation(200, 100);
			add (warning);
			warning.setSize(20, 20);
			warning.setLocation(200, 50);
			add(speedometer);
			speedometer.setSize(20,20);
			speedometer.setLocation(300, 350);
		}

		getModel().runCommand ("(set-visual-frequency near .1)");
		getModel().runCommand ("(set-visual-frequency car .1)");

		//they don't reset otherwise if you run the model multiple times in a row
		accelBrake = 0;
		speed = 0;
		previousLimit = "60";
		currentLimit = "60";

		getModel().getVision().addVisual ("near", "near", "near", nearLabel.getX(), nearLabel.getY(), 1, 1, 10);
		getModel().getVision().addVisual ("car", "car", "car", carLabel.getX(), carLabel.getY(), 1, 1, 100);
		getModel().getVision().addVisual ("speedometer", "speedometer", "speedometer", 260, 300, 1, 1, 1);
		addPeriodicUpdate (Env.sampleTime);

	}

	public void update (double time)
	{
		if (time <= endTime)
		{
			simulation.env.time = time - startTime; 
			updateVisuals();
			if(simulation.env.time>10) //only start after 10s
				updateSign(simulation.env.time);
			//updateInstructions(simulation.env.time);
			simulation.update();
			updateSpeedometer();
		}
		else 
		{
			getModel().stop();
			output("myData", simulation.samples);
		}

	}

	//	save to file
	void output(String filename, Vector<Sample> samples)
	{
		for (int i = 1 ; i < samples.size(); i++)
		{
			Sample s = samples.elementAt(i);
			if(i == 1) {
				output.add(s.listVars() + System.lineSeparator());
				output.add(s.toString() + System.lineSeparator());
			}else
				output.add(s.toString() + System.lineSeparator());
			}
		Model.print(output, "_driving_");
	}

	void updateInstructions(double time)
	{
		//reverse order randomly for participants
		if (nback_count == 0 && Math.round(Math.random()) == 1)
		{
			//keep the old order
		}else if(nback_count == 0)
		{
			nBack_list = new String[] {"2back", "1back", "4back", "3back", "0back", "4back", "1back", "0back", "3back", "2back"};
		}

		//add instructions
		if((int)time%180 == 0 && instructionsSeen == false)
		{
			currentNBack = nBack_list[nback_count];
			getModel().getVision().addVisual("instructions", "instructions", currentNBack, 200, 100, 50, 50);
			instructions.setLocation(200, 100);
			instructionsOnset = time;
			instructionsSeen = true;
			nback_count += 1;
		}else if(time - instructionsOnset >= 2)
		{
			getModel().getVision().removeVisual("instructions");
			instructionsSeen = false;
		}
	}

	void updateSign(double time)
	{
		String[] speedlimits = {"60", "70", "80", "90", "100", "110", "120", "130", "140"};		
		Env env = simulation.env;

		if((int)time%20 == 0 && signSeen == false && (speedI < speedlimits.length) )
		{
			if (env.simcar.nearPoint != null)

			{		
				//pick a random speed with delta<30
				previousLimit = currentLimit;
				while (Math.abs(Integer.parseInt(currentLimit) - Integer.parseInt(previousLimit)) > 30 || (currentLimit == previousLimit))
				{
					int rnd = new Random().nextInt(speedlimits.length);
					currentLimit = speedlimits[rnd];
				}

				Position newLoc = Road.location(env.simcar.fracIndex + 20 , 3);
				newLoc.y = 0.0;
				signPos = env.world2image(newLoc);

				getModel().getVision().addVisual ("speedsign", "speedsign", currentLimit, (int)signPos.x, (int)signPos.y, 1, 1, 100);
				speedsign.setLocation((int)signPos.x, (int)signPos.y);

				signOnset = time;
				signSeen = true;
			}
		} else if(time - signOnset >= 3)
		{
			getModel().getVision().removeVisual("speedsign");
			signSeen = false;
		} else
		{
			//not the prettiest solution, but for now it works
			signPos.x += 5;
			signPos.y += 3;
			getModel().getVision().moveVisual("speedsign", (int)signPos.x, (int)signPos.y);
			speedsign.setLocation((int)signPos.x , (int)signPos.y);
		}

	}

	void updateSpeedometer()
	{
		Simcar simcar = simulation.env.simcar;
		String speed = Integer.toString((int)Utilities.mps2mph(Utilities.mph2kph(simcar.speed)));
		getModel().getVision().removeVisual("speedometer");
		getModel().getVision().addVisual ("speedometer", "speedometer", speed, 260, 300, 1, 1, 1);
	}

	void updateVisuals ()
	{
		Env env = simulation.env;
		if (env.simcar.nearPoint != null)
		{
			Coordinate cn = env.world2image (env.simcar.nearPoint);
			Coordinate cc = env.world2image (env.simcar.carPoint);
			//Coordinate cc = env.world2image (env.simcar.farPoint);

			if (cn == null || cc == null) env.done = true;
			else
			{
				nearLabel.setLocation (cn.x, cn.y);
				carLabel.setLocation (cc.x, cc.y);
				getModel().getVision().moveVisual ("near", cn.x, cn.y);
				getModel().getVision().moveVisual ("car", cc.x, cc.y);
				//				speed = env.simcar.speed;

			}
		}
	}

	double minSigned (double x, double y)
	{
		return (x>=0) ? Math.min (x,y) : Math.max (x,-y);
	}

	void doSteer (double na, double dna, double dfa, double dt)
	{
		Simcar simcar = simulation.env.simcar;
		if (simcar.speed >= 10.0)
		{
			double dsteer = (dna * steerFactor_dna)
					+ (dfa * steerFactor_dfa)
					+ (minSigned (na, steerNaMax) * steerFactor_na * dt);
			dsteer *= simulation.driver.steeringFactor;
			simcar.steerAngle += dsteer;
		}
		else simcar.steerAngle = 0;
	}

	void doAccelerate (double fthw, double dthw, double dt, double tlimit)
	{
		Simcar simcar = simulation.env.simcar;
		if (simcar.speed >= 10.0)
		{
			double dacc = (dthw * accelFactor_dthw)
					+ (dt * (fthw - thwFollow) * accelFactor_thw);
			accelBrake += dacc;
			accelBrake = minSigned (accelBrake, 1.0);
		}
		else
		{
			accelBrake = .65 * (simulation.env.time / 3.0);
			accelBrake = minSigned (accelBrake, .65);
		}
		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
		simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
	}

	void keepLimit(double tlimit)
	{		
		imaginedSpeedlimit = Integer.toString((int)tlimit); //for sampling
		Simcar simcar = simulation.env.simcar;
		double speed = simcar.speed;
		tlimit = Utilities.mph2mps(Utilities.kph2mph(tlimit));
		double diff = (tlimit - speed);
		double time = simulation.env.time - startTime;

		//display warning if the speed limit is not kept
		if(Math.abs(diff)>1.39 && warningSeen == false && (time - signOnset)>5) //1.39 m/s is roughly 5 km/h
		{
			warningOnset = time;
			getModel().getVision().addVisual("warning", "warning", "warning", 200, 100, 50, 50);
			warning.setLocation(200, 50);
			warningSeen = true;
		}else if (time - warningOnset > 3)
		{
			getModel().getVision().removeVisual("warning");
			//warningOnset = time;
			warningSeen = false;
		}

		if (Math.abs(tlimit - speed) > 0)
		{
			//			double dacc = (dthw * accelFactor_dthw 1.2)
			//			+ (dt * (fthw - thwFollow 1.0) * accelFactor_thw 0.4);
			if(diff>1.0) {
				double dacc = (diff*10 * 1.2) + (0.25 * diff * 0.4);
				accelBrake += dacc;
				accelBrake = minSigned (accelBrake, 1.5); // 1.0
			}else
			{
				double dacc = (diff * 1.2) + (0.25 * diff * 0.4);
				accelBrake += dacc;
				accelBrake = minSigned (accelBrake, 1.0);
			}
		}
		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
		simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
	}

	boolean isCarStable (double na, double nva, double fva)
	{
		double f = 2.5;
		return (Math.abs(na) < .025*f) && (Math.abs(nva) < .0125*f) && (Math.abs(fva) < .0125*f);
	}

	double image2angle (double x, double d)
	{
		Env env = simulation.env;
		double px = env.simcar.p.x + (env.simcar.h.x * d);
		double pz = env.simcar.p.z + (env.simcar.h.z * d);
		Coordinate im = env.world2image (new Position (px, pz));
		try { return Math.atan2 (.5*(x-im.x), 450); }
		catch (Exception e) { return 0; }
	}

	public void eval (Iterator<String> it)
	{
		it.next();
		String cmd = it.next();
		if (cmd.equals ("do-steer"))
		{
			double na = Double.valueOf (it.next());
			double dna = Double.valueOf (it.next());
			double dfa = Double.valueOf (it.next());
			double dt = Double.valueOf (it.next());
			doSteer (na, dna, dfa, dt);
		}
		else if (cmd.equals ("do-accelerate"))
		{
			double fthw = Double.valueOf (it.next());
			double dthw = Double.valueOf (it.next());
			double dt = Double.valueOf (it.next());
			double tlimit = Double.valueOf(it.next());
			doAccelerate (fthw, dthw, dt, tlimit);
		}
		else if (cmd.equals ("keep-limit"))
		{
			double tlimit = Double.valueOf (it.next());
			keepLimit (tlimit);
		}
		else if (cmd.equals ("placeholder"))
		{
			//
		}
	}

	public boolean evalCondition (Iterator<String> it)
	{
		it.next();
		String cmd = it.next();
		if (cmd.equals ("is-car-stable") || cmd.equals ("is-car-not-stable"))
		{
			double na = Double.valueOf (it.next());
			double nva = Double.valueOf (it.next());
			double fva = Double.valueOf (it.next());
			boolean b = isCarStable(na,nva,fva);
			return cmd.equals("is-car-stable") ? b : !b;
		}
		else return false;
	}

	public double bind (Iterator<String> it)
	{
		try
		{
			it.next(); // (
			String cmd = it.next();
			if (cmd.equals ("image->angle"))
			{
				double x = Double.valueOf (it.next());
				double d = Double.valueOf (it.next());
				return image2angle (x, d);
			}
			else if (cmd.equals ("mp-time")) return simulation.env.time;
			else if (cmd.equals ("get-thw"))
			{
				double fd = Double.valueOf (it.next());
				double v = Double.valueOf (it.next());
				double thw = (v==0) ? 4.0 : fd/v;
				return Math.min (thw, 4.0);
			}
			else if (cmd.equals ("get-velocity")) return speed;
			else if (cmd.equals ("get-chunk-id"))
			{
				//every chunk needs a unique id. Time is unique so I just used that
				double cid = (int)simulation.env.time;
				return cid;
			}
			else if (cmd.equals ("get-num-sign"))
			{
				//number of signs passed
				sign_count += 1;
				return sign_count;
			}
			else if (cmd.equals ("get-num-rehearsal"))
			{
				//number of rehearals per iteration
				rehearsal_count += 1;
				return rehearsal_count;
			}
			else if (cmd.equals ("reset-rehearsal"))
			{
				//number of rehearsals in a loop
				rehearsal_count = 0;
				return rehearsal_count;
			}
			else return 0;
		}
		catch (Exception e)
		{
			e.printStackTrace();
			System.exit(1);
			return 0;
		}
	}

	void incNum(Integer rehearsal_count, boolean startover)
	{
		if (startover)
			rehearsal_count = 0;
		else
		{
			rehearsal_count += 1;
		}
	}


	public int numberOfSimulations () { return 1; }

	public Result analyze (Task[] tasks, boolean output)
	{
		return null;
	}

}
