package actr.tasks.driving;

import java.awt.BorderLayout;
// import java.io.BufferedWriter;
// import java.io.FileWriter;
// import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Vector;

import javax.swing.JLabel;

import actr.model.Model;
import actr.task.Result;
import actr.task.Task;

/**
 * The main Driving task class that sets up the simulation and starts periodic
 * updates.
 * 
 * @author Dario Salvucci
 */
public class Driving extends actr.task.Task {
	static Simulator simulator = null;

	Simulation simulation;
	JLabel nearLabel, farLabel, signLabel, carLabel, speedoLabel, mirrorLabel;

	final double scale = 0.6; // .85 //.6
	final double steerFactor_dfa = (16 * scale);
	final double steerFactor_dna = (4 * scale);
	final double steerFactor_na = (3 * scale);
	final double steerFactor_fa = (0 * scale);
	final double accelFactor_thw = (1 * .40);
	final double accelFactor_dthw = (3 * .40);
	final double steerNaMax = .07;
	final double thwFollow = 1.0;
	final double thwMax = 4.0;

	double startTime = 0, endTime = 180;
	double accelBrake = 0, speed = 0;

	static int minX = 174, maxX = (238 + 24), minY = 94, maxY = (262 + 32);
	static int centerX = (minX + maxX) / 2, centerY = (minY + maxY) / 2;

	String previousLimit = "0";
	int nback_count = 0;
	double signOnset = 0;
	double instructionsOnset = 0;
	double warningOnset = 0;
	static boolean instructionsSeen = false;
	static boolean warningSeen = false;
	static int speedI = 0;
	static String currentNBack = "";
	String[] nBack_list = { "2back", "3back", "0back", "1back", "4back", "0back", "3back", "4back", "1back", "2back" };
	double sign_count = 0;
	int rehearsal_count = 0;
	static String imaginedSpeedlimit = "";
	List<String> output = new ArrayList<String>();

	public Driving() {
		super();
		nearLabel = new JLabel(".");
		farLabel = new JLabel("X");
		carLabel = new JLabel(";");
		signLabel = new JLabel("*");
		speedoLabel = new JLabel("~");
		mirrorLabel = new JLabel("+");
	}

	public void start() {
		simulation = new Simulation(getModel());

		if (getModel().getRealTime()) {
			setLayout(new BorderLayout());
			if (simulator == null)
				simulator = new Simulator();
			add(simulator, BorderLayout.CENTER);
			simulator.useSimulation(simulation);
		} else {
			add(nearLabel);
			nearLabel.setSize(20, 20);
			nearLabel.setLocation(250, 250);
			add(farLabel);
			farLabel.setSize(20, 20);
			farLabel.setLocation(250, 250);
			add(speedoLabel);
			speedoLabel.setSize(20, 20);
			speedoLabel.setLocation(300, 350);
			add(mirrorLabel);
			mirrorLabel.setSize(20, 20);
			mirrorLabel.setLocation(230, 20);
			add(carLabel);
			carLabel.setSize(20, 20);
			carLabel.setLocation(250, 250);
			add(signLabel);
			signLabel.setSize(20, 20);
			signLabel.setLocation(250, 250);
		}

		getModel().runCommand("(set-visual-frequency near .1)");
		getModel().runCommand("(set-visual-frequency far .1)");
		getModel().runCommand("(set-visual-frequency car .1");

		// they don't reset otherwise if you run the model multiple times in a row
		accelBrake = 0;
		speed = 0;
		previousLimit = "60";

		getModel().getVision().addVisual("near", "near", "near", nearLabel.getX(), nearLabel.getY(), 1, 1, 10);
		getModel().getVision().addVisual("far", "far", "far", farLabel.getX(), farLabel.getY(), 1, 1, 100);
		getModel().getVision().addVisual("speedometer", "speedometer", "speedometer", 260, 300, 10, 10, 1);
		getModel().getVision().addVisual("mirror", "mirror", "road-clear", 230, 20, 10, 10, 1);
		addPeriodicUpdate(Env.sampleTime);
	}

	public void update(double time) {
		simulation.env.autocar.visible = true;
		if (time <= endTime) {
			simulation.env.time = time - startTime;
			updateVisuals();
			simulation.update();
		} else {
			getModel().stop();
			output("myData", simulation.samples);
		}
	}

	// save to file
	void output(String filename, Vector<Sample> samples) {
		for (int i = 1; i < samples.size(); i++) {
			Sample s = samples.elementAt(i);
			if (i == 1) {
				output.add(s.listVars() + System.lineSeparator());
				output.add(s.toString() + System.lineSeparator());
			} else
				output.add(s.toString() + System.lineSeparator());
		}
		Model.print(output, "_driving_");
	}

	void updateVisuals() {

		Env env = simulation.env;
		// egocar
		if (env.simcar.nearPoint != null) {
			Coordinate cn = env.world2image(env.simcar.nearPoint);
			Coordinate cf = env.world2image(env.simcar.farPoint);
			if (cn == null || cf == null) {
				env.done = true;
			} else {
				nearLabel.setLocation(cn.x, cn.y);
				farLabel.setLocation(cf.x, cf.y);
				getModel().getVision().moveVisual("near", cn.x, cn.y);
				getModel().getVision().moveVisual("far", cf.x, cf.y);
			}
		}

		// speed sign
		Coordinate cs = null;
		if (env.speedsign.signPos != null)
			cs = env.world2image(env.speedsign.signPos);
		if (getModel().getVision().visualObjects().contains("speedsign") == false && env.speedsign.newSign == true) {
			signLabel.setLocation(cs.x, cs.y);
			// if (cs.d < 30) {
			getModel().getVision().addVisual("speedsign", "speedsign", env.speedsign.speedlimit, cs.x, cs.y, 1, 1, 100);
			env.speedsign.visible = true;
			env.speedsign.newSign = false;
			// }
		} else if (cs != null)

		{
			signLabel.setLocation(cs.x, cs.y);
			getModel().getVision().moveVisual("speedsign", cs.x, cs.y);
		} else {
			getModel().getVision().removeVisual("speedsign");
			env.speedsign.visible = false;
		}

		// autocar
		if (env.autocar.p != null && env.autocar.visible) {
			Coordinate cc = env.world2image(env.autocar.p);

			if (cc != null) {
				if (getModel().getVision().visualObjects().contains("car")) {
					carLabel.setLocation(cc.x, cc.y);
					getModel().getVision().moveVisual("car", cc.x, cc.y);
					getModel().getVision().changeValue("car", env.autocar.distance);
				} else {
					if (cc.d > 3 && cc.d < 15) {
						getModel().getVision().removeVisual("car");
						getModel().getVision().addVisual("car", "car", env.autocar.distance, cc.x, cc.y, 1, 1);
					}
				}
			} else {
				getModel().getVision().removeVisual("car");
			}
			// double distance = Math.abs(env.autocar.fracIndex - env.simcar.fracIndex);
		}

		// speedometer
		String speed = Integer.toString((int) Utilities.mph2kph(Utilities.mps2mph(simulation.env.simcar.speed)));
		getModel().getVision().changeValue("speedometer", speed);
		env.done = true;
	}

	double minSigned(double x, double y) {
		return (x >= 0) ? Math.min(x, y) : Math.max(x, -y);
	}

	void doSteer(double na, double dna, double dfa, double dt) {
		Simcar simcar = simulation.env.simcar;
		if (simcar.speed >= 10.0) {
			double dsteer = (dna * steerFactor_dna) + (dfa * steerFactor_dfa)
					+ (minSigned(na, steerNaMax) * steerFactor_na * dt);
			dsteer *= simulation.driver.steeringFactor;
			simcar.steerAngle += dsteer;
		} else
			simcar.steerAngle = 0;
	}

	void doAccelerate(double fthw, double dthw, double dt) {
		Simcar simcar = simulation.env.simcar;
		if (simcar.speed >= 10.0) {
			double dacc = (dthw * accelFactor_dthw) + (dt * (fthw - thwFollow) * accelFactor_thw);
			accelBrake += dacc;
			accelBrake = minSigned(accelBrake, 1.0);
		} else {
			accelBrake = .65 * (simulation.env.time / 3.0);
			accelBrake = minSigned(accelBrake, .65);
		}
		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
		simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
		System.out.println("fthw: " + fthw + " dthw: " + dthw + " dt: " + dt);
	}

	void keepLimit(double tlimit) {
		imaginedSpeedlimit = Integer.toString((int) tlimit); // for sampling
		Simcar simcar = simulation.env.simcar;
		double speed = simcar.speed;
		tlimit = Utilities.mph2mps(Utilities.kph2mph(tlimit));
		double diff = (tlimit - speed);

		if (Math.abs(tlimit - speed) > 0) {
			// double dacc = (dthw * accelFactor_dthw 1.2)
			// + (dt * (fthw - thwFollow 1.0) * accelFactor_thw 0.4);
			double dacc = (diff * 1.2) + (0.25 * diff * 0.4);
			accelBrake += dacc;
			accelBrake = minSigned(accelBrake, 1.0);
			// System.out.println("dacc: " + dacc + " accelBrake: " + accelBrake + " diff: "
			// + diff);
		}
		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
		simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
	}

	boolean isCarStable(double na, double nva, double fva) {
		double f = 2.5;
		return (Math.abs(na) < .025 * f) && (Math.abs(nva) < .0125 * f) && (Math.abs(fva) < .0125 * f);
	}

	boolean isLaneChangeNeeded(Double dist, Double cthw) {
		if (cthw < 10 || dist < 20)
			return true;
		return false;
	}

	double image2angle(double x, double d) {
		Env env = simulation.env;
		double px = env.simcar.p.x + (env.simcar.h.x * d);
		double pz = env.simcar.p.z + (env.simcar.h.z * d);
		Coordinate im = env.world2image(new Position(px, pz));
		try {
			return Math.atan2(.5 * (x - im.x), 450);
		} catch (Exception e) {
			return 0;
		}
	}

	void changeLane() {
		Env env = simulation.env;
		env.simcar.lane = 1;
	}

	public void eval(Iterator<String> it) {
		it.next();
		String cmd = it.next();
		if (cmd.equals("do-steer")) {
			double na = Double.valueOf(it.next());
			double dna = Double.valueOf(it.next());
			double dfa = Double.valueOf(it.next());
			double dt = Double.valueOf(it.next());
			doSteer(na, dna, dfa, dt);
		} else if (cmd.equals("do-accelerate")) {
			double fthw = Double.valueOf(it.next());
			double dthw = Double.valueOf(it.next());
			double dt = Double.valueOf(it.next());
			doAccelerate(fthw, dthw, dt);
		} else if (cmd.equals("keep-limit")) {
			double tlimit = Double.valueOf(it.next());
			keepLimit(tlimit);
		} else if (cmd.equals("change-lane")) {
			changeLane();
		} else if (cmd.equals("placeholder")) {
			//
		}
	}

	public boolean evalCondition(Iterator<String> it) {
		it.next();
		String cmd = it.next();
		if (cmd.equals("is-car-stable") || cmd.equals("is-car-not-stable")) {
			double na = Double.valueOf(it.next());
			double nva = Double.valueOf(it.next());
			double fva = Double.valueOf(it.next());
			boolean b = isCarStable(na, nva, fva);
			return cmd.equals("is-car-stable") ? b : !b;
		} else if (cmd.equals("car-too-close") || cmd.equals("car-not-too-close")) {
			double cd = Double.valueOf(it.next());
			double cthw = Double.valueOf(it.next());
			boolean c = isLaneChangeNeeded(cd, cthw);
			return cmd.equals("car-too-close") ? c : !c; // XXX c : !c
		} else
			return false;
	}

	public double bind(Iterator<String> it) {
		try {
			it.next(); // (
			String cmd = it.next();
			if (cmd.equals("image->angle")) {
				double x = Double.valueOf(it.next());
				double d = Double.valueOf(it.next());
				return image2angle(x, d);
			} else if (cmd.equals("mp-time"))
				return simulation.env.time;
			else if (cmd.equals("get-thw")) {
				double fd = Double.valueOf(it.next());
				double v = Double.valueOf(it.next());
				double thw = (v == 0) ? 4.0 : fd / v;
				return Math.min(thw, 4.0);
			} else if (cmd.equals("get-velocity"))
				return speed;
			else if (cmd.equals("get-chunk-id")) {
				// time as unique id for chunks
				double cid = (int) simulation.env.time;
				return cid;
			} else if (cmd.equals("get-num-sign")) {
				// number of signs passed
				sign_count += 1;
				return sign_count;
			} else if (cmd.equals("get-num-rehearsal")) {
				// number of rehearals per iteration
				rehearsal_count += 1;
				return rehearsal_count;
			} else if (cmd.equals("reset-rehearsal")) {
				// number of rehearsals in a loop
				rehearsal_count = 0;
				return rehearsal_count;
			} else if (cmd.equals("autocar-lane")) {
				// what lane is the other car driving on?
				return simulation.env.autocar.lane;
			} else if (cmd.equals("simcar-lane")) {
				// what lane am I driving on?
				return simulation.env.simcar.lane;
			} else if (cmd.equals("placeholder")) {
				return 0;
			} else
				return 0;
		} catch (Exception e) {
			e.printStackTrace();
			System.exit(1);
			return 0;
		}
	}

	void incNum(int rehearsal_count, boolean startover) {
		rehearsal_count = startover ? 0 : rehearsal_count + 1;
	}

	public int numberOfSimulations() {
		return 1;
	}

	public Result analyze(Task[] tasks, boolean output) {
		return null;
	}

}
