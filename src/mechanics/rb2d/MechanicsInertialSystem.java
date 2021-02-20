package mechanics.rb2d;

import mechanics.tvg.MechanicsTVG;

import de.physolator.usr.*;

public class MechanicsInertialSystem extends PhysicalSystem {

	public RigidBody[] rigidBodyArray;

	@V(unit = "kg*m/s")
	public double energy;

	public MechanicsInertialSystem() {
		rigidBodyArray = new RigidBody[1];
		rigidBodyArray[0] = new RigidBody();
	}

	@Override
	public void f(double t, double dt) {
		
	}

	@Override
	public void g(double t, AfterEventDescription aed) {
		
	}

	@Override
	public void initSimulationParameters(SimulationParameters sp) {
		sp.fastMotionFactor = 1;
	}

	@Override
	public void initGraphicsComponents(GraphicsComponents g, Structure s, Recorder r, SimulationParameters sp) {
		MechanicsTVG mTVG = new MechanicsTVG(this, s, r);
		mTVG.geometry.setUserArea(-5, 5, -5, 5);
		g.addTVG(mTVG);
	}

	public static void main(String args[]) {
		start();
	}
}
