package mechanics.rb2d;

import de.physolator.usr.GraphicsComponents;
import de.physolator.usr.PhysicalSystem;
import de.physolator.usr.Recorder;
import de.physolator.usr.SimulationParameters;
import de.physolator.usr.Structure;

public class PongPS extends PhysicalSystem {

	
	public int zähler = 0;
	public double position = 0;

	@Override
	public void initGraphicsComponents(GraphicsComponents g, Structure s,
			Recorder r, SimulationParameters sp) {
		g.addTVG(new PongTVG(this,s,r));		
	}
	public static void main(String args[]) {
		start();
	}
	
	

}
