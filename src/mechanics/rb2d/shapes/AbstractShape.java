package mechanics.rb2d.shapes;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.tvg.TVG;

public abstract class AbstractShape {
	public abstract void paint(TVG tvg, Vector2D position, double phi);
	public abstract double getI(double m);
}
