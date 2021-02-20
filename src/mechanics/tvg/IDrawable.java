package mechanics.tvg;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.tvg.TVG;

public interface IDrawable {
	public void paint(TVG tvg, Vector2D translationVector, double rotationAngle);
}
