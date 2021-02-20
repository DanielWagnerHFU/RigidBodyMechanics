package mechanics.rb2d;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.tvg.TVG;
import mechanics.tvg.IDrawable;

public class RigidBody implements IDrawable{
	
	@Override
	public void paint(TVG tvg, Vector2D translationVector, double rotationAngle) {
		tvg.drawCircle(translationVector, 1);
	}

}
