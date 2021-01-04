package mechanics.rb2d.misc;

import de.physolator.usr.components.Vector2D;

public class Impactpoint {
	public Vector2D impactPoint;
	public Vector2D impactEdge;

	public Impactpoint(Vector2D impactPoint, Vector2D impactEdge) {
		this.impactPoint = impactPoint;
		this.impactEdge = impactEdge;
	}

	public Impactpoint(Vector2D impactPoint) {
		this.impactPoint = impactPoint;
	}
}
