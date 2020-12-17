package mechanics.rb2d;

import java.awt.geom.Line2D;
import java.awt.geom.Line2D.Double;

import de.physolator.usr.components.Vector2D;
import javafx.scene.shape.Line;
import mechanics.rb2d.shapes.Circle;

public class Impactpoint {
	public Vector2D impactPoint;
	public Vector2D impactEdge;
	public RigidBody rb_point;
	public RigidBody rb_edge;
	Line2D.Double impactEdgeLine;

	public Impactpoint(Vector2D impactPoint, Vector2D impactEdge, RigidBody rb_point, RigidBody rb_edge) {
		this.impactPoint = impactPoint;
		this.impactEdge = impactEdge;
		this.rb_point = rb_point;
		this.rb_edge = rb_edge;
	}
	
	public Impactpoint(Vector2D impactPoint, Vector2D impactEdge, RigidBody rb_point, RigidBody rb_edge, Line2D.Double impactEdgeLine) {
		this.impactPoint = impactPoint;
		this.impactEdge = impactEdge;
		this.rb_point = rb_point;
		this.rb_edge = rb_edge;
		this.impactEdgeLine = impactEdgeLine;
	}
}
