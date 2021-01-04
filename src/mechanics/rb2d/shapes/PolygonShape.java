package mechanics.rb2d.shapes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.tvg.Shape;
import de.physolator.usr.tvg.TVG;

public class PolygonShape extends AbstractShape{
	public Vector2D[] vertices;

	public PolygonShape() {
		this(new Vector2D[] { new Vector2D(-0.5, 0.5), new Vector2D(0.5, 0.5), new Vector2D(0.5, -0.5),
				new Vector2D(-0.5, -0.5) });
	}

	public PolygonShape(Vector2D[] vertexList) {
		vertices = vertexList;
	}

	@Override
	public void paint(TVG tvg, Vector2D position, double phi) {
		tvg.beginShape(Shape.POLYGON_LINE_LOOP);
		for (Vector2D e : vertices)
			tvg.vertex(position.x + cos(phi) * e.x + sin(phi) * e.y,
					position.y + sin(phi) * e.x - cos(phi) * e.y);
		tvg.endShape();
	}
}
