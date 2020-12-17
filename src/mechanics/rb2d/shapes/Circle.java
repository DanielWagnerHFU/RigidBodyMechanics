package mechanics.rb2d.shapes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.tvg.Shape;
import de.physolator.usr.tvg.TVG;

public class Circle extends AbstractShape {

	public double radius;

	public Circle() {
		this(1);
	}

	public Circle(double radius) {
		this.radius = radius;
	}


	@Override
	public void paint(TVG tvg, Vector2D position, double phi) {

		Vector2D rm = new Vector2D(radius, 0);
		Vector2D line = rotateVector2D(rm, phi);
		line.add(position);

		tvg.beginShape(Shape.POLYGON_LINE_LOOP);
		tvg.drawCircle(position, radius);
		tvg.drawLine(position.x, position.y, line.x, line.y);
		tvg.endShape();

	}

	private Vector2D rotateVector2D(Vector2D r, double rot) {
		return new Vector2D(r.x * cos(rot) - r.y * sin(rot), r.x * sin(rot) + r.y * cos(rot));
	}

	@Override
	public double getI(double m) {
		return m*radius*radius;
	}
	
	@Override
	public double getRadius() {
		return radius;
	}

}
