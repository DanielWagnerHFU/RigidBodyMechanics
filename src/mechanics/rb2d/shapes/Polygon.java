package mechanics.rb2d.shapes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeSet;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.tvg.Shape;
import de.physolator.usr.tvg.TVG;

public class Polygon extends AbstractShape{
	public Vector2D[] vertices;
	private double precision = 0.001;

	public Polygon(Vector2D[] vertexList) {
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

	@Override
	public double getI(double m) {
		TreeSet<Double> verticesX = new TreeSet<Double>();
		TreeSet<Double> verticesY = new TreeSet<Double>();
		for (Vector2D vertex : vertices) {
			verticesX.add(vertex.x);
			verticesY.add(vertex.y);
		}
		double maxX = verticesX.first();
		double minX = verticesX.last();
		double maxY = verticesY.first();
		double minY = verticesY.last();

		Vector2D gridPointer = new Vector2D(minX, minY);
		List<Vector2D> pointsInsidePolygon = new ArrayList<Vector2D>();
		while (gridPointer.x <= maxX) {
			while (gridPointer.y <= maxY) {
				if (pointInsidePolygon(gridPointer, vertices))
					pointsInsidePolygon.add(new Vector2D(gridPointer.x, gridPointer.y));
				gridPointer.y += precision;
			}
		}
		
		Vector2D centroid = getCentroid(pointsInsidePolygon);
		double momentOfInertia = getMomentOfInertia(m, centroid, pointsInsidePolygon);
		for(Vector2D vertex: vertices) {
			vertex.sub(centroid);
		}
				
		return momentOfInertia;
	}

	private double getMomentOfInertia(double m, Vector2D centroid, List<Vector2D> pointsInsidePolygon) {
		// TODO Auto-generated method stub
		return 0;
	}

	private Vector2D getCentroid(List<Vector2D> pointsInsidePolygon) {
		// TODO Auto-generated method stub
		return null;
	}

	private boolean pointInsidePolygon(Vector2D gridPointer, Vector2D[] vertices2) {
		// TODO Auto-generated method stub
		return false;
	}
}
