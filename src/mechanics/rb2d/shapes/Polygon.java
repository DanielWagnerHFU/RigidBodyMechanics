package mechanics.rb2d.shapes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import de.physolator.usr.components.Vector2D;
import de.physolator.usr.components.VectorMath;
import de.physolator.usr.tvg.Shape;
import de.physolator.usr.tvg.TVG;
import geometry.Polygon2D;

public class Polygon extends AbstractShape{
	public Vector2D[] vertices;
	private double precision = 0.02;

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
		LinkedList<Double> verticesX = new LinkedList<Double>();
		LinkedList<Double> verticesY = new LinkedList<Double>();
		for (Vector2D vertex : vertices) {
			verticesX.add(vertex.x);
			verticesY.add(vertex.y);
		}
		double[] verticesXArray = new double[verticesX.size()];
		double[] verticesYArray = new double[verticesY.size()];
		for (int i = 0; i < vertices.length; i++) {
			Vector2D vertex = vertices[i];
			verticesXArray[i] = vertex.x;
			verticesYArray[i] = vertex.y;
		}
		Collections.sort(verticesX);
		Collections.sort(verticesY);
		double minX = verticesX.getFirst();
		double maxX = verticesX.getLast();
		double minY = verticesY.getFirst();
		double maxY = verticesY.getLast();

		Vector2D gridPointer = new Vector2D(minX, minY);
		List<Vector2D> pointsInsidePolygon = new ArrayList<Vector2D>();
		Polygon2D polygon = new Polygon2D(verticesXArray, verticesYArray, verticesXArray.length);
		
		while (gridPointer.x <= maxX) {
			gridPointer.y = minY;
			while (gridPointer.y <= maxY) {
				if (pointInsidePolygon(gridPointer, vertices, polygon)) {
					pointsInsidePolygon.add(new Vector2D(gridPointer.x, gridPointer.y));
				}
				gridPointer.y += precision;
			}
			gridPointer.x += precision;
		}
		Vector2D centroid = getCentroid(pointsInsidePolygon);
		for(Vector2D vertex: vertices)
			vertex.sub(centroid);
				
		return getMomentOfInertia(m, centroid, pointsInsidePolygon);
	}

	private double getMomentOfInertia(double m, Vector2D centroid, List<Vector2D> pointsInsidePolygon) {
		double pointMass = m / pointsInsidePolygon.size();
		double I = 0;
		for(Vector2D point : pointsInsidePolygon) {
			double r = VectorMath.dist(centroid, point);
			I += (pointMass * (r * r));
		}
		return I;
	}

	private Vector2D getCentroid(List<Vector2D> pointsInsidePolygon) {
		Vector2D centroid = new Vector2D(0, 0);
		for(Vector2D point : pointsInsidePolygon) {
			centroid.add(point);
		}
		int k = pointsInsidePolygon.size();
		return new Vector2D(centroid.x/k,centroid.y/k);
	}

	private boolean pointInsidePolygon(Vector2D gridPointer, Vector2D[] vertices, Polygon2D polygon) {
		return polygon.contains(gridPointer.x, gridPointer.y);
	}

	@Override
	public double getRadius() {
		// TODO Auto-generated method stub
		return 0;
	}
}
