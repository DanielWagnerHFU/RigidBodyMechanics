package mechanics.tvg;

import de.physolator.usr.StructureElement;

public class PM2D {

	public Object object;

	public StructureElement rx, ry, vx, vy, ax, ay, Frx, Fry, Fnx, Fny, Fgx, Fgy, Fhx, Fhy, Fresx, Fresy;

	public double m;
	public double radius;
	public String name;

	public PM2D(Object object, StructureElement rx, StructureElement ry, StructureElement vx, StructureElement vy,
			StructureElement ax, StructureElement ay, StructureElement Fhx,	StructureElement Fhy, StructureElement Frx,	StructureElement Fry, double m, double radius,
			String name) {
		super();
		this.object = object;
		this.rx = rx;
		this.ry = ry;
		this.vx = vx;
		this.vy = vy;
		this.ax = ax;
		this.ay = ay;
		this.Frx = Frx;
		this.Fry = Fry;
		this.Fnx = Fnx;
		this.Fny = Fny;
		this.Fgx = Fgx;
		this.Fgy = Fgy;
		this.Fhx = Fhx;
		this.Fhy = Fhy;
		this.Fresx = Fresx;
		this.Fresy = Fresy;
		this.m = m;
		this.radius = radius;
		this.name = name;
	}

	public PM2D(Object object, StructureElement rx, StructureElement ry, StructureElement vx, StructureElement vy,
			StructureElement ax, StructureElement ay, double m, double radius, String name) {
		super();
		this.object = object;
		this.rx = rx;
		this.ry = ry;
		this.vx = vx;
		this.vy = vy;
		this.ax = ax;
		this.ay = ay;
		this.m = m;
		this.radius = radius;
		this.name = name;
	}

}
