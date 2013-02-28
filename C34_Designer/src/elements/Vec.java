package elements;

import java.awt.Dimension;
import java.awt.Point;
import java.awt.geom.Dimension2D;
import java.awt.geom.Point2D;

public class Vec {
	static public class Size2D extends Dimension2D {
		public double x, y;

		public Size2D(double width, double height) {
			this.x = width;
			this.y = height;
		}

		@Override
		public double getHeight() {
			return this.x;
		}

		@Override
		public double getWidth() {
			return this.y;
		}

		@Override
		public void setSize(double width, double height) {
			this.x = width;
			this.y = height;
		}
	}

	public static Vec byCort(double x, double y) {
		return new Vec(x, y);
	}

	public static Vec byPolar(double a, double r) {
		return new Vec(r * Math.cos(a), r * Math.sin(a));
	}

	public static double d2r(double d) {
		return d * Math.PI / 180.0;
	}

	static double distance(Vec a, Vec b) {
		return b.sub(a).len();
	}

	static double distance(Vec a, Vec b, Vec c) {
		return Math.abs((b.x - a.x) * (a.y - c.y) - (a.x - c.x) * (b.y - a.y))
				/ distance(a, b);
	}

	public static double r2d(double r) {
		return r * 180.0 / Math.PI;
	}

	public double x, y;

	public Vec(Dimension d) {
		this.x = d.getWidth();
		this.y = d.getHeight();
	}

	public Vec(Dimension2D d) {
		this.x = d.getWidth();
		this.y = d.getHeight();
	}

	public Vec(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public Vec(Point p) {
		this.x = p.x;
		this.y = p.y;
	}

	public Vec(Point2D p) {
		this.x = p.getX();
		this.y = p.getY();
	}

	public Vec(Vec v) {
		this.x = v.x;
		this.y = v.y;
	}

	public Vec add(Vec v) {
		return new Vec(this.x + v.x, this.y + v.y);
	}

	public double ang() {
		return Math.atan2(this.y, this.x);
	}

	public Vec changeAng(double a) {
		return Vec.byPolar(a, len());
	}

	public Vec changeLen(double l) {
		return Vec.byPolar(ang(), l);
	}

	public Dimension getDimension() {
		return new Dimension((int) Math.round(this.x), (int) Math.round(this.y));
	}

	public Dimension2D getDimension2D() {
		return new Size2D(this.x, this.y);
	}

	public int getIntX() {
		return (int) Math.round(this.x);
	}

	public int getIntY() {
		return (int) Math.round(this.y);
	}

	public Point getPoint() {
		return new Point((int) Math.round(this.x), (int) Math.round(this.y));
	}

	public Point2D getPoint2D() {
		return new Point2D.Double(this.x, this.y);
	}

	public double getX() {
		return this.x;
	}

	public double getY() {
		return this.y;
	}

	public double len() {
		return Math.sqrt(this.x * this.x + this.y * this.y);
	}

	public Vec offsetLen(double l) {
		return Vec.byPolar(ang(), len() + l);
	}

	public Vec rotate(double a) {
		return Vec.byPolar(ang() + a, len());
	}

	public Vec scale(double q) {
		return scale(q, q);
	}

	public Vec scale(double q, double w) {
		return new Vec(this.x * q, this.y * w);
	}

	public void set(Vec v) {
		this.x = v.x;
		this.y = v.y;
	}

	public void setAng(double v) {
		set(Vec.byPolar(v, len()));
	}

	public void setLen(double r) {
		set(Vec.byPolar(ang(), r));
	}

	public void setOffset(Vec v) {
		this.x += v.x;
		this.y += v.y;
	}

	public void setX(double v) {
		this.x = v;
	}

	public void setY(double v) {
		this.y = v;
	}

	public Vec sub(Vec v) {
		return new Vec(this.x - v.x, this.y - v.y);
	}

}
