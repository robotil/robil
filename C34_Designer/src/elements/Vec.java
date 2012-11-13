package elements;

import java.awt.Dimension;
import java.awt.Point;
import java.awt.geom.Dimension2D;
import java.awt.geom.Point2D;

public class Vec{
	public double x,y;
	public double getX(){ return x; }
	public double getY(){ return y; }
	public int getIntX(){ return (int)Math.round(x); }
	public int getIntY(){ return (int)Math.round(y); }

	public Point2D getPoint2D(){ return new Point2D.Double(x,y); }
	public Point getPoint(){ return new Point((int)Math.round(x), (int)Math.round(y)); }

	static public class Size2D extends Dimension2D{
		public double x,y;
		public double getHeight() {
			return x;
		}
		public double getWidth() {
			return y;
		}
		public void setSize(double width, double height) {
			x=width; y=height;
		}
		public Size2D(double width, double height) {
			x=width; y=height;
		}
	}
	public Dimension2D getDimension2D(){ return new Size2D(x,y); }
	public Dimension getDimension(){ return new Dimension((int)Math.round(x), (int)Math.round(y)); }
	
	public Vec(double x, double y){ this.x=x; this.y=y; }
	public static Vec byPolar(double a, double r){
		return new Vec(r*Math.cos(a), r*Math.sin(a));
	}
	public static Vec byCort(double x, double y){
		return new Vec(x, y);
	}
	public Vec(Point p){
		x = p.x; y = p.y;
	}
	public Vec(Point2D p){
		x = p.getX(); y = p.getY();
	}
	public Vec(Dimension d){
		x = d.getWidth(); y = d.getHeight();
	}
	public Vec(Dimension2D d){
		x = d.getWidth(); y = d.getHeight();
	}
	
	public double ang(){ return Math.atan2(y, x); }
	public double len(){ return Math.sqrt(x*x+y*y); }
	public void setX(double v){ x=v; }
	public void setY(double v){ y=v; }
	public void setAng(double v){ set(Vec.byPolar(v, len())); }
	public void setLen(double r){ set(Vec.byPolar(ang(), r)); }
	
	public Vec(Vec v){ x=v.x; y=v.y; }
	public void set(Vec v){ x=v.x; y=v.y; }
	
	public Vec add(Vec v){ return new Vec(x+v.x, y+v.y); }
	public Vec sub(Vec v){ return new Vec(x-v.x, y-v.y); }
	public Vec scale(double q, double w){ return new Vec(x*q, y*w); }
	public Vec scale(double q){ return scale(q,q); }
	public Vec rotate(double a){ return Vec.byPolar(ang()+a, len()); }
	public Vec changeAng(double a){ return Vec.byPolar(a, len()); }
	public Vec changeLen(double l){ return Vec.byPolar(ang(), l); }
	public Vec offsetLen(double l){ return Vec.byPolar(ang(), len()+l); }
	
	
	
	public static double d2r(double d) {
		return d*Math.PI/180.0;
	}
	public static double r2d(double r) {
		return r*180.0/Math.PI;
	}
	public void setOffset(Vec v) {
		x+=v.x;y+=v.y;
	}
	
	
	static double distance(Vec a, Vec b){
		return b.sub(a).len();
	}
	static double distance(Vec a, Vec b, Vec c){
		return Math.abs( (b.x-a.x)*(a.y-c.y) - (a.x-c.x)*(b.y-a.y) ) / distance(a,b);
	}
	
}
