/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package pathplanning;
import java.awt.geom.Point2D;
import java.awt.Point;

/**
 *
 * @author user
 */
public class Vec {
    protected double x;
    protected double y;
    static Vec byXY(double x, double y){
        return new Vec(x,y);
    }
    static Vec byPolar(double a, double l){
        return new Vec(Math.cos(a)*l, Math.sin(a)*l);
    }
    public double getX(){return x;}
    public double getY(){return y;}
    public void setX(double x){this.x=x;}
    public void setY(double y){this.y=y;}
    public double getLength(){return Math.sqrt(x*x+y*y);}
    public double getAngle(){return Math.atan2(y, x);}
    public void setLength(double len){ Vec v = Vec.byPolar(getAngle(), len); x=v.x; y=v.y;}
    public void setAngle(double ang){ Vec v = Vec.byPolar(ang, getLength()); x=v.x; y=v.y;}
    
    public Vec add(Vec v){return new Vec(x+v.x, y+v.y);}
    public Vec sub(Vec v){return new Vec(x-v.x, y-v.y);}
    public Vec zoom(double z){return new Vec(x*z, y*z);}
    public Vec scale(double zx, double zy){return new Vec(x*zx, y*zy);}
    
    
    public Vec(){x=0;y=0;}
    public Vec(double x, double y){
        this.x=x;this.y=y;
    }
    public Vec(Point2D point){
        x= point.getX();
        y= point.getY();
    }
    public Vec(Point point){
        x= point.getX();
        y= point.getY();
    }    
    public Point2D toPoint(){return new Point2D.Double(x,y);}
    public Point Int(){return new Point((int)Math.round(x),(int)Math.round(y));}
    
    @Override
    public String toString(){return "("+x+","+y+")";}

}
