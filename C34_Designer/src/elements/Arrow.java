package elements;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Map;

import javax.swing.Icon;

public class Arrow extends GElement {
	
	public interface ArrayElement{
		public void add(Arrow array);
		public Arrow getArray();
	}
	
	public GElement source = null;
	public ArrayList<GElement> targets = new ArrayList<GElement>();
	public int lastSelectedSegmentId = -1;
	
	private Arrow(){
		source = null;
	}
	
	public Arrow(GElement str, GElement end){
		source = str;
		if(end!=null) targets.add( end );
	}
	
	public GElement getSource(){
		return source;
	}
	public GElement getTarget(){
		if(targets.size()==0) return null;
		return targets.get(targets.size()-1);
	}

	static public class Creator extends GElement.Creator{
		ArrayList<GElement> els = new ArrayList<GElement>();
		public GElement newInstance(){
			if(ready()==false) return null;
			Arrow a = new Arrow(els.get(0), els.get(1));
			els.clear();
			return a;
		}
		public Icon getIcon(){
			return null;
		}
		public boolean ready(){
			return els.size()==2;
		}
		public boolean createOnEmptyPlace(){
			return false;
		}
		public void add(GElement selectedElement) {
			if(selectedElement instanceof Task == false) return;
			if(els.size()==1 && els.get(0)==selectedElement) return;
			if(els.size()<2)
				els.add(selectedElement);
		}
		public String getToolbarName() {
			return "Arrow";
		}
		@Override
		public String toolTip() {
			return "Create an Arrow between to tasks. Fist, select source task and then select distination task.";
		}
	}
	static public class Reconector extends GElement.Creator{
		ArrayList<GElement> els = new ArrayList<GElement>();
		Arrow inst = null;
		public Reconector(Arrow inst){
			this.inst = inst;
		}
		public GElement getInstance(){ return inst; }
		public GElement newInstance(){
			if(ready()==false) return null;
			//Arrow a = new Arrow(els.get(0), els.get(1));
			inst.source = els.get(0);
			inst.targets.remove(inst.getTarget());
			inst.add(els.get(1));
			els.clear();
			return inst;
		}
		public Icon getIcon(){
			return null;
		}
		public boolean ready(){
			return els.size()==2;
		}
		public boolean createOnEmptyPlace(){
			return false;
		}
		public void add(GElement selectedElement) {
			if(selectedElement instanceof Task == false) return;
			if(els.size()==1 && els.get(0)==selectedElement) return;
			if(els.size()<2)
				els.add(selectedElement);
		}
		public String getToolbarName() {
			return "Arrow Reconector";
		}
		@Override
		public String toolTip() {
			return "Reconect selected arrow. Fist, select source task and then select distination task.";
		}
	}
	
	static double line_y(Vec v1, Vec v2, double x){
		return (v2.y-v1.y)/(v2.x-v1.x)*(x - v1.x) + v1.y;
	}
	static double line_x(Vec v1, Vec v2, double y){
		return (y - v1.y)*(v2.x-v1.x)/(v2.y-v1.y) + v1.x;
	}
	
	public boolean segmentUnderMouse(GElement s, GElement e, Vec m){
		Vec c1 = s.getCenter();
		Vec c2 = e.getCenter();
		double mix=Math.min(c1.x, c2.x), miy=Math.min(c1.y, c2.y), max=Math.max(c1.x, c2.x), may=Math.max(c1.y,c2.y);
		if( !(mix<=m.x && m.x<=max /*&& miy<=m.y && m.y<=may*/) ) return false;
		return Vec.distance(c1, c2, m)<4;
	}
	public GElement underMouse(Point p){
		Vec v = new Vec(p);
		GElement s = source;
		for(int i=0;i<targets.size();i++){
			if(segmentUnderMouse(s, targets.get(i), v)){
				lastSelectedSegmentId = i;
				return this;
			}
			s = targets.get(i);
		}
		
		return null;
	}
	
	public void paintLastSegment(Graphics2D g, GElement source, GElement target){
		GraphProp gp = new GraphProp(g);
		
		g.setPaint(Color.black);
		
		Vec vloc = source.getCenter();
		Vec vctar = target.getCenter();
		//Vec border = target.getSize().scale(0.5);
		
		double ang = vctar.sub(vloc).ang();
		double dx = Math.abs(target.getSize().scale(0.5).x/Math.cos(ang));
		double dy = Math.abs(target.getSize().scale(0.5).y/Math.sin(ang));
		double d = dx <= target.getSize().scale(0.5).len() ? dx : dy;
		Vec border = new Vec(0, d);
		
		Vec vtar = vctar.sub(vloc).offsetLen(-border.len()).add(vloc);
		Vec vtar1 = vctar.sub(vloc).offsetLen(-border.len()-5*view.zoom).add(vloc);
		
		Point loc = vloc.getPoint();
		Point tar = vtar.getPoint();
		Point tar1 = vtar1.getPoint();
		g.drawLine(loc.x, loc.y, tar1.x, tar1.y);
		
		Vec v = vloc.sub(vtar).changeLen(10*view.zoom);
		Point s1 = v.rotate(Vec.d2r(+20)).add(vtar).getPoint();
		Point s2 = v.rotate(Vec.d2r(-20)).add(vtar).getPoint();
		g.fillPolygon(
				new int[]{tar.x, s1.x, s2.x }, 
				new int[]{tar.y, s1.y, s2.y }, 
				3);
		gp.restore();
	}
	public void paintSegment(Graphics2D g, GElement source, GElement target){
		GraphProp gp = new GraphProp(g);
		
		g.setPaint(Color.black);
		
		Vec vloc = source.getCenter();
		Vec vctar = target.getCenter();
		
		Point loc = vloc.getPoint();
		Point tar = vctar.getPoint();
		g.drawLine(loc.x, loc.y, tar.x, tar.y);
		
		gp.restore();
	}
	public void paint(Graphics2D g){
		if(source==null || targets.size()<1) return;
		GraphProp gp = new GraphProp(g);
		
		if(property.selected)
		{
			g.setStroke(new BasicStroke(3));
		}
		
		GElement s = source;
		for(int i=0;i<targets.size()-1;i++){
			paintSegment(g, s, targets.get(i));
			s = targets.get(i);
		}
		paintLastSegment(g, s, targets.get(targets.size()-1));
		gp.restore();
	}
	
	Vec getLocation(){
		return source.getCenter();
	}
	Vec getSize(){
		if(targets.size()<1) return new Vec(0,0);
		return targets.get(targets.size()-1).getCenter().sub(source.getCenter());
	}

	public void add(GElement e){
		targets.add(e);
		if(e instanceof ArrayElement){
			((ArrayElement)e).add(this);
		}
	}
	public void add(int i, GElement e){
		targets.add(i, e);
		if(e instanceof ArrayElement){
			((ArrayElement)e).add(this);
		}
	}
	
	@Override
	public GElement clone() {
		Arrow n = new Arrow();
		cloneInit(n);
		n.source = source;
		n.targets.addAll(targets);
		n.lastSelectedSegmentId = lastSelectedSegmentId;
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {
		source = link.get(source);
		ArrayList<GElement> old = targets;
		targets = new ArrayList<GElement>();
		for(GElement e: old) targets.add(link.get(e));
	}

	@Override
	public void modify() {
		// TODO Auto-generated method stub
		
	}
}
