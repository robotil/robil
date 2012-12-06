package elements;

import org.w3c.dom.Element;

import document.Parameters;

public class GProperty {
	
	public Vec loc = new Vec(0,0);
	public double heading = 0;
	public Vec size = new Vec(0,0);
	public boolean selected = false;
	
	public int dbg_time=Parameters.dbg_time;
	public boolean dbg_result=Parameters.dbg_result;
	
	public boolean running=false;

	public void setXml(Element xml) {
		
	}
	
	public String toString(){
		
		return "x=\""+loc.x+"\" y=\""+loc.y+"\"";
	}
	public String toString(View v){
		Vec l = loc.add(v.loc.scale(1/v.zoom));
		return "x=\""+l.x+"\" y=\""+l.y+"\"";
	}
	public String toStringForTask(View v){
		Vec l = loc.add(v.loc.scale(1/v.zoom));
		return "x=\""+l.x+"\" y=\""+l.y+"\""+" dbg_time=\""+dbg_time+"\" dbg_result=\""+(dbg_result?"true":"false")+"\"";
	}
	public String toStringForSTask(View v){
		Vec l = loc.add(v.loc.scale(1/v.zoom));
		return "x=\""+l.x+"\" y=\""+l.y+"\"";
	}

	public void setCenter(Vec p) {
		loc = p.sub(size.scale(0.5));
	}

}
