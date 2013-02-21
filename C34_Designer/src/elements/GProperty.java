package elements;

import org.w3c.dom.Element;

import document.Parameters;

public class GProperty {
	
	public Vec loc = new Vec(0,0);
	public double heading = 0;
	public Vec size = new Vec(0,0);
	public boolean selected = false;
	
	public boolean collapsed = false;
	
	public int test_time=Parameters.test_time;
	public boolean test_result=Parameters.test_result;
	
	public boolean running=false;
	
	public GProperty clone(){
		GProperty p = new GProperty();
		p.loc = new Vec(loc);
		p.heading= heading;
		p.size = new Vec(size);
		p.selected = selected;
		p.collapsed = collapsed;
		p.test_result = test_result;
		p.test_time = test_time;
		return p;
	}

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
		return "x=\""+l.x+"\" y=\""+l.y+"\""+" test_time=\""+test_time+"\" test_result=\""+(test_result?"true":"false")+"\"";
	}
	public String toStringForSTask(View v){
		Vec l = loc.add(v.loc.scale(1/v.zoom));
		return "x=\""+l.x+"\" y=\""+l.y+"\""+" collapsed=\""+(collapsed?"true":"false")+"\"";
	}

	public void setCenter(Vec p) {
		loc = p.sub(size.scale(0.5));
	}

}
