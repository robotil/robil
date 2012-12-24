package document;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.geom.GeneralPath;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Properties;
import java.util.UUID;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.stream.events.StartDocument;

import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import document.description.TaskDescription;

import elements.Arrow;
import elements.Joint;
import elements.Decorator;
import elements.GElement;
import elements.GProperty;
import elements.Modifier;
import elements.Task;
import elements.Vec;
import elements.View;
import elements.Arrow.Creator;

@SuppressWarnings("serial")
public class Document extends JPanel {
	
	public ArrayList<GElement> arrays = new ArrayList<GElement>();
	public ArrayList<GElement> elements = new ArrayList<GElement>();
	public View view = new View();
	public TaskDescription task_desc = null;
	
	public BTDesigner mainWindow = null;

	public void add(GElement el){
		el.setView(view);
		if(el instanceof Arrow)
			arrays.add(el);
		else
			elements.add(el);
	}
	
	@SuppressWarnings("unchecked")
	public void remove(GElement el){
		if(el instanceof Task){
			for(GElement e : (ArrayList<GElement>)arrays.clone()){
				Arrow a = (Arrow)e;
				if(a.getTarget()==el || a.getSource()==el){
					remove(e);
				}
			}
			elements.remove(el);
		}
		if(el instanceof Arrow){
			Arrow a = (Arrow)el;
			ArrayList<GElement> forrem = new ArrayList<GElement>();
			for(int i=0;i<a.targets.size()-1;i++){
				forrem.add(a.targets.get(i));
			}
			for(GElement e: forrem) remove(e);
			arrays.remove(el);
		}
		if(el instanceof Arrow.ArrayElement){
			elements.remove(el);
			Arrow.ArrayElement a = (Arrow.ArrayElement)el;
			a.getArray().targets.remove(el);
		}
	}
	
	public Document(BTDesigner mw){
		this.mainWindow = mw;
		view.loc = new Vec(0,0);
		view.zoom = 1;
		
		MouseHandler mh = new MouseHandler();
		addMouseListener(mh);
		addMouseMotionListener(mh);
		addMouseWheelListener(mh);
	
		try {
			task_desc = new TaskDescription(Parameters.path_to_description);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	public void renumberElements(ArrayList<GElement> elements){
		for(GElement el: elements){ if(el instanceof Task){ ((Task) el).seqNumber = 0; } }
		for(GElement el: elements){
			if(el instanceof Task){
				Task t = ((Task) el);
				if(t.type == Task.TYPE_sequenser || t.type == Task.TYPE_selector){
					int i=1;
					for(GElement e: getSubElements(t)){
						if(e instanceof Task){
							((Task) e).seqNumber = i++;
						}
					}
				}
			}
		}
	}
	public void paintElement(Graphics2D g, ArrayList<GElement> elements){
		for(GElement el: elements){	
			el.paintElement(g);			
		}
	}
	public void paint(Graphics g){
		g.setColor( Color.lightGray );
		g.fillRect(0, 0, getWidth(), getHeight());
		Graphics2D g2d = (Graphics2D)g;
		g2d.setRenderingHint(RenderingHints .KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		if(view.graphics==null){
			view.graphics = g2d;
			riceOnViewChange();
		}
		renumberElements(elements);
		hideCollapsed();
		paintElement(g2d, arrays);
		paintElement(g2d, elements);
		if(selectedElement!=null) selectedElement.paint(g2d);
		
	}
	
	private void hideCollapsed(){
		for(GElement e: elements) e.isVisiable = true;
		for(GElement e: arrays) e.isVisiable = true;
		for(GElement e: elements){
			if(e.getProperty().collapsed){
				for(GElement c : searchAllSubelements(e)) c.isVisiable=false;
			}
		}
	}
	
	private void riceOnViewChange(){
		for(GElement e: elements) if(e instanceof View.ChangesListener) ((View.ChangesListener)e).onViewChange();
		for(GElement e: arrays) if(e instanceof View.ChangesListener) ((View.ChangesListener)e).onViewChange();
	}
	
	public void loadPlan(String fname){
		setCurrentWorkingFile(fname);
		File file = new File(fname);
		org.w3c.dom.Document doc = null;
		try{
			doc = 
				DocumentBuilderFactory
					.newInstance()
						.newDocumentBuilder()
							.parse(file);
		}catch(javax.xml.parsers.ParserConfigurationException ex){
			ex.printStackTrace();
		} catch (SAXException ex) {
			ex.printStackTrace();
		} catch (IOException ex) {
			ex.printStackTrace();
		}
		toolSelectionClean();
		if(doc==null){
			tip.setText("ERROR: Can't load plan file : "+file.getAbsolutePath());
			return;
		}
		elements.clear(); arrays.clear();
		if(doc.getDocumentElement().getNodeName()!="plan"){
			tip.setText("ERROR: Xml file of plan does not contain plan tag");
			return;			
		}
		view.loc = new Vec(0,0);
		view.zoom = 1;
		lastX=0;lastY=0;
		loadPlan( doc.getDocumentElement() , null );
		repaint();
	}
	
	private double lastX=0,lastY=0;
	private void loadPlan(Element el, GElement rge){
		lastY+=40;
		NodeList nl = el.getChildNodes();
		if(nl!=null && nl.getLength()>0){
			for(int i=0;i<nl.getLength();i++){
				GElement ge = rge;
				if(nl.item(i).getNodeType() != Node.ELEMENT_NODE) continue;
				Element e = (Element)nl.item(i);
				if(ge instanceof Task){
					Arrow a = new Arrow(ge, null);
					add(a);
					ge = a;
				}
				GElement nge = null;
				String nodeName = e.getNodeName().toLowerCase();
				if(ge == null){
					if(nodeName.equals("seq") || nodeName.equals("sequenser")){
						nge = new Task();
						((Task)nge).type = Task.TYPE_sequenser;
						((Task)nge).text = e.getAttribute("name");
					}else
					if(nodeName.equals("sel") || nodeName.equals("selector")){
						nge = new Task();
						((Task)nge).type = Task.TYPE_selector;
						((Task)nge).text = e.getAttribute("name");
					}else
					if(nodeName.equals("par") || nodeName.equals("parallel")){
						nge = new Task();
						((Task)nge).type = Task.TYPE_parallel;
						((Task)nge).text = e.getAttribute("name");
					}else
					if(nodeName.equals("tsk") || nodeName.equals("task")){
						nge = new Task();
						((Task)nge).type = Task.TYPE_task;
						((Task)nge).text = e.getAttribute("name");
					}
					if(e.hasAttribute("x") && e.hasAttribute("y")){	
						lastX = nge.getProperty().loc.x = Double.parseDouble( e.getAttribute("x") );
						lastY = nge.getProperty().loc.y = Double.parseDouble( e.getAttribute("y") );		
					}else{
						lastX = nge.getProperty().loc.x = lastX+20;
						lastY = nge.getProperty().loc.y = lastY;
					}
					if(e.hasAttribute("collapsed")){	
						nge.getProperty().collapsed = Boolean.parseBoolean( e.getAttribute("collapsed") );	
					}else{
						nge.getProperty().collapsed = false;
					}
					if(e.hasAttribute("test_time")){	
						nge.getProperty().test_time = Integer.parseInt( e.getAttribute("test_time") );
					}
					if(e.hasAttribute("test_result")){	
						nge.getProperty().test_result = Boolean.parseBoolean( e.getAttribute("test_result") );
					}
					if(e.hasAttribute("id")){	
						nge.id = UUID.fromString( e.getAttribute("id") );
					}
					add(nge);
					loadPlan(e,nge);
				}else
				if(ge instanceof Arrow){
					GElement se = null;
					if(nodeName.equals("jnt") || nodeName.equals("joint")){
						Arrow a = (Arrow)ge;
						nge = new Joint();
						a.add(nge);
						loadPlan(e,ge);
					}else
					if(nodeName.equals("dec") || nodeName.equals("decorator")){
						Arrow a = (Arrow)ge;
						nge = new Decorator();
						((Decorator)nge).text = e.getAttribute("name");
						a.add(nge);
						se=ge;
					}else
					if(nodeName.equals("seq") || nodeName.equals("sequenser")){
						Arrow a = (Arrow)ge;
						nge = new Task();
						((Task)nge).type = Task.TYPE_sequenser;
						((Task)nge).text = e.getAttribute("name");
						a.add(nge);
						se=nge;
					}else
					if(nodeName.equals("sel") || nodeName.equals("seletor")){
						Arrow a = (Arrow)ge;
						nge = new Task();
						((Task)nge).type = Task.TYPE_selector;
						((Task)nge).text = e.getAttribute("name");
						a.add(nge);
						se=nge;
					}else
					if(nodeName.equals("par") || nodeName.equals("parallel")){
						Arrow a = (Arrow)ge;
						nge = new Task();
						((Task)nge).type = Task.TYPE_parallel;
						((Task)nge).text = e.getAttribute("name");
						a.add(nge);
						se=nge;
					}else
					if(nodeName.equals("tsk") || nodeName.equals("task")){
						Arrow a = (Arrow)ge;
						nge = new Task();
						((Task)nge).type = Task.TYPE_task;
						((Task)nge).text = e.getAttribute("name");
						a.add(nge);
						se=nge;
					}
					add(nge);
					if(e.hasAttribute("x") && e.hasAttribute("y")){	
						lastX=nge.getProperty().loc.x = Double.parseDouble( e.getAttribute("x") );
						lastY=nge.getProperty().loc.y = Double.parseDouble( e.getAttribute("y") );
					}else{
						lastX = nge.getProperty().loc.x = lastX+20;
						lastY = nge.getProperty().loc.y = lastY;
					}
					if(e.hasAttribute("collapsed")){	
						nge.getProperty().collapsed = Boolean.parseBoolean( e.getAttribute("collapsed") );	
					}else{
						nge.getProperty().collapsed = false;
					}
					if(e.hasAttribute("test_time")){	
						nge.getProperty().test_time = Integer.parseInt( e.getAttribute("test_time") );
					}
					if(e.hasAttribute("test_result")){	
						nge.getProperty().test_result = Boolean.parseBoolean( e.getAttribute("test_result") );
					}
					if(e.hasAttribute("id")){	
						nge.id = UUID.fromString( e.getAttribute("id") );
					}
					loadPlan(e, se);
				}
			}
		}
	}
	
	public ArrayList<GElement> getSubElements(GElement el){
		ArrayList<GElement> subels = new ArrayList<GElement>();
		for(GElement e : arrays){
			Arrow arr = (Arrow)e;
			if(arr.getSource() == el) subels.add(arr.getTarget());
		}
		Collections.sort(subels,  new Comparator<GElement>(){
			public int compare(GElement a, GElement b) {
		        return (int)( a.getProperty().loc.x - b.getProperty().loc.x );
		    }
		});
		return uniq(subels);
	}
	public ArrayList<GElement> getSubElementsOfOneLevel(GElement el){
		ArrayList<GElement> subels = new ArrayList<GElement>();
		for(GElement e : arrays){
			Arrow arr = (Arrow)e;
			if(arr.getSource() == el){
				subels.add(arr);
				subels.addAll(arr.targets);
			}
		}
		return uniq(subels);
	}
	public ArrayList<GElement> searchAllSubelements(GElement el){
		ArrayList<GElement> ret = uniq(searchAllSubelements(el, new ArrayList<GElement>()));
		ret.remove(el);
		return ret;
	}
	public ArrayList<GElement> searchAllSubelements(GElement el, boolean removeTopElement){
		ArrayList<GElement> ret = uniq(searchAllSubelements(el, new ArrayList<GElement>()));
		if(removeTopElement) ret.remove(el);
		return ret;
	}
	public ArrayList<GElement> searchAllSubelements(GElement el, ArrayList<GElement> subels){
		ArrayList<GElement> subels_onelevel = getSubElementsOfOneLevel(el);
		for(GElement e : subels_onelevel)if(subels.contains(e)==false){
			subels.add(e);
			searchAllSubelements(e, subels);
		}
		return subels;
	}
	public ArrayList<GElement> getSuperElements(GElement el){
		ArrayList<GElement> subels = new ArrayList<GElement>();
		for(GElement e : arrays){
			Arrow arr = (Arrow)e;
			if(arr.getTarget() == el) subels.add(arr.getSource());
		}
		Collections.sort(uniq(subels),  new Comparator<GElement>(){
			public int compare(GElement a, GElement b) {
		        return (int)( a.getProperty().loc.x - b.getProperty().loc.x );
		    }
		});
		return subels;
	}
	public ArrayList<GElement> getDecorators(GElement el){
		ArrayList<GElement> dec = new ArrayList<GElement>();
		for(GElement e : arrays){
			Arrow arr = (Arrow)e;
			for(GElement ae : arr.targets){
				if(ae instanceof Decorator){
					dec.add(ae);
				}
			}
		}
		return uniq(dec);
	}
	public ArrayList<GElement> getDecorators(Arrow arr){
		ArrayList<GElement> dec = new ArrayList<GElement>();
		for(GElement ae : arr.targets){
			if(ae instanceof Decorator){
				dec.add(ae);
			}
		}
		return uniq(dec);
	}
	public ArrayList<GElement> getArrow(GElement es, GElement et){
		ArrayList<GElement> subels = new ArrayList<GElement>();
		for(GElement e : arrays){
			Arrow arr = (Arrow)e;
			if((arr.getSource() == es || es==null) && (arr.getTarget() == et || et==null)) subels.add(arr);
		}
		return uniq(subels);
	}
	
	public ArrayList<GElement> uniq(ArrayList<GElement> arr){
		for(int s=0;s<arr.size();s++){
			for(int p=s+1;p<arr.size();p++){
				if(arr.get(s)==arr.get(p)){
					arr.remove(p); p--;
				}
			}
		}
		return arr;
	}
	
	public ArrayList<GElement> getRoot(){
		ArrayList<GElement> root = new ArrayList<GElement>();
		for(GElement e: elements)if(e instanceof Task){
			if( getArrow(null, e).size()==0 ) root.add(e);
		}
		//for(GElement r: root) System.out.print(""+r.toString()+" ");
		System.out.println();
		return root;
	}
	
	GElement selectedElement = null;
	Point mousePressed = null;
	
	public GElement.Creator creator = null;
	public boolean removeElement = false;
	public Modifier modifier = null;
	public JLabel tip = null;
	public final boolean cleanToolSelectionAfterUse = false;
	private String absoluteFilePath = "plan.xml";
	
	public String getAbsoluteFilePath() {
		if (absoluteFilePath == null) {
			return null;
		}
		return new String(absoluteFilePath);
	}

	public String getShortFilePath() {
		if (absoluteFilePath == null) {
			return null;
		}
		
		String[] splitted = absoluteFilePath.split("/");
		if (splitted.length == 0) {
			return null;
		}
		
		return new String(splitted[splitted.length - 1]);
	}
	
	public void toolSelectionClean(){
		creator = null;
		removeElement = false;
		modifier = null;
		if(tip!=null) tip.setText(Toolbar.TIP_move);
	}
	
	public class MouseHandler extends MouseAdapter{

		@Override
		public void mouseWheelMoved(MouseWheelEvent e) {
			int notches = e.getWheelRotation();
			if(notches<0 && view.zoom<0.3) return;
			if(notches>0 && view.zoom>5) return;
			double old_zoom = view.zoom;
			double new_zoom = view.zoom = view.zoom + (notches*0.1);
			Vec m = new Vec(e.getPoint());
			Vec d = view.loc.sub(m).scale(1.0/old_zoom).scale(new_zoom);
			view.loc = d.add(m);			
			repaint();
			super.mouseWheelMoved(e);
		}
		
		@Override
		public void mouseDragged(MouseEvent e) {
			if(mousePressed==null) return;
			if(selectedElement==null){
				Vec d = new Vec(e.getPoint()).sub(new Vec(mousePressed)).scale(1/view.zoom);
				view.loc.setOffset(new Vec(e.getPoint()).sub(new Vec(mousePressed)));
				mousePressed = e.getPoint();
			}else{
				Vec d = new Vec(e.getPoint()).sub(new Vec(mousePressed)).scale(1/view.zoom);
				selectedElement.getProperty().loc.setOffset(d);
				for(GElement el: searchAllSubelements(selectedElement)){
					el.getProperty().loc.setOffset(d);
				}
				mousePressed = e.getPoint();
			}
			repaint();
			super.mouseDragged(e);
		}

		@Override
		public void mousePressed(MouseEvent ev) {
			mousePressed = ev.getPoint();
			for(GElement el: elements){
				GElement e = el.underMouse(ev.getPoint());
				if(e!=null){
					selectedElement = e;
					selectedElement.getProperty().selected = true;
					repaint();
					break;
				}
			}
			if(selectedElement==null)
			for(GElement el: arrays){
				GElement e = el.underMouse(ev.getPoint());
				if(e!=null){
					selectedElement = e;
					selectedElement.getProperty().selected = true;
					repaint();
					break;
				}
			}

			super.mousePressed(ev);
		}

		@Override
		public void mouseReleased(MouseEvent e) {
			mousePressed = null;
			if(creator!=null){
				if(selectedElement!=null)
					creator.add(selectedElement);
				boolean checkCreator = creator!=null && creator.ready() && (
						(selectedElement==null && creator.createOnEmptyPlace()) ||
						(selectedElement!=null && !creator.createOnEmptyPlace())
						);
				if(checkCreator){
					Vec p = new Vec(e.getPoint()).sub(view.loc).scale(1/view.zoom);
					GElement el = creator.newInstance();
					if(el instanceof Arrow){
						Arrow a = (Arrow) el;
						if(
							getArrow(a.getSource(), a.getTarget()).size()>0 ||
							getArrow(a.getTarget(), a.getSource()).size()>0
						)
						el = null;
					}
					if(el!=null){
						el.setView(view);
						if(el instanceof View.ChangesListener) ((View.ChangesListener)el).onViewChange();
						el.getProperty().setCenter( p );
						add(el);
						el.modify();
						repaint();
					}
					if(cleanToolSelectionAfterUse) toolSelectionClean();
				}
			}
			if(removeElement && selectedElement!=null){
				remove(selectedElement);
				if(cleanToolSelectionAfterUse) toolSelectionClean();
			}
			if(modifier!=null && selectedElement!=null){
				modifier.set(selectedElement);
				if(cleanToolSelectionAfterUse) toolSelectionClean();
			}
			if(selectedElement==null) return;
//			/*DEBUG*/{
//				ArrayList<GElement> s = getSubElements(selectedElement);
//				System.out.print("Sub elements: ");
//				for(GElement ee: s){
//					if(ee instanceof Task) System.out.print( ((Task)ee).text +" ");
//				}
//				System.out.println();
//			}
			selectedElement.getProperty().selected=false;
			selectedElement = null;
			repaint();
			super.mouseReleased(e);
		}
		
	}
	
	public String xmlNameOfTask(String tname){
		if(tname==Task.TYPE_selector) return "sel";
		if(tname==Task.TYPE_sequenser) return "seq";
		if(tname==Task.TYPE_task) return "tsk";
		if(tname==Task.TYPE_parallel) return "par";
		return "";
	}
	public static final String tabulation = "   ";
	public String strTaskProperties(Task root){
		if(root.type.equals(Task.TYPE_task))
			return root.getProperty().toStringForTask(view)+" id=\""+root.id.toString()+"\"";
		return root.getProperty().toStringForSTask(view)+" id=\""+root.id.toString()+"\"";
	}
	public String strDecProperties(Decorator d){
		return d.getProperty().toString(view);
	}
	public String strJointProperties(Joint d){
		return "";
	}
	public String createXml(Task root, String tab){return createXml(root, tab, false);}
	public String createXml(Task root, String tab, boolean justNames){
		if(root.type == Task.TYPE_task) 
			return tab+"<"+xmlNameOfTask(root.type)+" name=\""+root.text+"\""+(justNames?"":" "+strTaskProperties(root))+" />";

		String xml = tab+"<"+xmlNameOfTask(root.type)+" name=\""+root.text+"\""+(justNames?"":" "+strTaskProperties(root))+"> ";
		ArrayList<GElement> subel = getSubElements(root);
		String ntab = tab+tabulation;
		for(GElement el: subel){
			xml+="\n";
			Arrow arr = (Arrow) getArrow(root, el).get(0);
			ArrayList<GElement> decor = arr.targets;//getDecorators(arr);
			String d_xml_p = "", d_xml_s = "", nntab = ntab;
			for(GElement ed: decor){ 
				if(ed instanceof Decorator){
					Decorator d = (Decorator)ed;
					d_xml_p = d_xml_p + nntab + "<dec name=\""+d.text+"\""+(justNames?"":" "+strDecProperties(d))+"> \n";
					d_xml_s = "\n"+nntab+"</dec>" + d_xml_s;
				}else
				if(ed instanceof Joint){
					Joint d = (Joint)ed;
					d_xml_p = d_xml_p + nntab + "<jnt "+d.getProperty().toString(view)+(justNames?"":" "+strJointProperties(d))+"> \n";
					d_xml_s = "\n"+nntab+"</jnt>" + d_xml_s;
				}
				nntab += tabulation;
			}
			String t_xml = createXml((Task)el, nntab, justNames);
			xml+=d_xml_p+t_xml+d_xml_s;
		}
		xml+="\n"+tab+"</"+xmlNameOfTask(root.type)+">";
		return xml;
	}

	public void compile(String destination) {
		setCurrentWorkingFile(destination);
		
		toolSelectionClean();
		ArrayList<GElement> root = getRoot();
		if(root.size()!=1){
			tip.setText("ERROR: Behavior TREE has to have ONE and only ONE root node");
			return;
		}
		for(GElement ea: arrays){
			if( ((Task)((Arrow)ea).getSource()).type == Task.TYPE_task ){
				tip.setText("ERROR: Task in BT is a TERMINAL node");
				return;				
			}
		}
		for(GElement ea: elements)if(ea instanceof Task){
			if( ((Task)ea).type!=Task.TYPE_task && getArrow(ea, null).size()==0 ){
				tip.setText("ERROR: Sequenser, Parallel or Selector in BT are a NON TERMINAL nodes");
				return;				
			}
		}
		Task rootTask = (Task) root.get(0);
		
		String xml = null;
		try{
			
			xml = createXml(rootTask, tabulation);
			System.out.println("XML : \n"+"<plan>\n"+xml+"\n</plan>");
			tip.setText("Compilation is OK");
			
		}catch(StackOverflowError e){
			tip.setText("ERROR: Behavior TREE has not to have CYCLES");
			return;
		}
		
		boolean saved = saveXmlToFile(xml, getCurrentWorkingFile(), true);
		if(saved) saveXmlToFile(createXml(rootTask, tabulation, true), getCurrentWorkingFileForXmlWithJustNames(), false);
	}
	
	public void compile() {
		compile(getCurrentWorkingFile());
	}
	
	private boolean saveXmlToFile(String xml, String filen, boolean withNotifications){
		xml = "<plan>\n"+xml+"\n</plan>";
				
		FileWriter w = null;File file=null;
		try{
			file =  new File(filen);
			w = new FileWriter(file);
			w.write(xml);
			w.close();
			if(withNotifications)tip.setText(tip.getText()+"; file created: "+file.getAbsolutePath());
		}catch(IOException e){
			if(withNotifications)tip.setText(tip.getText()+"; ERROR: writing to file is fault. file: "+file.getAbsolutePath()+", "+e.toString());
			return false;			
		}	
		return true;
	}

	public void setCurrentWorkingFile(String absoluteFilePath) {
		this.absoluteFilePath = absoluteFilePath;
	}
	
	private String getCurrentWorkingFile() {
		return this.absoluteFilePath;
	}
	private String addTextToFileName(String file, String text){
		ArrayList<String> s = new ArrayList<String>(); 
		for(String f: file.split("\\.")) s.add(f);
		s.add(s.size()-1, text); String r="";
		for(int i=0;i<s.size()-1;i++) r = r.concat(s.get(i))+"."; r=r.concat(s.get(s.size()-1));
		return r;
	}
	private String getCurrentWorkingFileForXmlWithJustNames() {
		return addTextToFileName(this.absoluteFilePath, "(just_names)");
	}

	public void cleanRunning(){
		for(GElement e: elements){
			e.getProperty().running = false;
		}
		repaint();
	}
	
	public void setRunning(ArrayList<String> ids){
		for(GElement e: elements) if( ids.contains(e.id.toString()) ){
			//System.out.println("id = "+e.id+" found.");
			e.getProperty().running = true;
		}
		repaint();
	}
	
	public ArrayList<String> extractIds(String text){
		ArrayList<String> ids = new ArrayList<String>();
		String[] lines = text.split("\\[id=");
		int l=1;
		for(String line: lines){
			if(l--<1){
				String linesp = line.split("\\]")[0];
				if(linesp.contains(","))
					for(String line1: linesp.split(","))
						ids.add(line1);
				else
					ids.add(linesp);
			}
		}
		return ids;
	}

	Process BTExecuter=null;
	public void test() {
		if(BTExecuter!=null){
			BTExecuter.destroy();
			return;
		}
		compile();
		if(tip.getText().startsWith("ERROR")) return;
		(new Thread(){public void run(){
			ArrayList<String> cmd = new ArrayList<String>();
			if(OSValidator.isUnix()){
				String[] f1 = new String[]{
						"./BTExecuter-lin.exe",
						"./BTExecuter-lin.bin",
						"./BTExecuter-lin.a",
						"./BTExecuter-lin.run",
						"./BTExecuter"
				};
				File f = null;
				for(String ff: f1){ f = new File(ff); if(f.exists()) break; }
				cmd.add(f.getAbsolutePath());
			}
			else if(OSValidator.isWindows())
				cmd.add("./BTExecuter-win.exe");
			//cmd.add("--test");
			cmd.add("-bt"); cmd.add(absoluteFilePath);
			cmd.add("-lu"); cmd.add((new File("lookup.xml")).getAbsolutePath());
			ProcessBuilder pb = new ProcessBuilder(cmd);
			try {
				Process p = pb.start();
				BTExecuter = p;
				BufferedReader stdout = new BufferedReader(new InputStreamReader( p .getInputStream() ));
				BufferedReader stderr = new BufferedReader(new InputStreamReader( p .getErrorStream() ));
				String line;
				String plan=""; int c=0;
				tip.setText("Execution running... (press \"Run\" again for stop running)");
				while( (line=stdout.readLine())!=null){
					System.out.println("BTExecuter STDOUT: "+line);
					if(line.startsWith("plan{")){ plan+=line; c++; }
					else if(plan.length()>0){
						plan+=line;
						if(line.contains("{")) c++;
						if(line.contains("}")) c--;
						if(c==0){
							//System.out.println(": "+plan);
							cleanRunning();
							setRunning(extractIds(plan));
							plan="";
						}
					}
				}
				while( (line=stderr.readLine())!=null){
					System.out.println("BTExecuter STDERR: "+line);
				}
			} catch (IOException e) {
				e.printStackTrace();
				tip.setText("ERROR: Can not run BTExecuter");
			}
			BTExecuter = null;
			cleanRunning();
			System.out.println("BTExecuter DONE");
			tip.setText("Execution done.");
		}}).start();
	}

}
