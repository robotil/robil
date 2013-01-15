package document.description;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerConfigurationException;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.w3c.dom.Text;
import org.xml.sax.SAXException;

public class TaskDescription{
	static final String STR_TASK_DESCRIPTON= "task_description";
	static final String STR_TASK = "task";  
	static final String STR_COND = "conditions";  
	static final String STR_CONS = "constraints";  
	static final String STR_EFF = "effects";  
	static final String STR_ALG = "algorithm";  
	static final String STR_NAME = "name";  
	
	
	static public class Task {
	
		public String conditions;
		public String constraints;
		public String effects;		
		public String algorithm;
		public String name;
		
		public String toString(){
			return toXmlAttr("","   ");
		}
		
		private String tabed(String tab, String t ){
			String[] tt = t.split("\n");
			String res = "";
			for(String _t: tt){
				if(_t.trim().length()<1) continue;
				res += tab+_t.trim()+'\n';
			}
			return res;
		}
		public String toXmlAttr(String tab, String ttab){
			return tab+"<"+STR_TASK+" "+STR_NAME+"=\""+name+"\">\n"+
					tab+ttab+"<"+STR_COND+">\n"+
					tabed(tab+ttab+ttab,conditions)+
					tab+ttab+"</"+STR_COND+">\n"+
					tab+ttab+"<"+STR_CONS+">\n"+
					tabed(tab+ttab+ttab,constraints)+
					tab+ttab+"</"+STR_CONS+">\n"+
					tab+ttab+"<"+STR_EFF+">\n"+
					tabed(tab+ttab+ttab,effects)+
					tab+ttab+"</"+STR_EFF+">\n"+
					tab+ttab+"<"+STR_ALG+">\n"+
					tabed(tab+ttab+ttab,algorithm)+
					tab+ttab+"</"+STR_ALG+">\n"+
					tab+"</"+STR_TASK+">";
		}
		public Element createXmlNode(Document doc){
			Element task = doc.createElement(STR_TASK);
			task.setAttribute(STR_NAME, name);
			Element cond = doc.createElement(STR_COND);
			cond.setTextContent(conditions);
			Element cons = doc.createElement(STR_CONS);
			cons.setTextContent(constraints);
			Element eff = doc.createElement(STR_EFF);
			eff.setTextContent(effects);
			Element alg = doc.createElement(STR_ALG);
			alg.setTextContent(algorithm);
			task.appendChild(cond);
			task.appendChild(cons);
			task.appendChild(eff);
			task.appendChild(alg);
			return task;
		}
		public Element search(String tagname, Node root){
			if(root.getNodeName().equals(tagname)) return (Element) root;
			NodeList nl = root.getChildNodes();
			for(int i=0;i<nl.getLength();i++){
				Node n = nl.item(i);
				Element el = search(tagname,n);
				if(el!=null) return el;
			}
			return null;
		}
		public String body(Node root){
			if(root.getNodeType()==Element.TEXT_NODE)
				return ((Text) root).getTextContent();
			NodeList nl = root.getChildNodes();
			for(int i=0;i<nl.getLength();i++){
				Node n = nl.item(i);
				String txt = body(n);
				if(txt!=null) return txt;
			}
			return null;
		}
		public boolean parse(Node el){
			Element task = search(STR_TASK, el);
			if(task==null) return false;
			name = task.getAttribute(STR_NAME);
			
			Element c1 = search(STR_COND, task);
			if(c1==null) conditions = "";
			else conditions = body(c1);
			
			Element c2 = search(STR_CONS, task);
			if(c2==null) constraints = "";
			else constraints = body(c2);
	
			Element e = search(STR_EFF, task);
			if(e==null) effects = "";
			else effects = body(e);
			
			Element a = search(STR_ALG, task);
			if(a==null) algorithm = "";
			else algorithm = body(a);
			
			return true;
		}

	}

	
	public static ArrayList<Task> parseDocument(Document doc){
		ArrayList<Task> list = new ArrayList<Task>();
		final Element root = doc.getDocumentElement();
		NodeList nl = null;
		if(root.getNodeName().equals(STR_TASK_DESCRIPTON)==false) 
			nl = root.getElementsByTagName(STR_TASK_DESCRIPTON);
		else
			nl = new NodeList() {
				public Node item(int index) {
					return root;
				}
				public int getLength() {
					return 1;
				}
			};
		for(int i=0;i<nl.getLength();i++){
			if(nl.item(i).getNodeType() != Element.ELEMENT_NODE) continue;
			NodeList nnl = ((Element) nl.item(i)).getElementsByTagName(STR_TASK);
			//System.out.println("task : "+nnl.getLength());
			for(int ii=0;ii<nnl.getLength();ii++){
				Task def = new Task();
				if( def.parse(nnl.item(ii)) ){
					list.add(def);
				}
			}
		}
		return list;
	}
	
	public TaskDescription(String fname) throws ParserConfigurationException, SAXException, IOException{
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		Document doc = dBuilder.parse(new File(fname));
		doc.getDocumentElement().normalize();

		ArrayList<Task> tasks = parseDocument(doc);
		for(Task task : tasks){
			map.put(task.name, task);
		}
	}
	
	public TaskDescription(){
	}	
	
	Map<String, Task> map = new HashMap<String, TaskDescription.Task>();
	
	public String toString(){
		String res="Tasks Description:";
		for(Task task : map.values()){
			res+="\n"+task.toString();
		}
		return res;
	}
	
	public List<String> getNames() {
		List<String> names = new ArrayList<String>();
		
		names.add("");
		names.add(" ");
		
		for (Task name : map.values()) 
			names.add(name.name);
				
		return names;
	}
	
	public Task get(String taskname){
		if(map.containsKey(taskname)) return map.get(taskname);
		return null;
	}
	
	public void put(String taskname, Task task){
		task.name = taskname;
		map.put(taskname, task);
	}
	
	public void load(String fname)throws ParserConfigurationException, SAXException, IOException{
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		Document doc = dBuilder.parse(new File(fname));
		doc.getDocumentElement().normalize();

		ArrayList<Task> tasks = parseDocument(doc);
		for(Task task : tasks){
			map.put(task.name, task);
		}
	}
	
	public void save(String fname) throws ParserConfigurationException{
		Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();
		Element root = doc.createElement(STR_TASK_DESCRIPTON);
		for(Task task : map.values()){
			root.appendChild(task.createXmlNode(doc));
		}
		doc.appendChild(root);
		TransformerFactory tFactory =
		TransformerFactory.newInstance();
		Transformer transformer;
		try {
			tFactory.setAttribute("indent-number", new Integer(4));
	        transformer = tFactory.newTransformer();
	        transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
	        transformer.setOutputProperty(OutputKeys.INDENT,     "yes");			

	        StreamResult result = new StreamResult(new FileWriter(fname));
	        
			transformer.transform(new DOMSource(doc), result);
		
		} catch (TransformerConfigurationException e) {
			e.printStackTrace();
		} catch (TransformerException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
//	public static void main(String[] args) throws ParserConfigurationException, SAXException, IOException {
//
//		String fname = "/tmp/task_desc";
//		TaskDescription td = new TaskDescription(fname);
//		td.save("/tmp/res.xml");
//		System.out.println("TEST FINISHED.");
//	}

}
