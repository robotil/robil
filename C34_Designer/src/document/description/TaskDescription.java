package document.description;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

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

public class TaskDescription {
	static public class TaskInfo {

		public String conditions;
		public String constraints;
		public String effects;
		public String algorithm;
		public String name;

		public String body(Node root) {
			if (root.getNodeType() == Element.TEXT_NODE)
				return ((Text) root).getTextContent();
			NodeList nl = root.getChildNodes();
			for (int i = 0; i < nl.getLength(); i++) {
				Node n = nl.item(i);
				String txt = body(n);
				if (txt != null)
					return txt;
			}
			return null;
		}

		public Element createXmlNode(Document doc) {
			Element task = doc.createElement(STR_TASK);
			task.setAttribute(STR_NAME, this.name);
			Element cond = doc.createElement(STR_COND);
			cond.setTextContent(this.conditions);
			Element cons = doc.createElement(STR_CONS);
			cons.setTextContent(this.constraints);
			Element eff = doc.createElement(STR_EFF);
			eff.setTextContent(this.effects);
			Element alg = doc.createElement(STR_ALG);
			alg.setTextContent(this.algorithm);
			task.appendChild(cond);
			task.appendChild(cons);
			task.appendChild(eff);
			task.appendChild(alg);
			return task;
		}

		public boolean parse(Node el) {
			Element task = search(STR_TASK, el);
			if (task == null)
				return false;
			this.name = task.getAttribute(STR_NAME);

			Element c1 = search(STR_COND, task);
			if (c1 == null)
				this.conditions = "";
			else
				this.conditions = body(c1);

			Element c2 = search(STR_CONS, task);
			if (c2 == null)
				this.constraints = "";
			else
				this.constraints = body(c2);

			Element e = search(STR_EFF, task);
			if (e == null)
				this.effects = "";
			else
				this.effects = body(e);

			Element a = search(STR_ALG, task);
			if (a == null)
				this.algorithm = "";
			else
				this.algorithm = body(a);

			return true;
		}

		public Element search(String tagname, Node root) {
			if (root.getNodeName().equals(tagname))
				return (Element) root;
			NodeList nl = root.getChildNodes();
			for (int i = 0; i < nl.getLength(); i++) {
				Node n = nl.item(i);
				Element el = search(tagname, n);
				if (el != null)
					return el;
			}
			return null;
		}

		private String tabed(String tab, String t) {
			String[] tt = t.split("\n");
			String res = "";
			for (String _t : tt) {
				if (_t.trim().length() < 1)
					continue;
				res += tab + _t.trim() + '\n';
			}
			return res;
		}

		@Override
		public String toString() {
			return toXmlAttr("", "   ");
		}

		public String toXmlAttr(String tab, String ttab) {
			return tab + "<" + STR_TASK + " " + STR_NAME + "=\"" + this.name
					+ "\">\n" + tab + ttab + "<" + STR_COND + ">\n"
					+ tabed(tab + ttab + ttab, this.conditions) + tab + ttab
					+ "</" + STR_COND + ">\n" + tab + ttab + "<" + STR_CONS
					+ ">\n" + tabed(tab + ttab + ttab, this.constraints) + tab
					+ ttab + "</" + STR_CONS + ">\n" + tab + ttab + "<"
					+ STR_EFF + ">\n" + tabed(tab + ttab + ttab, this.effects)
					+ tab + ttab + "</" + STR_EFF + ">\n" + tab + ttab + "<"
					+ STR_ALG + ">\n"
					+ tabed(tab + ttab + ttab, this.algorithm) + tab + ttab
					+ "</" + STR_ALG + ">\n" + tab + "</" + STR_TASK + ">";
		}

	}

	static final String STR_TASK_DESCRIPTON = "task_description";
	static final String STR_TASK = "task";
	static final String STR_COND = "conditions";
	static final String STR_CONS = "constraints";
	static final String STR_EFF = "effects";
	static final String STR_ALG = "algorithm";

	static final String STR_NAME = "name";

	public static ArrayList<TaskInfo> parseDocument(Document doc) {
		ArrayList<TaskInfo> list = new ArrayList<TaskInfo>();
		final Element root = doc.getDocumentElement();
		NodeList nl = null;
		if (root.getNodeName().equals(STR_TASK_DESCRIPTON) == false)
			nl = root.getElementsByTagName(STR_TASK_DESCRIPTON);
		else
			nl = new NodeList() {
				@Override
				public int getLength() {
					return 1;
				}

				@Override
				public Node item(int index) {
					return root;
				}
			};
		for (int i = 0; i < nl.getLength(); i++) {
			if (nl.item(i).getNodeType() != Element.ELEMENT_NODE)
				continue;
			NodeList nnl = ((Element) nl.item(i))
					.getElementsByTagName(STR_TASK);

			for (int ii = 0; ii < nnl.getLength(); ii++) {
				TaskInfo def = new TaskInfo();
				if (def.parse(nnl.item(ii))) {
					list.add(def);
				}
			}
		}
		return list;
	}

	private String _fileName = "";

	Map<String, TaskInfo> map = new HashMap<String, TaskDescription.TaskInfo>();

	public TaskDescription() {
	}

	public TaskDescription(String fname) throws ParserConfigurationException,
			SAXException, IOException {
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		Document doc = dBuilder.parse(new File(fname));
		doc.getDocumentElement().normalize();

		ArrayList<TaskInfo> tasks = parseDocument(doc);
		for (TaskInfo task : tasks) {
			this.map.put(task.name, task);
		}

		this._fileName = new File(fname).getCanonicalFile().getAbsolutePath();
	}

	public void addTaskDescription(String taskName, String taskDescription) {
		TaskInfo task = new TaskInfo();
		task.algorithm = taskDescription;
		task.name = taskName;
		this.map.put(taskName, task);
	}

	public TaskInfo get(String taskname) {
		if (this.map.containsKey(taskname))
			return this.map.get(taskname);
		return null;
	}

	public String getFilename() {
		return this._fileName;
	}

	public List<String> getNames() {
		List<String> names = new ArrayList<String>();

		names.add("");
		names.add(" ");

		for (TaskInfo name : this.map.values())
			names.add(name.name);

		return names;
	}

	public Vector<String> getNamesVector() {
		Vector<String> names = new Vector<String>();

		for (TaskInfo name : this.map.values())
			names.add(name.name);

		return names;
	}

	public void load(String fname) throws ParserConfigurationException,
			SAXException, IOException {
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		Document doc = dBuilder.parse(new File(fname));
		doc.getDocumentElement().normalize();

		ArrayList<TaskInfo> tasks = parseDocument(doc);
		for (TaskInfo task : tasks) {
			this.map.put(task.name, task);
		}

		this._fileName = new File(fname).getAbsolutePath();
	}

	public void put(String taskname, TaskInfo task) {
		task.name = taskname;
		this.map.put(taskname, task);
	}

	public void remove(String taskName) {
		if (!this.map.containsKey(taskName))
			return;

		this.map.remove(taskName);
	}

	public void save(String fname) throws ParserConfigurationException {
		Document doc = DocumentBuilderFactory.newInstance()
				.newDocumentBuilder().newDocument();
		Element root = doc.createElement(STR_TASK_DESCRIPTON);
		for (TaskInfo task : this.map.values()) {
			root.appendChild(task.createXmlNode(doc));
		}
		doc.appendChild(root);
		TransformerFactory tFactory = TransformerFactory.newInstance();
		Transformer transformer;
		try {
			tFactory.setAttribute("indent-number", new Integer(4));
			transformer = tFactory.newTransformer();
			transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
			transformer.setOutputProperty(OutputKeys.INDENT, "yes");

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

	@Override
	public String toString() {
		String res = "Tasks Description:";
		for (TaskInfo task : this.map.values()) {
			res += "\n" + task.toString();
		}
		return res;
	}

	public void update(String taskName, String taskDescription) {
		if (!this.map.containsKey(taskName))
			return;

		this.map.get(taskName).algorithm = taskDescription;
	}

	// public static void main(String[] args) throws
	// ParserConfigurationException, SAXException, IOException {
	//
	// String fname = "/tmp/task_desc";
	// TaskDescription td = new TaskDescription(fname);
	// td.save("/tmp/res.xml");
	// }

}
