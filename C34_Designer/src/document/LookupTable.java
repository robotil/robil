package document;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import logger.Log;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class LookupTable {
	HashMap<String, LookupTableRecord> _rowsHashMap = new HashMap<String, LookupTableRecord>();
	ArrayList<LookupTableRecord> _rows = new ArrayList<LookupTableRecord>();
	String _filename;
	
	public LookupTable(String fileName) {
		this._filename = fileName;
		load();
	}
	
	public LookupTableRecord getRow(int index) {
		return _rows.get(index);
	}
	
	public String[] getColumnNames() {
		return new String[] { "Task name", "Type", "Planner", "Filename" };
	}
	
	public int getColumnCount() {
		return getColumnNames().length;
	}
	
	public int getRowCount() {
		return _rows.size();
	}
	
	public String getFilename() {
		return this._filename;
	}
	
	public boolean containsTask(String taskName) {
		return getByTaskName(taskName) != null;
	}
	
	public LookupTableRecord getByTaskName(String taskName) {
		if (!_rowsHashMap.containsKey(taskName))
			return null;
		
		return _rowsHashMap.get(taskName);
	}
	
	public void add(String taskName, String type, String planner, String fileName) {
		add(new LookupTableRecord(taskName, type, planner, fileName));
	}
	
	public void add(LookupTableRecord row) {
		_rows.add(row);
		_rowsHashMap.put(row.getTaskName(), row);
	}

	public void remove(int row) {
		if (row >= 0 && row < getRowCount())
			_rows.remove(row);
	}

	public void load(String filename) {
		this._filename = filename;
		load();
	}

	public void load() {
		_rows.clear();
		_rowsHashMap.clear();
		try {
			DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
			Document document = dBuilder.parse(new File(getFilename()));
			document.getDocumentElement().normalize();
			
			Element root = document.getDocumentElement();
			
			if (root == null) {
				Log.e("Bad lookup table format (root element 'lookup' not found)");
				return;
			}
			
			NodeList tasks = root.getElementsByTagName("task");
			
			if (tasks == null) {
				Log.i("Lookup table is empty");
				return;
			}
			
			for (int i = 0; i < tasks.getLength(); i++) {
				
				if (!(tasks.item(i) instanceof Element))
					continue;
				
				Element taskElement = (Element)tasks.item(i);
				LookupTableRecord newRow = new LookupTableRecord(
						taskElement.getAttribute("name"),
						taskElement.getAttribute("type"),
						taskElement.getAttribute("planner"),
						taskElement.getAttribute("file-name"));
				add(newRow);
			}
		} catch (Exception e) {
			Log.e(e);
		}
	}
	
	public void save() {
		try {
			Document document = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();
			Element root = document.createElement("lookup");
			
			for (LookupTableRecord row : _rows)
				root.appendChild(row.getXmlElement(document));
	
			document.appendChild(root);
			
			TransformerFactory tFactory = TransformerFactory.newInstance();
			Transformer transformer;
		
			tFactory.setAttribute("indent-number", new Integer(4));
			transformer = tFactory.newTransformer();
			transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
			transformer.setOutputProperty(OutputKeys.INDENT, "yes");

			StreamResult result = new StreamResult(new FileWriter(getFilename()));
			transformer.transform(new DOMSource(document), result);

		} catch (Exception e) {
			Log.e(e);
		}
	}
}
