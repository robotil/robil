package document;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import javax.swing.JTable;
import javax.swing.table.TableModel;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.Result;
import javax.xml.transform.Source;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerConfigurationException;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import logger.Log;

import org.w3c.dom.Attr;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class PropertiesXmlHandler {

	final public static String lastPath = "BTDesigner.xml";

	private static Node createNode(Document doc, String name, String value) {
		Element element = doc.createElement(name);

		Attr attribute = doc.createAttribute("value");
		attribute.setValue(value);
		element.setAttributeNode(attribute);

		return element;

	}

	public static Map<String, String> getParameters() {
		Map<String, String> map = new HashMap<String, String>();

		try {
			for (int i = 0; i < Parameters.class.getFields().length; ++i) {
				String name = Parameters.class.getFields()[i].getName();
				String value = Parameters.class.getFields()[i].get(null).toString();
				map.put(name, value);
			}
		} catch (IllegalAccessException ex) {
			Log.e("Error getParameters: " + ex.getMessage());
		}

		return map;
	}

	public static void loadAndSetProperties() throws IOException,
			ParserConfigurationException, SAXException {
		loadAndSetProperties(lastPath);
	}

	public static void loadAndSetProperties(String path) throws IOException,
			ParserConfigurationException, SAXException {
		
		Log.d("Load properties from "+new File(path).getAbsolutePath());
		File xmlFile = new File(path);
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder db = dbFactory.newDocumentBuilder();
		Document doc = db.parse(xmlFile);

		doc.getDocumentElement().normalize();

		NodeList list = doc.getDocumentElement().getChildNodes();
		//Log.d("list size = " + list.getLength());
		Log.d("Properties :");
		for (int i = 0; i < list.getLength(); ++i) {
			Node n = list.item(i);
			//Log.d(n.getNodeName());

			if (n.getNodeName().trim().startsWith("#")) {
				continue;
			}
			String key = n.getNodeName().trim();
			String value = n.getAttributes().getNamedItem("value")
					.getTextContent();

			if( Parameters.set(key, value) ){
				Log.d("   "+key+" <- "+value);
			}
		}
		
		Parameters.onLoaded();
	}

	public static void saveToFile(String path) throws IOException,
			TransformerException, ParserConfigurationException,
			TransformerConfigurationException {
		try {
			Log.d("Save properties to "+new File(path).getAbsolutePath());
			File xmlFile = new File(path);
			DocumentBuilderFactory dbFactory = DocumentBuilderFactory
					.newInstance();
			DocumentBuilder db = dbFactory.newDocumentBuilder();
			Document doc = db.newDocument();

			Element root = doc.createElement("parameters");
			doc.appendChild(root);

			// use reflection to get all field values from Parameter class
			Map<String, String> map = getParameters();

			for (String name : map.keySet()) {
				root.appendChild(createNode(doc, name, map.get(name)));
			}

			Source xmlSource = new DOMSource(doc);
			Result result = new StreamResult(new FileOutputStream(xmlFile));
			TransformerFactory transformerFactory = TransformerFactory
					.newInstance();
			Transformer transformer = transformerFactory.newTransformer();
			transformer.setOutputProperty("indent", "yes");
			transformer.transform(xmlSource, result);
		} catch (Exception ex) {
			Log.e(ex.getMessage());
		}
	}

	public static void setParametersFromTable(JTable table) {

		// get updated values from table model
		TableModel model = table.getModel();
		Log.d("Update properties :");
		for (int i = 0; i < model.getRowCount(); ++i) {
			String key = model.getValueAt(i, 0).toString();
			String value = model.getValueAt(i, 1).toString();

			if( Parameters.set(key, value) ){
				Log.d("   "+key+" <- "+value);
			}
		}
		
		Parameters.onChanged();
	}
}
