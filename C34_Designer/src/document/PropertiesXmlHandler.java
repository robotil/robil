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

import org.w3c.dom.Attr;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class PropertiesXmlHandler {

	public static String lastPath = "BTDesigner.xml";

	public static void loadAndSetProperties(String path) throws IOException,
			ParserConfigurationException, SAXException {
		File xmlFile = new File(path);
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder db = dbFactory.newDocumentBuilder();
		Document doc = db.parse(xmlFile);

		doc.getDocumentElement().normalize();

		NodeList list = doc.getDocumentElement().getChildNodes();
		System.out.println("list size = " + list.getLength());
		for (int i = 0; i < list.getLength(); ++i) {
			Node n = list.item(i);
			System.out.println(n.getNodeName());

			if (n.getNodeName().trim().startsWith("#")) {
				continue;
			}
			String key = n.getNodeName().trim();
			String value = n.getAttributes().getNamedItem("value")
					.getTextContent();
			if (key.equals("test_time")) {
				Parameters.test_time = Integer.parseInt(value);
				
			} else if (key.equals("test_result")) {
				Parameters.test_result = Boolean.parseBoolean(value);
			} else if (key.equals("path_to_plans")) {
				Parameters.path_to_plans = new String(value);
			} else if (key.equals("path_to_images")) {
				Parameters.path_to_images = new String(value);
			} else if (key.equals("executer_service")) {
				Parameters.executer_service = new String(value);
			} else if (key.equals("path_to_lookup")) {
				Parameters.path_to_lookup = new String(value);
			} else if (key.equals("path_to_address")) {
				Parameters.path_to_address = new String(value);
			} else if (key.equals("path_to_description")) {
				Parameters.path_to_description = new String(value);
			} else {
				System.err.println("Error: unknown field in " + path);
			}
		}
	}

	private static Node createNode(Document doc, String name, String value) {
		Element element = doc.createElement(name);
		// element.appendChild(doc.createTextNode(name));

		Attr attribute = doc.createAttribute("value");
		attribute.setValue(value);
		element.setAttributeNode(attribute);

		return element;

		// create FirstName and LastName elements
		// Element firstName = doc.createElement("FirstName");
		// Element lastName = doc.createElement("LastName");
		//
		// firstName.appendChild(doc.createTextNode("First Name"));
		// lastName.appendChild(doc.createTextNode("Last Name"));
		//
		// // create contact element
		// Element contact = doc.createElement("contact");
		//
		// // create attribute
		// Attr genderAttribute = doc.createAttribute("gender");
		// genderAttribute.setValue("F");
		//
		// // append attribute to contact element
		// contact.setAttributeNode(genderAttribute);
		// contact.appendChild(firstName);
		// contact.appendChild(lastName);
		//
		// return contact;
	}

	public static Map<String, String> getParameters() {
		Map<String, String> map = new HashMap<String, String>();

		try {
			for (int i = 0; i < Parameters.class.getFields().length; ++i) {
				String name = Parameters.class.getFields()[i].getName();
				String value = Parameters.class.getFields()[i].get(name)
						.toString();
				map.put(name, value);
			}
		} catch (IllegalAccessException ex) {
			System.err.println("Error getParameters: " + ex.getMessage());
		}

		return map;
	}

	public static void setParametersFromTable(JTable table) {
		Map<String, String> map = new HashMap<String, String>();// getParameters();

		// get updated values from table model
		TableModel model = table.getModel();
		for (int i = 0; i < model.getRowCount(); ++i) {
			String key = model.getValueAt(i, 0).toString();
			String value = model.getValueAt(i, 1).toString();

			map.put(key, value);
		}

		// put values in Parameters class
		for (int i = 0; i < Parameters.class.getFields().length; ++i) {
			String name = Parameters.class.getFields()[i].getName();
			String value = map.get(name);

			if (name.equals("test_time")) {
				Parameters.test_time = Integer.parseInt(value);
			} else if (name.equals("test_result")) {
				Parameters.test_result = Boolean.parseBoolean(value);
			} else if (name.equals("path_to_plans")) {
				Parameters.path_to_plans = new String(value);
			} else if (name.equals("path_to_images")) {
				Parameters.path_to_images = new String(value);
			} else if (name.equals("executer_service")) {
				Parameters.executer_service = new String(value);
			} else if (name.equals("path_to_lookup")) {
				Parameters.path_to_lookup = new String(value);
			} else if (name.equals("path_to_address")) {
				Parameters.path_to_address = new String(value);
			} else if (name.equals("path_to_description")) {
				Parameters.path_to_description = new String(value);
			}
		}
	}

	public static void saveToFile(String path) throws IOException,
			TransformerException, ParserConfigurationException,
			TransformerConfigurationException {
		try {
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
			System.err.println(ex.getMessage());
		}
	}
}
