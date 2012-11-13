package document;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Image;
import java.io.File;
import java.io.IOException;


import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Element;
import org.xml.sax.SAXException;


public class BTDesigner extends JFrame{

	public final static String VERSION= "0.1.1";

	Document document = new Document(this);
	Toolbar toolbar = new Toolbar(document);
	
	public BTDesigner(){
		
		this.setTitle("Cogniteam BTDesigner "+BTDesigner.VERSION);
		
		setLocation(200, 50);
		setSize(new Dimension(1000,700));
		ImageIcon icon = new ImageIcon(getClass().getClassLoader().getResource("icons/CogniTeam.gif"));
		this.setIconImage(icon.getImage());
		setLayout(new BorderLayout());
		add(toolbar, BorderLayout.NORTH);
		add(document, BorderLayout.CENTER);
		
	}
	
	
	public static void main(String[] args) {
		org.w3c.dom.Document doc = null;
		try{
			doc = 
				DocumentBuilderFactory
					.newInstance()
						.newDocumentBuilder()
							.parse(new File("BTDesigner.xml"));
		}catch(javax.xml.parsers.ParserConfigurationException ex){
			ex.printStackTrace();
		} catch (SAXException ex) {
			ex.printStackTrace();
		} catch (IOException ex) {
			ex.printStackTrace();
		}
		Element el = (Element)(doc.getElementsByTagName("dbg_time").item(0));
		Parameters.dbg_time = Integer.parseInt(el.getAttribute("value"));
		el = (Element)(doc.getElementsByTagName("dbg_result").item(0));
		Parameters.dbg_result = Boolean.parseBoolean(el.getAttribute("value"));
		
		
		
		BTDesigner btd = new BTDesigner();
		btd.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		btd.setVisible(true);
	}

}
