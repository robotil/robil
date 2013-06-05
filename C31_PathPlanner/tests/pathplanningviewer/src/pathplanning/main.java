/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package pathplanning;

import javax.swing.JFrame;

/**
 *
 * @author dan
 */
public class main {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        MapBoard mb = new MapBoard();
        String[] config_path={
            "src/pathplanning/config.xml",
            "config.xml"
        };
        for(int i=0; i<config_path.length && !mb.loadConfigFile(config_path[i]);i++);
        mb.loadXmlFile("src/pathplanning/out.xml");
        mb.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        mb.setVisible(true);
    }
}
