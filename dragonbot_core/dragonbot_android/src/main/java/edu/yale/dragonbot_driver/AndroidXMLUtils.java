package edu.yale.dragonbot_driver;

import mcbmini.utils.XMLUtils;

import mcbmini.MCBMiniBoard;
import mcbmini.MCBMiniConstants;

import org.jdom.Document;
import org.jdom.Element;
import org.jdom.JDOMException;
import org.jdom.input.SAXBuilder;

import java.io.*;
import java.util.*;

public class AndroidXMLUtils extends XMLUtils {

  public static Element loadXMLStream(InputStream stream) throws Exception{
    SAXBuilder builder = new SAXBuilder();
    Document doc;

    Element root = null;
    try {
      doc = builder.build(stream);
      root = doc.getRootElement();
    } catch (JDOMException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    return root;
  }
 
  public static XMLResults parseMCBMiniConfigStream(InputStream stream) throws Exception{
    XMLResults res = new XMLResults();

    Element root = findMCBRoot( loadXMLStream(stream) );
    if( root == null ){
      throw new Exception("Can't find <MiniBoards> tag in xml file");
    }

    res.port_name = getOptional(root, "port");
    res.boards = new ArrayList<MCBMiniBoard>();

    if(root.getChild("MiniBoards") == null ) throw new RuntimeException("XML file has no MiniBoards list !");
    List<Element> miniBoards = root.getChild("MiniBoards").getChildren();

    for(Element board : miniBoards){
      // Parse board information
      MCBMiniBoard miniBoard = parseMCBMiniBoard(board);

      res.boards.add(miniBoard);
    }

    return res;
  }

  private static Element findMCBRoot(Element root_candidate){
    if( root_candidate.getChild("MiniBoards") != null ) return root_candidate;

    for(Object e : root_candidate.getChildren()){
      if( e instanceof Element ){
        Element root = findMCBRoot( (Element)e );
        if( root != null ) return root;
      }
    }
    return null;
  }

}
