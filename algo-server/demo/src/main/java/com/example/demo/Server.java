package com.example.demo;

import java.net.*;
import java.nio.charset.StandardCharsets;
import java.text.ParseException;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;

import java.io.*;

public class Server {
	public static void main(String[] args) throws IOException {
	      try {
	       ServerSocket serversocket = new ServerSocket(6000);
	       System.out.println("Waiting for connections");
	       Socket client = serversocket.accept();
	       main test = new main();
	       
	       final BufferedReader input = new BufferedReader(
	           new InputStreamReader(client.getInputStream(), StandardCharsets.UTF_8)); // changed
	       final OutputStream output = client.getOutputStream();
	 
	        String message = input.readLine(); // changed
	        System.out.println(message);
	        String x = test.findPathJson(message);
	              output.write(x.getBytes(StandardCharsets.UTF_8)); // changed
	 
	       //client.close();
	       //serversocket.close();
	 
	         } catch (IOException e) {
	             e.printStackTrace();
	         } 
	 //     catch (org.json.simple.parser.ParseException e) {
	 //   // TODO Auto-generated catch block
	 //   e.printStackTrace();
	 //  }
	         System.exit(0);
	     }
}
