package com.example.demo;

import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;
import org.springframework.web.bind.annotation.RestController;


@SpringBootApplication
@RestController
@ResponseBody
@Controller
public class DemoApplication {
	main test = new main();
	public static void main(String[] args) {
		// GUI running here, comment out if dw
//		main test = new main();
//		String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":[{\"x\":5,\"y\":13,\"id\":1,\"d\":6},{\"x\":15,\"y\":15,\"id\":2,\"d\":2},{\"x\":10,\"y\":15,\"id\":3,\"d\":0},{\"x\":5,\"y\":7,\"id\":5,\"d\":6},{\"x\":15,\"y\":7,\"id\":4,\"d\":2}],\"mode\":\"0\"}}";
//		
//		String outputjson = test.findPathJson(json);
//		
//		String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":["
//			    + "{\"x\":1,\"y\":18,\"id\":1,\"d\":4},"
//			    + "{\"x\":6,\"y\":12,\"id\":2,\"d\":0},"
//			    + "{\"x\":14,\"y\":16,\"id\":3,\"d\":6},"
//			    + "{\"x\":10,\"y\":6,\"id\":4,\"d\":2},"
//			    + "{\"x\":13,\"y\":2,\"id\":5,\"d\":2},"
//			    + "{\"x\":18,\"y\":9,\"id\":6,\"d\":6},"
//			    + "],\"mode\":\"0\"}}";
		
//		String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":["
//			    + "{\"x\":5,\"y\":3,\"id\":1,\"d\":0},"
//			    + "{\"x\":2,\"y\":9,\"id\":2,\"d\":0},"
//			    + "{\"x\":13,\"y\":8,\"id\":3,\"d\":4},"
//			    + "{\"x\":11,\"y\":14,\"id\":4,\"d\":6},"
//			    + "{\"x\":2,\"y\":17,\"id\":5,\"d\":2},"
//			    + "{\"x\":16,\"y\":17,\"id\":6,\"d\":2},"
//			    + "],\"mode\":\"0\"}}";
		
		// sem 2 run 2
//		String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":["
//			    + "{\"x\":11,\"y\":4,\"id\":1,\"d\":6},"
//			    + "{\"x\":6,\"y\":10,\"id\":2,\"d\":4},"
//			    + "{\"x\":1,\"y\":16,\"id\":3,\"d\":4},"
//			    + "{\"x\":13,\"y\":14,\"id\":4,\"d\":4},"
//			    + "{\"x\":18,\"y\":3,\"id\":5,\"d\":0},"
//			    + "{\"x\":18,\"y\":16,\"id\":6,\"d\":4},"
//			    + "],\"mode\":\"0\"}}";

		// 2022 sem 1 run 1
		String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":["
			    + "{\"x\":2,\"y\":12,\"id\":1,\"d\":0},"
			    + "{\"x\":10,\"y\":6,\"id\":2,\"d\":2},"
			    + "{\"x\":11,\"y\":18,\"id\":3,\"d\":4},"
			    + "{\"x\":19,\"y\":15,\"id\":4,\"d\":6},"
			    + "{\"x\":15,\"y\":9,\"id\":5,\"d\":0},"
			    + "{\"x\":18,\"y\":2,\"id\":6,\"d\":6},"
			    + "],\"mode\":\"0\"}}";
	
		main test = new main();
		test.createGUI();
		test.findPathJson(json);
//		SpringApplication.run(DemoApplication.class, args);
		
	}

	// @RequestMapping("/response")
	// public Map<String, String> sayHello() {
	// 	HashMap<String, String> map = new HashMap<>();
	// 	map.put("key", "value");
	// 	map.put("foo", "bar");
	// 	map.put("aa", "bb");
	// 	return map;
	// }

	@PostMapping("/api")
    public String postBody(@RequestBody JSONObject obstacles) {


		String output = test.callAlgo(obstacles);

        return output;
    }
	
	@PostMapping("/checklist")
    public String postBody(@RequestBody String obstacles) {

		String output = test.turnToNextSide(obstacles);
        return output;
    }
}

