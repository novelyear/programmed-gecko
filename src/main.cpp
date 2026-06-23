#include "pch.h"
#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"
#include "BodyChain.h"
#include "Renderer.h"
#include <string>

#define BODY_CONFIG_FILE "src/body-config.json"
using namespace nlohmann;

void initialize_body(BodyChain* body)
{
	try
	{
		std::ifstream file(BODY_CONFIG_FILE);
		if (!file.is_open())
		{
			std::cerr << "Failed to open: " << BODY_CONFIG_FILE << std::endl;
			return;
		}

		json j;
		file >> j;

		if (!j.contains("distance") ||
			!j.contains("min_angle") ||
			!j.contains("nodes"))
		{
			std::cerr << "Some essential data Miss!" << std::endl;
			return;
		}

		std::vector<std::vector<float>> body_nodes_data;
		for (const auto& v : j["nodes"]) {
			body_nodes_data.push_back({ v[0], v[1], v[2] });
		}
		body->createNodeFromArray(body_nodes_data);
		body->setDistance(j["distance"]);
		body->setMinAngle(j["min_angle"]);
	}
	catch (const std::exception& e) {
		std::cerr << "Error loading body data: " << e.what() << std::endl;
		return;
	}
}

int main()
{
	BodyChain body;
	initialize_body(&body);
	Renderer renderer(1600, 1200, "gecko");
	while (true) {
		auto bodyData = body.getNodeData();
		auto footData = std::vector<std::vector<float>>();
		renderer.render(bodyData, footData);
	}
	return 0;
}