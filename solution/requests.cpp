#include "requests.h"

#include <sstream>
#include <iomanip>
#include <unordered_set>
#include <iterator>

using namespace std;

RequestHolder Request::Create(Type type, int32_t id) {
	switch (type) {
	case Type::BUS :
		return std::make_unique<Requests::BusInfo>(id);
		break;
	case Type::STOP :
		return std::make_unique<Requests::StopInfo>(id);
		break;
	case Type::ROUTE :
		return std::make_unique<Requests::RouteInfo>(id);
		break;
	default:
		return nullptr;
	}
}

namespace Requests {
	Json::Node BusInfo::Process(const TransportManager& manager) const {
		auto bus_ptr = manager.GetBus(name);
		map<string, Json::Node> result;
		result["request_id"s] = Json::Node(request_id);

		if (bus_ptr) {
			auto bus_info_ptr = &bus_ptr->second;

			const int32_t stop_count =
				bus_info_ptr->type_ == TransportManager::Bus::Type::REGULAR ?
				bus_info_ptr->stops_.size() * 2 - 1 :
				bus_info_ptr->stops_.size();

			const auto length = manager.ComputeDistanceForStop(bus_ptr->first);

			const int32_t unique_stop_count = unordered_set<TransportManager::ConstStopRawPtr>(
				bus_info_ptr->stops_.begin(),
				bus_info_ptr->stops_.end()
			).size();
			// assigning map
			result["route_length"s] = Json::Node(static_cast<double>(length.by_default_));
			result["curvature"s] = Json::Node(static_cast<double>(length.Curvature()));
			result["stop_count"s] = Json::Node(static_cast<int32_t>(stop_count));
			result["unique_stop_count"s] = Json::Node(static_cast<int32_t>(unique_stop_count));
		}

		else {
			result["error_message"s] = Json::Node("not found"s);
		}
		return Json::Node(std::move(result));
	}

	void BusInfo::ParseFrom(const std::map<std::string, Json::Node>& request_map) {
		name = request_map.at("name"s).AsString();
	}

	Json::Node StopInfo::Process(const TransportManager& manager) const {
		auto stop_ptr = manager.GetStop(name);
		map<string, Json::Node> result;
		result["request_id"s] = Json::Node(request_id);

		if (stop_ptr) {
			vector<Json::Node> buses;
			for (string_view bus_view : stop_ptr->second.buses_) {
				buses.push_back(Json::Node(string(bus_view)));
			}
			result.emplace("buses"s, Json::Node(std::move(buses)));
		}
		else {
			result["error_message"s] = Json::Node("not found"s);
		}

		return Json::Node(std::move(result));
	}

	void StopInfo::ParseFrom(const std::map<std::string, Json::Node>& request_map) {
		name = request_map.at("name"s).AsString();
	}

	void RouteInfo::ParseFrom(const std::map<std::string, Json::Node>& request_map) {
		from = request_map.at("from"s).AsString();
		to = request_map.at("to"s).AsString();
	}

	Json::Node RouteInfo::Process(const TransportManager& manager, const Graph::Router<EdgeWeight>& router) const {
		auto route_info_opt = router.BuildRoute(
			manager.GetVertexIdByWaitStop(from),
			manager.GetVertexIdByWaitStop(to)
		);

		map<string, Json::Node> result;
		result["request_id"s] = request_id;

		if (!route_info_opt) {
			result["error_message"s] = "not found"s;
			return Json::Node{ std::move(result) };
		}

		auto& route_info = *route_info_opt;
		result["total_time"s] = route_info.weight.weight_;

		vector<Json::Node> route_elements;

		for (size_t idx = 0; idx < route_info.edge_count; ++idx) {
			const Graph::EdgeId edge_id = router.GetRouteEdge(route_info.id, idx);
			const Graph::Edge<EdgeWeight>& edge = manager.GetGraphEdge(edge_id);

			if (edge.weight.type_ == EdgeType::BUS) {
				const auto& edge_info = manager.GetEdgeInfoByEdgeId(edge_id);
				route_elements.push_back(Json::Node{ map<string, Json::Node>{
					{ "type"s, "Bus"s },
					{ "bus"s, edge_info.bus_ptr->first },
					{ "span_count"s, edge_info.stops_count },
					{ "time"s, edge.weight.weight_ },
				} });
				
			}

			else if (edge.weight.type_ == EdgeType::WAIT) {
				route_elements.push_back(Json::Node{ map<string, Json::Node>{
					{"type"s, "Wait"s},
					{ "stop_name"s, manager.GetStopByVertexId(edge.to)->first },
					{ "time"s, manager.GetBusWaitTime() }
				} });
			}
		}

		result["items"s] = std::move(route_elements);
		return Json::Node{ std::move(result) };
	}
}