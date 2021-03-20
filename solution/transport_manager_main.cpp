#include "transport_manager.h"
#include "requests.h"
#include "json.h"

#include <mutex>
#include <future>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <iomanip>
#include <tuple>
#include <unordered_set>
#include <string>
#include <string_view>
#include <utility>
#include <optional>

using namespace std;

class TransportManagerBuilder {
public:
	void SetBusSettings(int wait_time, double velocity_in_kmph) {
		wait_time_ = wait_time;
		velocity_in_kmph_ = velocity_in_kmph;
	}

	void AddQuery(const Json::Node& node) {
		const auto& mapped_node = node.AsMap();
		if (mapped_node.at("type"s).AsString() == "Stop"s) {
			stop_requests_.push_back(&node);
			if (mapped_node.count("road_distances"s) > 0 &&
				mapped_node.at("road_distances"s).AsMap().size() > 0) {
				distances_.push_back(&node);
			}
		}
		else if (mapped_node.at("type"s).AsString() == "Bus"s) {
			bus_requests_.push_back(&node);
		}
	}

	TransportManager Build() const {
		TransportManager manager(stop_requests_.size() * 2, wait_time_, velocity_in_kmph_);
		BuildStops(manager);
		BuildDistances(manager);
		BuildBuses(manager);
		return manager;
	}

private:

	void BuildStops(TransportManager& manager) const {
		for (auto stop_request : stop_requests_) {
			const auto& request_map = stop_request->AsMap();
			auto stop_ptr = manager.SetStop(
				request_map.at("name"s).AsString(),
				{ GeoCoordinates(
						request_map.at("latitude"s).AsDouble(),
						request_map.at("longitude"s).AsDouble()
				) }
			);

			auto [it, is_inserted] = manager.stop_to_vertex_id.insert({
				stop_ptr,
				manager.stop_to_vertex_id.size() + manager.wait_stop_to_vertex_id.size()
				});
			manager.vertex_id_to_stop.insert({ it->second, stop_ptr });

			auto [wait_it, is_wait_inserted] = manager.wait_stop_to_vertex_id.insert({
				stop_ptr,
				manager.stop_to_vertex_id.size() + manager.wait_stop_to_vertex_id.size()
				});

			manager.graph.AddEdge({
					wait_it->second,
					it->second,
					EdgeWeight(EdgeType::WAIT, manager.GetBusWaitTime())
				});
		}
	}

	void BuildDistances(TransportManager& manager) const {
		for (auto distances_from_stop : distances_) {
			const auto& request_map = distances_from_stop->AsMap();

			auto from_ptr = manager.GetStop(request_map.at("name"s).AsString());

			for (const auto& stop_to_distance : request_map.at("road_distances"s).AsMap()) {
				auto to_ptr = manager.GetStop(stop_to_distance.first);
				double distance = static_cast<double>(stop_to_distance.second.AsInt());
				if (!(from_ptr && to_ptr)) {
					throw invalid_argument(
						"Valid Stops were expected: " +
						request_map.at("name"s).AsString() + "&&" + stop_to_distance.first
					);
				}

				manager.distances_[from_ptr][to_ptr] = distance;
				if (
					manager.distances_.count(to_ptr) < 1 ||
					manager.distances_.at(to_ptr).count(from_ptr) < 1
					) {
					manager.distances_[to_ptr][from_ptr] = distance;
				}
			}
		}
	}

	void BuildBackEdgesForRegularBus(TransportManager& manager,
																	 const vector<TransportManager::ConstStopRawPtr>& stops,
																	 TransportManager::ConstBusRawPtr bus_ptr,
																	 Graph::VertexId vertex_id,
																	 int32_t stops_count, double weight) const {
		optional<TransportManager::ConstStopRawPtr> last_stop_opt;
		for (auto next_stop_ptr : Range(stops.crbegin(), stops.crend())) {
			if (last_stop_opt) {
				++stops_count;
				weight += manager.GetDistance((*last_stop_opt)->first, next_stop_ptr->first) / 1000.0 / manager.GetBusVelocity();
				const Graph::EdgeId edge_id = manager.graph.AddEdge({
						vertex_id,
						manager.GetVertexIdByWaitStop(next_stop_ptr->first),
						EdgeWeight(EdgeType::BUS, weight)
					});

				manager.edge_id_to_info.insert({ edge_id, TransportManager::EdgeInfo(stops_count, bus_ptr) });
			}
			last_stop_opt = next_stop_ptr;
		}
	}


	void BuildEdgesForBus(TransportManager& manager,
												const vector<TransportManager::ConstStopRawPtr>& stops,
												TransportManager::ConstBusRawPtr bus_ptr, bool needs_wayback) const {
		for(auto it = stops.begin(); it!=stops.end(); ++it) {
			auto stop_ptr = *it;

			int32_t stops_count = 0;
			double weight = 0.0;
			Graph::VertexId vertex_id = manager.GetVertexIdByStop(stop_ptr->first);

			TransportManager::ConstStopRawPtr last_stop_ptr = stop_ptr;
			for (TransportManager::ConstStopRawPtr next_stop_ptr : Range(next(it), stops.end())) {
				++stops_count;
				weight += ((manager.GetDistance(last_stop_ptr->first, next_stop_ptr->first) / 1000.0) / manager.GetBusVelocity());
				const Graph::EdgeId edge_id = manager.graph.AddEdge({
						vertex_id,
						manager.GetVertexIdByWaitStop(next_stop_ptr->first),
						EdgeWeight(EdgeType::BUS, weight)
					});

				manager.edge_id_to_info.insert({ edge_id, TransportManager::EdgeInfo(stops_count, bus_ptr) });

				last_stop_ptr = next_stop_ptr;
			}
			if (needs_wayback) {
				BuildBackEdgesForRegularBus(manager, stops, bus_ptr, vertex_id, stops_count, weight);
			}
		}
	}
	
	template <typename Container>
	void BuildReversedEdgesForBus(TransportManager& manager,
												const Container& stops,
												TransportManager::ConstBusRawPtr bus_ptr) const {
		for(auto it = stops.begin(); it!=stops.end(); ++it) {
			auto stop_ptr = *it;

			int32_t stops_count = 0;
			double weight = 0.0;
			Graph::VertexId vertex_id = manager.GetVertexIdByStop(stop_ptr->first);

			TransportManager::ConstStopRawPtr last_stop_ptr = stop_ptr;
			for (TransportManager::ConstStopRawPtr next_stop_ptr : Range(next(it), stops.end())) {
				++stops_count;
				weight += ((manager.GetDistance(last_stop_ptr->first, next_stop_ptr->first) / 1000.0) / manager.GetBusVelocity());
				const Graph::EdgeId edge_id = manager.graph.AddEdge({
						vertex_id,
						manager.GetVertexIdByWaitStop(next_stop_ptr->first),
						EdgeWeight(EdgeType::BUS, weight)
					});

				manager.edge_id_to_info.insert({ edge_id, TransportManager::EdgeInfo(stops_count, bus_ptr) });

				last_stop_ptr = next_stop_ptr;
			}
		}
	}




	void BuildBuses(TransportManager& manager) const {
		for (const auto& bus_request : bus_requests_) {
			const auto& request_map = bus_request->AsMap();

			auto bus_ptr = manager.SetBus(request_map.at("name"s).AsString(), {});

			TransportManager::Bus& bus = bus_ptr->second;

			bus.type_ = request_map.at("is_roundtrip"s).AsBool() ?
									TransportManager::Bus::Type::CIRCULAR :
									TransportManager::Bus::Type::REGULAR;

			const auto& stops = request_map.at("stops"s).AsArray();

			for (const auto& stop : stops) {
				auto stop_ptr = manager.GetStop(stop.AsString());
				if (!stop_ptr) {
					throw invalid_argument("valid stop expected: " + stop.AsString());
				}
				bus.stops_.push_back(stop_ptr);
				stop_ptr->second.buses_.insert(bus_ptr->first);
			}

			BuildEdgesForBus(manager, bus.stops_, bus_ptr, bus.type_ == TransportManager::Bus::Type::REGULAR);

			if (bus.type_ == TransportManager::Bus::Type::REGULAR) {
				BuildReversedEdgesForBus(manager, Range(bus.stops_.crbegin(), bus.stops_.crend()), bus_ptr);
			}
		}
	}
private:
	int wait_time_ = 0;
	double velocity_in_kmph_ = 0.0;
	std::vector<const Json::Node*> stop_requests_, bus_requests_, distances_;
};

TransportManagerBuilder ParseBaseRequests(const vector<Json::Node>& requests,
																					const map<string, Json::Node>& routing_settings)
{
	TransportManagerBuilder builder;

	builder.SetBusSettings(
		routing_settings.at("bus_wait_time"s).AsInt(),
		routing_settings.at("bus_velocity"s).AsDouble()
	);

	for (const auto& node : requests) {
		builder.AddQuery(node);
	}

	return builder;
}

const unordered_map<string_view, Request::Type> STR_TO_REQUEST_TYPE = {
	{"Bus", Request::Type::BUS},
	{"Stop", Request::Type::STOP},
	{"Route", Request::Type::ROUTE}
};

optional<Request::Type> ConvertRequestTypeFromString(string_view type_str) {
	if (const auto it = STR_TO_REQUEST_TYPE.find(type_str);
		it != STR_TO_REQUEST_TYPE.end()) {
		return it->second;
	}
	else {
		return nullopt;
	}
}

RequestHolder ParseRequest(const map<string, Json::Node>& request_map) {
	const auto req_type = ConvertRequestTypeFromString(request_map.at("type"s).AsString());
	if (!req_type) {
		return	nullptr;
	}
	RequestHolder request = Request::Create(*req_type, request_map.at("id"s).AsInt());
	if (request) {
		request->ParseFrom(request_map);
	}
	return request;
}

vector<RequestHolder> ReadStatRequests(const vector<Json::Node>& stat_requests) {
	vector<RequestHolder> requests;

	for (auto& request_node : stat_requests) {
		auto& request_map = request_node.AsMap();
		if (auto request = ParseRequest(request_map); request) {
			requests.push_back(std::move(request));
		}
	}
	return requests;
}

Json::Document ProcessRequests(TransportManager& manager,
															 Graph::Router<EdgeWeight>& router,
															 const vector<RequestHolder>& requests)
{
	vector<Json::Node> responses;
	for (const auto& req_holder : requests) {
		if (req_holder->type == Request::Type::BUS || req_holder->type == Request::Type::STOP) {
			const auto& request = static_cast<const Requests::Read<Json::Node>&>(*req_holder);
			responses.push_back(std::move(request.Process(manager)));
		}
		else if (req_holder->type == Request::Type::ROUTE) {
			const auto& request = static_cast<const Requests::RouteInfo&>(*req_holder);
			responses.push_back(std::move(request.Process(manager, router)));
		}
		// ...
	}
	return Json::Document{ Json::Node(std::move(responses)) };
}

void PrintResponses(const Json::Document& responses, ostream& os = cout) {
	Json::Print(os, responses);
}

int main() {
	Json::Document document = Json::Load(cin);
	auto& requests = document.GetRoot().AsMap();
	const vector<Json::Node>& base_requests = requests.at("base_requests"s).AsArray();
	const map<string, Json::Node>& routing_settings = requests.at("routing_settings"s).AsMap();

	auto tm_builder = ParseBaseRequests(base_requests, routing_settings);

	TransportManager transport_manager = tm_builder.Build();


	Graph::Router<EdgeWeight> router(transport_manager.GetGraph());

	auto stat_requests = requests.at("stat_requests"s).AsArray();
	auto stat_request_holders = ReadStatRequests(stat_requests);
	Json::Document responses = ProcessRequests(transport_manager, router, stat_request_holders);
	PrintResponses(responses);
	return 0;
}
