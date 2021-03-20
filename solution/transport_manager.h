#pragma once

#include "router.h"

#include <vector>
#include <unordered_map>
#include <set>
#include <string_view>
#include <optional>
#include <memory>


struct GeoCoordinates {
	GeoCoordinates(double latitude_degree, double longitude_degree);

	static double ToRadian(double degree);

	double latitude_, longitude_;
};

GeoCoordinates ParseGeoCoordinates(std::string_view str);

double ComputeDistanceForCoords(const GeoCoordinates& lhs, const GeoCoordinates& rhs);

enum class EdgeType {
	BUS,
	WAIT
};

struct EdgeWeight {
	EdgeWeight(double weight = 0.0);
	EdgeWeight(EdgeType type, double weight);

	EdgeType type_;
	double weight_;

	EdgeWeight& operator += (const EdgeWeight& other);
	operator double() const;
};
EdgeWeight operator + (const EdgeWeight& lhs, const EdgeWeight& rhs);

struct EdgeInfo;



class TransportManager {
public:
	struct Stop {
		GeoCoordinates coords_;
		std::set<std::string_view> buses_;
	};
	using Stops = std::unordered_map<std::string, Stop>;
	using StopRawPtr = Stops::pointer;
	using ConstStopRawPtr = Stops::const_pointer;

	struct Bus {
		enum class Type {
			CIRCULAR,
			REGULAR
		};

		Type type_ = Type::REGULAR;
		std::vector<ConstStopRawPtr> stops_;
	};

	using Buses = std::unordered_map<std::string, Bus>;

	using BusRawPtr = Buses::pointer;
	using ConstBusRawPtr = Buses::const_pointer;

	using Distances = std::unordered_map<ConstStopRawPtr, std::unordered_map<ConstStopRawPtr, double>>;

	struct Distance {
		double raw_ = 0.0;
		double by_default_ = 0.0;
		double Curvature() const;
	};

	struct RoutingSettings {
		RoutingSettings(int wait, double velocity_in_kmph);

		int wait_time;
		double velocity;
	};

	struct EdgeInfo {
		EdgeInfo();
		EdgeInfo(int32_t stops_count_, TransportManager::ConstBusRawPtr bus_ptr_);

		int32_t stops_count;
		TransportManager::ConstBusRawPtr bus_ptr;
	};

	friend class TransportManagerBuilder;

public:
	ConstStopRawPtr SetStop(std::string stop_id, Stop stop);
	StopRawPtr GetStop(const std::string& stop_id);
	ConstStopRawPtr GetStop(const std::string& stop_id) const;
	BusRawPtr SetBus(std::string bus_id, Bus info);
	ConstBusRawPtr GetBus(const std::string& bus_id) const;

	ConstStopRawPtr GetStopByVertexId(Graph::VertexId id) const;
	Graph::VertexId GetVertexIdByStop(const std::string& stop_id) const;
	Graph::VertexId GetVertexIdByWaitStop(const std::string& stop_id) const;
	const EdgeInfo& GetEdgeInfoByEdgeId(Graph::EdgeId id) const;

	const Graph::DirectedWeightedGraph<EdgeWeight>& GetGraph() const;
	const Graph::Edge<EdgeWeight>& GetGraphEdge(Graph::EdgeId edge_id) const;

	Distance ComputeDistanceForStop(const std::string& bus_id) const;

	int GetBusWaitTime() const;
	double GetBusVelocity() const;


	double GetDistance(const std::string& from, const std::string& to) const;
private:

	TransportManager(size_t stops_count, int bus_wait_time, double bus_velocity_in_kmph);

	RoutingSettings settings;
	Stops stops_;
	Buses buses_;
	Distances distances_;
	std::unordered_map<Graph::VertexId, ConstStopRawPtr> vertex_id_to_stop;
	std::unordered_map<ConstStopRawPtr, Graph::VertexId> stop_to_vertex_id;
	std::unordered_map<ConstStopRawPtr, Graph::VertexId> wait_stop_to_vertex_id;
	std::unordered_map<Graph::EdgeId, EdgeInfo> edge_id_to_info;
	Graph::DirectedWeightedGraph<EdgeWeight> graph;
};