#include "transport_manager.h"

#include <cmath>
#include <tuple>
#include <unordered_set>
#include <stdexcept>

using namespace std;

const double PI = 3.1415926535;
const double EARTH_RADIUS = 6371'000.0;

// GeoCoordinates

GeoCoordinates::GeoCoordinates(double latitude_degree, double longitude_degree)
	: latitude_(ToRadian(latitude_degree)),
	longitude_(ToRadian(longitude_degree)) {}

double GeoCoordinates::ToRadian(double degree) {
	return degree * (PI / 180.0);
}

double ComputeDistanceForCoords(const GeoCoordinates& lhs, const GeoCoordinates& rhs) {
	const double arc_distance = 
		sin(lhs.latitude_) * sin(rhs.latitude_) +
		cos(lhs.latitude_) * cos(rhs.latitude_) *
		cos(lhs.longitude_ - rhs.longitude_);

	return acos(arc_distance) * EARTH_RADIUS;
}
// GeoCoordinates end

EdgeWeight::EdgeWeight(double weight)
	: type_(EdgeType::BUS),
		weight_(weight)
{
}

EdgeWeight::EdgeWeight(EdgeType type, double weight)
	: type_(type),
		weight_(weight)
{
}

EdgeWeight::operator double() const {
	return weight_;
}

EdgeWeight& EdgeWeight::operator += (const EdgeWeight& other) {
	weight_ += other.weight_;
	return *this;
}

EdgeWeight operator + (const EdgeWeight& lhs, const EdgeWeight& rhs) {
	return EdgeWeight(lhs).operator+=(rhs);
}

TransportManager::RoutingSettings::RoutingSettings(int wait, double velocity_in_kmph)
	: wait_time(wait), velocity(velocity_in_kmph / 60.0) {}

TransportManager::TransportManager(size_t stops_count, int bus_wait_time, double bus_velocity_in_kmph)
	: graph(stops_count),
		settings( bus_wait_time, bus_velocity_in_kmph )
{}

double TransportManager::Distance::Curvature() const {
	return by_default_ / raw_;
}




TransportManager::ConstStopRawPtr TransportManager::SetStop(string stop_id, TransportManager::Stop stop) {
	auto [it, is_inserted] = stops_.emplace(std::move(stop_id), std::move(stop));
	return &(*it);
}

TransportManager::StopRawPtr TransportManager::GetStop(const std::string& stop_id) {
	if (auto it = stops_.find(stop_id); it != stops_.end()) {
		return &(*it);
	}
	return nullptr;
}

TransportManager::ConstStopRawPtr TransportManager::GetStop(const string& stop_id) const {
	if (auto it = stops_.find(stop_id); it != stops_.end()) {
		return &(*it);
	}
	return nullptr;
}


TransportManager::ConstStopRawPtr TransportManager::GetStopByVertexId(Graph::VertexId id) const {
	return vertex_id_to_stop.at(id);
}
Graph::VertexId TransportManager::GetVertexIdByStop(const std::string& stop_id) const {
	return stop_to_vertex_id.at(GetStop(stop_id));
}

Graph::VertexId TransportManager::GetVertexIdByWaitStop(const std::string& stop_id) const {
	return wait_stop_to_vertex_id.at(GetStop(stop_id));
}

const TransportManager::EdgeInfo& TransportManager::GetEdgeInfoByEdgeId(Graph::EdgeId id) const {
	return edge_id_to_info.at(id);
}


TransportManager::BusRawPtr TransportManager::SetBus(string bus_id, TransportManager::Bus info) {
	auto [it, is_inserted] = buses_.emplace(std::move(bus_id), std::move(info));
	return &(*it);
}

TransportManager::ConstBusRawPtr TransportManager::GetBus(const string& bus_id) const {
	if (auto it = buses_.find(bus_id); it != buses_.end()) {
		return &(*it);
	}
	return nullptr;
}

TransportManager::Distance TransportManager::ComputeDistanceForStop(const string& bus_id) const {
	auto bus_it = buses_.find(bus_id);
	if (bus_it == buses_.end()) {
		return { 0.0, 0.0};
	}

	
	optional<ConstStopRawPtr> last;
	Distance dist;

	for (auto stop_ptr : bus_it->second.stops_) {
		if (last) {
			const double raw_distance =
				ComputeDistanceForCoords((*last)->second.coords_, stop_ptr->second.coords_);
			dist.raw_ += raw_distance;
			dist.by_default_ += GetDistance((*last)->first, stop_ptr->first);

			if (bus_it->second.type_ == Bus::Type::REGULAR) {
				dist.raw_ += raw_distance;
				dist.by_default_ += GetDistance(stop_ptr->first, (*last)->first);
			}
		}
		last = stop_ptr;
	}
	return dist;
}

double TransportManager::GetDistance(const string& from, const string& to) const {
	auto from_it = stops_.find(from);
	auto to_it = stops_.find(to);
	if (from_it == stops_.end() || to_it == stops_.end()) {
		throw invalid_argument("Valid stops expected: " + from + "&&" + to);
	}

	ConstStopRawPtr from_ptr = &(*from_it), to_ptr = &(*to_it);
	if (distances_.count(from_ptr) < 1 || distances_.at(from_ptr).count(to_ptr) < 1) {
		return ComputeDistanceForCoords(from_ptr->second.coords_, to_ptr->second.coords_);
	}
	return distances_.at(from_ptr).at(to_ptr);
}

const Graph::DirectedWeightedGraph<EdgeWeight>& TransportManager::GetGraph() const {
	return graph;
}

int TransportManager::GetBusWaitTime() const {
	return settings.wait_time;
}

double TransportManager::GetBusVelocity() const {
	return settings.velocity;
}
const Graph::Edge<EdgeWeight>& TransportManager::GetGraphEdge(Graph::EdgeId edge_id) const {
	return graph.GetEdge(edge_id);
}

TransportManager::EdgeInfo::EdgeInfo()
	: stops_count(0),
	bus_ptr(nullptr)
{
}

TransportManager::EdgeInfo::EdgeInfo(int32_t stops_count_, TransportManager::ConstBusRawPtr bus_ptr_)
	: stops_count(stops_count_),
	bus_ptr(bus_ptr_)
{
}