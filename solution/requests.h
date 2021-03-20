#pragma once

#include "transport_manager.h"
#include "json.h"

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

struct Request;
using RequestHolder = std::unique_ptr<Request>;

struct Request {
	enum class Type {
		BUS,
		STOP,
		ROUTE
	};
	Request(Type t, size_t id) : type(t), request_id(id) {}
	static RequestHolder Create(Type type, int32_t id);
	virtual void ParseFrom(const std::map<std::string, Json::Node>& request_map) = 0;
	virtual ~Request() = default;
	const Type type;
	int32_t request_id;
};


namespace Requests {
	template <typename ReturnType>
	struct Read : Request {
		using Request::Request;
		virtual ReturnType Process(const TransportManager& manager) const = 0;
	};

	struct BusInfo : Read<Json::Node> {
		BusInfo(int32_t id) : Read(Type::BUS, id) {}

		void ParseFrom(const std::map<std::string, Json::Node>& request_map) override;

		Json::Node Process(const TransportManager& manager) const override;

		std::string name;
	};

	struct StopInfo : Read<Json::Node> {
		StopInfo(int32_t id) : Read(Type::STOP, id) {}

		void ParseFrom(const std::map<std::string, Json::Node>& request_map) override;

		Json::Node Process(const TransportManager& manager) const override;

		std::string name;
	};

	struct RouteInfo : Request {
		RouteInfo(int32_t id) : Request(Type::ROUTE, id) {}

		void ParseFrom(const std::map<std::string, Json::Node>& request_map) override;

		Json::Node Process(const TransportManager& manager, const Graph::Router<EdgeWeight>& router) const;
		std::string from;
		std::string to;
	};
}
