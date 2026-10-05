#include <catch2/catch_test_macros.hpp>

#include <httplib.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "ExampleConfig.h"
#include "demdl/ElevationClient.h"

using namespace std::chrono_literals;
using demdl::Config;
using demdl::ElevationClient;
using demdl::FetchResult;
using demdl::RetryPolicy;

namespace {

RetryPolicy fastPolicy() {
  RetryPolicy p;
  p.maxRetries = 2;
  p.baseDelay = 1ms;
  p.maxDelay = 2ms;
  p.jitter = 0.0;
  return p;
}

// The example configuration's box and layer on a 4 x 1 grid, requested from wmsUrl.
Config configFor(const std::string& wmsUrl) {
  Config c = exampleConfig();
  c.width = 4;
  c.height = 1;
  c.output = "unused.tif";
  c.wmsUrl = wmsUrl;
  return c;
}

// Serves canned GetFeatureInfo bodies on loopback. The pixel column I selects the behaviour: 0 a
// band value, 1 a plain body without a band, 2 a 500 and then a band value, 4 a 200 service
// exception report, any other column a 404. Every request's remote port and decoded parameters are
// recorded. httplib re-encodes the query on the wire (%2C goes out as ','), so tests compare the
// decoded parameters and never the raw request target.
struct CannedServer {
  httplib::Server server;
  std::thread thread;
  std::atomic<int> hitsForColumn2{0};
  std::atomic<int> hitsForColumn4{0};
  std::mutex mutex;
  std::set<int> remotePorts;                      // guarded by mutex; one port per connection
  std::map<std::string, std::string> lastParams;  // guarded by mutex; BBOX, LAYERS, I and J
  int port = 0;

  CannedServer() {
    server.Get("/wms", [this](const httplib::Request& req, httplib::Response& res) {
      {
        const std::lock_guard<std::mutex> lock(mutex);
        remotePorts.insert(req.remote_port);
        for (const char* key : {"BBOX", "LAYERS", "I", "J"}) lastParams[key] = req.get_param_value(key);
      }
      const std::string column = req.get_param_value("I");
      if (column == "0") {
        res.set_content("GetFeatureInfo results:\nBAND 0: 12.5\n", "text/plain");
      } else if (column == "1") {
        res.set_content("GetFeatureInfo results:\n", "text/plain");
      } else if (column == "2") {
        if (hitsForColumn2++ == 0) {
          res.status = 500;
        } else {
          res.set_content("BAND 0: 7\n", "text/plain");
        }
      } else if (column == "4") {
        ++hitsForColumn4;
        res.set_content(R"(<?xml version="1.0"?><ServiceExceptionReport>)"
                        R"(<ServiceException code="LayerNotDefined">no such layer</ServiceException>)"
                        R"(</ServiceExceptionReport>)",
                        "application/vnd.ogc.se_xml");
      } else {
        res.status = 404;
      }
    });
    port = server.bind_to_any_port("127.0.0.1");
    thread = std::thread([this] { server.listen_after_bind(); });
    server.wait_until_ready();
  }
  ~CannedServer() {
    server.stop();
    thread.join();
  }
  std::string url() const { return "http://127.0.0.1:" + std::to_string(port) + "/wms"; }
  std::size_t distinctRemotePorts() {
    const std::lock_guard<std::mutex> lock(mutex);
    return remotePorts.size();
  }
  std::map<std::string, std::string> latestParams() {
    const std::lock_guard<std::mutex> lock(mutex);
    return lastParams;
  }
};

}  // namespace

TEST_CASE("a band value is returned on the first attempt", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(r.value == 12.5f);
  CHECK(r.attempts == 1);
}

TEST_CASE("a plain 200 without a band is NoData", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(1, 0, stop);
  CHECK(r.kind == FetchResult::Kind::NoData);
  CHECK(r.attempts == 1);
}

TEST_CASE("a 500 is retried and the second attempt succeeds", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(2, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(r.value == 7.0f);
  CHECK(r.attempts == 2);
  CHECK(srv.hitsForColumn2.load() == 2);
}

TEST_CASE("a status that never recovers fails after the retries", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(3, 0, stop);  // 404 every time
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error == "http 404");
}

TEST_CASE("a 200 service exception is retried and then fails", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(4, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error == "unparseable body");
  CHECK(srv.hitsForColumn4.load() == 3);
}

TEST_CASE("a refused connection fails after the retries with a transport error", "[elevationclient]") {
  // Port 1 is privileged and never has a listener on a developer machine.
  ElevationClient client(configFor("http://127.0.0.1:1/wms"), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error.rfind("transport: ", 0) == 0);
}

TEST_CASE("a stop request cuts the backoff short", "[elevationclient]") {
  RetryPolicy slow;
  slow.maxRetries = 5;
  slow.baseDelay = 10s;
  slow.maxDelay = 10s;
  slow.jitter = 0.0;
  ElevationClient client(configFor("http://127.0.0.1:1/wms"), slow);
  std::atomic<bool> stop{false};
  std::thread stopper([&stop] {
    std::this_thread::sleep_for(150ms);
    stop = true;
  });
  const auto start = std::chrono::steady_clock::now();
  const FetchResult r = client.fetch(0, 0, stop);
  stopper.join();
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.error.rfind("stopped during backoff", 0) == 0);
  CHECK(std::chrono::steady_clock::now() - start < 5s);
}

TEST_CASE("keep-alive reuses one connection for successive fetches", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  for (int i = 0; i < 3; ++i) {
    const FetchResult r = client.fetch(0, 0, stop);
    CHECK(r.kind == FetchResult::Kind::Value);
  }
  // Every new connection arrives from a new ephemeral port.
  CHECK(srv.distinctRemotePorts() == 1);
}

TEST_CASE("the server receives the decoded BBOX, LAYERS, I and J", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(2, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  // South, west, north, east of the example box with nine decimals, as the server decodes them.
  const std::map<std::string, std::string> params = srv.latestParams();
  CHECK(params.at("BBOX") == "-34.216344700,18.418579100,-34.184541800,18.462524400");
  CHECK(params.at("LAYERS") == "Elevation_CCT_GLM2019_5m_DEM");
  CHECK(params.at("I") == "2");
  CHECK(params.at("J") == "0");
}
