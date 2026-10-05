#include <catch2/catch_test_macros.hpp>

#include <httplib.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/x509.h>
#include <openssl/x509v3.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ExampleConfig.h"
#include "TempDir.h"
#include "demdl/ElevationClient.h"

using namespace std::chrono_literals;
using demdl::Config;
using demdl::ElevationClient;
using demdl::FetchResult;
using demdl::RetryPolicy;
using demdl::describeUnparseableBody;

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

// A self-signed EC P-256 certificate for 127.0.0.1, made at test time so that no key is committed.
// It is valid from an hour ago to a day ahead and carries subjectAltName IP:127.0.0.1 and
// basicConstraints CA:TRUE, so that as a CA bundle it is its own trust anchor.
struct LoopbackCertificate {
  std::unique_ptr<EVP_PKEY, decltype(&EVP_PKEY_free)> key{EVP_PKEY_Q_keygen(nullptr, nullptr, "EC", "P-256"),
                                                          &EVP_PKEY_free};
  std::unique_ptr<X509, decltype(&X509_free)> cert{X509_new(), &X509_free};

  LoopbackCertificate() {
    REQUIRE(key != nullptr);
    REQUIRE(cert != nullptr);
    X509* const x = cert.get();
    X509_NAME* const name = X509_get_subject_name(x);
    const auto* const commonName = reinterpret_cast<const unsigned char*>("127.0.0.1");
    REQUIRE(X509_set_version(x, X509_VERSION_3) == 1);
    REQUIRE(ASN1_INTEGER_set(X509_get_serialNumber(x), 1) == 1);
    REQUIRE(X509_gmtime_adj(X509_getm_notBefore(x), -60 * 60) != nullptr);
    REQUIRE(X509_gmtime_adj(X509_getm_notAfter(x), 24 * 60 * 60) != nullptr);
    REQUIRE(X509_set_pubkey(x, key.get()) == 1);
    REQUIRE(X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC, commonName, -1, -1, 0) == 1);
    REQUIRE(X509_set_issuer_name(x, name) == 1);
    X509V3_CTX ctx{};
    X509V3_set_ctx(&ctx, x, x, nullptr, nullptr, 0);
    for (const auto& [nid, value] : {std::pair{NID_subject_alt_name, "IP:127.0.0.1"},
                                     std::pair{NID_basic_constraints, "critical,CA:TRUE"}}) {
      X509_EXTENSION* const extension = X509V3_EXT_conf_nid(nullptr, &ctx, nid, value);
      REQUIRE(extension != nullptr);
      const int added = X509_add_ext(x, extension, -1);
      X509_EXTENSION_free(extension);
      REQUIRE(added == 1);
    }
    REQUIRE(X509_sign(x, key.get(), EVP_sha256()) > 0);
  }

  // Writes the certificate as PEM, for use as a CA bundle.
  void writePem(const std::filesystem::path& path) const {
    std::FILE* const file = std::fopen(path.c_str(), "w");
    REQUIRE(file != nullptr);
    const int written = PEM_write_X509(file, cert.get());
    REQUIRE(std::fclose(file) == 0);
    REQUIRE(written == 1);
  }
};

// An httplib::SSLServer presenting the certificate. httplib 0.46 has no constructor that takes an
// X509* and an EVP_PKEY*, so the context callback installs them; SSL_CTX_use_certificate and
// SSL_CTX_use_PrivateKey take references of their own.
std::unique_ptr<httplib::Server> tlsServer(const LoopbackCertificate& certificate) {
  return std::make_unique<httplib::SSLServer>([&certificate](httplib::tls::ctx_t ctx) {
    auto* const ssl = static_cast<SSL_CTX*>(ctx);
    return SSL_CTX_use_certificate(ssl, certificate.cert.get()) == 1 &&
           SSL_CTX_use_PrivateKey(ssl, certificate.key.get()) == 1;
  });
}

// The pixel column I of a request, or -1 unless it is a single digit from 0 to 7.
int columnOf(const httplib::Request& req) {
  const std::string i = req.get_param_value("I");
  return i.size() == 1 && i[0] >= '0' && i[0] <= '7' ? i[0] - '0' : -1;
}

constexpr const char* kServiceExceptionReport =
    R"(<?xml version="1.0"?><ServiceExceptionReport>)"
    R"(<ServiceException code="LayerNotDefined">no such layer</ServiceException>)"
    R"(</ServiceExceptionReport>)";

// The City of Cape Town server's answer for a pixel in False Bay, recorded on 2026-10-05 (the sea fixture
// in test_WmsQuery.cpp): BAND 0 reads 0, and the mask band BAND 1 is 0 because there is no elevation.
constexpr const char* kSeaResponse =
    "Results for Layer \"Elevation_CCT_GLM2019_5m_DEM\":\r\n"
    "------------------------------------------------------------------------\r\n"
    "RED: 255\r\n"
    "GREEN: 255\r\n"
    "BLUE: 255\r\n"
    "FileName: CCT_GLM2019_5m_DEM_1D_v1.tif\r\n"
    "Timestamp: 1661765030000\r\n"
    "Point: 18.4623527386719,-34.1846837772321\r\n"
    "BAND 0: 0\r\n"
    "BAND 1: 0\r\n"
    "------------------------------------------------------------------------\r\n";

// What the server decoded from one request.
struct SeenRequest {
  std::map<std::string, std::string> params;  // BBOX, LAYERS, QUERY_LAYERS, I and J
  std::vector<std::string> userAgents;         // every User-Agent header value
};

// Serves canned GetFeatureInfo bodies on loopback, over plain HTTP or, given a certificate, over TLS.
// The pixel column I selects the behaviour: 0 a band value, 1 the real server's answer at sea, 2 a 500
// and then a band value, 4 a 200 service exception report, 5 a plain body without a band, any other
// column a 404. Hits are counted per column; the remote port of every request and what the latest one
// carried are recorded.
// httplib re-encodes the query on the wire (%2C goes out as ','), so tests compare the decoded
// parameters and never the raw request target.
struct CannedServer {
  std::unique_ptr<httplib::Server> server;
  std::string scheme;
  std::thread thread;
  std::array<std::atomic<int>, 8> hits{};  // indexed by column
  std::mutex mutex;
  std::set<int> remotePorts;  // guarded by mutex; one port per connection
  SeenRequest latest;         // guarded by mutex
  int port = 0;

  CannedServer() : CannedServer(std::make_unique<httplib::Server>(), "http") {}
  explicit CannedServer(const LoopbackCertificate& certificate)
      : CannedServer(tlsServer(certificate), "https") {}
  CannedServer(std::unique_ptr<httplib::Server> s, std::string urlScheme)
      : server(std::move(s)), scheme(std::move(urlScheme)) {
    REQUIRE(server->is_valid());
    server->Get("/wms", [this](const httplib::Request& req, httplib::Response& res) { respond(req, res); });
    port = server->bind_to_any_port("127.0.0.1");
    REQUIRE(port > 0);
    thread = std::thread([this] { server->listen_after_bind(); });
    server->wait_until_ready();
  }
  ~CannedServer() {
    server->stop();
    thread.join();
  }

  void respond(const httplib::Request& req, httplib::Response& res) {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      remotePorts.insert(req.remote_port);
      latest = SeenRequest{};
      for (const char* key : {"BBOX", "LAYERS", "QUERY_LAYERS", "I", "J"}) {
        latest.params[key] = req.get_param_value(key);
      }
      for (std::size_t i = 0; i < req.get_header_value_count("User-Agent"); ++i) {
        latest.userAgents.push_back(req.get_header_value("User-Agent", "", i));
      }
    }
    const int column = columnOf(req);
    const int hit = column < 0 ? 0 : ++hits[static_cast<std::size_t>(column)];  // 1 on the first
    if (column == 0) {
      res.set_content("GetFeatureInfo results:\nBAND 0: 12.5\n", "text/plain");
    } else if (column == 1) {
      res.set_content(kSeaResponse, "text/plain");
    } else if (column == 2) {
      if (hit == 1) {
        res.status = 500;
      } else {
        res.set_content("BAND 0: 7\n", "text/plain");
      }
    } else if (column == 4) {
      res.set_content(kServiceExceptionReport, "application/vnd.ogc.se_xml");
    } else if (column == 5) {
      res.set_content("GetFeatureInfo results:\n", "text/plain");
    } else {
      res.status = 404;
    }
  }

  std::string url() const { return scheme + "://127.0.0.1:" + std::to_string(port) + "/wms"; }
  std::size_t distinctRemotePorts() {
    const std::lock_guard<std::mutex> lock(mutex);
    return remotePorts.size();
  }
  SeenRequest latestRequest() {
    const std::lock_guard<std::mutex> lock(mutex);
    return latest;
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

TEST_CASE("a 200 whose mask band marks no data is NoData", "[elevationclient]") {
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
  CHECK(srv.hits[2].load() == 2);
}

TEST_CASE("a status that never recovers fails after the retries", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(3, 0, stop);  // 404 every time
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error == "http 404");
  CHECK(srv.hits[3].load() == 3);
}

TEST_CASE("a 200 service exception is retried and then fails", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(4, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error == "unparseable body");
  CHECK(srv.hits[4].load() == 3);
}

TEST_CASE("a plain 200 without a band is retried and then fails", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(5, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error == "unparseable body");
  CHECK(srv.hits[5].load() == 3);
}

TEST_CASE("a refused connection fails after the retries with a transport error", "[elevationclient]") {
  // On macOS unprivileged processes can bind low ports; the assumption is only that nothing listens
  // on port 1 on a developer machine, so the connection is refused at once.
  ElevationClient client(configFor("http://127.0.0.1:1/wms"), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error.rfind("transport: ", 0) == 0);
}

TEST_CASE("a stop request cuts the backoff short", "[elevationclient]") {
  CannedServer srv;
  RetryPolicy slow;
  slow.maxRetries = 5;
  slow.baseDelay = 10s;
  slow.maxDelay = 10s;
  slow.jitter = 0.0;
  ElevationClient client(configFor(srv.url()), slow);
  std::atomic<bool> stop{false};
  std::thread stopper([&stop] {
    std::this_thread::sleep_for(150ms);
    stop = true;
  });
  const auto start = std::chrono::steady_clock::now();
  const FetchResult r = client.fetch(3, 0, stop);  // a 404, then a 10 s backoff
  stopper.join();
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 1);
  CHECK(r.error == "stopped during backoff after: http 404");
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
  const SeenRequest seen = srv.latestRequest();
  CHECK(seen.params.at("BBOX") == "-34.216344700,18.418579100,-34.184541800,18.462524400");
  CHECK(seen.params.at("LAYERS") == "Elevation_CCT_GLM2019_5m_DEM");
  CHECK(seen.params.at("I") == "2");
  CHECK(seen.params.at("J") == "0");
}

TEST_CASE("a layer name with special characters arrives decoded", "[elevationclient]") {
  CannedServer srv;
  Config cfg = configFor(srv.url());
  cfg.layer = "Elevation Layer+1&2%";
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  // Locks in how httplib re-encodes the query: the name must survive it in both parameters.
  const SeenRequest seen = srv.latestRequest();
  CHECK(seen.params.at("LAYERS") == "Elevation Layer+1&2%");
  CHECK(seen.params.at("QUERY_LAYERS") == "Elevation Layer+1&2%");
}

TEST_CASE("requests identify themselves as dl_elevation", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(srv.latestRequest().userAgents == std::vector<std::string>{"dl_elevation/" DL_ELEVATION_VERSION});
}

TEST_CASE("certificate verification rejects an untrusted loopback certificate without retrying",
          "[elevationclient]") {
  const LoopbackCertificate certificate;
  CannedServer srv(certificate);
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 1);
  CHECK(r.error == "transport: " + httplib::to_string(httplib::Error::SSLServerVerification));
  CHECK(srv.hits[0].load() == 0);  // the handshake fails before any request is sent
}

TEST_CASE("insecure accepts an untrusted certificate", "[elevationclient]") {
  const LoopbackCertificate certificate;
  CannedServer srv(certificate);
  Config cfg = configFor(srv.url());
  cfg.insecure = true;
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(r.value == 12.5f);
}

TEST_CASE("a CA bundle trusts the loopback certificate", "[elevationclient]") {
  const LoopbackCertificate certificate;
  const TempDir dir;
  const std::filesystem::path bundle = dir.path / "bundle.pem";
  certificate.writePem(bundle);
  CannedServer srv(certificate);
  Config cfg = configFor(srv.url());
  cfg.caBundle = bundle.string();
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(r.value == 12.5f);
}

TEST_CASE("a missing CA bundle fails without retrying", "[elevationclient]") {
  const LoopbackCertificate certificate;
  const TempDir dir;
  CannedServer srv(certificate);
  Config cfg = configFor(srv.url());
  cfg.caBundle = (dir.path / "missing.pem").string();
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 1);
  CHECK(r.error == "transport: " + httplib::to_string(httplib::Error::SSLLoadingCerts));
  CHECK(srv.hits[0].load() == 0);  // loading the bundle fails before any request is sent
}

TEST_CASE("a CA bundle that failed to load keeps failing with the same error", "[elevationclient]") {
  const LoopbackCertificate certificate;
  const TempDir dir;
  CannedServer srv(certificate);
  Config cfg = configFor(srv.url());
  cfg.caBundle = (dir.path / "missing.pem").string();
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  // httplib loads the bundle once per client, so a second request would fail verification instead.
  for (int fetch = 0; fetch < 2; ++fetch) {
    const FetchResult r = client.fetch(0, 0, stop);
    CHECK(r.kind == FetchResult::Kind::Failed);
    CHECK(r.attempts == 1);
    CHECK(r.error == "transport: " + httplib::to_string(httplib::Error::SSLLoadingCerts));
  }
  CHECK(srv.hits[0].load() == 0);
}

TEST_CASE("a certificate that does not cover the host name fails without retrying", "[elevationclient]") {
  const LoopbackCertificate certificate;
  const TempDir dir;
  const std::filesystem::path bundle = dir.path / "bundle.pem";
  certificate.writePem(bundle);
  CannedServer srv(certificate);
  // The certificate's only subjectAltName is IP:127.0.0.1, which the name localhost does not match.
  Config cfg = configFor("https://localhost:" + std::to_string(srv.port) + "/wms");
  cfg.caBundle = bundle.string();
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 1);
  CHECK(r.error == "transport: " + httplib::to_string(httplib::Error::SSLServerHostnameVerification));
  CHECK(srv.hits[0].load() == 0);  // the host name is checked before any request is sent
}

TEST_CASE("a blank unparseable body is described by its size", "[elevationclient]") {
  CHECK(describeUnparseableBody("") == "warning: first unparseable response body was 0 bytes of whitespace");
  CHECK(describeUnparseableBody(" \t\r\n") ==
        "warning: first unparseable response body was 4 bytes of whitespace");
}

TEST_CASE("an unparseable body is shown up to 200 bytes", "[elevationclient]") {
  const std::string body = "<" + std::string(299, 'x');
  const std::string line = describeUnparseableBody(body);
  const std::string prefix = "warning: first unparseable response body (300 bytes, up to 200 shown): ";
  REQUIRE(line.rfind(prefix, 0) == 0);
  CHECK(line.substr(prefix.size()) == body.substr(0, 200));
}

TEST_CASE("an unparseable body is shown as printable ASCII on one line", "[elevationclient]") {
  // ESC, BEL, DEL, the UTF-8 encoding of the C1 control CSI (0xC2 0x9B) and a newline.
  const std::string body = std::string("<p>\x1b[31mred\a\x7f") + "\xC2\x9B" + "31m</p>\n";
  const std::string line = describeUnparseableBody(body);
  CHECK(line ==
        "warning: first unparseable response body (23 bytes, up to 200 shown): <p>.[31mred....31m</p>.");
  CHECK(line.find('\n') == std::string::npos);
}
