/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gmock/gmock.h>

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/transport/paho_mqtt_client.hpp"

extern char** environ;

namespace {

constexpr int kRestartBrokerPort = 18883;
constexpr auto kBrokerOutage = std::chrono::seconds(1);
constexpr auto kDeadline = std::chrono::seconds(20);
constexpr auto kPollInterval = std::chrono::milliseconds(50);

// A broker owned by the test so it can be stopped and restarted.
class BrokerProcess
{
public:
  explicit BrokerProcess(int port) : port_(std::to_string(port)) {}

  ~BrokerProcess()
  {
    stop();
  }

  BrokerProcess(const BrokerProcess&) = delete;
  BrokerProcess& operator=(const BrokerProcess&) = delete;

  bool start()
  {
    std::string program = "mosquitto";
    std::string port_flag = "-p";
    char* argv[] = {program.data(), port_flag.data(), port_.data(), nullptr};
    return posix_spawnp(&pid_, "mosquitto", nullptr, nullptr, argv, environ) ==
           0;
  }

  void stop()
  {
    if (pid_ <= 0) return;
    kill(pid_, SIGTERM);
    waitpid(pid_, nullptr, 0);
    pid_ = 0;
  }

  std::string url() const
  {
    return "tcp://localhost:" + port_;
  }

private:
  std::string port_;
  pid_t pid_ = 0;
};

bool wait_until(const std::function<bool()>& condition)
{
  const auto deadline = std::chrono::steady_clock::now() + kDeadline;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (condition()) return true;
    std::this_thread::sleep_for(kPollInterval);
  }
  return condition();
}

bool connect_when_ready(vda5050_core::transport::PahoMqttClient& client)
{
  return wait_until([&client]() {
    client.connect();
    return client.connected();
  });
}

}  // namespace

TEST(PahoMqttClientTest, PublishSubscribe)
{
  std::string broker = "tcp://localhost:1883";
  std::string topic = "/test/integration";
  std::string payload = "hello";
  int qos = 0;

  std::atomic_int message_count{0};

  auto listener =
    vda5050_core::transport::PahoMqttClient::make_shared(broker, "listener");
  ASSERT_NO_THROW(listener->connect());
  ASSERT_TRUE(listener->connected());
  ASSERT_NO_THROW(listener->subscribe(
    topic,
    [&](const std::string& topic_, const std::string& payload_) {
      message_count++;
      ASSERT_EQ(topic, topic_);
      ASSERT_EQ(payload, payload_);
    },
    qos));

  auto talker =
    vda5050_core::transport::create_default_client_shared(broker, "talker");
  ASSERT_NO_THROW(talker->connect());
  ASSERT_NO_THROW(talker->publish(topic, payload, qos));

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  ASSERT_EQ(message_count, 1);

  ASSERT_NO_THROW(talker->disconnect());
  ASSERT_NO_THROW(listener->disconnect());
}

TEST(PahoMqttClientTest, UnsubscribeStopsMessages)
{
  std::string broker = "tcp://localhost:1883";
  std::string topic = "/test/integration/unsubscribe";
  std::string payload = "hello";
  int qos = 0;

  std::atomic_int message_count{0};

  auto listener = vda5050_core::transport::PahoMqttClient::make_unique(
    broker, "unsub_listener");
  ASSERT_NO_THROW(listener->connect());
  ASSERT_NO_THROW(listener->subscribe(
    topic,
    [&](const std::string& /*topic_*/, const std::string& /*payload_*/) {
      message_count++;
    },
    qos));

  auto talker = vda5050_core::transport::create_default_client_unique(
    broker, "unsub_talker");
  ASSERT_NO_THROW(talker->connect());

  // Publish first message and verify it is received
  ASSERT_NO_THROW(talker->publish(topic, payload, qos));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(message_count.load(), 1);

  // Unsubscribe from the topic
  ASSERT_NO_THROW(listener->unsubscribe(topic));
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Publish second message after unsubscribe
  ASSERT_NO_THROW(talker->publish(topic, payload, qos));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Message count should still be 1 (no new messages received)
  ASSERT_EQ(message_count.load(), 1);

  ASSERT_NO_THROW(talker->disconnect());
  ASSERT_NO_THROW(listener->disconnect());
}

TEST(PahoMqttClient, LastWill)
{
  std::string broker = "tcp://localhost:1883";
  std::string topic = "/test/integration/unsubscribe";
  std::string payload = "hello";
  int qos = 0;

  auto client = vda5050_core::transport::PahoMqttClient::make_unique(
    broker, "last_will_client");
  ASSERT_NO_THROW(client->set_will(topic, payload, qos));
  ASSERT_NO_THROW(client->connect());
  ASSERT_TRUE(client->connected());

  ASSERT_EQ(client->connect_options().get_will_topic(), topic);
  ASSERT_EQ(client->connect_options().get_will_message()->to_string(), payload);

  ASSERT_NO_THROW(client->disconnect());
}

TEST(PahoMqttClientTest, FailedConnectionRemainsDisconnected)
{
  // Use an invalid broker endpoint to simulate connection failure
  std::string invalid_broker = "tcp://invalid.broker.address:1883";

  auto client = vda5050_core::transport::PahoMqttClient::make_shared(
    invalid_broker, "test_failed_connection");

  // Initial state should be disconnected
  ASSERT_FALSE(client->connected());

  // Attempt connection to invalid broker (should not throw)
  ASSERT_NO_THROW(client->connect());

  // connected() should return false after failed connection
  ASSERT_FALSE(client->connected());
}

TEST(PahoMqttClientTest, ResubscribesAfterBrokerRestart)
{
  BrokerProcess broker(kRestartBrokerPort);
  if (!broker.start()) GTEST_SKIP() << "mosquitto could not be started";

  const std::string topic = "/test/integration/resubscribe";
  std::atomic_int message_count{0};

  auto listener = vda5050_core::transport::PahoMqttClient::make_unique(
    broker.url(), "resubscribe_listener");
  ASSERT_TRUE(connect_when_ready(*listener));
  listener->subscribe(
    topic, [&](const std::string&, const std::string&) { message_count++; }, 0);

  auto talker = vda5050_core::transport::PahoMqttClient::make_unique(
    broker.url(), "resubscribe_talker");
  ASSERT_TRUE(connect_when_ready(*talker));
  ASSERT_TRUE(wait_until([&]() {
    talker->publish(topic, "before", 0);
    return message_count.load() > 0;
  }));

  broker.stop();
  std::this_thread::sleep_for(kBrokerOutage);
  ASSERT_TRUE(broker.start());
  ASSERT_TRUE(wait_until([&]() { return listener->connected(); }));
  ASSERT_TRUE(wait_until([&]() { return talker->connected(); }));

  const int received_before_restart = message_count.load();
  EXPECT_TRUE(wait_until([&]() {
    talker->publish(topic, "after", 0);
    return message_count.load() > received_before_restart;
  }));

  talker->disconnect();
  listener->disconnect();
}

TEST(PahoMqttClientTest, UnsubscribeWhileDisconnectedStaysUnsubscribed)
{
  BrokerProcess broker(kRestartBrokerPort);
  if (!broker.start()) GTEST_SKIP() << "mosquitto could not be started";

  const std::string kept_topic = "/test/integration/kept";
  const std::string dropped_topic = "/test/integration/dropped";
  std::atomic_int kept_count{0};
  std::atomic_int dropped_count{0};

  auto listener = vda5050_core::transport::PahoMqttClient::make_unique(
    broker.url(), "unsubscribe_offline_listener");
  ASSERT_TRUE(connect_when_ready(*listener));
  listener->subscribe(
    kept_topic, [&](const std::string&, const std::string&) { kept_count++; },
    0);
  listener->subscribe(
    dropped_topic,
    [&](const std::string&, const std::string&) { dropped_count++; }, 0);

  broker.stop();
  ASSERT_TRUE(wait_until([&]() { return !listener->connected(); }));
  listener->unsubscribe(dropped_topic);

  std::this_thread::sleep_for(kBrokerOutage);
  ASSERT_TRUE(broker.start());
  ASSERT_TRUE(wait_until([&]() { return listener->connected(); }));

  auto talker = vda5050_core::transport::PahoMqttClient::make_unique(
    broker.url(), "unsubscribe_offline_talker");
  ASSERT_TRUE(connect_when_ready(*talker));

  // Once kept arrives, re-subscribe is done, so dropped must not have arrived.
  ASSERT_TRUE(wait_until([&]() {
    talker->publish(dropped_topic, "dropped", 0);
    talker->publish(kept_topic, "kept", 0);
    return kept_count.load() > 0;
  }));
  EXPECT_EQ(dropped_count.load(), 0);

  talker->disconnect();
  listener->disconnect();
}
