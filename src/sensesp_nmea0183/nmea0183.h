#ifndef SENSESP_NMEA0183_NMEA0183_H_
#define SENSESP_NMEA0183_NMEA0183_H_

#include "sensesp/sensors/sensor.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/stream_producer.h"
#include "sensesp/system/task_queue_producer.h"
#include "sensesp/transforms/filter.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace sensesp::nmea0183 {

void ReportFailure(bool ok, const char* sentence);

/// Maximum length of a single NMEA sentence. The standard defined
/// maximum is 82, but let's give it a bit of margin.
constexpr int kNMEA0183InputBufferLength = 164;
/// Maximum number of comma-separated fields in one NMEA sentence.
constexpr int kNMEA0183MaxFields = 25;

class SentenceParser;

int CalculateChecksum(const char* buffer, char seed = 0);
void AddChecksum(String& sentence);

/**
 * @brief NMEA 0183 parser class.
 *
 **/
class NMEA0183Parser : public ValueConsumer<String> {
 public:
  NMEA0183Parser() : ValueConsumer<String>() {}

  void register_sentence_parser(SentenceParser* parser);
  virtual void set(const String& line) override;

 protected:
  // offset for each sentence field in the buffer
  int field_offsets[kNMEA0183MaxFields];
  void parse_sentence(const String& sentence);
  std::vector<SentenceParser*> sentence_parsers;
};

/**
 * @brief NMEA 0183 I/O task class.
 *
 * This class is responsible for writing and reading NMEA 0183 sentences from
 * a stream and parsing the read sentences. A task is created to run the
 * NMEA 0183 I/O operations. Parser output should be connected to consumers
 * using TaskQueueProducers.
 *
 */
class NMEA0183IOTask : public ValueConsumer<String> {
 public:
  NMEA0183IOTask(Stream* stream) : stream_(stream) {
    // Event loop for the task.
    task_event_loop_ = std::make_shared<reactesp::EventLoop>();

    // Consume strings and output them on the stream.
    task_input_producer_ =
        std::make_shared<TaskQueueProducer<String>>("", task_event_loop_, 10);
    task_input_producer_->connect_to(std::make_shared<LambdaConsumer<String>>(
        [this](const String& line) { stream_->println(line); }));

    // Produce strings from the stream.
    line_producer_ =
        std::make_shared<StreamLineProducer>(stream_, task_event_loop_);

    sentence_filter_ = std::make_shared<Filter<String>>([](const String& line) {
      return line.startsWith("!") || line.startsWith("$");
    });

    line_producer_->connect_to(sentence_filter_)->connect_to(&parser_);

    // Start the task after the event loop is running.
    event_loop()->onDelay(0, [this]() {
      xTaskCreatePinnedToCore(NMEA0183IOTask::start, "NMEA0183Task", 4096, this,
                              1, &nmea_task_, 0);
    });
  }

  NMEA0183Parser parser_;

  virtual void set(const String& line) override {
    task_input_producer_->set(line);
  }

 protected:
  Stream* stream_;
  TaskHandle_t nmea_task_;
  std::shared_ptr<reactesp::EventLoop> task_event_loop_;
  std::shared_ptr<TaskQueueProducer<String>> task_input_producer_;
  std::shared_ptr<StreamLineProducer>
      line_producer_;  // Raw lines produced by the stream
  std::shared_ptr<Filter<String>> sentence_filter_;

  static void start(void* this_task) {
    auto task = static_cast<NMEA0183IOTask*>(this_task);
    task->run();
  }

  void run() {
    while (true) {
      task_event_loop_->tick();
      // Reset watchdog
      vTaskDelay(1);
    }
  }
};

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_NMEA0183_H_
