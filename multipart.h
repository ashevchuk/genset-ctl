/*
 * multipart.h — streaming multipart/form-data parser for uploads.
 *
 * Fed the request body in arbitrary chunks; reports only parts that carry a
 * filename (the file field), and skips plain form fields, the way
 * WebServer's HTTPUpload did. No ESP-IDF dependencies, so it can be tested
 * on the host.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

class MultipartHandler {
public:
    virtual ~MultipartHandler() = default;
    virtual bool on_file_start(const std::string &field, const std::string &filename) = 0;
    // Returning false aborts the parse.
    virtual bool on_file_data(const uint8_t *data, size_t len) = 0;
    virtual bool on_file_end() = 0;
};

class MultipartParser {
public:
    // False if `content_type` carries no boundary.
    bool begin(const char *content_type, MultipartHandler *handler);
    // False on malformed input or when the handler aborts.
    bool feed(const uint8_t *data, size_t len);
    // True once the closing boundary has been seen.
    bool complete() const { return state_ == State::Done; }
    // True while inside a file part (an abort here leaves a partial file).
    bool in_file() const { return in_file_; }

private:
    enum class State { Preamble, AfterBoundary, Headers, Body, Done, Error };

    bool step(bool &progressed);
    bool start_part(const std::string &headers);
    bool emit(size_t len);

    MultipartHandler *handler_ = nullptr;
    std::string first_delim_;  // "--" + boundary
    std::string delim_;        // "\r\n--" + boundary
    std::string buf_;
    State state_ = State::Error;
    bool in_file_ = false;
};
