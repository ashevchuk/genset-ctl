#include "multipart.h"
#include <ctype.h>
#include <string.h>

#define MULTIPART_MAX_HEADER_BLOCK 4096

static bool istarts_with(const std::string &s, size_t pos, const char *prefix) {
    size_t n = strlen(prefix);
    if (pos + n > s.size()) return false;
    for (size_t i = 0; i < n; i++) {
        if (tolower((unsigned char)s[pos + i]) != tolower((unsigned char)prefix[i])) return false;
    }
    return true;
}

// Value of `key="..."` inside a header line, or empty if absent.
static std::string quoted_param(const std::string &line, const char *key) {
    std::string needle = std::string(key) + "=\"";
    size_t pos = 0;
    while ((pos = line.find(needle, pos)) != std::string::npos) {
        // Require a parameter boundary so "name" doesn't match inside "filename".
        if (pos == 0 || line[pos - 1] == ' ' || line[pos - 1] == ';') {
            size_t start = pos + needle.size();
            size_t end = line.find('"', start);
            return end == std::string::npos ? std::string() : line.substr(start, end - start);
        }
        pos += needle.size();
    }
    return std::string();
}

bool MultipartParser::begin(const char *content_type, MultipartHandler *handler) {
    handler_ = handler;
    buf_.clear();
    in_file_ = false;
    state_ = State::Error;
    if (content_type == nullptr) return false;
    std::string ct(content_type);
    size_t pos = 0;
    for (; pos < ct.size(); pos++) {
        if (istarts_with(ct, pos, "boundary=")) break;
    }
    if (pos >= ct.size()) return false;
    std::string boundary = ct.substr(pos + strlen("boundary="));
    size_t end = boundary.find(';');
    if (end != std::string::npos) boundary.resize(end);
    if (boundary.size() >= 2 && boundary.front() == '"' && boundary.back() == '"') {
        boundary = boundary.substr(1, boundary.size() - 2);
    }
    if (boundary.empty()) return false;
    first_delim_ = "--" + boundary;
    delim_ = "\r\n--" + boundary;
    state_ = State::Preamble;
    return true;
}

bool MultipartParser::emit(size_t len) {
    if (len == 0) return true;
    if (in_file_ && !handler_->on_file_data((const uint8_t *)buf_.data(), len)) return false;
    buf_.erase(0, len);
    return true;
}

bool MultipartParser::start_part(const std::string &headers) {
    std::string field, filename;
    bool has_filename = false;
    size_t start = 0;
    while (start < headers.size()) {
        size_t end = headers.find("\r\n", start);
        if (end == std::string::npos) end = headers.size();
        std::string line = headers.substr(start, end - start);
        if (istarts_with(line, 0, "Content-Disposition:")) {
            field = quoted_param(line, "name");
            has_filename = line.find("filename=\"") != std::string::npos;
            filename = quoted_param(line, "filename");
        }
        start = end + 2;
    }
    in_file_ = has_filename;
    if (in_file_) return handler_->on_file_start(field, filename);
    return true;
}

bool MultipartParser::step(bool &progressed) {
    progressed = false;
    switch (state_) {
        case State::Preamble: {
            size_t pos = buf_.find(first_delim_);
            if (pos == std::string::npos) {
                if (buf_.size() > first_delim_.size()) buf_.erase(0, buf_.size() - first_delim_.size());
                return true;
            }
            buf_.erase(0, pos + first_delim_.size());
            state_ = State::AfterBoundary;
            progressed = true;
            return true;
        }
        case State::AfterBoundary: {
            if (buf_.size() < 2) return true;
            if (buf_.compare(0, 2, "--") == 0) {
                state_ = State::Done;
            } else if (buf_.compare(0, 2, "\r\n") == 0) {
                state_ = State::Headers;
            } else {
                return false;
            }
            buf_.erase(0, 2);
            progressed = true;
            return true;
        }
        case State::Headers: {
            size_t pos = buf_.find("\r\n\r\n");
            if (pos == std::string::npos) return buf_.size() <= MULTIPART_MAX_HEADER_BLOCK;
            std::string headers = buf_.substr(0, pos);
            buf_.erase(0, pos + 4);
            if (!start_part(headers)) return false;
            state_ = State::Body;
            progressed = true;
            return true;
        }
        case State::Body: {
            size_t pos = buf_.find(delim_);
            if (pos == std::string::npos) {
                // Keep a tail that could be the start of a delimiter split
                // across chunks.
                if (buf_.size() >= delim_.size()) {
                    if (!emit(buf_.size() - (delim_.size() - 1))) return false;
                }
                return true;
            }
            if (!emit(pos)) return false;
            buf_.erase(0, delim_.size());
            if (in_file_) {
                in_file_ = false;
                if (!handler_->on_file_end()) return false;
            }
            state_ = State::AfterBoundary;
            progressed = true;
            return true;
        }
        case State::Done:
            buf_.clear();  // epilogue is ignored
            return true;
        case State::Error:
            return false;
    }
    return false;
}

bool MultipartParser::feed(const uint8_t *data, size_t len) {
    if (state_ == State::Error) return false;
    buf_.append((const char *)data, len);
    bool progressed = true;
    while (progressed) {
        if (!step(progressed)) {
            state_ = State::Error;
            return false;
        }
    }
    return true;
}
