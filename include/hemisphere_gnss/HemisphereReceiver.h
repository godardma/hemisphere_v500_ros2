#ifndef _DEF_HEMISPHERE_V500_HEMISPHERE_RECEIVER_H_
#define _DEF_HEMISPHERE_V500_HEMISPHERE_RECEIVER_H_

#include <iostream>
#include <vector>
#include <functional>
#include <mutex>

#include <munu_io/AsyncDevice.h>

#include <hemisphere_gnss/BinaryMsg.h>

namespace hemisphere_gnss {

class Buffer
{
    protected:

    std::vector<char> buffer_;
    size_t size_;

    public:

    Buffer(size_t bufferSize = 0) : buffer_(bufferSize), size_(0) {}
    void clear() { size_ = 0; }

    size_t size() const { return size_; }
    void resize(size_t size) { 
        if(buffer_.size() < size) {
            std::vector<char> newData(size);
            buffer_.swap(newData);
            std::memcpy(buffer_.data(), newData.data(), size_);
        }
        size_ = buffer_.size();
    }
    const char* data() const { return buffer_.data(); }
    char* data() { return buffer_.data(); }
    const std::vector<char>& buffer() const { return buffer_; }
    char operator[](size_t idx) const       { return buffer_[idx]; }
    char front() const { return buffer_[0]; }
    char back()  const { return buffer_[size_ - 1]; }

    auto begin() const { return buffer_.begin(); }
    auto end()   const { return buffer_.begin() + size_; }

    void insert(char c) {
        if(size_ >= buffer_.size()) {
            buffer_.push_back(c);
            size_ = buffer_.size();
        }
        else {
            buffer_[size_] = c;
            size_++;
        }
    }

    std::string str() const {
        std::ostringstream oss;
        for(auto c : *this) oss << c;
        return oss.str();
    }
};

Buffer& operator<<(Buffer& buf, char c) {
    buf.insert(c);
    return buf;
}

/**
 * class to handle reception of messages from a Hamisphere gnss antenna.
 */
template <typename AsyncDeviceT>
class HemisphereReceiver
{
    public:

    using Device = AsyncDeviceT;
    using NmeaCallback   = std::function<void(const std::string&)>;
    using BinaryCallback = std::function<void(const std::vector<char>&)>;

    enum ParseState {
        PARSE_UNDEFINED,
        PARSE_READ_NMEA,
        PARSE_READ_BINARY
    };

    protected:

    Device            device_;
    ParseState        state_;
    std::vector<char> chunk_;
    Buffer buffer_;
    //std::ostringstream buffer_;

    mutable std::mutex                  callbacksMutex_;
    mutable std::vector<NmeaCallback>   nmeaCallbacks_;
    mutable std::vector<BinaryCallback> binaryCallbacks_;
    
    void initiate_read();
    void consume_data(const char* data,
                      const boost::system::error_code& err,
                      size_t readCount);
    void handle_binary(const char* data,
                       const boost::system::error_code& err,
                       size_t readCount);
    void handle_binary_finish(const boost::system::error_code& err,
                              size_t readCount);
    void handle_nmea(const char* data,
                     const boost::system::error_code& err,
                     size_t readCount);

    public:

    HemisphereReceiver(boost::asio::io_service& service,
                       size_t chunkSize = 32) :
        device_(service),
        state_(PARSE_UNDEFINED),
        chunk_(chunkSize)
    {}

    template <class... Args>
    void open(Args... args);

    Device&       device()       { return device_; }
    const Device& device() const { return device_; }

    void add_nmea_callback(const NmeaCallback& callback) const;
    void add_binary_callback(const BinaryCallback& callback) const;
    void call_nmea_callbacks(const std::string& data) const;
    void call_binary_callbacks(const std::vector<char>& data) const;
};

template <typename D> template <class... Args>
void HemisphereReceiver<D>::open(Args... args)
{
    device_.open(args...);
    this->initiate_read();

    // device_.async_read_until('\n',
    //     std::bind(&HemisphereReceiver<D>::read_str_callback, this,
    //     std::placeholders::_1, std::placeholders::_2));
}

template <typename D>
void HemisphereReceiver<D>::initiate_read()
{
    device_.async_read(chunk_.size(), chunk_.data(),
                       std::bind(&HemisphereReceiver<D>::consume_data,
                                 this, chunk_.data(),
                                 std::placeholders::_1, std::placeholders::_2));  
}

template <typename D>
void HemisphereReceiver<D>::consume_data(const char* data,
                                         const boost::system::error_code& err,
                                         size_t readCount)
{
    if(err) {
        throw std::runtime_error("Got error on initiate_read");
    }

    // int start = 0;
    if(buffer_.size() == 0) {
        // discarding until $ character only if buffer was empty
        while(readCount > 0 && *data != '$') {
            readCount--;
            data++;
        }
    }
    
    // Getting enough data for BIN message header.
    while(readCount > 0 && buffer_.size() < sizeof(SBinaryMsgHeader)) {
        buffer_ << *data;
        readCount--;
        data++;
    }

    // Not enough. retrying
    if(buffer_.size() < sizeof(SBinaryMsgHeader)) {
        this->initiate_read();
        return;
    }

    // Just enough data for a BIN message header in buffer_
    if(std::string(buffer_.data(), 4) == "$BIN") {
        this->handle_binary(data, err, readCount);
    }
    else {
        // Not a BIN message, maybe ASCII NMEA sentence ?
        // Checking NMEA header
        for(int i = 0; i < 4; i++) {
            if(buffer_[i + 1] < 'A' || buffer_[i + 1] > 'Z') {
                // not a valid NMEA header. Discarding already read data and retrying.
                buffer_.clear();
                this->consume_data(data, err, readCount);
                return;
            }
        }

        // We probably have an NMEA sentence
        this->handle_nmea(data, err, readCount);
    }
}

template <typename D>
void HemisphereReceiver<D>::handle_binary(const char* data,
                                          const boost::system::error_code& err,
                                          size_t readCount)
{
    if(err) {
        throw std::runtime_error("Got handle_binary error");
    }
    auto header = reinterpret_cast<const SBinaryMsgHeader*>(buffer_.data());
    if(msg_size(*header) == 0) {
        buffer_.clear();
        this->consume_data(data, err, readCount);
        return;
    }

    for(; readCount > 0; readCount--) {
        buffer_ << *data;
        data++;
        if(buffer_.size() >= msg_size(*header)) {
            this->call_binary_callbacks(std::vector<char>(buffer_.begin(),
                                                          buffer_.end()));
            buffer_.clear();
            this->consume_data(data, err, readCount - 1);
            return;
        }
    }

    device_.async_read(chunk_.size(), chunk_.data(),
                       std::bind(&HemisphereReceiver<D>::handle_binary,
                                 this, chunk_.data(),
                                 std::placeholders::_1, std::placeholders::_2));  
    
    //size_t offset = buffer_.size();
    //buffer_.resize(msg_size(*header));
    //// could read directly into buffer instead of this.
    //device_.async_read(buffer_.size() - offset, buffer_.data() + offset,
    //                   std::bind(&HemisphereReceiver<D>::handle_binary_finish,
    //                             this, std::placeholders::_1, std::placeholders::_2));  
}

template <typename D>
void HemisphereReceiver<D>::handle_binary_finish(const boost::system::error_code& err,
                                                 size_t readCount)
{
    if(err) {
        throw std::runtime_error("Got handle_binary_finish error");
    }
    this->call_binary_callbacks(std::vector<char>(buffer_.begin(),
                                                  buffer_.end()));
    buffer_.clear();
    this->initiate_read();
}

template <typename D>
void HemisphereReceiver<D>::handle_nmea(const char* data,
                                        const boost::system::error_code& err,
                                        size_t readCount)
{
    if(err) {
        throw std::runtime_error("Got handle nmea error");
    }

    for(; readCount > 0; readCount--) {
        buffer_ << *data;
        data++;
        if(buffer_.back() == '\n') {
            // Got a full nmea message
            this->call_nmea_callbacks(buffer_.str());
            buffer_.clear();
            this->consume_data(data, err, readCount-1);
            return;
        }
    }

    device_.async_read(chunk_.size(), chunk_.data(),
                       std::bind(&HemisphereReceiver<D>::handle_nmea,
                                 this, chunk_.data(),
                                 std::placeholders::_1, std::placeholders::_2));  
}

template <typename D>
void HemisphereReceiver<D>::add_nmea_callback(const NmeaCallback& callback) const
{
    std::lock_guard<std::mutex> guard(callbacksMutex_);
    nmeaCallbacks_.push_back(callback);
}

template <typename D>
void HemisphereReceiver<D>::add_binary_callback(const BinaryCallback& callback) const
{
    std::lock_guard<std::mutex> guard(callbacksMutex_);
    binaryCallbacks_.push_back(callback);
}

template <typename D>
void HemisphereReceiver<D>::call_nmea_callbacks(const std::string& data) const
{
    std::vector<NmeaCallback> callbacks;
    {
        // Making a copy of the callbacks before calling them allows to avoid
        // dealocking if one of the callback wants to add a callback to this
        // object.
        std::lock_guard<std::mutex> guard(callbacksMutex_);
        callbacks = this->nmeaCallbacks_;
    }
    for(auto& c : callbacks) {
        c(data);
    }
}

template <typename D>
void HemisphereReceiver<D>::call_binary_callbacks(const std::vector<char>& data) const
{
    std::vector<BinaryCallback> callbacks;
    {
        // Making a copy of the callbacks before calling them allows to avoid
        // dealocking if one of the callback wants to add a callback to this
        // object.
        std::lock_guard<std::mutex> guard(callbacksMutex_);
        callbacks = this->binaryCallbacks_;
    }
    for(auto& c : callbacks) {
        c(data);
    }
}

} //namespace hemisphere_gnss

#endif //_DEF_HEMISPHERE_V500_HEMISPHERE_RECEIVER_H_
