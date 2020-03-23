/*
Copyright (c) 2019, ROSIN-project
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef RXROS2_INCLUDE_RXROS2_H_
#define RXROS2_INCLUDE_RXROS2_H_

#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <cassert>
#include <chrono>
#include <thread>
#include <memory>
#include <rxcpp/rx.hpp>
#include <rclcpp/rclcpp.hpp>


namespace rxros2
{
    /// rxros2::Node is a rclcpp::Node that will start the run method in a new thread when instantiated.
    /**
     * rxros2::Node is a rclcpp::Node, but it cannot be instantiated directly because it is abstract class.
     * Only by creating a Sub-class of rxros2::Node and by implementing the abstract method 'run' can an
     * instance be created. The 'run' method will be started in a new thread.
     */
    class Node: public rclcpp::Node
    {
    public:
        /// Constructor of a rxros2 Node.
        /**
         * The constructor of a rxros2::Node. It initialize the super class rxlcpp::Node and
         * spawns a new thread that executes the run() method. The run method is abstract
         * and must therefore be implemented by a sub-class of rxros2::Node
         *
         * @param node_name The name of the node.
         */
        Node(const std::string& node_name): rclcpp::Node(node_name) {
            std::thread trd([this]() { this->run(); });
            trd.detach();
        };
        ~Node() = default;

        /// run is an abstract method that should be implemented by a sub-class of rxros2::Node.
        virtual void run() = 0;
    }; // end of class Node

    /// create_node is a simple wrapper function that will create a shared_ptr instance of the rclcpp::Node
    /**
     * create_node is a simple wrapper function that will create a shared_ptr instance of the rclcpp::Node.
     * The idea is to allow simple creation of a rxros2 program that are not
     *
     * @param node_name The name of the node.
     * @return shared_ptr instance of a rclcpp::Node
     */
    std::shared_ptr<rclcpp::Node> create_node(const std::string& node_name) {
        return std::make_shared<rclcpp::Node>(node_name);
    }

    /// rxros2::Exception is a helper class to simplify the creation of exceptions.
    class Exception
    {
    private:
        Exception() = default;
    
    public:
        ~Exception() = default;
    
        static auto system_error(const int errCode, const std::string& msg) {
            return std::make_exception_ptr(std::system_error(std::error_code(errCode, std::generic_category()), msg));
        }
        static auto runtime_error(const std::string& msg) {
            return std::make_exception_ptr(std::runtime_error(msg));
        }
    }; // end of class Exception


    /**
     * Observables are asynchronous message streams.
     * They are the fundamental data structure used by RxROS2.
     * As soon as we have the observables RxCpp will provide us with a
     * number of functions and operators to manipulate the streams.
     */
    namespace observable
    {
        /// from_topic creates an observable data stream from a ROS2 topic.
        /**
         * An observable data stream is created from a topic by calling the from_topic function.
         * The function takes two arguments a name of the topic and a queue size.
         * In order to use the from_topic function it is important also to specify
         * the type of the topic messages.
         *
         * @tparam T Type of topic and also the type of the elements in the returned observable.
         * @param node A pointer to a rclcpp::Node.
         * @param topic The name of the topic.
         * @param queue_size The size of the queue associated to the topic.
         * @return An observable data stream from the specified topic.
         */
        template<class T>
        static auto from_topic(rclcpp::Node* node, const std::string& topic, const uint32_t queue_size = 10) {
            static typename rclcpp::Subscription<T>::SharedPtr subscription;
            auto observable = rxcpp::observable<>::create<std::shared_ptr<T>> (
                [=](rxcpp::subscriber<std::shared_ptr<T>> subscriber) {
                    auto callback = [=](const std::shared_ptr<T> val) {
                        subscriber.on_next(val);};
                    subscription = node->create_subscription<T>(topic, queue_size, callback);
                });
            return observable;
        }

        /// from_topic creates an observable data stream from a ROS2 topic.
        /**
         * An observable data stream is created from a topic by calling the from_topic function.
         * The function takes two arguments a name of the topic and a queue size.
         * In order to use the from_topic function it is important also to specify
         * the type of the topic messages.
         *
         * @tparam T Type of topic and also the type of the elements in the returned observable.
         * @param node A shared_ptr to a rclcpp::Node.
         * @param topic The name of the topic.
         * @param queue_size The size of the queue associated to the topic.
         * @return An observable data stream from the specified topic.
         */
        template<class T>
        static auto from_topic(std::shared_ptr<rclcpp::Node> node, const std::string& topic, const uint32_t queue_size = 10) {
            return from_topic<T>(node.get(), topic, queue_size);
        }


        /// from_device creates an observable data stream from a Linux device like “/dev/input/js0”.
        /**
         * The function from_device will turn a Linux block or character device like “/dev/input/js0”
         * into an observable message stream. from_device has as such nothing to do with ROS2,
         * but it provides an interface to low-level data types that are needed in order to create
         * e.g. keyboard and joystick observables. from_device takes as argument
         * the name of the device and a type of the data that are read from the device.
         *
         * @tparam T Type of device data and also the type of the elements in the returned observable.
         * @param device_name The name of the Linux device (e.g. “/dev/input/js0”)
         * @return An observable data stream from the specified linux device.
         */
        template<class T>
        static auto from_device(const std::string& device_name)
        {
            auto observable = rxcpp::observable<>::create<T>(
                [=](rxcpp::subscriber<T> subscriber) {
                    int fd = open(device_name.c_str(), O_RDONLY | O_NONBLOCK);
                    if (fd < 0)
                        subscriber.on_error(rxros2::Exception::system_error(errno, std::string("Cannot open device ") + device_name));
                    else {
                        fd_set readfds; // initialize file descriptor set.
                        FD_ZERO(&readfds);
                        FD_SET(fd, &readfds);
                        T event{};
                        bool errReported = false;
                        while (rclcpp::ok()) {
                            int rc = select(fd + 1, &readfds, nullptr, nullptr, nullptr);  // wait for input on keyboard device
                            if (rc == -1 && errno == EINTR) { // select was interrupted. This is not an error but we exit the loop
                                subscriber.on_completed();
                                close(fd);
                                break;
                            }
                            else if (rc == -1 || rc == 0) { // select failed and we issue an error.
                                subscriber.on_error(rxros2::Exception::system_error(errno, std::string("Failed to read device ") + device_name));
                                close(fd);
                                errReported = true;
                                break;
                            }
                            else if (FD_ISSET(fd, &readfds)) {
                                ssize_t sz = read(fd, &event, sizeof(T)); // read element from device.
                                if (sz == -1) {
                                    subscriber.on_error(rxros2::Exception::system_error(errno, std::string("Failed to read device ") + device_name));
                                    close(fd);
                                    errReported = true;
                                    break;
                                }
                                else if (sz == sizeof(T)) {
                                    subscriber.on_next(event); // populate the event on the
                                }
                            }
                        }
                        if (!errReported) {
                            subscriber.on_completed();
                        }
                    }
                });
            return observable;
        }
    } // end of namespace observable


    /**
     * One of the primary advantages of stream oriented processing is that we can apply
     * functional programming primitives on them. RxCpp operators are nothing but filters,
     * transformations, aggregations and reductions of the observable message streams
     * we created above.
     */
    namespace operators
    {
        /// sample_with_frequency will at regular intervals emit the last element of the observable message stream it was applied on.
        /**
         * The operator sample_with_frequency will at regular intervals emit the last element or message
         * of the observable message stream it was applied on - that is independent of whether it has
         * changed or not. This means that the observable message stream produced by sample_with_frequency
         * may contain duplicated messages if the frequency is too high and it may miss messages in case
         * the frequency is too low. This is the preferred way in ROS2 to publish messages on a topic and
         * therefore a needed operation.
         *
         * @param frequency The frequency in mHZ. A frequency of 1000 (mHZ) will emit a element every second.
         * @param cn Coordination A coordination on which sample_with_frequency will be synchronized.
         * @return a observable data stream where the data elements are distributed with a specified frequency.
         */
        template<class Coordination>
        static auto sample_with_frequency(const double frequency, Coordination cn) {
            return [=](auto&& source) {
                const std::chrono::milliseconds durationInMs(static_cast<long>(1000.0/frequency));
                return rxcpp::observable<>::interval(durationInMs, cn).with_latest_from(
                    [=](const auto intervalObsrv, const auto sourceObsrv) { return sourceObsrv; }, source);};
        }

        /// sample_with_frequency will at regular intervals emit the last element of the observable message stream it was applied on.
        /**
         * The operator sample_with_frequency will at regular intervals emit the last element or message
         * of the observable message stream it was applied on - that is independent of whether it has
         * changed or not. This means that the observable message stream produced by sample_with_frequency
         * may contain duplicated messages if the frequency is too high and it may miss messages in case
         * the frequency is too low. This is the preferred way in ROS2 to publish messages on a topic and
         * therefore a needed operation.
         *
         * @param frequency The frequency in HZ. A frequency of 10 (HZ) will emit a element 10 times every second.
         * @return a observable data stream where the data elements are distributed with a specified frequency.
         */
        static auto sample_with_frequency(const double frequency) {
            return [=](auto&& source) {
                const std::chrono::milliseconds durationInMs(static_cast<long>(1000.0/frequency));
                return rxcpp::observable<>::interval(durationInMs).with_latest_from(
                    [=](const auto intervalObsrv, const auto sourceObsrv) { return sourceObsrv; }, source);};
        }


        /// publish_to_topic will publish each element of an observable data stream to a ROS2 topic.
        /**
         * The publish_to_topic is a special operator as it does not modify the message steam it operates on.
         * However, it will take each message from the stream and publish it to a specific ROS2 topic.
         *
         * @tparam T Type of the elements in the observable message stream and the type of the published ROS2 messages.
         * @param node A pointer to a rclcpp::Node.
         * @param topic The name of the ROS2 topic to publish the messages to.
         * @param queue_size The size of the queue associated to ROS2 publisher.
         * @return The observable data stream it operates on, i.e. it is an identity operator.
         */
        template<class T>
        static auto publish_to_topic(rclcpp::Node* node, const std::string &topic, const uint32_t queue_size = 10) {
            return [=](auto&& source) {
                auto publisher = node->create_publisher<T>(topic, queue_size);
                source.subscribe ([=](const T& msg) {publisher->publish(msg);});
                return source;};
        }

        /// publish_to_topic will publish each element of an observable data stream to a ROS2 topic.
        /**
         * The publish_to_topic is a special operator as it does not modify the message steam it operates on.
         * However, it will take each message from the stream and publish it to a specific ROS2 topic.
         *
         * @tparam T Type of the elements in the observable message stream and the type of the published ROS2 messages.
         * @param node A shared_ptr to a rclcpp::Node.
         * @param topic The name of the ROS2 topic to publish the messages to.
         * @param queue_size The size of the queue associated to ROS2 publisher.
         * @return The observable data stream it operates on, i.e. it is an identity operator.
         */
        template<class T>
        static auto publish_to_topic(std::shared_ptr<rclcpp::Node> node, const std::string &topic, const uint32_t queue_size = 10) {
            return publish_to_topic<T>(node.get(), topic, queue_size);
        }


        /// call_service will send a request to a server node and return the responce as an observable data stream.
        /**
         * ROS2 provides a request/response model that allows messages to be send from one node (request)
         * and handled by another node (response). It is a typical client-server mechanism that can be
         * useful in distributed systems. RxROS2 only provides a means to send a request, i.e.
         * the client side. The server side will have to be created exactly the same way as it is done
         * it ROS2. To send a request the call_service operator is called. It take a node and a
         * service name as argument and a type that contains both the request and response part.
         * The observable data stream call_service operates on is the request part of the specified type
         * and it return a new observable data stream where the elements are the response part of the
         * specified type: observable<T::Request> -> call_service("service_name") -> observable<T::Response>
         *
         * @tparam T A type consisting of a request and response part.
         * @param node A pointer to a rclcpp::Node.
         * @param service_name The name of the service to be used for calculating the response.
         * @return a new observable message stream consisting of the responses from the services.
         */
        template<class T>
        static auto call_service(rclcpp::Node* node, const std::string& service_name) {
            return [=](auto&& source) {
                return rxcpp::observable<>::create<std::shared_ptr<typename T::Response>>(
                    [=](rxcpp::subscriber<std::shared_ptr<typename T::Request>> subscriber) {
                        auto client = node->create_client<T>(service_name);
                        bool service_ok = true;
                        while (service_ok && !client->wait_for_service(std::chrono::seconds(1)))
                            service_ok = rclcpp::ok();
                        if (service_ok) {
                            source.subscribe(
                                [=](const std::shared_ptr<typename T::Request> request) {
                                    auto future = client->async_send_request(request);
                                    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::executor::FutureReturnCode::SUCCESS)
                                        subscriber.on_next(future.get());
                                    else
                                        subscriber.on_error(rxros2::Exception::runtime_error(std::string("Client call failed"))); },
                                [=](const std::exception_ptr error) { subscriber.on_error(error); },
                                [=]() { subscriber.on_completed(); });
                        }
                        else
                            subscriber.on_error(rxros2::Exception::runtime_error(std::string("Client call interrupted while waiting for service")));});};
        }

        /// call_service will send a request to a server node and return the responce as an observable data stream.
        /**
        * ROS2 provides a request/response model that allows messages to be send from one node (request)
        * and handled by another node (response). It is a typical client-server mechanism that can be
        * useful in distributed systems. RxROS2 only provides a means to send a request, i.e.
        * the client side. The server side will have to be created exactly the same way as it is done
        * it ROS2. To send a request the call_service operator is called. It take a node and a
        * service name as argument and a type that contains both the request and response part.
        * The observable data stream call_service operates on is the request part of the specified type
        * and it return a new observable data stream where the elements are the response part of the
        * specified type: observable<T::Request> -> call_service("service_name") -> observable<T::Response>
        *
        * @tparam T A type consisting of a request and response part.
        * @param node A pointer to a rclcpp::Node.
        * @param service_name The name of the service to be used for calculating the response.
        * @return a new observable message stream consisting of the responses from the services.
        */
        template<class T>
        static auto call_service(std::shared_ptr<rclcpp::Node> node, const std::string& service_name) {
            return call_service<T>(node.get(), service_name);
        }
    } // end namespace operators
} // end namespace rxros2


#endif // RXROS2_INCLUDE_RXROS2_H_
