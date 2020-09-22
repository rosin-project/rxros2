/*
Copyright (c) 2020, ROSIN-project

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
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
#include <rclcpp_action/rclcpp_action.hpp>


namespace rxros2
{
    /// rxros2::Node is a rclcpp::Node that contains the necessary means to create and execute a RxROS2 program.
    /**
     * rxros2::Node is a rclcpp::Node that contains the necessary means to create and execute a RxROS2 program.
     * It contains in particular a abstract method 'run' that must be implemented by a subclass of rxros2::Node.
     * The 'run' method is the rxros2::Node's main function. It should contain the RxROS2 code that is necessary
     * to implement the node. The 'run' method can be called by the subclass either by calling 'run' directly
     * from the subclass or it can be called by by the 'start' method provided by the rxros2::Node in which case
     * it will execute in a dedicated thread.
     */
    class Node: public rclcpp::Node
    {
    private:
        std::list<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
        std::list<rclcpp::PublisherBase::SharedPtr> publishers;

    public:
        /// Constructor of a rxros2 Node.
        /**
         * The constructor of a rxros2::Node.
         * It initialize the super class rxlcpp::Node by the specified name of the node.
         *
         * @param node_name The name of the node.
         */
        explicit Node(const std::string& node_name): rclcpp::Node(node_name) {}
        ~Node() = default;

        /// run is a abstract method that should be implemented by a sub-class of rxros2::Node.
        virtual void run() = 0;

        /// start calls the abstract run method in a dedicated thread.
        void start() {
            std::thread trd([this]() { this->run(); });
            trd.detach();
        }

        template<typename T,typename CBT>
        typename rclcpp::Subscription<T>::SharedPtr add_subscription(const std::string& topic_name, const rclcpp::QoS& qos, CBT&& callback) {
            auto subscription = create_subscription<T>(topic_name, qos, callback);
            subscriptions.push_back(subscription);
            return subscription;
        }

        template<typename T>
        typename rclcpp::Publisher<T>::SharedPtr add_publisher(const std::string& topic_name, const rclcpp::QoS& qos) {
            auto publisher = create_publisher<T>(topic_name, qos);
            publishers.push_back(publisher);
            return publisher;
        }
    }; // end of class Node

    /// create_node is a simple wrapper function that will create a shared_ptr instance of the rxros2::Node
    /**
     * create_node is a simple wrapper function that will create a shared_ptr instance of the rxros2:Node.
     * The idea is to allow a simple creation of a rxros2 node without having first to create a dedicated
     * subclass of rxros2::Node
     *
     * @param node_name The name of the node.
     * @return shared_ptr instance of a rxros2::Node
     */
    std::shared_ptr<Node> create_node(const std::string& node_name) {
        struct MinimalNode: public Node
        {
            explicit MinimalNode(const std::string& node_name): Node(node_name) {}
            void run() {exit(-1);}
        }; // end of class MinimalNode
        return std::make_shared<MinimalNode>(node_name);
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
         * from_topic creates an observable data stream from a ROS2 topic.
         * The function takes two arguments a name of the topic and a queue size.
         * In order to use the from_topic function it is important also to specify
         * the type of the topic messages.
         *
         * @tparam T Type of topic and also the type of the elements in the returned observable.
         * @param node A pointer to a Node.
         * @param topic The name of the topic.
         * @param qos The quality of service for the ROS2 subscription.
         * @return An observable data stream from the specified topic.
         */
        template<class T>
        static auto from_topic(Node* node, const std::string& topic_name, const rclcpp::QoS& qos = 10) {
            auto observable = rxcpp::observable<>::create<std::shared_ptr<T>> (
                [=](rxcpp::subscriber<std::shared_ptr<T>> subscriber) {
                    node->add_subscription<T>(topic_name, qos, [=](const std::shared_ptr<T> val) {subscriber.on_next(val);});
                });
            return observable;
        }

        /// from_topic creates an observable data stream from a ROS2 topic.
        /**
         * from_topic creates an observable data stream from a ROS2 topic.
         * The function takes two arguments a name of the topic and a queue size.
         * In order to use the from_topic function it is important also to specify
         * the type of the topic messages.
         *
         * @tparam T Type of topic and also the type of the elements in the returned observable.
         * @param node A shared_ptr to a Node.
         * @param topic The name of the topic.
         * @param qos The quality of service for the ROS2 subscription.
         * @return An observable data stream from the specified topic.
         */
        template<class T>
        static auto from_topic(const std::shared_ptr<Node>& node, const std::string& topic_name, const rclcpp::QoS& qos = 10) {
            return from_topic<T>(node.get(), topic_name, qos);
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
                    int fd = open(device_name.c_str(), (O_RDONLY | O_NONBLOCK));
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
     * functional programming primitives on them also known a operators in reactive programming.
     * RxCpp operators are nothing but filters, transformations, aggregations and reductions of
     * the observable message streams we created above.
     */
    namespace operators
    {
        /// sample_with_frequency will at regular intervals emit the last element of the observable message stream it was applied on.
        /**
         * sample_with_frequency will at regular intervals emit the last element or message
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
         * sample_with_frequency will at regular intervals emit the last element or message
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
         * publish_to_topic will publish each element of an observable data stream to a ROS2 topic.
         * It is a special operator as it does not modify the message steam it operates on.
         *
         * @tparam T Type of the elements in the observable message stream and the type of the published ROS2 messages.
         * @param node A pointer to a Node.
         * @param topic The name of the ROS2 topic to publish the messages to.
         * @param qos The quality of service for the ROS2 publisher.
         * @return The observable data stream it operates on, i.e. it is an identity operator.
         */
        template<class T>
        static auto publish_to_topic(Node* node, const std::string &topic_name, const rclcpp::QoS& qos = 10) {
            return [=](auto&& source) {
                auto publisher = node->add_publisher<T>(topic_name, qos);
                source.subscribe([=](const T& msg) {publisher->publish(msg);});
                return source;};
        }

        /// publish_to_topic will publish each element of an observable data stream to a ROS2 topic.
        /**
         * publish_to_topic will publish each element of an observable data stream to a ROS2 topic.
         * It is a special operator as it does not modify the message steam it operates on.
         *
         * @tparam T Type of the elements in the observable message stream and the type of the published ROS2 messages.
         * @param node A shared_ptr to a Node.
         * @param topic The name of the ROS2 topic to publish the messages to.
         * @param qos quality of service for the ROS2 publisher.
         * @return The observable data stream it operates on, i.e. it is an identity operator.
         */
        template<class T>
        static auto publish_to_topic(const std::shared_ptr<Node>& node, const std::string &topic_name, const rclcpp::QoS& qos = 10) {
            return publish_to_topic<T>(node.get(), topic_name, qos);
        }


        /// send_request will send a request to a server node and return the response as a new observable data stream.
        /**
         * ROS2 provides a request/response model that allows messages to be send from one node (request)
         * and handled by another node (response). It is a typical client-server mechanism that can be
         * useful in distributed systems. RxROS2 provides a means to send a request, i.e. the client side.
         * To send a request the send_request operator is called. It take a node and a service name as argument
         * and a type that contains both the request and response part. The observable data stream send_request
         * operates on is the request part of the specified type and it return a new observable data stream where
         * the elements are the response part of the specified type:
         * observable<T::Request> -> send_request("service_name") -> observable<T::Response>
         *
         * @tparam T A type consisting of a request and response part.
         * @param node A pointer to a Node.
         * @param service_name The name of the service to be used for calculating the response.
         * @return a new observable message stream consisting of the responses from the services.
         */
        template<class T>
        static auto send_request(Node* node, const std::string& service_name) {
            return [=](auto&& source) {
                return rxcpp::observable<>::create<std::shared_ptr<typename T::Response>>(
                    [=](rxcpp::subscriber<std::shared_ptr<typename T::Response>> subscriber) {
                        auto client = node->create_client<T>(service_name);
                        client->wait_for_service();
                        source.subscribe(
                            [=](const std::shared_ptr<typename T::Request> request) {
                                auto future = client->async_send_request(request);
                                future.wait(); // should have used rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) == rclcpp::executor::FutureReturnCode::SUCCESS
                                subscriber.on_next(future.get());});});};
        }

        /// send_request will send a request to a server node and return the response as a new observable data stream.
        /**
         * ROS2 provides a request/response model that allows messages to be send from one node (request)
         * and handled by another node (response). It is a typical client-server mechanism that can be
         * useful in distributed systems. RxROS2 provides a means to send a request, i.e. the client side.
         * To send a request the send_request operator is called. It take a node and a service name as argument
         * and a type that contains both the request and response part. The observable data stream send_request
         * operates on is the request part of the specified type and it return a new observable data stream where
         * the elements are the response part of the specified type:
         * observable<T::Request> -> send_request("service_name") -> observable<T::Response>
         *
         * @tparam T A type consisting of a request and response part.
         * @param node A shared pointer pointer to a Node.
         * @param service_name The name of the service to be used for calculating the response.
         * @return a new observable message stream consisting of the responses from the services.
         */
        template<class T>
        static auto send_request(const std::shared_ptr<Node>& node, const std::string& service_name) {
            return send_request<T>(node.get(), service_name);
        }

        /// send_goal will send an goal to an action server and return the result as a new observable data stream.
        /**
         * RxROS2 provides a means to send an action (goal) from a action client to an action server that will
         * return a result. To send a goal the send_goal operator is called. It take a node and a action
         * name as argument and a type that contains both the goal, feedback and result part of the action.
         * The observable data stream send_goal operates on is the goal part of the specified type and
         * it return a new observable data stream where the elements are the result part of the specified type:
         * observable<T::Goal> -> send_goal("action_name") -> observable<T::Result>
         *
         * @tparam T A type consisting of a goal, feedback and result part.
         * @param node A pointer to a Node.
         * @param service_name The name of the service to be used for calculating the result.
         * @return a new observable message stream consisting of the results of the goals.
         */
        template<class T>
        static auto send_goal(Node* node, const std::string& action_name) {
            return [=](auto&& source) {
                return rxcpp::observable<>::create<std::shared_ptr<typename T::Result>>(
                    [=](rxcpp::subscriber<std::shared_ptr<typename T::Result>> subscriber) {
                        auto action_client = rclcpp_action::create_client<T>(node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_logging_interface(), node->get_node_waitables_interface(), action_name);
                        action_client->wait_for_action_server();
                        source.subscribe(
                            [=](const std::shared_ptr<typename T::Goal> goal) {
                                subscriber.on_next(action_client->send_goal(goal));});});};
        }

        /// send_goal will send an goal to an action server and return the result as a new observable data stream.
        /**
         * RxROS2 provides a means to send an action (goal) from a action client to an action server that will
         * return a result. To send a goal the send_goal operator is called. It take a node and a action
         * name as argument and a type that contains both the goal, feedback and result part of the action.
         * The observable data stream send_goal operates on is the goal part of the specified type and
         * it return a new observable data stream where the elements are the result part of the specified type:
         * observable<T::Goal> -> send_goal("action_name") -> observable<T::Result>
         *
         * @tparam T A type consisting of a goal, feedback and result part.
         * @param node A shared pointer to a Node.
         * @param service_name The name of the service to be used for calculating the result.
         * @return a new observable message stream consisting of the results of the goals.
         */
        template<class T>
        static auto send_goal(const std::shared_ptr<Node>& node, const std::string& action_name) {
            return send_goal<T>(node.get(), action_name);
        }
    } // end namespace operators
} // end namespace rxros2


#endif // RXROS2_INCLUDE_RXROS2_H_
