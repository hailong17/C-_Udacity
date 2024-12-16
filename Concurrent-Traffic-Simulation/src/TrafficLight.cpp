#include <iostream>
#include <random>
#include <chrono>
#include "TrafficLight.h"

// Implementation of class "MessageQueue"

template <typename T>
T MessageQueue<T>::receive()
{
    // Khóa mutex và đợi đến khi có tin nhắn trong hàng đợi.
    std::unique_lock<std::mutex> lock(_mutex);
    _condition.wait(lock, [this] { return !_queue.empty(); });

    // Lấy phần tử từ đầu hàng đợi (FIFO)
    T msg = std::move(_queue.front());
    _queue.pop_front();

    // Trả về tin nhắn.
    return msg;
}


template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.

    std::lock_guard<std::mutex> lock(_mutex);

    _queue.emplace_back(std::move(msg));
    _condition.notify_one();
}


// Implementation of class "TrafficLight"

TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
    while(true)
    {
        if (_messages.receive() == TrafficLightPhase::green)
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class.
    auto thread = std::thread(&TrafficLight::cycleThroughPhases, this);
    threads.emplace_back(std::move(thread));
}

void TrafficLight::cycleThroughPhases()
{
    // Tạo bộ sinh số ngẫu nhiên cho chu kỳ (4 đến 6 giây)
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist(4000, 6000); // thời gian chu kỳ (ms)

    // Thiết lập thời gian chu kỳ đầu tiên
    int cycleDuration = dist(rng);

    // Ghi lại thời điểm bắt đầu
    auto lastUpdate = std::chrono::steady_clock::now();

    while (true)
    {
        // Chờ 1ms để giảm tải CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // Tính thời gian đã trôi qua
        auto timeNow = std::chrono::steady_clock::now();
        auto timeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - lastUpdate).count();

        // Nếu đã hết chu kỳ, chuyển pha và gửi thông báo
        if (timeElapsed >= cycleDuration)
        {
            // Chuyển trạng thái đèn giao thông
            if (_currentPhase == TrafficLightPhase::red)
            {
                _currentPhase = TrafficLightPhase::green;
            }
            else
            {
                _currentPhase = TrafficLightPhase::red;
            }

            // Cập nhật thời gian và chu kỳ tiếp theo
            lastUpdate = timeNow;
            cycleDuration = dist(rng);

            // Gửi trạng thái mới qua hàng đợi
            _messages.send(std::move(_currentPhase));
        }
    }
}
