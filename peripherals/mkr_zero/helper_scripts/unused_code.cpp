//Timer class header
/**
 * @brief Timer class for managing time-based events
 * 
 * Provides timing functionality with:
 * - Millisecond precision using Arduino millis()
 * - Auto-reset capability
 * - Pause/Resume support
 * - Overflow protection
 * - Bounds checking for duration
 * 
 * States:
 * - RUN: Timer is active and counting
 * - STOP: Timer is paused
 * - EXPIRED: Timer has reached its set duration
 */
class Timer{
    unsigned long m_setTime;
    bool autoReset;

    public:
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;
        Timer(unsigned long time, bool autoReset);
        ~Timer();
        bool startTimer();
        void endTimer();
        bool stopTimer();
        bool resumeTimer();
        void resetTimer();

        void setTimer(unsigned long time);
        void setAutoReset(bool autoReset);

        unsigned long getTimer() const;
        unsigned long getElapsedTime();
        unsigned long getRemainingTime() const;
        bool isAutoReset() const;
        TimerState timerState();
    private:
        unsigned long zTime;
        unsigned long stopTime;
        TimerState state;
        static constexpr unsigned long MIN_TIME_MS = 1;
        static constexpr unsigned long MAX_TIME_MS = ULONG_MAX;
        unsigned long timer(bool reset);
        static inline unsigned long Timer::checkTime(unsigned long time);
};

enum class TimerState{
    RUN,
    STOP,
    EXPIRED
};

enum class Task{
    GPS,
    SERVO,
    SEGLED,
    HTTP,
    WIFI,
    CAN,
    COMMS
};

struct TaskTimer{
    Timer* timer;
    Task task;
};


//Timer class member func

/**
 * @brief Construct a new Timer:: Timer object
 * @return Timer object
 */
Timer::Timer(unsigned long time, bool autoReset):
    m_setTime(checkTime(time)),
    autoReset(autoReset),
    state(TimerState::EXPIRED),
    zTime(millis()),
    stopTime(0){
    //do nothing
    }

/**
 * @brief Destroy the Timer:: Timer object
 * @return void
 */
Timer::~Timer(){
    //do nothing
}

/**
 * @brief Timer function
 * @param reset
 * @return unsigned long
 */
unsigned long Timer::timer(bool reset){
    unsigned long cTime = millis();
    if (reset){
        zTime = cTime;
        return 0;
    }
    return cTime >= zTime ? cTime - zTime : MAX_TIME_MS - zTime + cTime;
}

/**
 * @brief Start the timer
 * @return bool
 * @note If the timer is already running, it will not start the timer
 * @note If the timer is expired and auto reset is false, it will not start the timer. Use resetTimer() to start the timer again.
 */
bool Timer::startTimer(){
    if (state == TimerState::EXPIRED && !autoReset || state == TimerState::RUN){
        return false;
    }
    timer(true);
    state = TimerState::RUN;
    return true;
}

/**
 * @brief Stop the timer
 * @return void
 * @note If the timer is already stopped, it will not stop the timer
 */
void Timer::endTimer(){
    state = TimerState::EXPIRED;
    stopTime = 0;
}

/**
 * @brief Stop the timer
 * @return void
 * @note If the timer is already stopped, it will not stop the timer
 */
bool Timer::stopTimer(){
    if (state == TimerState::RUN){
        state = TimerState::STOP;
        stopTime = timer(false);
        return true;
    }
    return false;
}

/**
 * @brief Resume the timer
 * @return void
 * @note If the timer is already running, it will not resume the timer
 */
bool Timer::resumeTimer(){
    if (state == TimerState::STOP){
        state = TimerState::RUN;
        unsigned long tempTime = millis();
        zTime = tempTime >= stopTime ? tempTime - stopTime : MAX_TIME_MS - stopTime + tempTime;
        stopTime = 0;
        return true;
    }
    return false;
}

/**
 * @brief Reset the timer
 * @return void
 * @note If the timer is already running, it will reset the timer
 */
void Timer::resetTimer(){
    timer(true);
    state = TimerState::RUN;
    stopTime = 0;
}

/**
 * @brief Set the timer
 * @param time
 * @return void
 * @note If the time is less than 1ms, it will set the time to 1ms
 * @note If the time is greater than ULONG_MAX, it will set the time to ULONG_MAX
 */
void Timer::setTimer(unsigned long time){
    m_setTime = checkTime(time);
}

/**
 * @brief Set the auto reset
 * @param autoReset
 * @return void
 */
void Timer::setAutoReset(bool autoReset){
    this->autoReset = autoReset;
}

/**
 * @brief Get the timer
 * @return unsigned long
 */
unsigned long Timer::getTimer() const {
    return m_setTime;
}

/**
 * @brief Get the elapsed time
 * @return unsigned long
 */
unsigned long Timer::getElapsedTime() {
    switch (state){
        case TimerState::RUN:
            return timer(false);
        case TimerState::STOP:
            return zTime;
        case TimerState::EXPIRED:
            return m_setTime;
    }
}

/**
 * @brief Get the remaining time
 * @return unsigned long
 * @note If the timer is not running, it will return 0
 */
unsigned long Timer::getRemainingTime() const {
    switch (state){
        case TimerState::RUN:
            unsigned long elapsedTime = timer(false);
            return m_setTime >= elapsedTime ? m_setTime - elapsedTime : 0;
        case TimerState::STOP:
            return m_setTime >= stopTime ? m_setTime - stopTime : 0;
        case TimerState::EXPIRED:
            return 0;
    }
}

/**
 * @brief Get the auto reset
 * @return bool
 */
bool Timer::isAutoReset() const {
    return autoReset;
}

/**
 * @brief Check the state of the timer
 * @return TimerState enum
 * @note Run continuously to update state and timing multiple tasks
 */
TimerState Timer::timerState(){
    if (state != TimerState::RUN){
        return state;
    }
    if (timer(false) >= m_setTime){
        if (autoReset){
            timer(true);
            state = TimerState::RUN;
        } else {
            state = TimerState::EXPIRED;
        }
    }
    return state;
}

static inline unsigned long Timer::checkTime(unsigned long time){
    return time < MIN_TIME_MS ? MIN_TIME_MS : (time > MAX_TIME_MS ? MAX_TIME_MS : time);
}


