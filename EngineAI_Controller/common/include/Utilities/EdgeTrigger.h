//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_EDGETRIGGER_H
#define ZQ_HUMANOID_EDGETRIGGER_H
template <typename T>
class EdgeTrigger
{
public:
    EdgeTrigger(T initial_state) : _state(initial_state) {}

    bool trigger(T &x)
    {
        if (_state == x)
        {
            return false;
        }
        else
        {
            _state = x;
            return true;
        }
    }

private:
    T _state;
};

#endif // ZQ_HUMANOID_EDGETRIGGER_H
