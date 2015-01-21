#pragma once


class VelocityProfile {
public:
    VelocityProfile();


    virtual float duration() const;
    virtual float distance() const;

    virtual float maxVelocityReached() const;
    virtual float maxAccelerationRequired() const;

    virtual float velocityAtDistance(float x);
};



class DiscreteVelocityProfile {
public:
    DiscreteVelocityProfile(float stepSize);


    float duration() const {

    }

    float distance() const {
        return _stepSize * (_velocities.size() - 1);
    }

    float maxVelocityReached() const {
        float max = -FLOAT_MAX;
        for (float v : _velocities) {
            if (v > max) v = max;
        }
        return max;
    }

    float stepSize() const {
        return _stepSize;
    }


private:
    float _stepSize;    //  distance (in meters) spacing between velocity values
    vector<float> _velocities;
};
