#include <math.h>

class PID
{
private:
    struct heading_param
    {
        float kp;
        float ki;
        float kd;
    }headingParams;

    struct base_param
    {
        float kp;
        float ki;
        float kd;
    }baseParams;

    struct error
    {
        float proportional;
        float integral;
        float derivative;
        float previous;
    }err;

    struct velocity
    {
        float linear;
        float angular;
        float angular_filter;
    }vel;

    struct out
    {
        float heading;
        float linear;
    }output;


    float u;

public:
    void setBaseParam(float kp_, float ki_, float kd_)
    {
        baseParams.kp = kp_;
        baseParams.ki = ki_;
        baseParams.kd = kd_;
    };

    void setHeadingParam(float kp_, float ki_, float kd_)
    {
        baseParams.kp = kp_;
        baseParams.ki = ki_;
        baseParams.kd = kd_;
    };

    float control_base(float error, float speed)
    {
        err.proportional = error;
        
        err.integral += err.proportional;

        err.derivative = (err.proportional - err.previous);

        err.previous = err.proportional;

        u = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * err.derivative;

        return fmax(-speed, fmin(u, speed));
    }

    float control_base_(float error, float speed )
    {
        // if(condition)
        // {
        err.proportional = error;
        
        if(err.proportional > 180)
        {
            err.proportional -= 360;
        }
        else if(err.proportional < -180)
        {
            err.proportional += 360;
        }
        // }

            // else{
            //     err.proportional = error;
            // }

        err.integral += err.proportional;

        err.derivative = (err.proportional - err.previous);

        err.previous = err.proportional;

        // float u = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * err.derivative;
        float uT = headingParams.kp * err.proportional + headingParams.ki * err.integral + headingParams.kd * err.derivative;
        return fmax(-speed, fmin(uT, speed));
    }

    float gerErrorP()const
    {
        return err.proportional;
    }
    float getErrorI()const
    {
        return err.integral;
    }
    float getErrorD()const
    {
        return err.derivative;
    }
    float getU()const
    {
        return u;
    }
};