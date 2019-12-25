#include "pwmout_api.h"
#include "gsl/gsl"
#include "pwmout_instance.h"
using namespace gsl;
/** Initialize the pwm out peripheral and configure the pin
 *
 * @param obj The pwmout object to initialize
 * @param pin The pwmout pin to initialize
 */
void pwmout_init(pwmout_t *obj, PinName pin)
{
    auto instance = new Pwmout_instance(pin);    
    Expects(instance);
    obj->instance = instance;
}
/** Deinitialize the pwmout object
 *
 * @param obj The pwmout object
 */
void pwmout_free(pwmout_t *obj)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    delete instance;
    obj->instance = 0;
}
/** Set the output duty-cycle in range <0.0f, 1.0f>
 *
 * Value 0.0f represents 0 percentage, 1.0f represents 100 percent.
 * @param obj     The pwmout object
 * @param percent The floating-point percentage number
 */
void pwmout_write(pwmout_t *obj, float percent)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    instance->write(percent);
}
/** Read the current float-point output duty-cycle
 *
 * @param obj The pwmout object
 * @return A floating-point output duty-cycle
 */
float pwmout_read(pwmout_t *obj)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    return instance->read();
}
/** Set the PWM period specified in seconds, keeping the duty cycle the same
 *
 * Periods smaller than microseconds (the lowest resolution) are set to zero.
 * @param obj     The pwmout object
 * @param seconds The floating-point seconds period
 */
void pwmout_period(pwmout_t *obj, float seconds)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    instance->period(seconds);
}
/** Set the PWM period specified in miliseconds, keeping the duty cycle the same
 *
 * @param obj The pwmout object
 * @param ms  The milisecond period
 */
void pwmout_period_ms(pwmout_t *obj, int ms)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    instance->period_ms(ms);
}
/** Set the PWM period specified in microseconds, keeping the duty cycle the same
 *
 * @param obj The pwmout object
 * @param us  The microsecond period
 */
void pwmout_period_us(pwmout_t *obj, int us)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    instance->period_us(us);
}
/** Set the PWM pulsewidth specified in seconds, keeping the period the same.
 *
 * @param obj     The pwmout object
 * @param seconds The floating-point pulsewidth in seconds
 */
void pwmout_pulsewidth(pwmout_t *obj, float seconds)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    instance->pulse_width(seconds);
}

/** Set the PWM pulsewidth specified in miliseconds, keeping the period the same.
 *
 * @param obj The pwmout object
 * @param ms  The floating-point pulsewidth in miliseconds
 */
void pwmout_pulsewidth_ms(pwmout_t *obj, int ms)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    instance->pulse_width_ms(ms);
}
/** Set the PWM pulsewidth specified in microseconds, keeping the period the same.
 *
 * @param obj The pwmout object
 * @param us  The floating-point pulsewidth in microseconds
 */
void pwmout_pulsewidth_us(pwmout_t *obj, int us)
{
    auto instance = reinterpret_cast<Pwmout_instance *>(obj->instance);
    Expects(instance);
    instance->pulse_width_us(us);
}
