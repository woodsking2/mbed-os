#include "pwmout_instance.h"
#include "gsl/gsl"
#include "mbed_debug.h"
#include "pwmout_manager.h"
#include <tuple>
#include <cmath>
extern "C"
{
#include "default_config.h"
#include "hw_timer.h"
}
using namespace std;
using namespace gsl;
class Pwmout_instance::Impl
{
  public:
    Impl(PinName pin);
    ~Impl();
    void write(float percent);
    float read();
    void period(float seconds);
    void period_ms(int ms);
    void period_us(int us);
    void pulse_width(float seconds);
    void pulse_width_ms(int ms);
    void pulse_width_us(int us);

  private:
    static constexpr auto invalid_duty_cycle = -1.f;
    static constexpr auto invalid_pulse_width = -1.f;
    Pwmout_manager::Type m_type;
    PinName m_pin;
    float m_duty_cycle;  // 0.f-1.f, 负数表示使用 m_pulse_width
    float m_period;      // seconds
    float m_pulse_width; // seconds, 负数表示使用 m_duty_cycle
    bool m_pwm_enable;
    uint8_t m_hw_prescaler;
    HW_TIMER_CLK_SRC m_hw_source;
    uint16_t m_hw_frequency;
    uint16_t m_hw_duty_cycle;

    tuple<HW_TIMER_CLK_SRC, uint16_t, uint16_t, uint8_t> calculate_hw_param();
    HW_TIMER_ID get_hw_id();
    void set_pin_pwm();
    void set_pin_gpio(bool value);
    void release_pin();
    HW_GPIO_FUNC get_pin_func();
    void setup_pwm();
};
tuple<HW_TIMER_CLK_SRC, uint16_t, uint16_t, uint8_t> Pwmout_instance::Impl::calculate_hw_param()
{
    HW_TIMER_CLK_SRC source{};
    float frequency{};
    float duty_cycle{};
    int source_clock{};
    int prescaler{1};
    float multiple{};
    Expects(m_period <= 2.f && m_period >= 1.f / 16000000.f);
    if (m_period <= 1.f / (16 * 1024))
    {
        source = HW_TIMER_CLK_SRC_INT;
        source_clock = 32 * 1024;
        debug("lp clock\n");
    }
    else
    {
        source = HW_TIMER_CLK_SRC_EXT;
        source_clock = 32000000;
        debug("32m clock\n");
    }
    frequency = source_clock * m_period;
    if (frequency > 0x10000)
    {
        prescaler = (frequency + 0xFFFF) / 0x10000;
        frequency /= prescaler;
        debug("prescaler %d, frequency %d\n", prescaler, frequency);
    }
    if (m_duty_cycle >= 0.f)
    {
        duty_cycle = frequency * m_duty_cycle;
        debug("percent duty %d\n", duty_cycle);
    }
    else
    {
        duty_cycle = source_clock * m_pulse_width;
        debug("pulse_width duty %d\n", duty_cycle);
    }

    return make_tuple(source, round(frequency - 1), round(duty_cycle), prescaler - 1);
}
HW_TIMER_ID Pwmout_instance::Impl::get_hw_id()
{
    switch (m_type)
    {
    case Pwmout_manager::Type::timer_1:
        return HW_TIMER;
    case Pwmout_manager::Type::timer_3:
        return HW_TIMER3;
    case Pwmout_manager::Type::timer_4:
        return HW_TIMER4;
    }
    Ensures(false);
    return HW_TIMER;
}
void Pwmout_instance::Impl::set_pin_gpio(bool value)
{
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    hw_timer_disable(get_hw_id());
    hw_gpio_set_pin_function(PinName_to_port(m_pin), PinName_to_pin(m_pin), HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
    if (value)
    {
        hw_gpio_set_active(PinName_to_port(m_pin), PinName_to_pin(m_pin));
    }
    else
    {
        hw_gpio_set_inactive(PinName_to_port(m_pin), PinName_to_pin(m_pin));
    }
    hw_gpio_pad_latch_enable(PinName_to_port(m_pin), PinName_to_pin(m_pin));
    hw_gpio_pad_latch_disable(PinName_to_port(m_pin), PinName_to_pin(m_pin));
    if (m_pwm_enable)
    {
        hw_sys_pd_com_disable();
    }
    m_pwm_enable = false;
}
Pwmout_instance::Impl::Impl(PinName pin) : m_type(Pwmout_manager::get_instance().acquire(pin)), m_pin(pin), m_duty_cycle(0.f), m_period(0.001f), m_pulse_width(invalid_pulse_width), m_pwm_enable(false)
{
}
HW_GPIO_FUNC Pwmout_instance::Impl::get_pin_func()
{
    switch (m_type)
    {
    case Pwmout_manager::Type::timer_1:
        return HW_GPIO_FUNC_TIM_PWM;
    case Pwmout_manager::Type::timer_3:
        return HW_GPIO_FUNC_TIM3_PWM;
    case Pwmout_manager::Type::timer_4:
        return HW_GPIO_FUNC_TIM4_PWM;
    }
    Ensures(false);
    return HW_GPIO_FUNC_TIM_PWM;
}
void Pwmout_instance::Impl::set_pin_pwm()
{
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });

    hw_gpio_set_pin_function(PinName_to_port(m_pin), PinName_to_pin(m_pin), HW_GPIO_MODE_OUTPUT, get_pin_func());
    hw_gpio_pad_latch_enable(PinName_to_port(m_pin), PinName_to_pin(m_pin));
    hw_gpio_pad_latch_disable(PinName_to_port(m_pin), PinName_to_pin(m_pin));
}
void Pwmout_instance::Impl::release_pin()
{
    hw_gpio_set_default(m_pin);
}
Pwmout_instance::Impl::~Impl()
{
    hw_timer_disable(get_hw_id());
    Pwmout_manager::get_instance().release(m_type);
}
void Pwmout_instance::Impl::write(float percent)
{
    Expects(percent <= 1.f && percent >= 0.f);
    m_duty_cycle = percent;
    m_pulse_width = invalid_pulse_width;
    setup_pwm();
}
float Pwmout_instance::Impl::read()
{
    if (m_duty_cycle >= 0.f)
    {
        return m_duty_cycle;
    }
    return m_pulse_width / m_period;
}
void Pwmout_instance::Impl::period(float seconds)
{
    m_period = seconds;
    setup_pwm();
}
void Pwmout_instance::Impl::period_ms(int ms)
{
    m_period = ms / 1000.f;
    setup_pwm();
}
void Pwmout_instance::Impl::period_us(int us)
{
    m_period = us / 1000000.f;
    setup_pwm();
}
void Pwmout_instance::Impl::pulse_width(float seconds)
{
    Expects(seconds <= m_period);
    m_duty_cycle = invalid_duty_cycle;
    m_pulse_width = seconds;
    setup_pwm();
}
void Pwmout_instance::Impl::pulse_width_ms(int ms)
{
    auto pulse_width = ms / 1000.f;
    Expects(pulse_width <= m_period);
    m_duty_cycle = invalid_duty_cycle;
    m_pulse_width = pulse_width;
    setup_pwm();
}
void Pwmout_instance::Impl::pulse_width_us(int us)
{
    auto pulse_width = us / 1000000.f;
    Expects(pulse_width <= m_period);
    m_duty_cycle = invalid_duty_cycle;
    m_pulse_width = pulse_width;
    setup_pwm();
}

void Pwmout_instance::Impl::setup_pwm()
{
    Expects(m_pulse_width < 0.f || m_duty_cycle < 0.f);
    Expects(m_pulse_width >= 0.f || m_duty_cycle >= 0.f);
    if (m_duty_cycle == 0.f || m_pulse_width == 0.f)
    {
        debug("set pwm low\n");
        set_pin_gpio(false);
        return;
    }
    if (m_duty_cycle == 1.f || m_pulse_width == 1.f)
    {
        debug("set pwm high\n");
        set_pin_gpio(true);
        return;
    }
    debug("m_period: %f, m_duty_cycle: %f, m_pulse_width:%f\n", m_period, m_duty_cycle, m_pulse_width);
    auto [source, frequency, duty_cycle, prescaler] = calculate_hw_param();
    auto _ = finally([&]() {
        m_hw_source = source;
        m_hw_frequency = frequency;
        m_hw_duty_cycle = duty_cycle;
        m_hw_prescaler = prescaler;
        debug("hw_source: %d, hw_freq: %d hw_duty_cycle: %d prescaler:%d\n", m_hw_source, m_hw_frequency, m_hw_duty_cycle, prescaler);
    });
    if (m_pwm_enable)
    {
        debug("m_pwm_enable modify\n");
        // hw_timer_set_prescaler
        if (prescaler != m_hw_prescaler)
        {
            hw_timer_set_prescaler(get_hw_id(), prescaler);
        }
        if (source != m_hw_source)
        {
            hw_timer_set_clk(get_hw_id(), source);
        }
        if (m_hw_frequency != frequency)
        {
            hw_timer_set_pwm_freq(get_hw_id(), frequency);
        }
        if (m_hw_duty_cycle != duty_cycle)
        {
            hw_timer_set_pwm_duty_cycle(get_hw_id(), duty_cycle);
        }
        return;
    }
    debug("m_pwm setup\n");
    m_pwm_enable = true;
    hw_sys_pd_com_enable();
    set_pin_pwm();
    timer_config timer_cfg = {
        .clk_src = source,
        .prescaler = prescaler,
        .mode = HW_TIMER_MODE_TIMER,
        .timer =
            {
                .direction = HW_TIMER_DIR_UP,
                .reload_val = 0,
                .free_run = true,
            },
        .pwm =
            {
                .pin = PinName_to_pin(m_pin),
                .port = PinName_to_port(m_pin),
                .pwm_active_in_sleep = false,
                .frequency = frequency,
                .duty_cycle = duty_cycle,
            },
    };
    hw_timer_init(get_hw_id(), &timer_cfg);
    hw_timer_set_pwm_freq(get_hw_id(), frequency);
    hw_timer_set_pwm_duty_cycle(get_hw_id(), duty_cycle);
    hw_timer_enable(get_hw_id());
}
Pwmout_instance::Pwmout_instance(PinName pin) : m_impl(make_unique<Pwmout_instance::Impl>(pin))
{
}
Pwmout_instance::~Pwmout_instance() = default;
void Pwmout_instance::write(float percent)
{
    m_impl->write(percent);
}
float Pwmout_instance::read()
{
    return m_impl->read();
}
void Pwmout_instance::period(float seconds)
{
    m_impl->period(seconds);
}
void Pwmout_instance::period_ms(int ms)
{
    m_impl->period_ms(ms);
}
void Pwmout_instance::period_us(int us)
{
    m_impl->period_us(us);
}
void Pwmout_instance::pulse_width(float seconds)
{
    m_impl->pulse_width(seconds);
}
void Pwmout_instance::pulse_width_ms(int ms)
{
    m_impl->pulse_width_ms(ms);
}
void Pwmout_instance::pulse_width_us(int us)
{
    m_impl->pulse_width_us(us);
}