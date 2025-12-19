#pragma once

#include <charconv>
#include <cstdio>
#include <algorithm>

template <UartTransport Uart>
C4001<Uart>::C4001(Uart &io) : _io(io) {}

template <UartTransport Uart>
bool C4001<Uart>::start()
{
    return sendCmd("sensorStart") && waitForOK(CMD_TIMEOUT_MS);
}

template <UartTransport Uart>
bool C4001<Uart>::stop()
{
    return sendCmd("sensorStop") && waitForOK(CMD_TIMEOUT_MS);
}

template <UartTransport Uart>
bool C4001<Uart>::reset()
{
    return sendCmd("resetSystem") && waitForOK(CMD_TIMEOUT_MS);
}

template <UartTransport Uart>
bool C4001<Uart>::saveConfig()
{
    return sendCmd("saveConfig") && waitForOK(CMD_TIMEOUT_MS);
}

template <UartTransport Uart>
bool C4001<Uart>::setMode(Mode mode)
{
    stop();
    sendCmd(mode == Mode::Speed ? "setRunApp 1" : "setRunApp 0");
    saveConfig();
    return start();
}

template <UartTransport Uart>
bool C4001<Uart>::getStatus(RadarStatus &status)
{
    if (!readLine(300)) { return false; }
    return parseStatus(_rxBuf, status);
}

template <UartTransport Uart>
bool C4001<Uart>::motionDetected()
{
    RadarStatus s{};
    if (!getStatus(s)) { return false; }
    return s.running;
}

template <UartTransport Uart>
bool C4001<Uart>::readTarget(RadarTarget &target)
{
    if (!readLine(200)) { return false; }
    return parseTarget(_rxBuf, target);
}

template <UartTransport Uart>
bool C4001<Uart>::setSensitivity(uint8_t trigger, uint8_t keep)
{
    if (trigger > 9 || keep > 9) { return false; }
    std::array<char, 32> cmd{};
    snprintf(cmd.data(), cmd.size(), "setSensitivity %u %u", 
             trigger, keep);
    if (!stop()) { return false; }
    if (!sendCmd(cmd.data())) { return false; }
    if (!saveConfig()) { return false; }
    return start();
}

template <UartTransport Uart>
bool C4001<Uart>::setDetectionRange(uint16_t min_cm,
                                    uint16_t max_cm,
                                    uint16_t trig_cm)
{
    std::array<char, 48> cmd1{};
    std::array<char, 32> cmd2{};
    snprintf(cmd1.data(), cmd1.size(), "setRange %.1f %.1f",
             min_cm / 100.f, max_cm / 100.f);
    snprintf(cmd2.data(), cmd2.size(), "setTrigRange %.1f",
             trig_cm / 100.f);
    if (!stop()) { return false; }
    if (!sendCmd(cmd1.data())) { return false; }
    if (!sendCmd(cmd2.data())) { return false; }
    if (!saveConfig()) { return false; }
    return start();
}

template <UartTransport Uart>
bool C4001<Uart>::setLatency(uint8_t trigger_cs, uint16_t keep_s)
{
    std::array<char, 40> cmd{};
    snprintf(cmd.data(), cmd.size(), "setLatency %.2f %.1f",
             trigger_cs / 100.f, keep_s / 2.f);

    if (!stop()) { return false; }
    if (!sendCmd(cmd.data())) { return false; }
    if (!saveConfig()) { return false; }
    return start();
}

template <UartTransport Uart>
bool C4001<Uart>::setMicroMotion(Switch enable)
{
    std::array<char, 24> cmd{};
    snprintf(cmd.data(), cmd.size(), "setMicroMotion %u",
             static_cast<uint8_t>(enable));

    if (!stop()) { return false; }
    if (!sendCmd(cmd.data())) { return false; }
    if (!saveConfig()) { return false; }
    return start();
}

template <UartTransport Uart>
template <typename T>
bool C4001<Uart>::parse_number(std::string_view s, T &out)
{
    auto [ptr, ec] =
        std::from_chars(s.data(), s.data() + s.size(), out);
    return ec == std::errc{};
}

template <UartTransport Uart>
bool C4001<Uart>::parse_float(std::string_view s, float &out)
{
    auto [ptr, ec] =
        std::from_chars(s.data(), s.data() + s.size(), out);
    return ec == std::errc{};
}

template <UartTransport Uart>
bool C4001<Uart>::sendCmd(std::string_view cmd)
{
    _io.flush();
    const auto len = static_cast<int>(cmd.size());
    return _io.write(
                reinterpret_cast<const uint8_t*>(cmd.data()),
                len) == len &&
            _io.write(
                reinterpret_cast<const uint8_t*>("\n"),
                1) == 1;
}

template <UartTransport Uart>
bool C4001<Uart>::readLine(uint32_t timeout_ms)
{
    _rxBuf.fill('\0');
    size_t idx = 0;
    uint8_t c;

    while (idx < _rxBuf.size() - 1)
    {
        if (_io.read(&c, 1, timeout_ms) <= 0) { return false; }
        if (c == '\n') { return true; }
        _rxBuf[idx++] = c;
    }
    return false;
}

template <UartTransport Uart>
bool C4001<Uart>::waitForOK(uint32_t timeout_ms)
{
    if (!readLine(timeout_ms))
    {
        return false;
    }
    return std::string_view{_rxBuf.data()}.find("OK") != std::string_view::npos;
}

template <UartTransport Uart>
bool C4001<Uart>::parseStatus(std::span<const char> line, 
                              RadarStatus &status)
{
    if (line.size() < 6 ||
        std::string_view{line.data(), 6} != "$DFHPD")
    {
        return false;
    }
    status.running = true;
    status.mode = Mode::Presence;
    status.initialized = true;
    return true;
}

template <UartTransport Uart>
bool C4001<Uart>::parseTarget(std::span<const char> line,
                              RadarTarget &target)
{
    constexpr std::string_view prefix = "$DFDMD";

    // $DFDMD,n,?,range,speed,energy
    if (line.size() < prefix.size() ||
        std::string_view(line.data(), prefix.size()) != prefix)
    {
        return false;
    }

    std::string_view view{line.data(), line.size()};
    size_t pos = 0;
    auto next = [&]() -> std::string_view
    {
        if (pos >= view.size()) { return {}; }
        const auto end = view.find(',', pos);
        auto token = view.substr(pos, end - pos);
        pos = (end == std::string_view::npos) ? view.size() : end + 1;
        return token;
    };

    // "$DFDMD"
    auto tok = next();
    if (tok != prefix) { return false; }

    // Target count
    tok = next();
    if (tok.empty() || !parse_number(tok, target.count)) { return false; }

    // Junk
    tok = next();
    if (tok.empty()) { return false; }

    // Range (m -> cm)
    float range_m{};
    tok = next();
    if (tok.empty() || !parse_float(tok, range_m)) { return false; }
    target.range_cm = range_m * 100.0f;

    // Speed (m/s)
    tok = next();
    if (tok.empty() || !parse_float(tok, target.speed_mps)) { return false; }

    // Energy
    tok = next();
    if (tok.empty() || !parse_number(tok, target.energy)) { return false; }

    return true;
}
