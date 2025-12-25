#pragma once

#include "uart_concepts.h"
#include <cstdint>
#include <array>
#include <span>
#include <string_view>

/**
 * @brief Driver for the C4001 millimeter-wave human presence module.
 *
 * This class implements a transport-agnostic interface to the C4001 radar
 * using a UART transport defined by the @ref UartTransport concept.
 *
 * @tparam Uart Transport type satisfying UartTransport
 */
template <UartTransport Uart>
class C4001
{
public:

    /**
     * @brief Radar operating mode.
     */
    enum class Mode : uint8_t
    {
        Presence = 0,   ///< Presence detection mode
        Speed = 1,      ///< Speed detection mode
    };

    /**
     * @brief Generic on/off switch.
     */
    enum class Switch : uint8_t
    {
        Off = 0,
        On = 1,
    };

    /**
     * @brief Radar runtime status.
     */
    struct RadarStatus
    {
        bool running;       ///< Radar is running
        Mode mode;          ///< Current operating mode
        bool initialized;   ///< Device is initialized
    };

    /**
     * @brief Single radar target data.
     */
    struct RadarTarget
    {
        uint8_t count;   ///< Number of detected targets
        float range_cm;  ///< Distance in cm
        float speed_mps; ///< Speed in m/s
        uint16_t energy; ///< Signal energy
    };

    /**
     * @brief Radar configuration parameters.
     */
    struct RadarConfig
    {
        uint16_t min_range_cm;       ///< Minimum detection range
        uint16_t max_range_cm;       ///< Maximum detection range
        uint16_t trigger_range_cm;   ///< Trigger range threshold
        uint16_t keep_timeout_s;     ///< Keep-alive timeout
        uint8_t trigger_sensitivity; ///< Trigger sensitivity (0–9)
        uint8_t keep_sensitivity;    ///< Keep sensitivity (0–9)
        uint8_t trigger_delay_cs;    ///< Trigger delay
    };

    /**
     * @brief Construct a C4001 driver.
     *
     * @param io Reference to UART transport
     */
    explicit C4001(Uart &io);

    /** @name Device control */
    ///@{

    /**
     * @brief Start the radar.
     * @return true on success
     */
    bool start();

    /**
     * @brief Stop the radar.
     * @return true on success
     */
    bool stop();

    /**
     * @brief Reset the device.
     * @return true on success
     */
    bool reset();

    /**
     * @brief Persist current configuration to flash.
     * @return true on success
     */
    bool saveConfig();

    ///@}

    /** @name Configuration */
    ///@{

    /**
     * @brief Set radar operating mode.
     *
     * @param mode Desired mode
     * @return true on success
     */
    bool setMode(Mode mode);

    /**
     * @brief Set detection sensitivity.
     *
     * @param trigger Trigger sensitivity (0–9)
     * @param keep Keep sensitivity (0–9)
     * @return true on success
     */
    bool setSensitivity(uint8_t trigger, uint8_t keep);

    /**
     * @brief Set detection range parameters.
     *
     * @param min_cm Minimum range in cm
     * @param max_cm Maximum range in cm
     * @param trig_cm Trigger range in cm
     * @return true on success
     */
    bool setDetectionRange(uint16_t min_cm,
                           uint16_t max_cm,
                           uint16_t trig_cm);

    /**
     * @brief Set Gpio pin output.
     * 
     * @param polarity
     *  - Off: GPIO is LOW when triggered
     *  - On: GPIO is HIGH when triggered
     * @return true on success
     */
    bool setGpioPolarity(Switch polarity);

    /**
     * @brief Set detection latency.
     *
     * @param trigger_cs Trigger delay in centiseconds
     * @param keep_s Keep timeout in seconds
     * @return true on success
     */
    bool setLatency(uint8_t trigger_cs, uint16_t keep_s);

    /**
     * @brief Enable or disable micro-motion detection.
     *
     * @param enable On or Off
     * @return true on success
     */
    bool setMicroMotion(Switch enable);

    ///@}

    /** @name Data access */
    ///@{

    /**
     * @brief Retrieve radar status.
     *
     * @param status Output status structure
     * @return true if status was successfully parsed
     */
    bool getStatus(RadarStatus &status);

    /**
     * @brief Check if motion is currently detected.
     *
     * @return true if motion detected
     */
    bool motionDetected();

    /**
     * @brief Read a single radar target.
     *
     * @param target Output target structure
     * @return true if target data was successfully parsed
     */
    bool readTarget(RadarTarget &target);

    ///@}

private:
    Uart &_io;

    static constexpr uint32_t CMD_TIMEOUT_MS = 1000;
    static constexpr uint16_t RX_BUFFER_SIZE = 128;
    std::array<char, RX_BUFFER_SIZE> _rxBuf{};

    /**
     * @brief Parse an integer value from a string.
     */
    template <typename T>
    static bool parse_number(std::string_view s, T &out);

    /**
     * @brief Parse a float value from a string.
     */
    static inline bool parse_float(std::string_view s, float &out);

    /**
     * @brief Send a command over UART.
     */
    inline bool sendCmd(std::string_view cmd);

    /**
     * @brief Read a newline-terminated response.
     */
    bool readLine(uint32_t timeout_ms);

    /**
     * @brief Parse radar status line.
     */
    inline bool parseStatus(std::span<const char> line, 
                            RadarStatus &status);

    /**
     * @brief Parse radar target line.
     */
    bool parseTarget(std::span<const char> line, 
                     RadarTarget &target);
};

#include "c4001.tpp"
