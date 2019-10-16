#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

//#define ENABLE_ABS_TO_CPR_MAP 
//#define kABS_TO_CPR_MAP_SIZE 4000

class Encoder {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_UNSTABLE_GAIN = 0x01,
        ERROR_CPR_OUT_OF_RANGE = 0x02,
        ERROR_NO_RESPONSE = 0x04,
        ERROR_UNSUPPORTED_ENCODER_MODE = 0x08,
        ERROR_ILLEGAL_HALL_STATE = 0x10,
        ERROR_INDEX_NOT_FOUND_YET = 0x20,
        ERROR_ABS_SPI_TIMEOUT = 0x40,
        ERROR_ABS_SPI_COM_FAIL = 0x80,
        ERROR_ABS_SPI_NOT_READY = 0x100,
    };

    enum Mode_t : uint32_t
    {
        MODE_INCREMENTAL             = ( 1 << 0 ),
        MODE_HALL                    = ( 1 << 1 ),
        MODE_SINCOS                  = ( 1 << 2 ),
        MODE_SPI_ABS_AMS_INCREMENTAL = ( 1 << 3 ), // Not true ABS. ABS only used at startup (for when its to nosiy for constant use)
        MODE_SPI_ABS_CUI             = ( 1 << 4 ),
        MODE_SPI_ABS_AMS             = ( 1 << 5 ),
    };
    const uint32_t MODE_FLAG_SPI = ( MODE_SPI_ABS_AMS_INCREMENTAL | MODE_SPI_ABS_CUI | MODE_SPI_ABS_AMS );
    const uint32_t MODE_FLAG_ABS = ( MODE_SPI_ABS_CUI | MODE_SPI_ABS_AMS );
    
    struct Config_t {
        Encoder::Mode_t mode = Encoder::MODE_INCREMENTAL;
        bool use_index = false;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        bool zero_count_on_find_idx = true;
        int32_t abs_cpr = (2048 * 8); // Default resolution of AMS-5047 encoder
        int32_t cpr = (2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        int32_t offset = 0;        // Offset between encoder count and rotor electrical phase
        float offset_float = 0.0f; // Sub-count phase alignment offset
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 1000.0f;
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool idx_search_unidirectional = false; // Only allow index search in known direction
        bool ignore_illegal_hall_state = false; // dont error on bad states like 000 or 111
        uint16_t abs_spi_cs_gpio_pin = 0;

#ifdef ENABLE_ABS_TO_CPR_MAP
        int32_t dbg_mapping_idx = 0;
        int32_t dbg_mapping_sample = 0;
        bool use_mapping = false;
        int16_t abs_to_cpr_map[kABS_TO_CPR_MAP_SIZE];
#endif
    };

    Encoder(const EncoderHardwareConfig_t& hw_config,
                     Config_t& config);
    
    void setup();
    void set_error(Error_t error);
    bool do_checks();

    void enc_index_cb();
    void set_idx_subscribe(bool override_enable = false);
    void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    bool calib_enc_offset(float voltage_magnitude);

    void clear_mapping();
    void update_mapping(int32_t index);
    void enable_mapping();

    bool run_index_search();
    bool run_direction_find();
    bool run_abs_to_cpr();
    bool run_offset_calibration();
    void sample_now();
    bool update();



    const EncoderHardwareConfig_t& hw_config_;
    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error_t error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    float phase_ = 0.0f;    // [count]
    float pos_estimate_ = 0.0f;  // [count]
    float pos_cpr_ = 0.0f;  // [count]
    float vel_estimate_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;

    int32_t enc_sampler_hz = current_meas_hz;
    float calib_scan_response_ = 0.0f; // debug report from offset calib

    int16_t tim_cnt_sample_ = 0; // 
    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC

    bool abs_spi_init();
    bool abs_spi_start_transaction();
    void abs_spi_cb();
    void abs_spi_cs_pin_init();
    uint16_t abs_spi_dma_tx_[2] = {0xFFFF, 0x0000};
    uint16_t abs_spi_dma_rx_[2];
    bool abs_spi_pos_updated_ = false;
    GPIO_TypeDef* abs_spi_cs_port_;
    uint16_t abs_spi_cs_pin_;
    uint32_t abs_spi_cr1;
    uint32_t abs_spi_cr2;
	
    float sincos_sample_s_ = 0.0f;
    float sincos_sample_c_ = 0.0f;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("error", &error_),
            make_protocol_property("is_ready", &is_ready_),
            make_protocol_property("index_found", const_cast<bool*>(&index_found_)),
            make_protocol_property("shadow_count", &shadow_count_),
            make_protocol_property("count_in_cpr", &count_in_cpr_),
            make_protocol_property("interpolation", &interpolation_),
            make_protocol_ro_property("phase", &phase_),
            make_protocol_property("pos_estimate", &pos_estimate_),
            make_protocol_property("pos_cpr", &pos_cpr_),
            make_protocol_ro_property("hall_state", &hall_state_),
            make_protocol_property("vel_estimate", &vel_estimate_),
            make_protocol_property("pos_abs", &pos_abs_),
            make_protocol_ro_property("calib_scan_response", &calib_scan_response_),
            make_protocol_ro_property("enc_sampler_hz", &enc_sampler_hz),
            make_protocol_ro_property("pll_kp", &pll_kp_),
            make_protocol_ro_property("pll_ki", &pll_ki_),
            make_protocol_object("config",
                make_protocol_property("mode", &config_.mode,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->abs_spi_init(); }, this),
                make_protocol_property("use_index", &config_.use_index,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->set_idx_subscribe(); }, this),
                make_protocol_property("abs_spi_cs_gpio_pin", &config_.abs_spi_cs_gpio_pin,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->abs_spi_cs_pin_init(); }, this),
                make_protocol_property("find_idx_on_lockin_only", &config_.find_idx_on_lockin_only,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->set_idx_subscribe(); }, this),
                make_protocol_property("pre_calibrated", &config_.pre_calibrated,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->check_pre_calibrated(); }, this),
                make_protocol_property("zero_count_on_find_idx", &config_.zero_count_on_find_idx),
                make_protocol_property("abs_cpr", &config_.abs_cpr),
                make_protocol_property("cpr", &config_.cpr),
                make_protocol_property("offset", &config_.offset),
                make_protocol_property("offset_float", &config_.offset_float),
                make_protocol_property("enable_phase_interpolation", &config_.enable_phase_interpolation),
                make_protocol_property("bandwidth", &config_.bandwidth,
                    [](void* ctx) { static_cast<Encoder*>(ctx)->update_pll_gains(); }, this),
                make_protocol_property("calib_range", &config_.calib_range),
                make_protocol_property("calib_scan_distance", &config_.calib_scan_distance),
                make_protocol_property("calib_scan_omega", &config_.calib_scan_omega),
                make_protocol_property("idx_search_unidirectional", &config_.idx_search_unidirectional),
                make_protocol_property("ignore_illegal_hall_state", &config_.ignore_illegal_hall_state)
#ifdef ENABLE_ABS_TO_CPR_MAP
                make_protocol_property("dbg_mapping_idx", &config_.dbg_mapping_idx),
                make_protocol_property("dbg_mapping_sample", &config_.dbg_mapping_sample),
                make_protocol_property("use_mapping",  &config_.use_mapping)
#endif                
            ),
            make_protocol_function("set_linear_count", *this, &Encoder::set_linear_count, "count")
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Encoder::Error_t)

#endif // __ENCODER_HPP
