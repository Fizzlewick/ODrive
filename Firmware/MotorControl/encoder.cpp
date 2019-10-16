
#include "odrive_main.h"


Encoder::Encoder(const EncoderHardwareConfig_t& hw_config,
                Config_t& config) :
        hw_config_(hw_config),
        config_(config)
{
    update_pll_gains();

    if (config.pre_calibrated && (config.mode == Encoder::MODE_HALL || config.mode == Encoder::MODE_SINCOS)) {
        is_ready_ = true;
    }
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}

void Encoder::setup() {
    HAL_TIM_Encoder_Start(hw_config_.timer, TIM_CHANNEL_ALL);
    set_idx_subscribe();

    if(config_.mode & MODE_FLAG_SPI){
        abs_spi_cs_pin_init();
        abs_spi_init();
    }
}

void Encoder::set_error(Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}

//--------------------
// Hardware Dependent
//--------------------

// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// (maybe by attaching the interrupt on start search, synergistic with following)
void Encoder::enc_index_cb() {
    if (config_.use_index) {
        set_circular_count(0, false);
        if (config_.zero_count_on_find_idx)
            set_linear_count(0); // Avoid position control transient after search
        if (config_.pre_calibrated) {
            is_ready_ = true;
        } else {
            // We can't use the update_offset facility in set_circular_count because
            // we also set the linear count before there is a chance to update. Therefore:
            // Invalidate offset calibration that may have happened before idx search
            is_ready_ = false;
        }
        index_found_ = true;
    }

    // Disable interrupt
    GPIO_unsubscribe(hw_config_.index_port, hw_config_.index_pin);
}

void Encoder::set_idx_subscribe(bool override_enable) {
    if (config_.use_index && (override_enable || !config_.find_idx_on_lockin_only)) {
        GPIO_subscribe(hw_config_.index_port, hw_config_.index_pin, GPIO_PULLDOWN,
                enc_index_cb_wrapper, this);
    } else if (!config_.use_index || config_.find_idx_on_lockin_only) {
        GPIO_unsubscribe(hw_config_.index_port, hw_config_.index_pin);
    }
}

void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

void Encoder::check_pre_calibrated() {
    if (!is_ready_)
        config_.pre_calibrated = false;
    if (config_.mode == MODE_INCREMENTAL && !index_found_)
        config_.pre_calibrated = false;
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    pos_estimate_ = (float)count;
    tim_cnt_sample_ = count;

    //Write hardware last
    hw_config_.timer->Instance->CNT = count;

    cpu_exit_critical(prim);
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count, bool update_offset) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    if (update_offset) {
        config_.offset += count - count_in_cpr_;
        config_.offset = mod(config_.offset, config_.cpr);
    }

    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_ = (float)count_in_cpr_;

    cpu_exit_critical(prim);
}

bool Encoder::run_index_search() {
    config_.use_index = true;
    index_found_ = false;
    if (!config_.idx_search_unidirectional && axis_->motor_.config_.direction == 0) {
        axis_->motor_.config_.direction = 1;
    }
    set_idx_subscribe();

    bool status = axis_->run_lockin_spin(axis_->config_.calibration_lockin);
    return status;
}

bool Encoder::run_direction_find() {
    int32_t init_enc_val = shadow_count_;
    bool orig_finish_on_distance = axis_->config_.calibration_lockin.finish_on_distance;
    axis_->config_.calibration_lockin.finish_on_distance = true;
    axis_->motor_.config_.direction = 1; // Must test spin forwards for direction detect logic
    bool status = axis_->run_lockin_spin(axis_->config_.calibration_lockin);
    axis_->config_.calibration_lockin.finish_on_distance = orig_finish_on_distance;

    if (status) {
        // Check response and direction
        if (shadow_count_ > init_enc_val + 8) {
            // motor same dir as encoder
            axis_->motor_.config_.direction = 1;
        } else if (shadow_count_ < init_enc_val - 8) {
            // motor opposite dir as encoder
            axis_->motor_.config_.direction = -1;
        } else {
            axis_->motor_.config_.direction = 0;
        }
    }

    return status;
}

int32_t ModDist( int32_t a, int32_t b, int32_t val_max )
{
    int32_t half_max = val_max / 2;
    a = a - half_max;
    b = b - half_max;

    int32_t diff = a - b;

    if ( diff < half_max )
        diff = diff + val_max;
    if ( diff > half_max )
        diff = diff - val_max;

    return diff;
}

void Encoder::clear_mapping()
{
#ifdef ENABLE_ABS_TO_CPR_MAP    
    config_.use_mapping = false;
    memset( config_.abs_to_cpr_map, 0, sizeof(int16_t) * kABS_TO_CPR_MAP_SIZE);
#endif
}

void Encoder::update_mapping( int32_t index )
{
#ifdef ENABLE_ABS_TO_CPR_MAP    
    if ( index >= 0 && index < kABS_TO_CPR_MAP_SIZE )
    {
        float abs_to_cpr_ratio = ( (float)config_.cpr / (float)config_.abs_cpr );

        float pos_abs_float    = ( pos_abs_ * abs_to_cpr_ratio ); 
              pos_abs_float    = fmodf_pos( pos_abs_float, config_.cpr );
        
        int32_t pos_abs_i      = (int32_t)( pos_abs_float + 0.5f );              
                pos_abs_i      = mod( pos_abs_i, config_.cpr );

        int16_t abs_to_cpr = (int16_t)( count_in_cpr_ - pos_abs_i );

        config_.abs_to_cpr_map[ index ] = abs_to_cpr;
    }
#endif    
}

void Encoder::enable_mapping()
{
#ifdef ENABLE_ABS_TO_CPR_MAP    
    config_.use_mapping = true;
#endif    
}

bool Encoder::run_abs_to_cpr() 
{
    const int32_t max_counts  = 1024;

    int32_t first_sample = 0;
    int32_t diff_accum   = 0;
    int32_t sample_count = 0;

    axis_->run_control_loop([&](){

        int32_t sample = pos_abs_;        
        if (sample_count == 0)
        {
            first_sample = sample;
        }
        else
        {
            diff_accum += ModDist( sample, first_sample, config_.abs_cpr );
        }

        return  ( sample_count++ < max_counts ); 
    });

    // Map from ABS_CPR to normal CPR
    int32_t pos_abs_filtered_i = 0;
    {
        float diff_avg   = ( (float)diff_accum / (float)sample_count );
        float avg_sample = first_sample + diff_avg;

        if ( avg_sample < 0 )
            avg_sample += config_.abs_cpr;
        if ( avg_sample >= config_.abs_cpr )
            avg_sample -= config_.abs_cpr;

        float abs_to_cpr_ratio = ( (float)config_.cpr / (float)config_.abs_cpr );
        
        float pos_abs_filtered_float = ( avg_sample * abs_to_cpr_ratio ); 
              pos_abs_filtered_float = fmodf_pos( pos_abs_filtered_float, config_.cpr );
        
        pos_abs_filtered_i = (int32_t)( pos_abs_filtered_float + 0.5f );              
        pos_abs_filtered_i = mod( pos_abs_filtered_i, config_.cpr );
    }

    int32_t abs_to_cpr = 0;
#ifdef ENABLE_ABS_TO_CPR_MAP    
    if ( ( config_.use_mapping ) && ( pos_abs_filtered_i >= 0 && pos_abs_filtered_i < kABS_TO_CPR_MAP_SIZE ) )
    {
        abs_to_cpr = config_.abs_to_cpr_map[ pos_abs_filtered_i ];
    }
#endif

    int32_t new_cpr = pos_abs_filtered_i + abs_to_cpr;
            new_cpr = mod( new_cpr, config_.cpr );    

    uint32_t prim = cpu_enter_critical();
    {
        // Update states
        count_in_cpr_ = new_cpr;
        pos_cpr_      = (float)count_in_cpr_;

        // Update states
        pos_estimate_ = (float)count_in_cpr_;

        shadow_count_ = count_in_cpr_;
        tim_cnt_sample_ = count_in_cpr_;
        hw_config_.timer->Instance->CNT = count_in_cpr_;
    }
    cpu_exit_critical(prim);

    return true;
}


// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
// TODO: Do the scan with current, not voltage!
bool Encoder::run_offset_calibration() {
    static const float start_lock_duration = 1.0f;
    static const int num_steps = (int)(config_.calib_scan_distance / config_.calib_scan_omega * (float)current_meas_hz);

    // Require index found if enabled
    if (config_.use_index && !index_found_) {
        set_error(ERROR_INDEX_NOT_FOUND_YET);
        return false;
    }

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;

    float voltage_magnitude;
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_HIGH_CURRENT)
        voltage_magnitude = axis_->motor_.config_.calibration_current * axis_->motor_.config_.phase_resistance;
    else if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL)
        voltage_magnitude = axis_->motor_.config_.calibration_current;
    else
        return false;

    // go to motor zero phase for start_lock_duration to get ready to scan
    int i = 0;
    axis_->run_control_loop([&](){
        if (!axis_->motor_.enqueue_voltage_timings(voltage_magnitude, 0.0f))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);
        return ++i < start_lock_duration * current_meas_hz;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    int32_t init_enc_val = shadow_count_;
    int64_t encvaluesum = 0;

    // scan forward
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(config_.calib_scan_distance * (float)i / (float)num_steps - config_.calib_scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * our_arm_cos_f32(phase);
        float v_beta = voltage_magnitude * our_arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += shadow_count_;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    // Check response and direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        axis_->motor_.config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        axis_->motor_.config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_NO_RESPONSE);
        return false;
    }

    //TODO avoid recomputing elec_rad_per_enc every time
    // Check CPR
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = config_.calib_scan_distance / elec_rad_per_enc;
    calib_scan_response_ = fabsf(shadow_count_-init_enc_val);
    if(fabsf(calib_scan_response_ - expected_encoder_delta)/expected_encoder_delta > config_.calib_range)
    {
        set_error(ERROR_CPR_OUT_OF_RANGE);
        return false;
    }

    // scan backwards
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(-config_.calib_scan_distance * (float)i / (float)num_steps + config_.calib_scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * our_arm_cos_f32(phase);
        float v_beta = voltage_magnitude * our_arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += shadow_count_;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    config_.offset = encvaluesum / (num_steps * 2);
    int32_t residual = encvaluesum - ((int64_t)config_.offset * (int64_t)(num_steps * 2));
    config_.offset_float = (float)residual / (float)(num_steps * 2) + 0.5f; // add 0.5 to center-align state to phase

    is_ready_ = true;
    return true;
}

static bool decode_hall(uint8_t hall_state, int32_t* hall_cnt) {
    switch (hall_state) {
        case 0b001: *hall_cnt = 0; return true;
        case 0b011: *hall_cnt = 1; return true;
        case 0b010: *hall_cnt = 2; return true;
        case 0b110: *hall_cnt = 3; return true;
        case 0b100: *hall_cnt = 4; return true;
        case 0b101: *hall_cnt = 5; return true;
        default: return false;
    }
}


bool Encoder::abs_spi_init(){
    if ((config_.mode & MODE_FLAG_SPI) == 0x0)
        return false;

    uint32_t cr1,cr2;
    cr1 = hw_config_.spi->Instance->CR1;
    cr2 = hw_config_.spi->Instance->CR2;

    SPI_HandleTypeDef * spi = hw_config_.spi;
    spi->Init.Mode = SPI_MODE_MASTER;
    spi->Init.Direction = SPI_DIRECTION_2LINES;
    spi->Init.DataSize = SPI_DATASIZE_16BIT;
    spi->Init.CLKPolarity = SPI_POLARITY_LOW;
    spi->Init.CLKPhase = SPI_PHASE_2EDGE;
    spi->Init.NSS = SPI_NSS_SOFT;
    spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    spi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi->Init.TIMode = SPI_TIMODE_DISABLE;
    spi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi->Init.CRCPolynomial = 10;

    HAL_SPI_DeInit(spi);
    HAL_SPI_Init(spi);
    //stash our configuration
    abs_spi_cr1 = hw_config_.spi->Instance->CR1;
    abs_spi_cr2 = hw_config_.spi->Instance->CR2;

    hw_config_.spi->Instance->CR1 = cr1;
    hw_config_.spi->Instance->CR2 = cr2;
    return true;
}

bool Encoder::abs_spi_start_transaction(){
    if (config_.mode & MODE_FLAG_SPI){
        if(hw_config_.spi->State != HAL_SPI_STATE_READY){
            set_error(ERROR_ABS_SPI_NOT_READY);
            return false;
        }
        //apply the stashed configuration
        hw_config_.spi->Instance->CR1 = abs_spi_cr1;
        hw_config_.spi->Instance->CR2 = abs_spi_cr2;
        HAL_GPIO_WritePin(abs_spi_cs_port_, abs_spi_cs_pin_, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_DMA(hw_config_.spi,(uint8_t*)abs_spi_dma_tx_,(uint8_t*)abs_spi_dma_rx_,1);
    }
    return true;
}

uint8_t parity(uint16_t v){
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}


void Encoder::abs_spi_cb(){
    HAL_GPIO_WritePin(abs_spi_cs_port_, abs_spi_cs_pin_, GPIO_PIN_SET);
    switch (config_.mode) {
        case MODE_SPI_ABS_AMS: 
        case MODE_SPI_ABS_AMS_INCREMENTAL:
        {
        uint8_t parity_calc, parity_bit;
        parity_calc = parity(abs_spi_dma_rx_[0]&0x7FFF);
        parity_bit = abs_spi_dma_rx_[0] >>15;

        if(parity_calc == parity_bit)
        {
            pos_abs_ = abs_spi_dma_rx_[0] & 0x3FFF;

            // We are going to ignore values all high or low
            // This might happen in normal operation, but its unlikely
            // The filter will handle these cases
            if(pos_abs_ != 0  && pos_abs_ != 0x3FFF)
            { 
                abs_spi_pos_updated_ = true;
            }
        }
        }break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }
    is_ready_ = true;
}

void Encoder::abs_spi_cs_pin_init(){
    // Decode cs pin
    abs_spi_cs_port_ = get_gpio_port_by_pin(config_.abs_spi_cs_gpio_pin);
    abs_spi_cs_pin_ = get_gpio_pin_by_pin(config_.abs_spi_cs_gpio_pin);

    // Init cs pin
    HAL_GPIO_DeInit(abs_spi_cs_port_, abs_spi_cs_pin_);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = abs_spi_cs_pin_;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(abs_spi_cs_port_, &GPIO_InitStruct);

    // Write pin high
    HAL_GPIO_WritePin(abs_spi_cs_port_, abs_spi_cs_pin_, GPIO_PIN_SET);
}

void Encoder::sample_now() {

    switch (config_.mode) {
        case MODE_INCREMENTAL: {
            tim_cnt_sample_ = (int16_t)hw_config_.timer->Instance->CNT;
        } break;

        case MODE_HALL: {
            // do nothing: samples already captured in general GPIO capture
        } break;

        case MODE_SPI_ABS_AMS_INCREMENTAL:{
            tim_cnt_sample_ = (int16_t)hw_config_.timer->Instance->CNT;
        } break;

        case MODE_SPI_ABS_CUI:
        case MODE_SPI_ABS_AMS: {
            // do nothing, samples already captured
        } break;        

        case MODE_SINCOS: {
            sincos_sample_s_ = (get_adc_voltage(GPIO_3_GPIO_Port, GPIO_3_Pin) / 3.3f) - 0.5f;
            sincos_sample_c_ = (get_adc_voltage(GPIO_4_GPIO_Port, GPIO_4_Pin) / 3.3f) - 0.5f;
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }
}

bool Encoder::update() {
    // update internal encoder state.
    int32_t delta_enc = 0;
    switch (config_.mode) {
        case MODE_INCREMENTAL: {
            //TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            //or use 64 bit
            int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; //sign extend
        } break;

        case MODE_HALL: {
            int32_t hall_cnt;
            if (decode_hall(hall_state_, &hall_cnt)) {
                delta_enc = hall_cnt - count_in_cpr_;
                delta_enc = mod(delta_enc, 6);
                if (delta_enc > 3)
                    delta_enc -= 6;
            } else {
                if (!config_.ignore_illegal_hall_state) {
                    set_error(ERROR_ILLEGAL_HALL_STATE);
                    return false;
                }
            }
        } break;
        
        case MODE_SPI_ABS_AMS:
        case MODE_SPI_ABS_CUI:{
            if(abs_spi_pos_updated_ == false)
            {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                if (spi_error_rate_ > 0.005f)
                    set_error(ERROR_ABS_SPI_COM_FAIL);
            }
            else
            {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
            }

            abs_spi_pos_updated_ = false;
            delta_enc = pos_abs_ - count_in_cpr_;
            delta_enc = mod(delta_enc, config_.cpr);
            if (delta_enc > config_.cpr/2)
                delta_enc -= config_.cpr;

        } break;

        case MODE_SPI_ABS_AMS_INCREMENTAL: {
            if(abs_spi_pos_updated_ == false)
            {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                if (spi_error_rate_ > 0.005f)
                    set_error(ERROR_ABS_SPI_COM_FAIL);
            }
            else
            {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
            }

            //TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            //or use 64 bit
            int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; //sign extend
        } break;      

		
        case MODE_SINCOS: {
            float phase = fast_atan2(sincos_sample_s_, sincos_sample_c_);
            int fake_count = (int)(1000.0f * phase);
            //CPR = 6283 = 2pi * 1k

            delta_enc = fake_count - count_in_cpr_;
            delta_enc = mod(delta_enc, 6283);
            if (delta_enc > 6283/2)
                delta_enc -= 6283;
        } break;
        
        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           return false;
        } break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    if(config_.mode & MODE_FLAG_ABS)
        count_in_cpr_ = pos_abs_;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_ += current_meas_period * vel_estimate_;
    pos_cpr_      += current_meas_period * vel_estimate_;
    // discrete phase detector
    float delta_pos     = (float)(shadow_count_ - (int32_t)floorf(pos_estimate_));
    float delta_pos_cpr = (float)(count_in_cpr_ - (int32_t)floorf(pos_cpr_));
    delta_pos_cpr = wrap_pm(delta_pos_cpr, 0.5f * (float)(config_.cpr));
    // pll feedback
    pos_estimate_ += current_meas_period * pll_kp_ * delta_pos;
    pos_cpr_      += current_meas_period * pll_kp_ * delta_pos_cpr;
    pos_cpr_ = fmodf_pos(pos_cpr_, (float)(config_.cpr));
    vel_estimate_      += current_meas_period * pll_ki_ * delta_pos_cpr;
    bool snap_to_zero_vel = false;
    if (fabsf(vel_estimate_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_ = 0.0f; //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.offset_float);
    // ph = fmodf(ph, 2*M_PI);
    phase_ = wrap_pm_pi(ph);

#ifdef ENABLE_ABS_TO_CPR_MAP
    if ( config_.dbg_mapping_idx >= 0 && config_.dbg_mapping_idx < kABS_TO_CPR_MAP_SIZE )
    {
        config_.dbg_mapping_sample = config_.abs_to_cpr_map[ config_.dbg_mapping_idx ];
    }
#endif

    return true;
}
