int main(void){
        // old main function
    /*
    bool recovery_active = false;

    // LoRa
    
    InitADCRef();
    InitCoilGPIO(); 
    gpioUnlock();
    ConfigCoilPWM(); 

    InitACCL();
    ConfigureACCL();

    InitBMP();
    ConfigureBMP();

    ground_pressure = CalibrateGroundPressure(100);
    initial_altitude = PressureToAltitude(ground_pressure);

    current_flight_state = PREFLIGHT;

    // LoRa to ensure init was done succesfully 

    __enable_interrupt(); 

    while(1){
        
        //Read from sensors here
        

        // 1. Check if new BMP sample ready
        if (bmp_data_ready) {
            bmp_data_ready = false;
            GetPressure();
                if (available_samples > 0) {
                    // read the next sample from buffer
                    //BMPData raw = bmpBuffer[read_index]; // RAW WTF???
                    const volatile BMPData& raw = bmpBuffer[read_index];
                    //I think this is meant to be raw_pressure?

                    // advance read_index and decrease available_samples
                    read_index = (read_index + 1) % BMP_BUFFER_SIZE;
                    available_samples--;

                    // convert to altitude and push to sliding window
                    // float altitude = PressureToAltitude(bmpBuffer[read_index].pressure);
                    float altitude = PressureToAltitude(raw.pressure); //This is meant to raw_pressure?
                    AltWindow_Push(altitude);
                }
        }


        // Update Flight State // should pass in small sample of data for flight determination.
        UpdateFlightState();

        // State-depened behaviour
        switch (current_flight_state) {
            case PREFLIGHT:
                break;
            case FLIGHT:
                break;
            case LANDED:
                if (!recovery_active){
                    // Enter recovery
                    RecoveryMode();
                    recovery_active = true;
                }
                break;
        }
    }
    */

    
    // Successful pulsing of PWM using LED
    /*
    WDTCTL = WDTPW + WDTHOLD; // Stop Watchdog Timer
    InitCoilGPIO(); 
    gpioUnlock();
    ConfigCoilPWM(); 

    while (1) {
		for (uint16_t i = 0; i < 100; i++) {
            SetCoilPWM(i);
            __delay_cycles(50 * PWM_PERIOD);
		}
		for (uint16_t i = 100; i > 0; i--) {
			SetCoilPWM(i);
            __delay_cycles(50 * PWM_PERIOD);
		}
	}
    */

    // Code to test coil PWM on PCB
    /*
    WDTCTL = WDTPW + WDTHOLD;
    InitCoilGPIO(); 
    gpioUnlock();
    ConfigCoilPWM(); 

    while (1) {
        SetCoilPWM(30);
    }
    */
    
    // Code to test fan PWM
    /*
    WDTCTL = WDTPW + WDTHOLD; // Stop Watchdog Timer
    InitFanGPIO(); 
    gpioUnlock();
    ConfigFanPWM();
    SetFanPWM(100);
    */
    


    // Successful use of background timer interrupts with toggling LEDs (other task)
    /*
    WDTCTL = WDTPW | WDTHOLD;

    // Initialise timer
    ConfigBackgroundTimer();

    // Init PWM
    InitCoilGPIO(); 
    gpioUnlock();
    ConfigCoilPWM(); 
    InitClock16MHz();
    
    // Init LEDs
    P1DIR |= RED_LED; // Equivalent of P1DIR |= BIT0; due to the #DEFINE at the top of the program
    P6DIR |= GREEN_LED;

    // Turn LEDs off.
    P1OUT &= ~RED_LED;
    P6OUT &= ~GREEN_LED;

    __enable_interrupt();

    // Being 40 seconds timer
    start_delay_seconds(40);   // start 40 seconds timer
    SetCoilPWM(100); // Turn LED ON

    // Set initial values - red LED on, green LED off.
    P1OUT |= RED_LED;
    P6OUT &= ~GREEN_LED;

    while (1)
    {
        if (timer_expired)
        {
            timer_expired = 0;
            // Execute task after 40 seconds end
            SetCoilPWM(0); // Turn LED OFF (Mock PWM Coil)
        }

        // other background work here. in this case it would be LoRa
        // state machines, IO, comms, etc.

        P1OUT ^= RED_LED;
        P6OUT ^= GREEN_LED;
        __delay_cycles(1600000); // 100ms delay at 16MHz
    }
    */



    //ADC Testing:
    /*
    WDTCTL = WDTPW | WDTHOLD;    
    InitADCRef();               // I believe setting the reference can be put after InitADCGPIO

    // P1DIR |= BIT0;             // RED LED as output
    // P1OUT &= ~BIT0;            // Turn RED LED off

    // Configure P5.0
    InitADCGPIO();

    ConfigADC();

    PM5CTL0 &= ~LOCKLPM5;   // Unlock GPIO pins

    uint16_t chamber_temperature;
    uint16_t battery_temperature;

    while(1)
    {
        // Start Conversion
        chamber_temperature = GetChamberTemp();
        //battery_temperature = GetBatteryTemp();
        
    }
    */
    
    
    // Reading BMP data (pressure data in Pa), and using it in UpdateFlightState(I2C)
    /*
    volatile float pressure;
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer. 

    InitSensorsGPIO();
    ConfigSensorsI2C();
    ConfigBMPI2C();
    CalibrateBMPI2C(&ground_pressure_i2c, &initial_altitude_i2c); 
    //ConfigACCLI2C();

    while (1) {
        ProcessBMPDataI2C(); 
        UpdateFlightStateI2C();
    }
    */


    // Successful GPS testing: watch LAT LON. Correct values when fixed. ZERO otherwise.
    /*
    WDTCTL = WDTPW | WDTHOLD;
    InitClock16MHz();
    Software_Trim();

    P1DIR |= BIT0;             // RED LED as output
    P1OUT &= ~BIT0;            // Turn RED LED off
    ClearGPSBuffer();
    InitGPSGPIO(); 
    ConfigGPSUART();
    
    while(1){
        __bis_SR_register(GIE); // Enter LPM0, Enable Interrupt
        if (gps_line_ready) {
            gps_line_ready = 0;

            // Loops through buffer to clear NULL
            // --- Add this BEFORE calling parse_gpgga --- sanitise buffer before passing to remove NULL which stops strtok from working
            // for (int i = 0; i < gps_index; i++) {
            //     if (gps_buffer[i] == 0) gps_buffer[i] = ',';  // replace nulls with comma
            // }
            // gps_buffer[gps_index] = '\0'; // ensure string is properly terminated
            
            //parse_gngga((char*)gps_buffer);   // attempt to parase after every sentence

            // Parse gngga only
            if (is_gngga((char*)gps_buffer)) {
                parse_gngga((char*)gps_buffer);
            } 
        }
    }
    */

    // LoRa Code
    /*
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    InitLoRaGPIO();
    PM5CTL0 &= ~LOCKLPM5;   // Unlock GPIO
    spi_B1_init();

    lora_configure(
            BANDWIDTH125K,              // 125 khz
            CODINGRATE4_5,              // Coding rate 4/5
            CRC_ENABLE,                 // Enable CRC
            EXPLICIT_HEADER_MODE,       // Explicit header
            POLARITY_NORMAL_MODE,       // Normal IQ
            PREAMBLE_LENGTH,            // Preamble 8
            SPREADINGFACTOR128,         // SF7
            SYNC_WORD_RESET,            // 0x12
            radioChipSelPin
    );
    
    LoRaTX(); 
    */
 
    // GPS testing for main
    /*
    while(1){
        TransmitGPS();
    }
    */



    // Subsystem list
/*
    coil pwm    done and tested
    fan pwm     done and tested
    bmp         done and tested
    accl        done and tesded
    adc         done and tested
    gps         done and tested
    lora        done and tested
    background timer    done and tested
    regulator enable    done

*/
}