classdef Adafruit_MAX31856
    properties (Access = public)
        arduino;
        cs;
        spi;
        
        MAX3156_REG_READ            =  uint8(hex2dec('00'));    % Read command
        MAX3156_REG_WRITE           =  uint8(hex2dec('80'));    % Write command
        
        MAX31856_CR0_REG            =  uint8(hex2dec('00'));    % Config 0 register
        MAX31856_CR0_AUTOCONVERT    =  uint8(hex2dec('80'));    % Config 0 Auto convert flag
        MAX31856_CR0_1SHOT          =  uint8(hex2dec('40'));    % Config 0 one shot convert flag
        MAX31856_CR0_OCFAULT1       =  uint8(hex2dec('20'));    % Config 0 open circuit fault 1 flag
        MAX31856_CR0_OCFAULT0       =  uint8(hex2dec('10'));    % Config 0 open circuit fault 0 flag
        MAX31856_CR0_CJ             =  uint8(hex2dec('08'));    % Config 0 cold junction disable flag
        MAX31856_CR0_FAULT          =  uint8(hex2dec('04'));    % Config 0 fault mode flag
        MAX31856_CR0_FAULTCLR       =  uint8(hex2dec('02'));    % Config 0 fault clear flag

        MAX31856_CR1_REG            =  uint8(hex2dec('01'));    % Config 1 register
        MAX31856_MASK_REG           =  uint8(hex2dec('02'));    % Fault Mask register
        MAX31856_CJHF_REG           =  uint8(hex2dec('03'));    % Cold junction High temp fault register
        MAX31856_CJLF_REG           =  uint8(hex2dec('04'));    % Cold junction Low temp fault register
        MAX31856_LTHFTH_REG         =  uint8(hex2dec('05'));    % Linearized Temperature High Fault Threshold Register, MSB
        MAX31856_LTHFTL_REG         =  uint8(hex2dec('06'));    % Linearized Temperature High Fault Threshold Register, LSB
        MAX31856_LTLFTH_REG         =  uint8(hex2dec('07'));    % Linearized Temperature Low Fault Threshold Register, MSB
        MAX31856_LTLFTL_REG         =  uint8(hex2dec('08'));    % Linearized Temperature Low Fault Threshold Register, LSB
        MAX31856_CJTO_REG           =  uint8(hex2dec('09'));    % Cold-Junction Temperature Offset Register 
        MAX31856_CJTH_REG           =  uint8(hex2dec('0A'));    % Cold-Junction Temperature Register, MSB
        MAX31856_CJTL_REG           =  uint8(hex2dec('0B'));    % Cold-Junction Temperature Register, LSB
        MAX31856_LTCBH_REG          =  uint8(hex2dec('0C'));    % Linearized TC Temperature, Byte 2 
        MAX31856_LTCBM_REG          =  uint8(hex2dec('0D'));    % Linearized TC Temperature, Byte 1
        MAX31856_LTCBL_REG          =  uint8(hex2dec('0E'));    % Linearized TC Temperature, Byte 0
        MAX31856_SR_REG             =  uint8(hex2dec('0F'));    % Fault Status Register

        MAX31856_FAULT_CJRANGE      =  uint8(hex2dec('80'));    % Fault status Cold Junction Out-of-Range flag
        MAX31856_FAULT_TCRANGE      =  uint8(hex2dec('40'));    % Fault status Thermocouple Out-of-Range flag
        MAX31856_FAULT_CJHIGH       =  uint8(hex2dec('20'));    % Fault status Cold-Junction High Fault flag
        MAX31856_FAULT_CJLOW        =  uint8(hex2dec('10'));    % Fault status Cold-Junction Low Fault flag
        MAX31856_FAULT_TCHIGH       =  uint8(hex2dec('08'));    % Fault status Thermocouple Temperature High Fault flag
        MAX31856_FAULT_TCLOW        =  uint8(hex2dec('04'));    % Fault status Thermocouple Temperature Low Fault flag
        MAX31856_FAULT_OVUV         =  uint8(hex2dec('02'));    % Fault status Overvoltage or Undervoltage Input Fault flag
        MAX31856_FAULT_OPEN         =  uint8(hex2dec('01'));    % Fault status Thermocouple Open-Circuit Fault flag

        MAX31856_TCTYPE_B           = uint8(bin2dec('0000'));
        MAX31856_TCTYPE_E           = uint8(bin2dec('0001'));
        MAX31856_TCTYPE_J           = uint8(bin2dec('0010'));
        MAX31856_TCTYPE_K           = uint8(bin2dec('0011'));
        MAX31856_TCTYPE_N           = uint8(bin2dec('0100'));
        MAX31856_TCTYPE_R           = uint8(bin2dec('0101'));
        MAX31856_TCTYPE_S           = uint8(bin2dec('0110'));
        MAX31856_TCTYPE_T           = uint8(bin2dec('0111'));
        MAX31856_VMODE_G8           = uint8(bin2dec('1000'));
        MAX31856_VMODE_G32          = uint8(bin2dec('1100'));
        
        MAX31856_NOISE_FILTER_50HZ  = 50;
        MAX31856_NOISE_FILTER_60HZ  = 60;        
    end
    
    methods (Access = public)
        function obj = Adafruit_MAX31856(com_port, board_type, spi_cs)
            obj.arduino = arduino(com_port, board_type, 'Libraries', 'SPI');
            obj.cs = spi_cs;
            disp(obj.arduino);
            obj.spi = spidev(obj.arduino, obj.cs, 'BitOrder', 'msbfirst', 'Mode', 1, 'BitRate', 500000);
            obj.begin();
        end
        
        function reply = setThermocoupleType(obj, type)
            t = obj.readRegister8(obj.MAX31856_CR1_REG);
            t = t(2);
            t = bitand(t, hex2dec('F0'));
            t = bitor(t, bitand(uint8(type), hex2dec('0F')));
            reply = obj.writeRegister8(obj.MAX31856_CR1_REG, t);
        end    
        
        function [] = setTempFaultThreshholds(obj, flow, fhigh)
            flow = flow * 16;
            low = uint16(flow);
            fhigh = fhigh * 16;
            high = uint16(fhigh);
            obj.writeRegister8(obj.MAX31856_LTHFTH_REG, bitsrl(high, 8));
            obj.writeRegister8(obj.MAX31856_LTHFTL_REG, high);
            obj.writeRegister8(obj.MAX31856_LTLFTH_REG, bitsrl(low, 8));
            obj.writeRegister8(obj.MAX31856_LTLFTL_REG, low);
        end
        
        function [] = setColdJunctionFaultThreshholds(obj, low, high)
            low = int8(low);
            high = int8(high);
            obj.writeRegister8(obj.MAX31856_CJLF_REG, low);
            obj.writeRegister8(obj.MAX31856_CJHF_REG, high);
        end
        
        function reply = setNoiseFilter(obj, noiseFilter)
            t = obj.readRegister8(obj.MAX31856_CR0_REG);
            t = t(2);
            if noiseFilter == obj.MAX31856_NOISE_FILTER_50HZ
                t = bitor(t, hex2dec('01'));
            else
                t = bitand(t, hex2dec('FE'));
            end
            reply = obj.writeRegister8(obj.MAX31856_CR0_REG, t);
        end

        function [] = begin(obj)
             obj.writeRegister8(obj.MAX31856_MASK_REG, hex2dec('00'));
             obj.writeRegister8(obj.MAX31856_CR0_REG, obj.MAX31856_CR0_OCFAULT0);
             obj.setThermocoupleType(obj.MAX31856_TCTYPE_K);
             obj.setNoiseFilter(obj.MAX31856_NOISE_FILTER_50HZ);
        end
        
        function max31856_thermocoupletype_t = getThermocoupleType(obj)
            t = uint8(obj.readRegister8(obj.MAX31856_CR1_REG));
            t = t(2);
            t = bitand(t, hex2dec('0F'));
            max31856_thermocoupletype_t = t; 
        end
        
        function fault = readFault(obj)
            fault = obj.readRegister8(obj.MAX31856_SR_REG);
            fault = fault(2);
        end
        
        function reply = oneShotTemperature(obj)
            obj.writeRegister8(obj.MAX31856_CJTO_REG, hex2dec('00'));
            t = uint8(obj.readRegister8(obj.MAX31856_CR0_REG));
            t = t(2);
            t = bitand(t, bitcmp(obj.MAX31856_CR0_AUTOCONVERT));
            t = bitor(t, obj.MAX31856_CR0_1SHOT);
            reply = obj.writeRegister8(obj.MAX31856_CR0_REG, t);
            pause(0.1);
        end
        
        function coldJunctionTemperature = readCJTemperature(obj)
           obj.oneShotTemperature();
           temp16 = obj.readRegister16(obj.MAX31856_CJTH_REG);
           tempfloat = double(temp16);
           tempfloat = tempfloat / 256.0;
           coldJunctionTemperature = tempfloat;
        end
        
        function thermocoupleTemperature = readThermocoupleTemperature(obj)
           obj.oneShotTemperature();
           temp24 = obj.readRegister24(obj.MAX31856_LTCBH_REG);
           if bitand(temp24, hex2dec('800000')) ~= 0
               temp24 = bitor(temp24, hex2dec('FF0000'));
           end
           temp24 = bitsrl(temp24, 5);
           tempfloat = double(temp24);
           tempfloat = tempfloat * 0.0078125;
           thermocoupleTemperature = tempfloat;
        end
    end
    
    methods (Access = private)
        function buffer = readRegisterN(obj, addr, buffer, n)
            i = 1;
            addr = bitand(addr, hex2dec('7F'));
            buffer = [buffer, addr];
            while n > 0
                buffer = [buffer, hex2dec('FF')]; %#ok<AGROW>
                i = i + 1;
                n = n - 1;
            end
            buffer = obj.spixfer(buffer);
        end
        
        function reply = readRegister8(obj, addr)
            ret = [];
            reply = obj.readRegisterN(addr, ret, 1);
        end
        
        function reply = readRegister16(obj, addr)
            buffer = [];
            buffer = obj.readRegisterN(addr, buffer, 2);
            buffer = uint16(buffer);
            reply = buffer(2);
            reply = bitsll(reply, 8);
            reply = bitor(reply, buffer(3));
        end
        
        function reply = readRegister24(obj, addr)
            buffer = [];
            buffer = obj.readRegisterN(addr, buffer, 3);
            buffer = int32(buffer);
            reply = buffer(2);
            reply = bitsll(reply, 8);
            reply = bitor(reply, buffer(3));
            reply = bitsll(reply, 8);
            reply = bitor(reply, buffer(4));
            reply = int32(reply);
        end
        
        function reply = writeRegister8(obj, addr, data)
            addr = bitor(addr, obj.MAX3156_REG_WRITE);
            reply = obj.spixfer([addr, data]);
        end
        
        function reply = spixfer(obj, x)
            reply = writeRead(obj.spi, x, 'uint8');
        end        
    end    

end

