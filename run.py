import os
import sys

VERILATE_COMMAND_TRACE_I2C = "verilator --assert -I./rtl/verilog --Wall --trace --cc ./rtl/verilog/i2c_write_master.v --exe test/tb_i2c_write_master.cpp"
VERILATE_COMMAND_TRACE_OLED = "verilator --assert -I./rtl/verilog --Wall --trace --cc ./rtl/verilog/oled_config.v --exe test/tb_oled_config.cpp"
MAKE_COMMAND_I2C = "make -C obj_dir -f Vi2c_write_master.mk"
MAKE_COMMAND_OLED = "make -C obj_dir -f Voled_config.mk"
SAVE_COMMAND_I2C = "./obj_dir/Vi2c_write_master"
SAVE_COMMAND_OLED = "./obj_dir/Voled_config"
CLEAN_COMMAND = "rm -r ./obj_dir"

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 script.py [i2c|oled]")
        sys.exit(1)

    target = sys.argv[1].lower()

    if target == "i2c":
        os.system(VERILATE_COMMAND_TRACE_I2C)
        os.system(MAKE_COMMAND_I2C)
        os.system(SAVE_COMMAND_I2C)
    elif target == "oled":
        os.system(VERILATE_COMMAND_TRACE_OLED)
        os.system(MAKE_COMMAND_OLED)
        os.system(SAVE_COMMAND_OLED)
    else:
        print("Unknown target:", target)
        print("Usage: python3 script.py [i2c|oled]")
        sys.exit(1)

    os.system(CLEAN_COMMAND)

if __name__ == "__main__":
    main()
