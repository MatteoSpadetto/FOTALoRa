'''
 __    __  ________  __    __  _______                      ______   __    __  ________  ______    ______   ________  __    __ 
/  |  /  |/        |/  |  /  |/       \         __         /      \ /  |  /  |/        |/      \  /      \ /        |/  \  /  |
$$ |  $$ |$$$$$$$$/ $$ |  $$ |$$$$$$$  |       /  |       /$$$$$$  |$$ |  $$ |$$$$$$$$//$$$$$$  |/$$$$$$  |$$$$$$$$/ $$  \ $$ |
$$ |__$$ |$$ |__    $$  \/$$/ $$ |__$$ |     __$$ |__     $$ |__$$ |$$ |  $$ |   $$ |  $$ |  $$ |$$ | _$$/ $$ |__    $$$  \$$ |
$$    $$ |$$    |    $$  $$<  $$    $$/     /  $$    |    $$    $$ |$$ |  $$ |   $$ |  $$ |  $$ |$$ |/    |$$    |   $$$$  $$ |
$$$$$$$$ |$$$$$/      $$$$  \ $$$$$$$/      $$$$$$$$/     $$$$$$$$ |$$ |  $$ |   $$ |  $$ |  $$ |$$ |$$$$ |$$$$$/    $$ $$ $$ |
$$ |  $$ |$$ |_____  $$ /$$  |$$ |             $$ |       $$ |  $$ |$$ \__$$ |   $$ |  $$ \__$$ |$$ \__$$ |$$ |_____ $$ |$$$$ |
$$ |  $$ |$$       |$$ |  $$ |$$ |             $$/        $$ |  $$ |$$    $$/    $$ |  $$    $$/ $$    $$/ $$       |$$ | $$$ |
$$/   $$/ $$$$$$$$/ $$/   $$/ $$/                         $$/   $$/  $$$$$$/     $$/    $$$$$$/   $$$$$$/  $$$$$$$$/ $$/   $$/ 
'''     

# Used for LoRa Firmware Update Over the Air project. Embedded system course University of Trento.
# This pyhton code is an ".hex" files pares that autogenerates an C ".h" file.
# In the header file addresses and data are stored.
# To use it, just import the ".hex" file, change the name of the file in script and run
# with "python pars_hex.py" on terminal.

import sys                                                                                                                  

arr_aaaa = [] # Array for addresses
arr_dd = [] # Array for data

flag_eof = 0 # End Of File flag
str_tmp_ll = "" # String to store data size
str_tmp_aa = "" # String to store address
aaaa_counter = 0 # Counter to get the size of addresses array
dd_counter = 0 # Counter to get the size of the data array

# HEX PARSER

# Open the ".hex" file and paresing it into arrays
with open('test_to_send.hex', 'r') as file_in:
    while file_in: # Cycling all file lines
        if(flag_eof == 0):
            line_data = file_in.readline()
            line_data = line_data[1:]
            # Handling the not used lines --> TODO: make it autonomus
            if (str(line_data) == '020000040800F2\n'): # If it is the first line, pass
                print("begin")
                pass

            elif (str(line_data) == '0400000508001031AE\n'): # If it is the second-last line, pass
                pass

            elif (str(line_data) == '00000001FF\n'): # If it is the EOF line, pass
                flag_eof = 1
                print("stop")
                pass

            else:
                print(line_data)
                # Get size concatenating chars, converting string to hex base and store in tmp
                str_tmp_ll = "0x" + line_data[0]
                str_tmp_ll = str_tmp_ll + line_data[1]
                tmp_ll = int(str_tmp_ll, 16)
                hex_ll = hex(tmp_ll)
                print(hex_ll)
                # Get address concatenating chars, converting string to hex base and store in tmp
                str_tmp_aa = "0x" + line_data[2]
                str_tmp_aa = str_tmp_aa + line_data[3]
                str_tmp_aa = str_tmp_aa + line_data[4]
                str_tmp_aa = str_tmp_aa + line_data[5]
                tmp_aa = int(str_tmp_aa, 16)
                hex_aa = hex(tmp_aa)
                arr_aaaa.append(hex_aa)
                # Storing also the middle address from the main ones for right memory writing
                tmp_aa = tmp_aa + 0x08
                hex_aa = hex(tmp_aa)
                arr_aaaa.append(hex_aa)
                print(hex_aa)
                # Get data storing 4 bytes in each elemet of the tmp array with size 4 then store the tmp in the data array
                char_counter = 8
                word_counter = 0
                arr_counter = 0
                arr_tmp_dd = ["0x", "0x", "0x", "0x"]
                while (char_counter < len(line_data) - 3):
                    if (word_counter == 8): # If the line data are all parsed then reset conunters
                        word_counter = 0
                        arr_counter = arr_counter + 1
                        char_counter = char_counter - 1
                    else: # If the line data are not all parsed then continue to parse
                        arr_tmp_dd[arr_counter] = arr_tmp_dd[arr_counter] + \
                            (line_data[char_counter])
                        word_counter = word_counter + 1
                    char_counter = char_counter + 1
                arr_counter = 0
                print(arr_tmp_dd)
                arr_dd.append(arr_tmp_dd)
                i = 0
                while i < len(arr_tmp_dd):
                    if (str(arr_tmp_dd[i]) != "0x"): # If the value is not 0x the number of data is increased
                        dd_counter = dd_counter + 1
                    i = i + 1
        else:
            break
m = 0
while m < len(arr_aaaa):
    if (str(arr_aaaa[m]) != str(arr_aaaa[m - 1])): # In order to avoid multiple equal addresses
        aaaa_counter = aaaa_counter + 1
    m = m + 1
    
# Debug arrays values
print("ARR_AAAA:")
print(arr_aaaa)
print("ARR_DD:")
print(arr_dd)
print("len_dd")
print(dd_counter)
print("len_aaaa")
print(aaaa_counter)

# HEADER FILE AUTO-GENERATION

# Open the ".h" file to store the addresses and data
with open('parsed_hex.h', 'w') as file_out:
    file_out.write("#ifndef _PARSED_HEX_H\r\n")
    file_out.write("#define _PARSED_HEX_H\r\n\r\n")

    # Addresses array
    file_out.write("int arr_aaaa[")
    file_out.write(str(aaaa_counter)) # Addresses array size
    file_out.write("] = {")
    eoa = 0 # Hanling the End Of Array
    while eoa < len(arr_aaaa):
        if (str(arr_aaaa[eoa]) != str(arr_aaaa[eoa - 1])):
            if (eoa != (len(arr_aaaa) - 1)): # Write the element and "," on ".h" file
                file_out.write(str(arr_aaaa[eoa]))
                file_out.write(", ")
            else: # Hanling the last element to not write the "," at the end
                file_out.write(str(arr_aaaa[eoa]))
        eoa = eoa + 1
    file_out.write("};\r\n")

    # Data array
    file_out.write("int arr_dd[")
    file_out.write(str(dd_counter)) # Data array size
    file_out.write("] = {")
    eom = 0 # Handling End Of Matrix
    while eom < (len(arr_dd)):
        eoa = 0 # Hanling End Of Array
        while eoa < 4:
            if ((eoa == 3) & (eom == len(arr_dd) - 1)): # Hanling the last element to not write the "," at the end
                file_out.write(str(arr_dd[eom][eoa]))
            else:
                if (str(arr_dd[eom][eoa]) != "0x" ): # Write the element and "," on ".h" file
                    file_out.write(str(arr_dd[eom][eoa]))
                    file_out.write(", ")
            eoa = eoa + 1
        eom = eom + 1
    file_out.write("};\r\n")

    file_out.write("#endif\r\n")
    file_out.close()