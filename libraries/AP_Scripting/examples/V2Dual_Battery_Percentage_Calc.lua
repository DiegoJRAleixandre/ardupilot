
local battery_instance = 0
local percentage_filt = nil
local exp_filter = 0.99
local battery_type

-- Lookup table values to calculate battery percentage from voltage
-- battery number 0, from sheet 'Onbo B 4.1A', assuming total mah of 16000
local LiPo_volt = {19.317000, 19.870091, 20.312182, 20.659182, 20.947545, 21.186273, 21.385545, 21.552818, 21.692909, 21.811545, 21.892273, 21.949000, 21.996091, 22.036000, 22.079000, 22.119000, 22.154727, 22.184545, 22.211091, 22.238636, 22.266818, 22.289455, 22.311000, 22.330909, 22.347045, 22.361364, 22.375000, 22.388409, 22.403727, 22.418636, 22.429000, 22.441818, 22.453273, 22.458000, 22.469000, 22.477000, 22.482318, 22.491000, 22.496273, 22.506727, 22.515818, 22.525364, 22.538000, 22.551455, 22.569000, 22.584273, 22.600182, 22.620636, 22.640182, 22.660091, 22.680273, 22.703182, 22.726000, 22.753364, 22.777000, 22.805000, 22.833455, 22.864909, 22.894636, 22.926409, 22.962455, 23.003000, 23.042000, 23.083727, 23.127000, 23.167455, 23.212000, 23.260000, 23.303818, 23.349364, 23.393182, 23.439364, 23.482864, 23.526182, 23.567818, 23.601000, 23.633818, 23.668000, 23.703000, 23.741045, 23.788364, 23.843000, 23.907182, 23.985273, 24.074545, 24.172455, 24.254182, 24.307455, 24.346000, 24.379091, 24.426000, 24.478909, 24.537000, 24.594000, 24.655136, 24.716000, 24.779364, 24.845818, 24.918000, 25.122000}
local LiPo_pct = {4.375000, 5.340909, 6.306818, 7.272727, 8.238636, 9.204545, 10.170455, 11.136364, 12.102273, 13.068182, 14.034091, 15.000000, 15.965909, 16.931818, 17.897727, 18.863636, 19.829545, 20.795455, 21.761364, 22.727273, 23.693182, 24.659091, 25.625000, 26.590909, 27.556818, 28.522727, 29.488636, 30.454545, 31.420455, 32.386364, 33.352273, 34.318182, 35.284091, 36.250000, 37.215909, 38.181818, 39.147727, 40.113636, 41.079545, 42.045455, 43.011364, 43.977273, 44.943182, 45.909091, 46.875000, 47.840909, 48.806818, 49.772727, 50.738636, 51.704545, 52.670455, 53.636364, 54.602273, 55.568182, 56.534091, 57.500000, 58.465909, 59.431818, 60.397727, 61.363636, 62.329545, 63.295455, 64.261364, 65.227273, 66.193182, 67.159091, 68.125000, 69.090909, 70.056818, 71.022727, 71.988636, 72.954545, 73.920455, 74.886364, 75.852273, 76.818182, 77.784091, 78.750000, 79.715909, 80.681818, 81.647727, 82.613636, 83.579545, 84.545455, 85.511364, 86.477273, 87.443182, 88.409091, 89.375000, 90.340909, 91.306818, 92.272727, 93.238636, 94.204545, 95.170455, 96.136364, 97.102273, 98.068182, 99.034091, 100.000000}

-- battery number 1, from sheet 'F1 4.1A', assuming total mah of 22000
local LiIon_voltage = {18.202000, 18.242045, 18.489818, 18.726000, 18.949909, 19.148636, 19.333182, 19.497091, 19.646091, 19.792182, 19.943636, 20.110000, 20.256727, 20.371727, 20.452727, 20.514000, 20.566000, 20.611091, 20.657455, 20.709545, 20.763273, 20.823909, 20.882000, 20.943091, 20.999273, 21.052909, 21.100545, 21.149636, 21.191455, 21.239091, 21.284727, 21.321273, 21.365364, 21.405000, 21.440727, 21.479455, 21.517727, 21.558091, 21.596545, 21.639455, 21.675909, 21.719045, 21.764000, 21.810364, 21.858000, 21.907000, 21.957636, 22.008909, 22.062091, 22.118182, 22.176455, 22.225545, 22.283091, 22.338727, 22.397000, 22.449000, 22.509000, 22.562182, 22.618818, 22.673455, 22.729000, 22.780182, 22.838909, 22.889909, 22.945182, 22.997636, 23.049000, 23.105000, 23.151000, 23.209909, 23.266818, 23.321182, 23.382182, 23.440545, 23.503909, 23.569000, 23.633364, 23.697000, 23.765000, 23.830727, 23.894091, 23.957000, 24.015182, 24.069818, 24.113818, 24.151182, 24.186000, 24.213455, 24.240000, 24.261364, 24.284273, 24.306000, 24.334455, 24.361000, 24.402455, 24.441545, 24.496000, 24.576818, 24.691364, 25.092000}
local LiIon_pct = {1.286364, 2.283471, 3.280579, 4.277686, 5.274793, 6.271901, 7.269008, 8.266116, 9.263223, 10.260331, 11.257438, 12.254545, 13.251653, 14.248760, 15.245868, 16.242975, 17.240083, 18.237190, 19.234298, 20.231405, 21.228512, 22.225620, 23.222727, 24.219835, 25.216942, 26.214050, 27.211157, 28.208264, 29.205372, 30.202479, 31.199587, 32.196694, 33.193802, 34.190909, 35.188017, 36.185124, 37.182231, 38.179339, 39.176446, 40.173554, 41.170661, 42.167769, 43.164876, 44.161983, 45.159091, 46.156198, 47.153306, 48.150413, 49.147521, 50.144628, 51.141736, 52.138843, 53.135950, 54.133058, 55.130165, 56.127273, 57.124380, 58.121488, 59.118595, 60.115702, 61.112810, 62.109917, 63.107025, 64.104132, 65.101240, 66.098347, 67.095455, 68.092562, 69.089669, 70.086777, 71.083884, 72.080992, 73.078099, 74.075207, 75.072314, 76.069421, 77.066529, 78.063636, 79.060744, 80.057851, 81.054959, 82.052066, 83.049174, 84.046281, 85.043388, 86.040496, 87.037603, 88.034711, 89.031818, 90.028926, 91.026033, 92.023140, 93.020248, 94.017355, 95.014463, 96.011570, 97.008678, 98.005785, 99.002893, 100.000000}


-- load Variable for the battery index
local battery_num = battery:get_serial_number(battery_instance)
if battery_num == 0 then
    gcs:send_text(3,"Configured for LiPo battery")

elseif battery_num == 1 then
    gcs:send_text(3,"Configured for Solid State battery")

else
    battery_num = 0
    gcs:send_text(1,"Unknown battery index, check BATT_SERIAL_NUM")

end

function linear_interpolate(low_output, high_output, var_value, var_low, var_high)
    local p = (var_value - var_low) / (var_high - var_low)
    return low_output + p * (high_output - low_output)
end

function interp_table(voltage_tab, pct_tab, voltage)
    -- find the correct point in the table

    -- check extremes
    if voltage <= voltage_tab[1] then
        -- voltage lower than min value in table
        return pct_tab[1]
    end
    if voltage >= voltage_tab[#voltage_tab] then
        -- voltage hight than max value
        return pct_tab[#voltage_tab]
    end

    for i = 1, #voltage_tab-1 do
        if voltage >= voltage_tab[i] and voltage < voltage_tab[i+1] then
            return linear_interpolate(pct_tab[i], pct_tab[i+1], voltage, voltage_tab[i], voltage_tab[i+1])
        end
    end
end

-- test interpolation table
--[[
local file = io.open('table_test_LiPo.csv', 'w')
for i = 28, 18, -0.01 do
    file:write(string.format('%0.2f, %0.2f\n',i,interp_table(LiPo_volt,LiPo_pct,i)))
end
file:close()
local file = io.open('table_test_LiIon.csv', 'w')
for i = 28, 18, -0.01 do
    file:write(string.format('%0.2f, %0.2f\n',i,interp_table(LiIon_voltage,LiIon_pct,i)))
end
file:close()
--]]


function update()

    if not battery:healthy(battery_instance) then
        -- the battery monitor is not healthy, don't change anything
        return update, 1000
    end

    -- to use raw voltage
    --local voltage = battery:voltage(battery_instance)

    -- to use compensated voltage
    local voltage = battery:voltage_resting_estimate(battery_instance)

    -- Check the battery number we should be using
    local batt_num = battery:get_serial_number(battery_instance)
    if batt_num ~= battery_type then
        if batt_num == 0 then
            gcs:send_text(3,"LiPo Selected")

        elseif batt_num == 1 then
            gcs:send_text(3,"Solid State Selected")

        else
            gcs:send_text(0,"Unknown battery index, check BATT_SERIAL_NUM")
            -- set battery percentage to 0 to discourage flying with incorrect params
            battery:reset_remaining(battery_instance, 0)
            return update, 10000

        end
        battery_type = batt_num

        -- clear the filter on type change
        percentage_filt = nil
    end

    -- Calculate the remaining percentage from voltage from the interpolation table
    local percentage
    if battery_type == 0 then
        --  LiPo curve
        percentage = interp_table(LiPo_volt, LiPo_pct, voltage)
    else
        -- SS Li-Ion 4A curve
        percentage = interp_table(LiIon_voltage, LiIon_pct, voltage)
    end

    -- constrain 0 to 100
    percentage = math.max(math.min(percentage,100),0)

    if percentage_filt then
        -- filter new reading
        percentage_filt = (percentage_filt * exp_filter) + (percentage * (1 - exp_filter))
    else
        -- init, set filter to latest reading
        percentage_filt = percentage
    end

    -- update the battery percentage
    -- note that current draw will still decrease the remaining percentage until we update it again
    battery:reset_remaining(battery_instance, percentage_filt)

    -- debug print
    -- gcs:send_text(0, string.format("Battery Set to: %.1f",percentage_filt))

    -- run at 1hz
    return update, 1000

end

-- give battery monitor 10 seconds to come good on boot
return update, 1000
