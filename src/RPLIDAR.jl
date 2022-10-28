# module RPLIDAR

using LibSerialPort
using Dates
using FixedPointNumbers
using StaticArrays
using TypedTables
using Dierckx
using PlotlyJS

mutable struct Replayer
    index::Int
    bytes::Vector{Vector{UInt8}}
end

Base.write(::Replayer, data) = nothing

function Base.read(r::Replayer)::Vector{UInt8}
    r.index = mod(r.index, length(r.bytes)) + 1
    r.bytes[r.index]
end

Base.close(::Replayer) = nothing

function go_live(sp = open_rplidar("COM3"))

    buffer = UInt8[]

    trace1 = scatter(;x=Float64[], y=Float64[],  mode="markers")
    l = Layout()
    p = plot([trace1], l)
    display(p)

    ref_time = now()

    try    
        write(sp, [0xa5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22])
   
        prev_time = (now() - ref_time).value * 0.001

        while true
            sleep(0.2)   
            time = (now() - ref_time).value * 0.001

            data = read(sp)
            measurements = process_next!(buffer, data, prev_time, time)
           
            xy = distance_angle_to_XY.(measurements);

            trace1[:x] = getindex.(xy,1)
            trace1[:y] = getindex.(xy,2)
            react!(p, [trace1], l)
            
            prev_time = time
        end

    catch e
        if e isa InterruptException
            println("Done processing.")
        else
            rethrow(e)
        end
    finally
        write(sp, [0xa5, 0x25])
        sleep(0.25)
        read(sp)
        read(sp)
        close(sp)
    end


end


function replay(filename)

    ref_time, times, bytes = get_times_and_bytes(filename)

    deltas = diff(times)
    push!(deltas, deltas[end])

    buffer = UInt8[]

    time = times[1]

    trace1 = scatter(;x=Float64[], y=Float64[],  mode="markers")
    l = Layout()

    p = plot([trace1], l)
    display(p)

    map(bytes, deltas) do data, dt
        
        sleep(dt)

        measurements = process_next!(buffer, data, time - dt, time)
        time = time + dt

        xy = distance_angle_to_XY.(measurements);

        trace1[:x] = getindex.(xy,1)
        trace1[:y] = getindex.(xy,2)
        react!(p, [trace1], l)
        
    end

end

function process_next!(buffer::Vector{UInt8}, next_bytes::Vector{UInt8}, time_start::Float64, time_stop::Float64)

    append!(buffer, next_bytes)

    indices = find_all(buffer, [(b -> b & 0xf0 == 0xa0), (b -> b & 0xf0 == 0x50)])
    indices = filter(ii -> (ii + 84) in indices, indices)

    messages = map(i -> buffer[i:(i + 83)], indices)

    if length(messages) < 2
        return []
    end

    dt = (time_stop - time_start) / (length(messages)-1)

    times = time_start .+ dt .* (0:(length(messages)-1))

    responses = decypfer_response.(messages, times)

    these_responses = responses[1:end-1]
    next_responses = responses[2:end]

    measurements = collect(Iterators.flatten(get_measurements.(these_responses, next_responses)))

    deleteat!(buffer, (1:length(buffer)) .< (sum(length.(messages[1:end-1]))))

    println(length(buffer))

    return measurements
end


function open_rplidar(port_name)
    sp = open(port_name, 115200; mode = SP_MODE_READ_WRITE, ndatabits=8, parity=SP_PARITY_NONE, nstopbits=1)
    
    sleep(0.2)
    read(sp)
    sleep(0.2)

    write(sp, [0xa5, 0x52])
    sleep(0.1)
    response = read(sp)

    should_be = [ 
        0xa5
        0x5a
        0x03
        0x00
        0x00
        0x00
        0x06
        0x00
        0x00
        0x00
        ]
    
    if all(should_be .== response)
        println("Good to go!")
    else
        println("ERROR!")
        close(sp)
    end
    
    return sp
end

function start_and_log(sp, target, t = 31536000)

    format = dateformat"Y m d H M S s"

    write(sp, [0xa5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22])
    
    stop = now() + Second(round(Int,t))

    while now() < stop
        sleep(0.2)   
        data = read(sp)
        time = Dates.format(now(), format)
        write(target, "<entry>")
        write(target, time)
        write(target, " $(length(data))")
        write(target, "</entry>")
        write(target, data)
        println(length(data))
    end
    
    write(sp, [0xa5, 0x25])
    sleep(0.1)
    read(sp)
    
    return nothing

end

function go(filename, portname = "COM3")
    
    file = open(filename, "w")
    sp = open_rplidar(portname)

    try    
        start_and_log(sp, file)
    finally
        write(sp, [0xa5, 0x25])
        sleep(0.25)
        read(sp)
        read(sp)

        close(file)
        close(sp)
    end

end

function equals(byte)
    b -> b == byte
end

function find_all(bytes::AbstractVector, magic_bytes::Vector{UInt8})
    find_all(bytes, map(u -> equals(u), magic_bytes))
end

function find_all(bytes::AbstractVector, magic_string::AbstractString)
    find_all(bytes, UInt8.(collect(magic_string)))
end

function find_all(bytes::AbstractVector, magic_bytes::Vector{<:Function})

    i_magic = 1

    found = Int[]

    for i_bytes in 1:length(bytes)

        if magic_bytes[i_magic](bytes[i_bytes])
            i_magic += 1;
        elseif magic_bytes[1](bytes[i_bytes])
            i_magic = 2
        else
            i_magic = 1;
        end

        if i_magic > length(magic_bytes)
            push!(found, i_bytes - length(magic_bytes) + 1)
            i_magic = 1
        end
    end

    return found
end

function time_from_string(s)
    s = split(s, ['-', ' ', '.'])

    time = DateTime(
        parse(Int, s[1]),
        parse(Int, s[2]),
        parse(Int, s[3]),
        parse(Int, s[4]),
        parse(Int, s[5]),
        parse(Int, s[6]),
        parse(Int, s[7])
     )

end

function get_times_and_bytes(filename)

    data = read(filename)
    starts = find_all(data, "<entry>") .+ 7
    stops = find_all(data, "</entry>") .-1
    data_starts = stops .+ 9

    stamps = map(starts, stops) do a,b
        string(Char.(data[a:b])...)
    end

    lengths = map(s -> parse(Int, s[end]), split.(stamps))
    time_stamps = time_from_string.(stamps)
    
    ref_time = time_stamps[1]
    ref_time = DateTime(Year(ref_time), Month(ref_time), Day(ref_time))

    rel_times = getproperty.(time_stamps .- Ref(ref_time), :value) .* 0.001

    bytes = map(data_starts, lengths) do start, len
        data[start:(start + len - 1)]
    end

    return ref_time, rel_times, bytes

end

function read_log_file(filename)

    ref_time, times, bytes = get_times_and_bytes(filename)

    # data = read(filename)
    # starts = find_all(data, "<entry>") .+ 7
    # stops = find_all(data, "</entry>") .-1
    # data_starts = stops .+ 9

    # stamps = map(starts, stops) do a,b
    #     string(Char.(data[a:b])...)
    # end

    # lengths = map(s -> parse(Int, s[end]), split.(stamps))
    # time_stamps = time_from_string.(stamps)
    
    # ref_time = time_stamps[1]
    # ref_time = DateTime(Year(ref_time), Month(ref_time), Day(ref_time))

    # times = getproperty.(time_stamps .- Ref(ref_time), :value) .* 0.001

    # bytes = map(data_starts, lengths) do start, len
    #     data[start:(start + len - 1)]
    # end

    all_bytes = collect(Iterators.flatten(bytes))
    lengths = length.(bytes)

    # @show length(all_bytes)
    # @show sum(lengths)

    good_times = lengths .> 50;

    times = times[good_times]
    byte_moments = cumsum(lengths[good_times])
    spline = Spline1D(byte_moments, times, k = 1, bc = "extrapolate")

    indices = find_all(all_bytes, [(b -> b & 0xf0 == 0xa0), (b -> b & 0xf0 == 0x50)])
    indices = filter(ii -> (ii + 84) in indices, indices)

    messages = map(i -> all_bytes[i:(i + 83)], indices)
    seconds = spline.(indices)

    measurements = get_measurements(decypfer_response.(messages, seconds))

    return (; measurements, ref_time) #Table(messages = messages, rel_time = seconds, date_time = time_stamps[1] .+ Millisecond.(round.(Int, seconds .* 1000)))
end

function get_measurements(this_message, next_message)
    
    k = 1:32
    cabins = this_message.cabins[ceil.(Int, k / 2)]

    angle_diff_k = mod(next_message.start_angle - this_message.start_angle, 360.0) / 32.0

    dt = (next_message.time - this_message.time) / 32

    get_measurement.(cabins, k, this_message.start_angle, angle_diff_k, this_message.time, dt)

end

function get_measurements(responses)
    collect(Iterators.flatten(get_measurements.(responses[1:end-1], responses[2:end])))
end

function decypfer_response(message, time)
   
    checksum = (message[1] & 0x0f) + (message[2] << 4)
    
    start_angle = Int16(message[3]) | (Int16(message[4] & 0x7f) << 8)

    start_angle_value = reinterpret(Fixed{Int16, 6}, start_angle)
    
    reset = (message[4] >> 7) == 0x01

    cabins = Iterators.partition(message[5:end], 5)

    return (
        reset = reset,
        start_angle = start_angle_value,
        cabins = decypfer_cabin.(cabins),
        time = time
    )
end

function decypfer_cabin(cabin)

    s1 = (Int8(cabin[1] & 0x0f) << 7) >> 3
    s2 = (Int8(cabin[3] & 0x0f) << 7) >> 3
    d1 = Int8(cabin[5] & 0xf)
    d2 = Int8(cabin[5] >> 4)

    dtheta1 = Int16(s1 | d1)
    dtheta2 = Int16(s2 | d2)

    distance1 = (UInt16(cabin[1]) >> 1) | (UInt16(cabin[2]) << 7)
    distance2 = (UInt16(cabin[3]) >> 1) | (UInt16(cabin[4]) << 7)

    return (
        distance1 = distance1 * 0.0005,
        distance2 = distance2 * 0.0005,
        dtheta1 = reinterpret(Fixed{Int16, 11}, dtheta1),
        dtheta2 = reinterpret(Fixed{Int16, 11}, dtheta2)
    )
end

function unwrap(signal, period=360)
    result = zeros(length(signal))
    result[2:end] .= cumsum(rem.(diff(signal), period, RoundNearest))
    result .+= rem(signal[1],360, RoundNearest)
end

function distance_angle_to_XY(distance_angle)
    angle = deg2rad(distance_angle[2])
    distance = distance_angle[1]
    x = distance * sin(angle)
    y = distance * cos(angle)
    return SVector{2}(x,y)
end

function get_measurement(cabin, k, start_angle, angle_offset_per_k, time, dt_per_k)
    
    dtheta = isodd(k) ? cabin.dtheta1 : cabin.dtheta2
    distance = isodd(k) ? cabin.distance1 : cabin.distance2
    angle = start_angle + (angle_offset_per_k * k) + dtheta

    return SVector{3}(distance, angle, time + k * dt_per_k)
end

# end # module
