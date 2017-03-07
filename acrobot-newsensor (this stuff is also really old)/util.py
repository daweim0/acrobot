def despike_slope(data):
    for i in range(2,len(data)):
        # remove sloped spikes
        if data[i] > data[i-2] and (data[i-1] > data[i] or data[i-1] < data[i-2]):
            data[i-1] = (data[i] + data[i-2]) / 2
        elif data[i] < data[i-2] and (data[i-1] > data[i] or data[i-1] > data[i-2]):
            data[i-1] = (data[i] + data[i-2]) / 2
    
def despike_flat(data):
    for i in range(2,len(data)):
        # remove single point spikes
        if data[i] == data[i-2] and data[i] != data[i-1]:
            data[i-1] = data[i]

        # remove double point spikes
        if data[i] == data[i-3]:
            if data[i] != data[i-1]: data[i-1] = data[i]
            if data[i] != data[i-2]: data[i-2] = data[i]

            
# expects integer values
def cleanup(data):
    despike_flat(data)
    despike_slope(data)

def draw_fit(dt,d, slen = 50):
    t = arange(dt[0],dt[-1],2)        # points to apply fit to
    a,b,c,g = polyfit(dt,d,3)          # cubic fit
    f = a*t**3 + b*t**2 + c*t + g      # generate the line
    l = d[-2]
    lt = dt[-2]
    st = array([lt-slen, lt, lt+slen])    # time for slope line
    slope = 3*a*lt**2 + 2*b*lt + c
    s = array([l - slope*slen, l, l + slope*slen])
    plot(t,f,'c')
    plot(st,s,'o-m')
