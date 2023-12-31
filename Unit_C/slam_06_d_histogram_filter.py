# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""

    # --->>> Copy your previous code here.

    return Distribution(distribution.offset + delta, distribution.value)  # Replace this by your own result.


def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""

    # --->>> Copy your previous code here.
    dist_list = []
    offs = a.offset + b.offset 
    for a_val in a.values:
        res = []
        for b_val in b.values:
           res.append(a_val*b_val)
        dist_list.append(Distribution(offs,res))
        offs += 1
        
    c = Distribution.sum(dist_list)
    return c  # Replace this by your own result.


def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Copy your previous code here.
    start = min([a.start(),b.start()])
    stop = max([a.stop(),b.stop()])
    multi_dist = []
    
    for i in xrange (start,stop):
        a_val = a.value(i)
        b_val = b.value(i)
        multi_dist.append(a_val*b_val)
    
    d = Distribution(start,multi_dist)
    Distribution.normalize(d)
    
    return d   # Modify this to return your result.


if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
