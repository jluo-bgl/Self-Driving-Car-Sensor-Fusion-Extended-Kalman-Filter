# Write a program that will iteratively update and
# predict based on the location measurements
# and inferred motions shown below.

def update(mean1, var1, measurement, measurements_uncertainty):
    new_mean = float(measurements_uncertainty * mean1 + var1 * measurement) / (var1 + measurements_uncertainty)
    new_var = 1./(1. / var1 + 1. / measurements_uncertainty)
    return [new_mean, new_var]

def predict(mean1, var1, motion, motion_uncertainty):
    new_mean = mean1 + motion
    new_var = var1 + motion_uncertainty
    return [new_mean, new_var]

measurements = [5., 6., 7., 9., 10.]
motions = [1., 1., 2., 1., 1.]
measurement_uncertainty = 4.
motion_uncertainty = 2.

current_mean = 0.
current_uncertainty = 10000.
# current_uncertainty = 0.000000001

#Please print out ONLY the final values of the mean
#and the variance in a list [mu, sig].

for n in range(len(measurements)):
    [current_mean, current_uncertainty] = update(current_mean, current_uncertainty, measurements[n], measurement_uncertainty)
    print("update new mean: {} new uncertainty: {}".format(current_mean, current_uncertainty))
    [current_mean, current_uncertainty] = predict(current_mean, current_uncertainty, motions[n], motion_uncertainty)
    print("predict new mean: {} new uncertainty: {}".format(current_mean, current_uncertainty))


print("final {} {}".format(current_mean, current_uncertainty))
