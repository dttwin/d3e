"""
Define common functions for the main_TLSCAV.py script file
"""

## IMPORT MODULES
import openpyxl
import scipy.stats
import warnings
import matplotlib.pyplot as plt
import numpy as np
from math import log

## FUNCTIONS
def frange(start, end=None, inc=None):
    """A range function, that does accept float increments"""

    if end == None:
        end = start + 0.0
        start = 0.0

    if inc == None:
        inc = 1.0

    L = []
    while 1:
        next = start + len(L) * inc
        if inc > 0 and next >= end:
            break
        elif inc < 0 and next <= end:
            break
        L.append(next)

    return L

def getExcelTargetList(filename):
    """Get Values of Each Row of the First Collum of an Excel file"""

    book = openpyxl.load_workbook(filename, data_only=True)
    sheet = book.active
    rows = sheet.rows
    file_input_vals = []
    for row in rows:
        for cell in row:
            val = cell.value
            # print('LWB: type(cell.value)=', type(cell.value))
            file_input_vals.append(val)

    return file_input_vals

def merge_two_dicts(x, y):
    z = x.copy()  # start with x's keys and values
    z.update(y)  # modifies z with y's keys and values & returns None
    return z

def check_distribution(dataset,name_set):
    """Check which distribution each set of the dataset follows (it should be a 1-D array of observations of
    random variables). The argument should be a Python list (not Numpy array) in which each main index (collumn) is
    a set of data. This function is based on a blog post by Andre Dietrich, currently
    found at http://www.aizac.info/simple-check-of-a-sample-against-80-distributions/"""

    # just for surpressing warnings
    warnings.simplefilter('ignore')

    # list of all available CONTINUOUS distributions
    cdfs = [
         "norm",  # Normal (Gaussian)
        #"alpha",  # Alpha
        #"anglit",  # Anglit
        #"arcsine",  # Arcsine
        #"beta",  # Beta
        #"betaprime",  # Beta Prime
        #"bradford",  # Bradford
        #"burr",  # Burr
        #"cauchy",  # Cauchy
        #"chi",  # Chi
        #"chi2",  # Chi-squared
        #"cosine",  # Cosine
        #"dgamma",  # Double Gamma
        #"dweibull",  # Double Weibull
        "erlang",  # Erlang
        #"expon",  # Exponential
        #"exponweib",  # Exponentiated Weibull
        "exponpow",  # Exponential Power
        #"fatiguelife",  # Fatigue Life (Birnbaum-Sanders)
        #"foldcauchy",  # Folded Cauchy
        #"f",  # F (Snecdor F)
        #"fisk",  # Fisk
        #"foldnorm",  # Folded Normal
        #"frechet_r",  # Frechet Right Sided, Extreme Value Type II
        #"frechet_l",  # Frechet Left Sided, Weibull_max
        "gamma",  # Gamma
        #"gausshyper",  # Gauss Hypergeometric
        "genexpon",  # Generalized Exponential
        #"genextreme",  # Generalized Extreme Value
        "gengamma",  # Generalized gamma
        #"genlogistic",  # Generalized Logistic
        #"genpareto",  # Generalized Pareto
        #"genhalflogistic",  # Generalized Half Logistic
        #"gilbrat",  # Gilbrat
        #"gompertz",  # Gompertz (Truncated Gumbel)
        #"gumbel_l",  # Left Sided Gumbel, etc.
        #"gumbel_r",  # Right Sided Gumbel
        #"halfcauchy",  # Half Cauchy
        #"halflogistic",  # Half Logistic
        #"halfnorm",  # Half Normal
        #"hypsecant",  # Hyperbolic Secant
        "invgamma",  # Inverse Gamma
        #"invweibull",  # Inverse Weibull
        #"johnsonsb",  # Johnson SB
        #"johnsonsu",  # Johnson SU
        #"laplace",  # Laplace
        "logistic",  # Logistic
        #"loggamma",  # Log-Gamma
        #"loglaplace",  # Log-Laplace (Log Double Exponential)
        #"lognorm",  # Log-Normal
        #"lomax",  # Lomax (Pareto of the second kind)
        #"maxwell",  # Maxwell
        #"mielke",  # Mielke's Beta-Kappa
        #"nakagami",  # Nakagami
        #"ncx2",  # Non-central chi-squared
        #"ncf",   #Non-central F
        #"nct",  # Non-central Student's T
        #"pareto",  # Pareto
        #"powerlaw",  # Power-function
        #"powerlognorm",  # Power log normal
        #"powernorm",  # Power normal
        #"rdist",  # R distribution
        #"reciprocal",  # Reciprocal
        #"rayleigh",  # Rayleigh
        #"rice",  # Rice
        #"recipinvgauss",  # Reciprocal Inverse Gaussian
        #"semicircular",  # Semicircular
        #"t",  # Student's T
        "triang",  # Triangular
        #"truncexpon",  # Truncated Exponential
        #"tukeylambda",  # Tukey-Lambda
        #"uniform",  # Uniform
        #"vonmises",  # Von-Mises (Circular)
        #"wald",  # Wald
        #"weibull_min",  # Minimum Weibull (see Frechet)
        #"weibull_max",  # Maximum Weibull (see Frechet)
        #"wrapcauchy",  # Wrapped Cauchy
        #"kstwobign", # Kolmogorov-Smirnov two-sided test for Large N
         "ksone"  # Kolmogorov-Smirnov one-sided (no stats)
        ]

    dictionary_dataset = dict((cdf, [0, 0, 0]) for cdf in cdfs)
    length_dataset = len(dataset)

    print("\nBest fits for one set of data of the dataset. Contains:\n\n1) name;\n2) p-value for KS test (p-values < "
          "significance level (usually 0.05) = the values DON'T follow the distribution, and p-values > 95% = sample "
          "follows distribution);\n3) the D statistic, which is the absolute max distance (supremum) between the CDFs "
          "of the data and the distribution (closer to 0 the better);\n4) number of parameters (degrees of freedom)")

    for set in range(0,length_dataset):
        data = np.array(dataset[set])

        dictionary_set = dict((cdf, [0, 0, 0]) for cdf in cdfs)
        new_cdfs = []
        for cdf in cdfs:
            # fit our data set against every probability distribution
            parameters = eval("scipy.stats." + cdf + ".fit(data)")
            # Applying the Kolmogorov-Smirnof one sided test
            D, p = scipy.stats.kstest(data, cdf, args=parameters)

            # The D statistic is the absolute max distance (supremum) between the CDFs of the two samples.
            # The closer this number is to 0 the more likely it is that the two samples were drawn from the same
            # distribution

            # The p-value is used to reject the null hypothesis (that the two samples were drawn from the same
            # distribution)
            # If the p-value is less than your significance level (usually 0.05) we reject the null hypothesis
            # If p-values < significance level (usually 0.05) = the values DON'T follow the distribution.


            # Discard all distributions that are very poor fits (not used for analyzing each set separated)
            if np.isnan(p):
                # Not the cdf append for the set when analyzing all dataset
                dictionary_set[cdf][0] = float('inf')
                dictionary_set[cdf][1] = float('inf')
                dictionary_set[cdf][2] = len(parameters)
            else:
                dictionary_set[cdf][0] = p
                dictionary_set[cdf][1] = D
                dictionary_set[cdf][2] = len(parameters)

                negative_log_likelihood = eval("scipy.stats." + cdf + ".nnlf(" + str(parameters) + ",data)")
                if negative_log_likelihood < 1e10 and p > 0:
                    new_cdfs.append(cdf)
                    dictionary_dataset[cdf][0] -= log(p)
                    dictionary_dataset[cdf][1] += negative_log_likelihood
                    dictionary_dataset[cdf][2] = len(parameters)

        cdfs = new_cdfs[:]

        # Sort Best Distributions with lowest D
        num_distrib_fitted = min(5,len(cdfs))
        if num_distrib_fitted > 0:
            print("\nSet %s out of %s sets. The %d best fit distributions for set %s are:" % (set, length_dataset,
                                                                                              num_distrib_fitted,
                                                                                              name_set[set]))
            best = sorted(dictionary_set.items(), key=lambda x: x[1][1])
            best_cdfs = []
            for ind in range(0,num_distrib_fitted):
                best_cdfs.append(best[ind][0])
            print("name".ljust(14) + "p-value".ljust(20) + "Statictic D".ljust(24) + "# parameters")
            for cdf in best_cdfs:
                # Print Best Distributions for the Set
                print(cdf.ljust(14) + str(dictionary_set[cdf][0]).ljust(20) + str(dictionary_set[cdf][1]).ljust(24), \
                      str(dictionary_set[cdf][2]))

            # Create and Plot the Histogram of each flow
            plt.figure()
            plt.hist(data, bins='auto')  # arguments are passed to np.histogram
            plt.title("Histogram of %s" % name_set[set])
            plt.show(block=False)
            plt.pause(1)
        else:
            print("\nNot found a good fit distribution for set %s out of %s sets." % (set, length_dataset))



    print("\n\nBest fits for the entire dataset. Contains:\n1) name;\n2) sum of negative log p-values for KS test "
          "(lower values indicate better fit - 0.022 is 95% confidence that the dataset follows the distribution);\n3)"
          " sum of negative log likelihoods (lower is better - also 0.022 is 95%);\n4) number of parameters (degrees "
          "of freedom)")
    num_distrib_fitted = min(5,len(cdfs))
    if num_distrib_fitted > 0:
        print("\nThe %d best fit distributions for the dataset:" % num_distrib_fitted)
        best = sorted(dictionary_dataset.items(), key=lambda x: x[1])
        best_cdfs = []
        for ind in range(0,num_distrib_fitted):
            best_cdfs.append(best[ind][0])
        print("name".ljust(14) + "neg log p KS-test".ljust(20) + "neg log likelihood".ljust(24) + "# parameters")
        for cdf in best_cdfs:
            # Print Best Distributions for the whole dataset (aggregation of each set of the datase)
            print(cdf.ljust(14) + str(dictionary_dataset[cdf][0]).ljust(20) + str(dictionary_dataset[cdf][1]).ljust(24),
                  str(dictionary_dataset[cdf][2]))
    else:
        print("\nNot found a good fit distribution for dataset.")

    input("\n\nPress any key to close")