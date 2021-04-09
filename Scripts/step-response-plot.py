from math import ceil
import argparse
import sys
import os
import matplotlib.pyplot as plt

"""
Input data file syntax:

# comment
TIME1 INPUT1 OUTPUT1
TIME2 INPUT2 OUTPUT2
...
"""

def zero_centered_range(range_max, range_half_ticks):
    range_step = ceil(range_max / range_half_ticks)
    h = [range_step + i * range_step for i in range(range_half_ticks)]
    l = [-x for x in h]
    l.reverse()
    return l + [0] + h
    
def do_plot_step_response(t, i, o, degrees, name="step response plot"):
    fig = plt.figure(name)
    ax1 = plt.axes()
    ax2 = ax1.twinx()
    
    ax1.set_xlabel("Time (ms)")
    ax1.set_ylabel("Output (%)")
    ax2.set_ylabel("Input (°)")
    
    ax1.plot(t, o, color="dodgerblue")
    ax2.plot(t, i, linestyle="dashed", color="limegreen")
    ax1.axhline(y=0,  linestyle="dotted", color="gray")
    
    r1 = zero_centered_range(100, 10)
    r2 = zero_centered_range(degrees, 10)

    ax1.set_yticks(r1)
    ax2.set_yticks(r2)
    
    ax1.set_ylim(r1[0], r1[len(r1) - 1])
    ax2.set_ylim(r2[0], r2[len(r2) - 1])
    
    
    fig.tight_layout()
    plt.show()
    
def plot_step_response(data_file, verbose=False, degrees=90):
    try:
        with open(data_file, errors="ignore") as f:
            data = []
            t = []
            i = []
            o = []
            
            for l in f.readlines():
                try:
                    l = l.strip()
                    if not l.startswith("#"):
                        entry = l.split()
                        if len(entry) == 3:
                            t.append(float(entry[0]))
                            i.append(float(entry[1]))
                            o.append(float(entry[2]))
                    elif verbose:
                        print(l)
                except Exception as e:
                    print(e, file=sys.stderr)
                    
            do_plot_step_response(t, i, o, degrees=degrees, 
                                  name=os.path.basename(data_file))
    except OSError:
        print(f"Failed to open '{data_file}'", file=sys.stderr)
        
    
def main():
    parser = argparse.ArgumentParser(description=
         "Plot step response graph"
     )
    parser.add_argument("data_file", metavar="DATA_FILE",
                        type=str,
                        help="path of the data file")
    parser.add_argument("-v", "--verbose",                                          
                        action="store_const", const=True, default=False,            
                        dest="verbose",                                             
                        help="print comment lines")
    parser.add_argument("-d", "--degrees",
                        type=int, default=90,
                        dest="degrees",
                        help=f"set y-axis scale max and min degrees (default is 90)")  

    
    args = vars(parser.parse_args())
    data_file = args.get("data_file")
    verbose = args.get("verbose")
    degrees = args.get("degrees")
    plot_step_response(data_file, verbose=verbose, degrees=degrees)
    
    
if __name__ == "__main__":
    main()