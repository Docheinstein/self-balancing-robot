import argparse
import sys
import matplotlib.pyplot as plt
import numpy as np

"""
Input data file syntax:

# comment
TIME1 INPUT1 OUTPUT1
TIME2 INPUT2 OUTPUT2
...
"""

def do_plot_step_response(t, i, o):
    fig = plt.figure()
    ax1 = plt.axes()
    ax2 = ax1.twinx()
    
    ax1.set_xlabel("Time (ms)")
    ax1.set_ylabel("Output (%)")
    ax2.set_ylabel("Input (°)")
    
    ax1.plot(t, o, color="dodgerblue")
    ax2.plot(t, i, linestyle="dashed", color="limegreen")
    ax1.axhline(y=0,  linestyle="dotted", color="gray")
    
    ax1.set_yticks(range(-100, 100 + 1, 10))
    ax2.set_yticks(range(-90, 90 + 1, 10))
    
    ax1.set_ylim(-100, 100)
    ax2.set_ylim(-90, 90)
    
    fig.tight_layout()
    plt.show()
    
def plot_step_response(data_file):
    try:
        with open(data_file) as f:
            t, i, o = zip(*[l.split() for l in f.readlines() 
                            if not l.startswith("#")])
            
            t = [float(x.strip()) for x in t]
            i = [float(x.strip()) for x in i]
            o = [float(x.strip()) for x in o]
            
            do_plot_step_response(t, i, o)
    except OSError:
        print(f"Failed to open '{data_file}'", file=sys.stderr)
        
    
def main():
    parser = argparse.ArgumentParser(description=
         "Plot step response graph"
     )
    parser.add_argument("data_file", metavar="DATA_FILE",
                        type=str,
                        help="path of the data file")
    
    args = vars(parser.parse_args())
    data_file = args.get("data_file")
    plot_step_response(data_file)
    
    
if __name__ == "__main__":
    main()