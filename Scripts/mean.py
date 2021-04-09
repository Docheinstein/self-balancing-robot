import statistics
import sys
import argparse
import numpy as np

"""
Input data file syntax:

# comment
VALUE
...
"""

    
def compute_mean(data_file, iqr):
    try:
        with open(data_file, errors="ignore") as f:
            values = []
            ignored = 0
            for l in f.readlines():
                l = l.strip()
                if not l or l.startswith("#"):
                    continue
                try:
                    values.append(float(l))
                except:
                    ignored += 1
                    pass
            values_len = len(values)
            q1, q2 = np.percentile(values, iqr)
            values = [v for v in values if q1 <= v <= q2]
            
            m = statistics.mean(values)
            s = statistics.stdev(values)
            s_percentage = abs(100 * s / m)
            print(f"iqr   = {iqr}")
            print(f"count = {len(values)}/{values_len}")
            print("----------------")
            print(f"mean  = {m:.5f}")
            print(f"std   = {s:.5f} ({s_percentage:.2f}% of mean)")
            if ignored > 0:
                print("----------------")
                print(f"WARN: ignored {ignored} lines")
    except OSError:
        print(f"Failed to open '{data_file}'", file=sys.stderr)
        
    
def main():
    parser = argparse.ArgumentParser(description=
         "Mean of values"
     )
    parser.add_argument("data_file", metavar="DATA_FILE",
                        type=str,
                        help="path of the data file")
    parser.add_argument("-q", "--iqr", metavar="BOUND",
                        nargs=2, type=int,
                        dest="iqr", default=[0, 100],
                        help="interquartile range")

    
    args = vars(parser.parse_args())
    data_file = args.get("data_file")
    iqr = args.get("iqr")
    compute_mean(data_file, iqr=iqr)
    
    
if __name__ == "__main__":
    main()
