import argparse

"""
    α * dt
τ = ------
    1 - α
    
=>
      τ
α = ------
    τ + dt
"""

def compute_params(time_const, sample_rate):
    dt = 1 / sample_rate
    alpha = time_const / (time_const + dt)
    print(f"tau   = {time_const}s")
    print(f"dt    = {sample_rate}Hz")
    print("-------------------")
    print(f"    a = {alpha:.6f}")
    print(f"1 - a = {1 - alpha:.6f}")


def main():
    parser = argparse.ArgumentParser(description=
         "Compute complementary filter parameters for a time constant and sample rate"
     )
    parser.add_argument("time_const", metavar="TIME_CONST",
                        type=float,
                        help="complementary filter time constant (s)")
    parser.add_argument("sample_rate", metavar="SAMPLE_RATE",
                        type=float,
                        help="sensors sample rate (Hz)")
    
    args = vars(parser.parse_args())
    time_const = args.get("time_const")
    sample_rate = args.get("sample_rate")
    compute_params(time_const, sample_rate)
    

if __name__ == "__main__":
    main()