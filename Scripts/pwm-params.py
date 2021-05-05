import argparse
from math import ceil

"""
                    Fclock
Fpwm = --------------------------
            (PSC + 1) * (ARR + 1)
            

=>
                    Fclock
(ARR + 1) = --------------------------
                Fpwm * (PSC + 1)
                    
"""


def compute_params(clock_freq, pwm_freq,
                   proposals_max=10):
    UINT16_MAX = 2**16 - 1
    
    proposals = 0
    print("-" * 54)
    print(f"Fclock = {clock_freq}")
    print("-" * 54)
    print("Fpwm (Hz)".ljust(12) + " | " + "Tpwm (ms)".ljust(12) + 
            " | " + "PSC".ljust(12) + " | " + "ARR".ljust(12))
    print("-" * 54)
    
    clock_freq = float(clock_freq)

    for psc in range(0, UINT16_MAX + 1):
        arr = ceil((clock_freq / (pwm_freq * (psc + 1))) - 1)
        if arr <= UINT16_MAX:
            pwm_freq_real = clock_freq / ((psc + 1) * (arr + 1))
            period_real = 1000 / pwm_freq_real
            print(f"{pwm_freq_real:.2f}".ljust(12) + " | " +
                  f"{period_real:.2f}".ljust(12) + " | " +
                  f"{psc}".ljust(12) + " | " + 
                  f"{arr}".ljust(12))
        
            proposals += 1
            if proposals > proposals_max:
                break
    print("-" * 54)


def main():
    parser = argparse.ArgumentParser(description=
         "Compute PWM parameters for a given frequency"
     )
    parser.add_argument("pwm_freq", metavar="PWM_FREQ",
                        type=float,
                        help="frequency of the pwm (Hz)")
    parser.add_argument("--clock-freq", metavar="CLOCK_FREQ", dest="clock_freq",
                        type=float, nargs="?",
                        default=80000000, # 80MHz
                        help="frequency of the clock generator (Hz)")
    
    args = vars(parser.parse_args())
    clock_freq = args.get("clock_freq")
    pwm_freq = args.get("pwm_freq")
    compute_params(clock_freq, pwm_freq)
    

if __name__ == "__main__":
    main()
