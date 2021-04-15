import argparse

"""
                    Fclock
Fmotor = --------------------------
            (PSC + 1) * (ARR + 1)
            

=>
                    Fclock
(ARR + 1) = --------------------------
                Fmotor * (PSC + 1)
                    
"""


def compute_params(clock_freq, motor_freq,
                   arr_min=0, proposals_max=10):
    UINT16_MAX = 2**16 - 1
    ARR_MIN = 32
    
    proposals = 0
    print("-" * 54)
    print(f"Fclock = {clock_freq}")
    print("-" * 54)
    print("Fmotor (Hz)".ljust(12) + " | " + "Tmotor (ms)".ljust(12) + " | " + "PSC".ljust(12) + " | " + "ARR".ljust(12))
    print("-" * 54)
    
    for psc in range(0, UINT16_MAX + 1):
        arr = int((clock_freq / (motor_freq * (psc + 1))) - 1)
        
        if arr_min <= arr <= UINT16_MAX:
            motor_freq_real = clock_freq / ((psc + 1) * (arr + 1))
            period_real = 1000 / motor_freq_real
            print(f"{motor_freq_real:.2f}".ljust(12) + " | " +
                  f"{period_real:.2f}".ljust(12) + " | " +
                  f"{psc}".ljust(12) + " | " + 
                  f"{arr}".ljust(12))
        
            proposals += 1
            if proposals > proposals_max:
                break
    print("-" * 54)


def main():
    parser = argparse.ArgumentParser(description=
         "Compute PWM parameters for a given frequency of the motor"
     )
    parser.add_argument("motor_freq", metavar="MOTOR_FREQ",
                        type=float,
                        help="frequency of the motor (Hz)")
    parser.add_argument("--clock-freq", metavar="CLOCK_FREQ", dest="clock_freq",
                        type=float, nargs="?",
                        default=80000000, # 80MHz
                        help="frequency of the clock generator (Hz)")
    
    args = vars(parser.parse_args())
    clock_freq = args.get("clock_freq")
    motor_freq = args.get("motor_freq")
    compute_params(clock_freq, motor_freq)
    

if __name__ == "__main__":
    main()
