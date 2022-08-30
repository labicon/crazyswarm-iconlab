#!/usr/bin/env python3

import argparse
import subprocess

def main():
    parser = argparse.ArgumentParser(description="Wrapper for crazyflie_tools to multiplex across crazyflies")
    parser.add_argument("-s", "--subcommand", required=True, help="Subcommand of crazyflie_tools to be ran")
    parser.add_argument("-c", "--channel", default=80, help="Channel of this Crazyflie")
    parser.add_argument("-u", "--uids", nargs="+", required=True, help="UID's of each of the crazyflies in hex")
    args = parser.parse_args()
    
    for uid in args.uids:
        cmd = ["rosrun", "crazyflie_tools", args.subcommand, f"--uri=radio://0/{args.channel}/2M/E7E7E7E70{uid}"]
        print(f"Running command: {' '.join(cmd)}")
        subprocess.run(cmd)

if __name__ == "__main__":
    main()
