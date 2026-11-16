#!/usr/bin/env python3
"""Compatibility entry for the online SLAM stationary verifier.

Fixed-map validation was removed with the fixed-map task route. This now runs
live-sensor SLAM with an explicitly labelled test-only zero-odometry fixture.
It does not load a map file, start a chassis, or submit navigation motion goals.
"""
from verify_online_slam import main

if __name__ == '__main__':
    raise SystemExit(main())
