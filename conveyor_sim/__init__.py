"""Conveyor (试管流水线) tick-based simulator.

Public entrypoints:
- ConveyorSystem (from conveyor_sim.sim)
- SimpleScheduler (from conveyor_sim.scheduler)
"""
from .sim import ConveyorSystem
from .scheduler import SimpleScheduler
from .models import StationStart, DiverterTransfer
