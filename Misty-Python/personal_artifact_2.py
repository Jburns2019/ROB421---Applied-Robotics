from mistyPy.Robot import Robot
from mistyPy.Events import Events
import time
from functools import partial

def distance_reading(misty: Robot, event: dict=None):
    # if event != None and event['message']['inHazard']:
    report = event['message']
    report.pop('averageTimeMs', None)
    report.pop('created', None)
    if not 'Down' in report['sensorPosition']:
        print(report)
    
def start(misty: Robot):
    misty.RegisterEvent(event_name='time_of_flight', event_type=Events.TimeOfFlight,  callback_function = partial(distance_reading, misty), keep_alive=True)
    
    misty.KeepAlive()
    while True:
        time.sleep(1)

if __name__ == '__main__':
    ipAddress = '192.168.0.116'
    try:
        misty = Robot(ipAddress)
        # start(misty)
    except:
        print(f"Connection to {ipAddress} failed.")
