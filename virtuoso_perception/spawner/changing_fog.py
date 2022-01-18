import asyncio

import pygazebo
from pygazebo.msg.v11 import world_stats_pb2, fog_pb2, color_pb2

#HOST, PORT = 'jehan-VirtualBox', 42729

async def subscriber_loop():
    print('hello1')
    manager = await pygazebo.connect()#(HOST, PORT))
    print(manager)
    

    async def callback2(data):
        print(data)
        message = world_stats_pb2.WorldStatistics.ParseFromString(data)
        print(message)

    subscriber = manager.subscribe('/gazebo/world/world_stats', 'gazebo.msgs.WorldStatistics', callback2)
    print(dir(subscriber),'\n')

    print(subscriber.__dict__)

async def publisher_loop():
    manager = await pygazebo.connect()
    
    publisher = manager.advertise('/gazebo/default/fog', 'gazebo.msgs.Fog')

    fog_color = color_pb2.Color()
    fog_color.r = 1
    fog_setting = fog_pb2.Fog(type=2, color=fog_color, start=0)
    fog_setting.density = 1000

    await publisher.publish(fog_setting)
    await asyncio.sleep(1.0)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(publisher_loop())
