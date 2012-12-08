import inspect
from collections import namedtuple
from geometry_msgs.msg import Transform


rotation = namedtuple("Rotation",["x","y","z"])
rotation.x=1
rotation.y=1
rotation.z=1
translation = namedtuple("Translation",["x","y","z","Theta"])
translation.x=0
translation.y=0
translation.z=0
translation.Theta=0

t1=Transform(rotation,translation)

print t1
print t1.rotation.x
print t1.rotation.y
print t1.rotation.z
print inspect(Transform.__init__)
