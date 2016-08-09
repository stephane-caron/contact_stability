# contact\_stability

ROS package for checking multi-contact stability.

## Installation

Clone the repository:

```
git clone --recursive https://github.com/stephane-caron/contact_stability.git
```

Link it into your Catkin workspace and launch the service corresponding to the
criterion you want to compute:

```
roslaunch contact_stability pendular.launch  # pendular ZMP support area
roslaunch contact_stability static.launch    # static-equilibrium COM polygon
roslaunch contact_stability all.launch       # everything
```

## Criteria

- **Pendular ZMP support areas:** in locomotion, most of today's walking
  pattern generators regulate the angular momentum to a constant value (Linear
  Pendulum Mode). In this case, the motion is contact stable if and only if the
  whole-body ZMP lies in the [pendular ZMP support
  area](http://arxiv.org/abs/1510.03232).

- **Static-equilibrium COM polygon:** when the robot is not moving, contact
  stability is enforced if and only if the (horizontal projection of the)
  center of mass lies in this polygon.

## Algorithms

- **Bretl and Lall:** the algorithm introduced by Bretl and Lall in [this
  paper](http://dx.doi.org/10.1109/TRO.2008.2001360), and used in a wealth of
  subsequent works [[1](https://dx.doi.org/10.1177/0278364914527855),
  [2](http://dx.doi.org/10.1109/TMECH.2015.2409479),
  [3](https://hal.archives-ouvertes.fr/hal-01201060/))]. It can also be applied
  to [ZMP support areas](http://arxiv.org/abs/1510.03232).

- **Double-description method:** an algorithm used to convert between halfspace
  and vertex representation of polyhedral sets. Has been applied to compute the
  [static-equilibrium polygon](https://scaron.info/research/ijhr-2016.html) as
  well as the [pendular ZMP support area](http://arxiv.org/abs/1510.03232). It
  is faster but less numerically stable than the other method (see e.g. the
  [Appendix in this paper](https://hal.archives-ouvertes.fr/hal-01349880)).

To choose between both, set the ``ALGORITHM`` global variable in the server
script ([pendular\_server.py](/scripts/pendular_server.py) or
[static\_server.py](/scripts/static_server.py)).

## Usage

Let us compute the pendular ZMP support area. First, initialize ROS and make a
service proxy:

```python
    rospy.init_node('my_node')
    rospy.wait_for_service(
        '/contact_stability/pendular/compute_support_area')
    compute_support_area = rospy.ServiceProxy(
        '/contact_stability/pendular/compute_support_area',
        contact_stability.srv.SupportArea)
```

Next, you will need to prepare a list of ``Contact`` messages. Assuming you
already have a list ``contacts`` describing your surface contacts (e.g. a list
of ``pymanoid.Contact`` objects):

```python
    ros_contacts = [
        contact_stability.msg.Contact(
            position=geometry_msgs.msg.Point(
                contact.p[0],      # x
                contact.p[1],      # y
                contact.p[2]),     # z
            orientation=geometry_msgs.msg.Quaternion(
                contact.pose[1],   # x
                contact.pose[2],   # y
                contact.pose[3],   # z
                contact.pose[0]),  # w comes first in OpenRAVE convention
            halflen=contact.X,     # half-length of the contact surface
            halfwidth=contact.Y,   # half-width of the contact surface
            friction=contact.friction)
        for contact in contacts]
```

Finally, create a ``SupportAreaRequest`` and call the ROS service with:

```python
    p_com = geometry_msgs.msg.Point(com.x, com.y, com.z)
    req = contact_stability.srv.SupportAreaRequest(
        contacts=ros_contacts,
        mass=robot.mass,
        p_in=p_com,  # area depends on COM position
        z_out=z_out)
    res = compute_support_area(req)
```

The ``res`` object then contains the vertex representation of your support
area:

```python
    vertices = [array([v.x, v.y, v.z]) for v in res.vertices]
    rays = [array([r.x, r.y, r.z]) for r in res.rays]
```
