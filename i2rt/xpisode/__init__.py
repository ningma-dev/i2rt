"""i2rt robots as xpisode device peers (https://github.com/ningma-dev/xpisode).

Needs the ``xpisode`` extra: ``pip install "i2rt[xpisode]"``. See ``examples/xpisode``.
"""

from i2rt.xpisode.yam import YamFollower, YamLeader

__all__ = ["YamFollower", "YamLeader"]
